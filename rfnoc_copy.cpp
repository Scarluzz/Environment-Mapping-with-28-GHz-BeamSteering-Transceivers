
#include <uhd/rfnoc/actions.hpp>
#include <uhd/rfnoc/block_id.hpp>
#include <uhd/rfnoc/duc_block_control.hpp>
#include <uhd/rfnoc/mb_controller.hpp>
#include <uhd/rfnoc/radio_control.hpp>
#include <uhd/rfnoc/replay_block_control.hpp>
#include <uhd/rfnoc_graph.hpp>
#include <uhd/types/tune_request.hpp>
#include <uhd/utils/graph_utils.hpp>
#include <uhd/utils/math.hpp>
#include <uhd/utils/safe_main.hpp>
#include <boost/program_options.hpp>
#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <boost/system.hpp>
#include <chrono>
#include <csignal>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <fstream>
#include <memory>
#include <thread>

namespace po = boost::program_options;


using std::cout;
using std::endl;
using namespace std::chrono_literals;

///////////////////////////////////////////////////////////////////////////////

static volatile bool stop_signal_called = false;

class NMEA_frame_manager : public std::enable_shared_from_this<NMEA_frame_manager>{
	public :
		NMEA_frame_manager(std::shared_ptr<uhd::rfnoc::rfnoc_graph> graph, std::chrono::seconds chrono, std::string nmea_frames_filename) : 
			interval_(chrono),  context_(), graph_(graph),   running_(false), nmea_frames_filename_(nmea_frames_filename), 
            mb_vector_(), file_counter_(1), frame_counter_(0) {
                std::string path = nmea_frames_filename + std::to_string(file_counter_) +".txt";
                data_out_file_ = std::ofstream(path, std::ios::out | std::ios::trunc);
                tree_ = graph_->get_tree();
                nb_mb_ = graph_->get_num_mboards();
                for(size_t i(0); i < nb_mb_; i++){
                    mb_vector_.emplace_back(graph_->get_mb_controller(i));
                }
                std::cout << "NMEA frame manager was constructed" << std::endl;
			}

		void stop_running(){
			running_ = false;
			nmea_timer_->cancel();
            nmea_thread_.join();
            std::cout << "NMEA frame manager was properly destroyed..." << std::endl;
		}

		void start_running(){
			if(!running_){
                std::cout << "Attempting to start NMEA frame manager..." << std::endl;
				running_ = true;
                nmea_timer_ = std::make_unique<boost::asio::steady_timer>(context_, interval_);
                start_timer();
                nmea_thread_ = std::thread([this]() { 
                try{ 
                    context_.run();
                }
                catch(...){
                    std::cerr << "An exception occured in NMEA frame manager, thread needs to stop !";
                }

                });
                std::cout << "NMEA frame manager successfuly started..." << std::endl;
			}
		}


	private :
		std::chrono::seconds interval_;
    	boost::asio::io_context context_;
    	std::unique_ptr<boost::asio::steady_timer> nmea_timer_;
		std::thread nmea_thread_;
		std::shared_ptr<uhd::rfnoc::rfnoc_graph> graph_;
		std::atomic<bool> running_;
		std::string nmea_frames_filename_;
        std::ofstream data_out_file_;
        std::shared_ptr<uhd::property_tree> tree_;
        std::vector<std::shared_ptr<uhd::rfnoc::mb_controller>> mb_vector_;
        unsigned int file_counter_;
        unsigned int frame_counter_;
        size_t nb_mb_;

		void start_timer() {
			auto self = shared_from_this();
        		nmea_timer_->expires_after(interval_);
       			nmea_timer_->async_wait([this, self](const boost::system::error_code& ec) {
            			if (!ec && running_) {
                			self->collect_nmea_frame(); 
					        start_timer();
            			}
        		});
    		}

		void collect_nmea_frame(){
			std::cout << "Collecting nmea frame..." << std::endl;
            std::unique_ptr<uhd::sensor_value_t> nmea_gpgga;
            std::unique_ptr<uhd::sensor_value_t> nmea_gprmc;
            if(frame_counter_ == 0){
                data_out_file_.close();
                file_counter_++;
                std::string path = nmea_frames_filename_ + std::to_string(file_counter_) +".txt";
                data_out_file_ = std::ofstream(path, std::ios::out | std::ios::trunc);
            }
            for(size_t i(0); i < nb_mb_; i++){
                nmea_gpgga = std::make_unique<uhd::sensor_value_t>(mb_vector_.at(i)->get_sensor("gps_gpgga"));
                nmea_gprmc = std::make_unique<uhd::sensor_value_t>(mb_vector_.at(i)->get_sensor("gps_gprmc"));

                data_out_file_ << nmea_gpgga->to_pp_string() << std::endl;
                data_out_file_ << nmea_gprmc->to_pp_string() << std::endl;
            }

            frame_counter_ == 20 ? frame_counter_ = 0 : frame_counter_++;
        }

};

/*void verify_gpsdo_sync(const uhd::rfnoc::rfnoc_graph &graph)
{
	uhd::time_spec_t time_last_pps = usrp->get_time_last_pps();
	while (time_last_pps == usrp->get_time_last_pps()) {
		boost::this_thread::sleep(boost::posix_time::milliseconds(1));
	}

	// Sleep a little to make sure all devices have seen a PPS edge
	boost::this_thread::sleep(boost::posix_time::milliseconds(200));
s
	// Compare times across all mboards
	bool all_matched = true;
	uhd::time_spec_t mboard0_time = usrp->get_time_last_pps(0);
	for (size_t mboard = 1; mboard < usrp->get_num_mboards(); ++mboard) {
		uhd::time_spec_t mboard_time = usrp->get_time_last_pps(mboard);
		if (mboard_time != mboard0_time) {
			all_matched = false;
			std::cerr << boost::format(
				"ERROR: Times are not aligned: USRP "
				"0=%0.9f, USRP %d=%0.9f")
				% mboard0_time.get_real_secs()
				% mboard
				% mboard_time.get_real_secs()
			          << std::endl;
		}
	}
	if (all_matched) {
		std::cout << "SUCCESS: USRP times aligned" << std::endl;
	} else {
		std::cout << "ERROR: USRP times are not aligned" << std::endl;
		exit(EXIT_FAILURE);
	}
}*/

void sync_gpsdo(const std::shared_ptr<uhd::rfnoc::rfnoc_graph>& graph)

{
	std::shared_ptr<uhd::rfnoc::mb_controller> usrp;
	uhd::time_spec_t gps_time;
	for(size_t mboard = 0; mboard < graph->get_num_mboards(); ++mboard) {
		usrp = graph->get_mb_controller(mboard);
		std::cout << "Board's name : " << usrp->get_mboard_name() << std::endl; 
		std::cout << "Current time source for mboard " << mboard << " : " << usrp->get_time_source() << std::endl;
		usrp->set_clock_source("gpsdo");
		usrp->set_time_source("gpsdo");
		std::cout << "Current time source for mboard " << mboard << " : " << usrp->get_time_source() << std::endl;
		std::cout << "Number of time keepers available : " << usrp->get_num_timekeepers() << std::endl;
		if(!usrp->get_sensor("ref_locked").to_bool()) {
			std::cerr << boost::format(
				"GPS ref not locked on board %zu")
				% mboard;
			exit(EXIT_FAILURE);
		}
		
		std::cout << "GPS refs locked. Verifying constellation lock..." << std::endl;
		size_t num_failed = 0;
		uhd::sensor_value_t gps_info(usrp->get_sensor("gps_gpgga"));
		std::cout << boost::format(
			"gps_gppda_info:\n"
			"\tName: %s\n"
			"\tValue: %s\n"
			"\tUnit: %s\n"
			"\tType: %s\n"
			"\tConverted:%s\n")
			% gps_info.name
			% gps_info.value
			% gps_info.unit
			% gps_info.type
			% gps_info.to_pp_string()
		          << std::endl;
		
		while(!(usrp->get_sensor("gps_locked").to_bool())) {
			++num_failed;
                        std::cerr << boost::format(
					"GPS constellation not locked on board %zu.\nRetrying (%zu/100)...\n")
					% mboard % num_failed;
			boost::this_thread::sleep(boost::posix_time::seconds(2));
			if(num_failed > 100) {
				std::cerr << boost::format(
					"GPS not locked on board %zu."
					" Wait a few minutes and try again.\n")
					% mboard;
				exit(EXIT_FAILURE);
			}
		}
		auto time_keeper = usrp->get_timekeeper(0);
		gps_time = uhd::time_spec_t(
		int64_t(usrp->get_sensor(
			"gps_time").to_int()));
		time_keeper->set_time_next_pps(gps_time);
		//usrp->set_time_next_pps(gps_time, mboard);
		
		//boost::this_thread::sleep(boost::posix_time::seconds(2));
	}
	//gps_time = uhd::time_spec_t(
	//		int64_t(graph->get_mb_controller(0)->get_sensor(
	//			"gps_time").to_int()));

	//graph->synchronize_devices(gps_time, false);
	boost::this_thread::sleep(boost::posix_time::seconds(2));
}

// Ctrl+C handler
void sig_int_handler(int)
{
    stop_signal_called = true;
}


int UHD_SAFE_MAIN(int argc, char* argv[])
{
    // We use sc16 in this example, but the replay block only uses 64-bit words
    // and is not aware of the CPU or wire format.
    std::string wire_format("sc16");
    std::string cpu_format("sc16");

    /************************************************************************
     * Set up the program options
     ***********************************************************************/
    std::string args, tx_args, file, ant, ref;
    double rate, freq, gain, bw, lo_offset;
    size_t radio_id, radio_chan, replay_id, replay_chan, nsamps;

    po::options_description desc("Allowed Options");
    // clang-format off
    desc.add_options()
        ("help", "help message")
        ("args", po::value<std::string>(&args)->default_value(""), "multi uhd device address args")
        ("tx_args", po::value<std::string>(&tx_args), "Block args for the transmit radio")
        ("radio_id", po::value<size_t>(&radio_id)->default_value(0), "radio block to use (e.g., 0 or 1).")
        ("radio_chan", po::value<size_t>(&radio_chan)->default_value(0), "radio channel to use")
        ("replay_id", po::value<size_t>(&replay_id)->default_value(0), "replay block to use (e.g., 0 or 1)")
        ("replay_chan", po::value<size_t>(&replay_chan)->default_value(0), "replay channel to use")
        ("nsamps", po::value<size_t>(&nsamps)->default_value(0), "number of samples to play (0 for infinite)")
        ("file", po::value<std::string>(&file)->default_value("usrp_samples.dat"), "name of the file to read binary samples from")
        ("freq", po::value<double>(&freq), "RF center frequency in Hz")
        ("lo-offset", po::value<double>(&lo_offset), "Offset for frontend LO in Hz (optional)")
        ("rate", po::value<double>(&rate), "rate of radio block")
        ("gain", po::value<double>(&gain), "gain for the RF chain")
        ("ant", po::value<std::string>(&ant), "antenna selection")
        ("bw", po::value<double>(&bw), "analog front-end filter bandwidth in Hz")
        ("ref", po::value<std::string>(&ref), "clock reference (internal, external, mimo, gpsdo)")
        ("dot", "instead of a textual representation, generate a dot graph of the RFNoC connections")
    ;
    // clang-format on
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    // Print help message
    if (vm.count("help")) {
        cout << "UHD/RFNoC Replay samples from file " << desc << endl;
        cout << "This application uses the Replay block to playback data from a file to "
                "a radio"
             << endl
             << endl;
        return EXIT_FAILURE;
    }


    /************************************************************************
     * Create device and block controls
     ***********************************************************************/
    std::cout << std::endl;
    std::cout << "Creating the RFNoC graph with args: " << args << "..." << std::endl;
    auto graph = uhd::rfnoc::rfnoc_graph::make(args); //args contient l'adresse du multi uhd device... un rfnoc_graph est l'objet fondamental pour gérer une session uhd avec un device rfnoc capable.
    std::cout << "Detected mother boards : " << graph->get_num_mboards() << std::endl;
    size_t nbr_boards =  graph->get_num_mboards();


    // Create handle for radio object
    //uhd::rfnoc::block_id_t radio_ctrl_id(0, "Radio", radio_id); //Crée un objet pour identifier un RFNoC block. arg 1 : identifiant du device, arg 2 : Type de block, arg 3 : numéro 
    //auto radio_ctrl = graph->get_block<uhd::rfnoc::radio_control>(radio_ctrl_id); //utilise le graph pour obtenir un objet permettant le contrôle effectif du bloc RFNoC (à l'aide de son ID)
    std::vector<uhd::rfnoc::block_id_t> radio_ctrl_ids;
    std::vector<uhd::rfnoc::block_id_t> replay_ctrl_ids;

    for(size_t i(0); i < nbr_boards; i++){
	   radio_ctrl_ids.emplace_back(i, "Radio", radio_id);
	   replay_ctrl_ids.emplace_back(i, "Replay", replay_id);
	   if(!graph->has_block(replay_ctrl_ids[i])){
		cout << "Unable to find block \"" << replay_ctrl_ids[i] << "\"" << endl;
        	return EXIT_FAILURE;
	   }
    }


    auto radio_ctrl = graph->get_block<uhd::rfnoc::radio_control>(radio_ctrl_ids[0]);
    auto replay_ctrl = graph->get_block<uhd::rfnoc::replay_block_control>(replay_ctrl_ids[0]); //Même principe que le radio block mais pour le replay block
    auto radio_ctrls = std::vector<decltype(radio_ctrl)>();
    auto replay_ctrls = std::vector<decltype(replay_ctrl)>();
    
    radio_ctrls.emplace_back(radio_ctrl);
    replay_ctrls.emplace_back(replay_ctrl);

    for (size_t i(1); i < nbr_boards; i++){
	    radio_ctrls.emplace_back(graph->get_block<uhd::rfnoc::radio_control>(radio_ctrl_ids[i]));
	    replay_ctrls.emplace_back(graph->get_block<uhd::rfnoc::replay_block_control>(replay_ctrl_ids[i]));
    }
	
    std::vector<uhd::rfnoc::duc_block_control::sptr> duc_ctrls = std::vector<uhd::rfnoc::duc_block_control::sptr>();
    std::vector<size_t> duc_chans = std::vector<size_t>();
    uhd::rfnoc::duc_block_control::sptr duc_ctrl;
    // Connect replay to radio
    for (size_t i(0); i < nbr_boards; i++){

    	auto edges = uhd::rfnoc::connect_through_blocks(
        	graph, replay_ctrl_ids[i], replay_chan, radio_ctrl_ids[i], radio_chan); //Pas une connection directe, connecte au travers d'autres blocks

    	// Check for a DUC connected to the radio
    	//uhd::rfnoc::duc_block_control::sptr duc_ctrl;
    	size_t duc_chan = 0;
    	for (auto& edge : edges) {
        	auto blockid = uhd::rfnoc::block_id_t(edge.dst_blockid);
        	if (blockid.match("DUC")) {
            		duc_ctrl = graph->get_block<uhd::rfnoc::duc_block_control>(blockid);
            		duc_chan = edge.dst_port;
			duc_ctrls.emplace_back(duc_ctrl);
			duc_chans.emplace_back(duc_chan);
            		break;
        	}
    	}
    }

    duc_ctrl = duc_ctrls[0];
    size_t duc_chan = duc_chans[0];

    std::cout << "Reporting the blocks..." << std::endl;

    for(size_t i (0); i < nbr_boards; i++){
	std::cout << "MOTHERBOARD NBR " << i << std::endl;
	
    	std::cout << "Using Radio Block:  " << radio_ctrl_ids[i] << ", channel " << radio_chan
        	      << std::endl;
    	std::cout << "Using Replay Block: " << replay_ctrl_ids[i] << ", channel " << replay_chan
        	      << std::endl;
    	if (duc_ctrl) {
        	std::cout << "Using DUC Block:    " << duc_ctrls[i]->get_block_id() << ", channel "
                	  << duc_chans[i] << std::endl;
    	}	
    }

    

    /************************************************************************
     * Set up streamer to Replay block and commit graph
     ***********************************************************************/
    uhd::device_addr_t streamer_args;
    uhd::stream_args_t stream_args(cpu_format, wire_format);
    uhd::tx_streamer::sptr tx_stream;
    uhd::tx_metadata_t tx_md;

    stream_args.args = streamer_args;
    tx_stream        = graph->create_tx_streamer(nbr_boards, stream_args);
    std::cout << "Streamer successfully created with " << tx_stream->get_num_channels() << " channels..." << std::endl;
    for (size_t i(0); i < nbr_boards; i++){
	    graph->connect(tx_stream, i, replay_ctrls[i]->get_block_id(), replay_chan);
    }
    //graph->connect(tx_stream, 0, replay_ctrl->get_block_id(), replay_chan);
    graph->commit();
    std::cout << "Active connections:" << std::endl;
    if (vm.count("dot")) {
        std::cout << graph->to_dot() << std::endl;
    } else {
        for (auto& edge : graph->enumerate_active_connections()) {
            std::cout << "* " << edge.to_string() << std::endl;
        }
    }

    /************************************************************************
     * Set up radio
     ***********************************************************************/
    // Set clock reference
    if (vm.count("ref")) {
        // Lock mboard clocks
	if(ref == "gpsdo"){
		sync_gpsdo(graph);
	}
	else{
        	for (size_t i = 0; i < graph->get_num_mboards(); ++i) {
            		graph->get_mb_controller(i)->set_clock_source(ref);
			graph->get_mb_controller(i)->set_time_source(ref);
			auto time_keeper = graph->get_mb_controller(i)->get_timekeeper(0);
			time_keeper->set_time_next_pps(uhd::time_spec_t(int64_t(0)));

        	}
	}
    }

    // Apply any radio arguments provided
    if (vm.count("tx_args")) {
	for (auto it : radio_ctrls){
		(it)->set_tx_tune_args(tx_args, radio_chan);	
	}
    }

    // Set the center frequency
    if (!vm.count("freq")) {
        std::cerr << "Please specify the center frequency with --freq" << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << std::fixed;
    std::cout << "Requesting TX Freq: " << (freq / 1e6) << " MHz..." << std::endl;
    uhd::tune_request_t tune_request(freq);
    if (vm.count("lo-offset")) {
        std::cout << boost::format("Setting TX LO Offset: %f MHz...") % (lo_offset / 1e6)
                  << std::endl;
        tune_request = uhd::tune_request_t(freq, lo_offset);
    }

    if (vm.count("int-n")) {
        tune_request.args = uhd::device_addr_t("mode_n=integer");
    }

    auto tune_req_action = uhd::rfnoc::tune_request_action_info::make(tune_request);
    tune_req_action->tune_request = tune_request;
    for (size_t i(0); i < nbr_boards; i++){
    	replay_ctrls[i]->post_output_action(tune_req_action, 0);
    }
    std::cout << "TX Freq at Radio: " << (radio_ctrl->get_tx_frequency(radio_chan) / 1e6)
              << " MHz..." << std::endl
              << std::endl;

    std::cout << std::resetiosflags(std::ios::fixed);

    // Set the sample rate
    if (vm.count("rate")) {
	    for (size_t i(0); i < nbr_boards; i++){
		duc_ctrl = duc_ctrls[i];
		radio_ctrl = radio_ctrls[i];
        	std::cout << std::fixed;
        	std::cout << "Requesting TX Rate: " << (rate / 1e6) << " Msps..." << std::endl;
        	if (duc_ctrl) {
            		std::cout << "DUC block found." << std::endl;
            		duc_ctrl->set_input_rate(rate, duc_chan);
            		std::cout << "  Interpolation value is "
                      	<< duc_ctrl->get_property<int>("interp", duc_chan) << std::endl;
            		rate = duc_ctrl->get_input_rate(duc_chan);
        	} else {
            		rate = radio_ctrl->set_rate(rate);
        	}
        	std::cout << "Actual TX Rate: " << (rate / 1e6) << " Msps..." << std::endl
                  << std::endl;
        	std::cout << std::resetiosflags(std::ios::fixed);
	    }
    }

    // Set the RF gain
    if (vm.count("gain")) {
	for (size_t i(0); i < nbr_boards; i++){

		std::cout << "Regarding Motherboard n° " << i << std::endl;
        	std::cout << std::fixed;
        	std::cout << "Requesting TX Gain: " << gain << " dB..." << std::endl;
        	radio_ctrls[i]->set_tx_gain(gain, radio_chan);
        	std::cout << "Actual TX Gain: " << radio_ctrls[i]->get_tx_gain(radio_chan) << " dB..."
                  	<< std::endl
                  	<< std::endl;
        	std::cout << std::resetiosflags(std::ios::fixed);
	}
    }

    // Set the analog front-end filter bandwidth
    if (vm.count("bw")) {
	for (size_t i(0); i < nbr_boards; i++){
        	std::cout << std::fixed;
        	std::cout << "Requesting TX Bandwidth: " << (bw / 1e6) << " MHz..." << std::endl;
        	radio_ctrls[i]->set_tx_bandwidth(bw, radio_chan);
        	std::cout << "Actual TX Bandwidth: "
                  	<< (radio_ctrls[i]->get_tx_bandwidth(radio_chan) / 1e6) << " MHz..."
                  	<< std::endl
                  	<< std::endl;
        	std::cout << std::resetiosflags(std::ios::fixed);
	}
    }

    // Set the antenna
    if (vm.count("ant")) {
	    for (size_t i(0); i < nbr_boards; i++){
        	radio_ctrls[i]->set_tx_antenna(ant, radio_chan);
	    }
    }

    // Allow for some setup time
    std::this_thread::sleep_for(std::chrono::milliseconds(200));


    /************************************************************************
     * Read the data to replay
     ***********************************************************************/
    // Constants related to the Replay block
    const size_t replay_word_size =
        replay_ctrl->get_word_size(); // Size of words used by replay block
    const size_t sample_size = 4; // Complex signed 16-bit is 32 bits per sample

    //char* files_bufr[nbr_boards];
    //uhd::tx_streamer::buffs_type data_buffer(&files_bufr[0], nbr_boards);

    std::cout << "Replay word size : " << replay_word_size << std::endl;
    std::cout << "Memory size : " << replay_ctrl->get_mem_size() << std::endl;
    //Open the file
    //std::cout << file << std::endl;

     std::vector<decltype(file)> data_files = std::vector<decltype(file)>();

    for (size_t i = 0; i < nbr_boards; i++){
	data_files.push_back(file + std::__cxx11::to_string(i+1) + ".dat");
	std::cout << "Data files number " << i << " : " << data_files[i] << std::endl;
    }
    std::vector <const void*> tx_buf_ptr_void(nbr_boards, nullptr);
    char* tx_buf_ptr;
    size_t samples_to_replay(0);
    size_t words_to_replay(0);
    for (size_t i = 0; i < nbr_boards; i++){
    file = data_files[i];
    

    std::ifstream infile(file.c_str(), std::ifstream::binary);
    if (!infile.is_open()) {
        std::cerr << "Could not open specified file" << std::endl;
        return EXIT_FAILURE;
    }

    // Get the file size
    	infile.seekg(0, std::ios::end);
	long unsigned int file_size =(long unsigned int)  infile.tellg();
    	infile.seekg(0, std::ios::beg);

    	std::cout << "File size : " << file_size << std::endl;

    // Calculate the number of 64-bit words and samples to replay
    	words_to_replay   = file_size / replay_word_size;
    	samples_to_replay = file_size / sample_size;

    	int modulo_2 = samples_to_replay%2;

    	if(modulo_2){
	    	std::cout << "Warning : Attempting to transmit an uneven number of samples.. The last one has to be dropped..." << std::endl;
	    	samples_to_replay -= 1;
    	}

    // Create buffer
    	//std::vector<char> tx_buffer(samples_to_replay * sample_size); //New buffer at each iteration ! 
    	tx_buf_ptr = new char[samples_to_replay*sample_size];
    //const void* tx_buf_ptr_void [nbr_boards];
    	infile.read(tx_buf_ptr, std::streamsize (samples_to_replay * sample_size));
    	infile.close();
    	tx_buf_ptr_void[i] = tx_buf_ptr;//tx_buffer.data();

     }
    //tx_buf_ptr_void[0] = &tx_buffer[0];
    //tx_buf_ptr_void[1] = &tx_buffer[0];
    uhd::tx_streamer::buffs_type data_buffer(tx_buf_ptr_void.data(), nbr_boards);

    if(samples_to_replay == 0 || words_to_replay == 0){
	std::cout << "A weird error occured during the files reading..." << std::endl;
	return -1;
    }
    //files_bufr[0] = &tx_buffer[0];
    //files_bufr[1] = &tx_buffer[0];


    // Read file into buffer, rounded down to number of words
    //infile.read(tx_buf_ptr, samples_to_replay * sample_size);
    //infile.close();

    //Reprocess the word size

    /*std::vector<std::complex<float>> float_data(samples_to_replay);
    for (size_t i = 0; i < samples_to_replay; i++) {
        float real_part = static_cast<float>(tx_buf_ptr[2 * i]) / 32768.0f;   // Normaliser entre -1 et 1
        float imag_part = static_cast<float>(tx_buf_ptr[2 * i + 1]) / 32768.0f;
        float_data[i] = std::complex<float>(real_part, imag_part);
    }*/

    /************************************************************************
     * Configure replay block
     ***********************************************************************/
    // Configure a buffer in the on-board memory at address 0 that's equal in
    // size to the file we want to play back (rounded down to a multiple of
    // 64-bit words). Note that it is allowed to playback a different size or
    // location from what was recorded.
    std::cout << "Start configuring replay block..." <<std::endl;
    uint32_t replay_buff_addr = 0;
    uint32_t replay_buff_size = (uint32_t) (samples_to_replay * sample_size);
    //uint32_t replay_buff_size = samples_to_replay*replay_word_size;
    for(size_t i(0); i < nbr_boards; i++){
    	replay_ctrls[i]->record(replay_buff_addr, replay_buff_size, replay_chan);
    }
    	
    // Display replay configuration
    cout << "Replay file size:     " << replay_buff_size << " bytes (" << words_to_replay
         << " qwords, " << samples_to_replay << " samples)" << endl;

    cout << "Record base address:  0x" << std::hex
         << replay_ctrl->get_record_offset(replay_chan) << std::dec << endl;
    cout << "Record buffer size:   " << replay_ctrl->get_record_size(replay_chan)
         << " bytes" << endl;
    cout << "Record fullness:      " << replay_ctrl->get_record_fullness(replay_chan)
         << " bytes" << endl
         << endl;

    // Restart record buffer repeatedly until no new data appears on the Replay
    // block's input. This will flush any data that was buffered on the input.
    uint64_t sum(0);
    cout << "Emptying record buffer..." << endl;
    do {
        replay_ctrl->record_restart(replay_chan);
	replay_ctrls[1]->record_restart(replay_chan);
	sum = 0;

        // Make sure the record buffer doesn't start to fill again
        auto start_time = std::chrono::steady_clock::now();
        do {
            sum+=  replay_ctrl->get_record_fullness(replay_chan);
	    sum +=  replay_ctrls[1]->get_record_fullness(replay_chan);
            if (sum != 0)
                break;
        } while (start_time + 250ms > std::chrono::steady_clock::now());
    } while (sum);
    cout << "Record fullness:      " << replay_ctrl->get_record_fullness(replay_chan)
         << " bytes" << endl
         << endl;

    /************************************************************************
     * Send data to replay (== record the data)
     ***********************************************************************/
    cout << "Sending data to be recorded..." << endl;
    tx_md.start_of_burst = true;
    tx_md.end_of_burst   = true;
    tx_md.has_time_spec = false;
    // We use a very big timeout here, any network buffering issue etc. is not
    // a problem for this application, and we want to upload all the data in one
    // send() call.
    //*(tx_buf_ptr + samples_to_replay*sample_size) = 10;
std::cout << "Data will be sent during next instruction..." << tx_stream->get_num_channels() << std::endl;
    size_t num_tx_samps = tx_stream->send(data_buffer, samples_to_replay, tx_md, 20.0);
    std::cout << "Data was sent..." << std::endl;
    if (num_tx_samps != samples_to_replay) {
        cout << "ERROR: Unable to send " << samples_to_replay << " samples (sent "
             << num_tx_samps << ")" << endl;
        return EXIT_FAILURE;
    }

    /************************************************************************
     * Wait for data to be stored in on-board memory
     ***********************************************************************/
    cout << "Waiting for recording to complete..." << endl;
    while (replay_ctrl->get_record_fullness(replay_chan) < replay_buff_size) {
        std::this_thread::sleep_for(150ms);
    }
    for (size_t i(0); i < nbr_boards; i++){
    	cout << "Record fullness:      " << replay_ctrls[i]->get_record_fullness(replay_chan)
         	<< " bytes" << endl
         	<< endl;
    }

    std::cout << "SENSOR LIST ON THE FIRST MOTHERBOARD : " << std::endl;

    for (const auto& name : graph->get_mb_controller(0)->get_sensor_names()) {
        std::cout << name << ": " << graph->get_mb_controller(0)->get_sensor(name).to_pp_string() << std::endl;
    }


    /************************************************************************
     * Start replay of data
     ***********************************************************************/
    auto gps_time = uhd::time_spec_t(
			int64_t(graph->get_mb_controller(0)->get_sensor(
				"gps_time").to_int()) + 15);
   //auto gps_time = uhd::time_spec_t(int64_t(15)); 

    //auto gps_time = uhd::time_spec_t(int64_t(15));

    std::cout << "Emission start GPS time : " <<int64_t(graph->get_mb_controller(0)->get_sensor("gps_time").to_int()) << std::endl;
    auto nmea_frame_manager = std::make_shared<NMEA_frame_manager>(graph, std::chrono::seconds(120), "data/GPS_data_1104C");


    if (nsamps <= 0) {

        // Replay the entire buffer over and over
	 uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
         stream_cmd.time_spec = gps_time;
	 stream_cmd.stream_now = false;
        //const bool repeat = true;
        cout << "Issuing replay command for " << samples_to_replay
             << " samps in continuous mode..." << endl;
        //uhd::time_spec_t time_spec = uhd::time_spec_t(0.0);

        if(ref == "gpsdo"){
            nmea_frame_manager->start_running();    
        }
        else{
            std::cout << "Frame manager was not started..." << std::endl;
        }              
	for(size_t i(0); i < nbr_boards; i++){
        	replay_ctrls[i]->config_play(
            	replay_buff_addr, replay_buff_size, replay_chan);//, time_spec, repeat);
		replay_ctrls[i]->issue_stream_cmd(stream_cmd);
	}
        // Setup SIGINT handler (Ctrl+C)
        std::signal(SIGINT, &sig_int_handler);
	std::cout << "Stream commands were issued, check the USRP's Rx LED for monitoring the emission start..." << std::endl;
        cout << "Replaying data (Press Ctrl+C to stop)..." << endl;
        while (not stop_signal_called) {
            std::this_thread::sleep_for(100ms);
        }
        // Remove SIGINT handler

        if(ref == "gpsdo"){
	        nmea_frame_manager->stop_running();
        }
	std::signal(SIGINT, SIG_DFL);
        cout << endl << "Stopping replay..." << endl;
	for(size_t i(0); i < nbr_boards; i++){
        	replay_ctrls[i]->stop(replay_chan);
	}
        std::cout << "Letting device settle..." << std::endl;
        std::this_thread::sleep_for(1s);
    } else {
        // Replay nsamps, wrapping back to the start of the buffer if nsamps is
        // larger than the buffer size.
        replay_ctrl->config_play(replay_buff_addr, replay_buff_size, replay_chan);
        uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
        stream_cmd.num_samps = nsamps;
        cout << "Issuing replay command for " << nsamps << " samps..." << endl;
        stream_cmd.stream_now = true;
        replay_ctrl->issue_stream_cmd(stream_cmd, replay_chan);
        std::cout << "Waiting until replay buffer is clear..." << std::endl;
        const double stream_duration = static_cast<double>(nsamps) / rate;
        std::this_thread::sleep_for(
            std::chrono::milliseconds(static_cast<int64_t>(stream_duration * 1000))
            + 500ms); // Slop factor
    }

    //delete[] tx_buf_ptr_void;

    return EXIT_SUCCESS;
}
