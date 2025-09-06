#include <tlkcore_lib/tlkcore_lib.hpp>
#include <common_lib.h>
#include <iostream>
#include <vector>
#include <string.h>
#include <chrono>
#include <thread>
#include <array> 
#include <memory>
#include <csignal>
#include <string>
#include <fstream>
#include <stdint.h>

#include <uhd/rfnoc/actions.hpp>
#include <uhd/rfnoc/block_id.hpp>
#include <uhd/rfnoc/duc_block_control.hpp>
#include <uhd/rfnoc/mb_controller.hpp>
#include <uhd/rfnoc/radio_control.hpp>
#include <uhd/rfnoc/replay_block_control.hpp>
#include <uhd/rfnoc_graph.hpp>

#include <uhd/utils/graph_utils.hpp>
#include <uhd/utils/math.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/static.hpp>
#include <uhd/utils/thread.hpp>
#include <uhd/utils/thread_priority.hpp>
#include <uhd/types/tune_request.hpp>

#include <uhd/exception.hpp>
#include <uhd/usrp/multi_usrp.hpp>

#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <boost/system.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include <boost/math/special_functions/round.hpp>   
#include <boost/program_options.hpp>


namespace po = boost::program_options;
using namespace tlkcore;

bool stop_signal_called = false;

int init_beam_config(tlkcore_lib::tlkcore_ptr service, std::string sn_Tx_BBox, std::string sn_Rx_BBox, float target_freq ){
    
    float gain_db = 1;
    int theta = 0;
    int phi = 0;

    rf_mode_t mode = MODE_TX;
    printf("[Main] set TX beam with gain/theta/phi: %.1f/%d/%d\r\n", gain_db, theta, phi);
    service->set_beam_angle(sn_Tx_BBox, target_freq, mode, gain_db, theta, phi);

    mode = MODE_RX;
    printf("[Main] set RX beam with gain/theta/phi: %.1f/%d/%d\r\n", gain_db, theta, phi);
    service->set_beam_angle(sn_Rx_BBox, target_freq, mode, gain_db, theta, phi);

    return 0;
}

void sig_int_handler(int)
{
    stop_signal_called = true;
}

bool isLittleEndian() {
    uint16_t x = 0x1;
    char* ptr = reinterpret_cast<char*>(&x);
    return (*ptr == 1);
}

int main(int argc, char* argv[]){
   // We use sc16 in this example, but the replay block only uses 64-bit words
    // and is not aware of the CPU or wire format.
    std::string wire_format("sc16");
    std::string cpu_format("fc32");

    /************************************************************************
     * Set up the program options
     ***********************************************************************/
    std::string args, tx_args, file, ant_bb, ref, dot;
    double rate, freq_bb, gain, bw, lo_offset;
    size_t radio_id, radio_chan, replay_id, replay_chan, nsamps;

    po::options_description desc("Allowed Options");
    // clang-format off
    desc.add_options()
        ("help", "help message")
        ("args", po::value<std::string>(&args)->default_value("addr=192.168.100.51")) // Deleted the multi device option... hopefully not a problem
        ("tx_args", po::value<std::string>(&tx_args)->default_value("addr=192.168.100.51"), "Block args for the transmit radio")
        ("radio_id", po::value<size_t>(&radio_id)->default_value(0), "radio block to use (e.g., 0 or 1).")
        ("radio_chan", po::value<size_t>(&radio_chan)->default_value(0), "radio channel to use")
        ("replay_id", po::value<size_t>(&replay_id)->default_value(0), "replay block to use (e.g., 0 or 1)")
        ("replay_chan", po::value<size_t>(&replay_chan)->default_value(0), "replay channel to use")   
        ("nsamps", po::value<size_t>(&nsamps)->default_value(80000), "number of samples to play (0 for infinite)")
        ("file", po::value<std::string>(&file)->default_value("usrp_samples_1Packet.dat"), "name of the file to read binary samples from")
        ("freq-bb", po::value<double>(&freq_bb)->default_value(4000000000), "RF center frequency in Hz") // Given IF carrying frequency is chosen to be 4GHz here. (not 28GHZ yet !)
        //("lo-offset", po::value<double>(&lo_offset), "Offset for frontend LO in Hz (optional)")
        ("rate", po::value<double>(&rate)->default_value(1000000), "rate of radio block")
        ("gain", po::value<double>(&gain)->default_value(5), "gain for the RF chain")
        ("ant_bb", po::value<std::string>(&ant_bb)->default_value("TX/RX"), "antenna selection")
        //("bw", po::value<double>(&bw), "analog front-end filter bandwidth in Hz")
        ("ref", po::value<std::string>(&ref)->default_value("internal"), "clock reference (internal, external, mimo, gpsdo)")
        ("dot", po::value<std::string>(&dot)->default_value("dot"), "instead of a textual representation, generate a dot graph of the RFNoC connections")
    ;
    // clang-format on
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    // Print help message
    if (vm.count("help")) {
        std::cout << "UHD/RFNoC Replay samples from file " << desc << std::endl;
        std::cout << "This application uses the Replay block to playback data from a file to "
                "a radio"
             << std::endl
             << std::endl;
        return EXIT_FAILURE;
    }


    /************************************************************************
     * Create device and block controls and Graph from USRP Device
     ***********************************************************************/

     /*Graph creation*/
    std::cout << "Creating the RFNoC graph using only Tx USRP at ip : " << args << "..." << std::endl;
    auto graph = uhd::rfnoc::rfnoc_graph::make(args); //Graph principal pour gérer la connexion des blocks...

    /*Check number of MotherBoards*/
    std::cout << "Number of detected mother boards : " << graph->get_num_mboards() << std::endl; // Normalement uniquement 1 seul dans mon cas...
    size_t nbr_boards =  graph->get_num_mboards();

    /*Creating a Radio Block*/
    uhd::rfnoc::block_id_t radio_ctrl_id(0, "Radio", radio_id);
    auto radio_ctrl = graph->get_block<uhd::rfnoc::radio_control>(radio_ctrl_id);

    /*Creating a Replay Block*/
    uhd::rfnoc::block_id_t replay_ctrl_id(0, "Replay", replay_id);
    auto replay_ctrl = graph->get_block<uhd::rfnoc::replay_block_control>(replay_ctrl_id);

    /*Verify if those blocks are present in the main Graph*/
    if(!graph->has_block(replay_ctrl_id) || !graph->has_block(radio_ctrl_id))
    {
		std::cout << "Unable to find block \"" << replay_ctrl_id << "\" OR \"" << radio_ctrl_id << "\"" << std::endl;
        return EXIT_FAILURE;
    }

    /*Connect Replay--->---Radio */
    auto edges = uhd::rfnoc::connect_through_blocks(graph, replay_ctrl_id, replay_chan, radio_ctrl_id, radio_chan); //Pas une connection directe, connecte au travers d'autres blocks


    /*Get DUC Block between Replay-->--[X]-->---Radio */
    uhd::rfnoc::duc_block_control::sptr duc_ctrl;
    size_t duc_chan;

    	for (auto& edge : edges)
        {
        	auto blockid = uhd::rfnoc::block_id_t(edge.dst_blockid);
        	if (blockid.match("DUC"))
            {
            		duc_ctrl = graph->get_block<uhd::rfnoc::duc_block_control>(blockid);
            		duc_chan = edge.dst_port;
            		break;
        	}
    	}

    /*Reporting the created connected blocks...*/

    std::cout << "Reporting the blocks..." << std::endl;

    std::cout << "TX USRP MOTHERBOARD NBR : " << nbr_boards-1 << std::endl;

    std::cout << "Using Radio Block:  " << radio_ctrl_id << ", channel " << radio_chan << std::endl;

    std::cout << "Using Replay Block: " << replay_ctrl_id << ", channel " << replay_chan << std::endl;

    std::cout << "Using DUC Block: " << duc_ctrl->get_block_id() << ", channel " << duc_chan << std::endl;


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

    graph->connect(tx_stream, 0, replay_ctrl->get_block_id(), replay_chan);

    graph->commit();
    std::cout << "Active connections:" << std::endl;

    if (vm.count("dot")) 
    {
        std::cout << graph->to_dot() << std::endl;
    } 
    else 
    {
        for (auto& edge : graph->enumerate_active_connections()) 
        {
            std::cout << "* " << edge.to_string() << std::endl;
        }
    }


    /************************************************************************
     * Set up radio
     ***********************************************************************/

    // Set clock reference
    if (vm.count("ref")) {
        graph->get_mb_controller()->set_clock_source(ref);
        graph->get_mb_controller()->set_time_source(ref);
        // INCLUDE in final merged code ! (might be a source of unstable sampling between TX & RX !)
        auto time_keeper = graph->get_mb_controller()->get_timekeeper(0);
        time_keeper->set_time_next_pps(uhd::time_spec_t(int64_t(0)));
    }

    // Apply any radio arguments provided
    if (vm.count("tx_args")) {
        radio_ctrl->set_tx_tune_args(tx_args, radio_chan);	
    }

    // Set the center frequency
    if (!vm.count("freq-bb")) {
        std::cerr << "Please specify the center frequency with --freq" << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << std::fixed;
    std::cout << "Requesting TX Freq: " << (freq_bb / 1e6) << " MHz..." << std::endl;
    uhd::tune_request_t tune_request(freq_bb);
    if (vm.count("lo-offset")) {
        std::cout << boost::format("Setting TX LO Offset: %f MHz...") % (lo_offset / 1e6) << std::endl;
        tune_request = uhd::tune_request_t(freq_bb, lo_offset);
    }

    if (vm.count("int-n")) {
        tune_request.args = uhd::device_addr_t("mode_n=integer");
    }


    auto tune_req_action = uhd::rfnoc::tune_request_action_info::make(tune_request);
    tune_req_action->tune_request = tune_request;
    replay_ctrl->post_output_action(tune_req_action, 0);

    //radio_ctrl->set_tx_frequency(freq, radio_chan);

    std::cout << "TX Freq at Radio: " << (radio_ctrl->get_tx_frequency(radio_chan) / 1e6) << " MHz..." << std::endl << std::endl;

    std::cout << std::resetiosflags(std::ios::fixed);

    // Set the sample rate
    if (vm.count("rate")) {
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
        std::cout << "Actual TX Rate: " << (rate / 1e6) << " Msps..." << std::endl << std::endl;
        std::cout << std::resetiosflags(std::ios::fixed);

    }

    // Set the RF gain
    if (vm.count("gain")) {
        std::cout << std::fixed;
        std::cout << "Requesting TX Gain: " << gain << " dB..." << std::endl;
        radio_ctrl->set_tx_gain(gain, radio_chan);
        std::cout << "Actual TX Gain: " << radio_ctrl->get_tx_gain(radio_chan) << " dB..." << std::endl << std::endl;
        std::cout << std::resetiosflags(std::ios::fixed);
    }

    // Set the analog front-end filter bandwidth
    if (vm.count("bw")) {
        std::cout << std::fixed;
        std::cout << "Requesting TX Bandwidth: " << (bw / 1e6) << " MHz..." << std::endl;
        radio_ctrl->set_tx_bandwidth(bw, radio_chan);
        std::cout << "Actual TX Bandwidth: " << (radio_ctrl->get_tx_bandwidth(radio_chan) / 1e6) << " MHz..." << std::endl << std::endl;
        std::cout << std::resetiosflags(std::ios::fixed);
    }

    // Set the antenna
    if (vm.count("ant_bb")) {
        std::cout << "Requesting TX Antenna to use : " << ant_bb << "..." << std::endl;
        radio_ctrl->set_tx_antenna(ant_bb, radio_chan);
        std::vector<std::string> Ant_out = radio_ctrl->get_tx_antennas(radio_chan);
        std::cout << "Actual TX Antenna used : ";
        for (int i = 0; i < Ant_out.size(); i++){

            std::cout << Ant_out[i] << ", ";
        }
        std::cout << "..." << std::endl;
    }

    // Allow for some setup time
    std::this_thread::sleep_for(std::chrono::milliseconds(200));


    /************************************************************************
     * Read the data to replay
     ***********************************************************************/
    // Constants related to the Replay block
    const size_t replay_word_size = replay_ctrl->get_word_size(); // Size of words used by replay block
    const size_t CPU_sample_size = 8; // On utilise fc32 => 8bytes/sample
    replay_ctrl->set_record_type(cpu_format);

    std::cout << "Replay word size : " << replay_word_size << std::endl;

    std::cout << "Memory size : " << replay_ctrl->get_mem_size() << std::endl;


    size_t samples_to_replay(0);
    size_t words_to_replay(0);

    std::ifstream infile(file.c_str(), std::ifstream::binary);
    if (!infile.is_open()) {
        std::cerr << "Could not open specified file" << std::endl;
        return EXIT_FAILURE;
    }

    // Get the file size
    infile.seekg(0, std::ios::end);
	long unsigned int file_size =(long unsigned int)  infile.tellg();
    infile.seekg(0, std::ios::beg);
    std::cout << "File size : " << file_size << "Bytes!" << std::endl;


    // Calculate the number of 64-bit words and samples to replay
    words_to_replay   = file_size / replay_word_size;
    samples_to_replay = file_size / CPU_sample_size;

    std::cout << "Number of samples to replay : " << samples_to_replay << std::endl;
    std::cout << "Number of words to replay : " << words_to_replay << std::endl;

    int modulo_2 = samples_to_replay%2;

    if(modulo_2){
        std::cout << "Warning : Attempting to transmit an uneven number of samples.. The last one has to be dropped..." << std::endl;
        samples_to_replay -= 1;
    }

    //nsamps = samples_to_replay; // will be used when Replaying !

    char* tx_buf_ptr = new char[samples_to_replay*CPU_sample_size];

    std::cout << "Buffer storing the signal is of length : " << samples_to_replay*CPU_sample_size << " Bytes !\n";

    infile.read(tx_buf_ptr, std::streamsize (samples_to_replay * CPU_sample_size));
    infile.close();


    /*std::vector <const void*> tx_buf_ptr_void(nbr_boards, nullptr);
    tx_buf_ptr_void[0] = tx_buf_ptr;*/

    uhd::tx_streamer::buffs_type data_buffer(tx_buf_ptr);

    //uhd::tx_streamer::buffs_type data_buffer(tx_buf_ptr_void.data(), nbr_boards);

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
    const size_t OTW_sample_size = 4; // The replay buffer is part of the RFNoC sc16 OTW path, thus each sample weights half of their CPU format !
    uint32_t replay_buff_size = (uint32_t) (samples_to_replay * OTW_sample_size);


    std::cout << "Current Input DataType (At recording port...): " << replay_ctrl->get_record_type() << std::endl;
    std::cout << "Current Output DataType (only sc16 is available on X310 USRP model): " << replay_ctrl->get_play_type() << std::endl;
    
    replay_ctrl->record(replay_buff_addr, replay_buff_size, replay_chan);

    	
    // Display replay configuration
    std::cout << "Replay file size(OTW is sc16 !): " << replay_buff_size << " bytes (" << words_to_replay << " qwords, " << samples_to_replay << " samples)" << std::endl;

    std::cout << "Record base address:  0x" << std::hex << replay_ctrl->get_record_offset(replay_chan) << std::dec << std::endl;
    std::cout << "Record buffer size: " << replay_ctrl->get_record_size(replay_chan) << " bytes" << std::endl;
    std::cout << "Record fullness: " << replay_ctrl->get_record_fullness(replay_chan) << " bytes" << std::endl << std::endl;

    // Restart record buffer repeatedly until no new data appears on the Replay
    // block's input. This will flush any data that was buffered on the input.

    uint64_t sum(0);
    std::cout << "Emptying record buffer..." << std::endl;

    do {
        replay_ctrl->record_restart(replay_chan);
	    sum = 0;
        // Make sure the record buffer doesn't start to fill again
        auto duration = std::chrono::milliseconds(250);
        auto start_time = std::chrono::steady_clock::now();
        do {
            sum+=  replay_ctrl->get_record_fullness(replay_chan);
            if (sum != 0)
                break;
        } 
        
        while (std::chrono::steady_clock::now() - start_time < duration);
    }

    while (sum);
    std::cout << "Record fullness: " << replay_ctrl->get_record_fullness(replay_chan) << " bytes" << std::endl << std::endl;


    /************************************************************************
     * Send data to replay (== record the data)
     ***********************************************************************/
    std::cout << "Sending data to be recorded..." << std::endl;
    tx_md.start_of_burst = true;
    tx_md.end_of_burst   = true;
    tx_md.has_time_spec = false;
    
    // We use a very big timeout here, any network buffering issue etc. is not
    // a problem for this application, and we want to upload all the data in one
    // send() call.
    //*(tx_buf_ptr + samples_to_replay*sample_size) = 10;

    std::cout << "Data will be sent during next instruction..." << tx_stream->get_num_channels() << std::endl;
    std::cout << "The max number of samples per buffer per packet is : " << tx_stream->get_max_num_samps() << std::endl;
    size_t num_tx_samps = tx_stream->send(data_buffer, samples_to_replay, tx_md, 20.0);


    std::cout << "Data was sent..." << std::endl;

    if (num_tx_samps != samples_to_replay) {
        std::cout << "ERROR: Unable to send " << samples_to_replay << " samples (sent " << num_tx_samps << ")" << std::endl;
        return EXIT_FAILURE;
    }

    /************************************************************************
     * Wait for data to be stored in on-board memory
     ***********************************************************************/
    std::cout << "Waiting for recording to complete..." << std::endl;
    while (replay_ctrl->get_record_fullness(replay_chan) < replay_buff_size) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        std::cout << "Recording... Current fullness : " << replay_ctrl->get_record_fullness(replay_chan) << "..." << std::endl;
        std::cout << "Recording... Current position : " << replay_ctrl->get_record_position() << std::endl;
        
    }

    std::cout << "Record fullness: " << replay_ctrl->get_record_fullness(replay_chan) << " bytes" << std::endl << std::endl;

    std::cout << "SENSOR LIST ON THE FIRST MOTHERBOARD : " << std::endl;

    for (const auto& name : graph->get_mb_controller()->get_sensor_names()) {
        std::cout << name << ": " << graph->get_mb_controller()->get_sensor(name).to_pp_string() << std::endl;
    }


    /************************************************************************
     * Start replay of data
     ***********************************************************************/
    auto gps_time = uhd::time_spec_t(int64_t(graph->get_mb_controller(0)->get_sensor("gps_time").to_int()) + 15);

    //auto Choosen_time = uhd::time_spec_t(int64_t(15)); 

    std::cout << "Emission start GPS time : " <<int64_t(graph->get_mb_controller(0)->get_sensor("gps_time").to_int()) << std::endl;
    //auto nmea_frame_manager = std::make_shared<NMEA_frame_manager>(graph, std::chrono::seconds(120), "data/GPS_data_1104C");

    std::this_thread::sleep_for(std::chrono::seconds(10));

    if (nsamps <= 0) {
        auto time_keeper = graph->get_mb_controller()->get_timekeeper(0);
        uhd::time_spec_t TxTimePPS = time_keeper->get_time_now();

        std::cout << "Tx USRP time is : " << TxTimePPS.get_full_secs() << "," << TxTimePPS.get_frac_secs() << "sec.\n";
        
        
        // Replay the entire buffer over and over.
        uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
        uhd::time_spec_t time_spec = uhd::time_spec_t(10.0);
        stream_cmd.time_spec = time_spec;

        stream_cmd.stream_now = false;

        std::cout << "Issuing replay command for " << samples_to_replay << " samps in continuous mode..." << std::endl;

        replay_ctrl->config_play(replay_buff_addr, replay_buff_size, replay_chan);//, time_spec, repeat);
        replay_ctrl->issue_stream_cmd(stream_cmd);

        //replay_ctrl->play(replay_buff_addr, replay_buff_size, replay_chan,  time_keeper->get_time_now() + uhd::time_spec_t(0,0.1), true);
        
        // Setup SIGINT handler (Ctrl+C)
        std::signal(SIGINT, &sig_int_handler);
	    std::cout << "Stream commands were issued, check the USRP's Rx LED for monitoring the emission start..." << std::endl;
        std::cout << "Replaying data (Press Ctrl+C to stop)..." << std::endl;

        while (not stop_signal_called) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // Remove SIGINT handler

        /*if(ref == "gpsdo"){
	        nmea_frame_manager->stop_running();
        }*/

	    std::signal(SIGINT, SIG_DFL);
        std::cout << std::endl << "Stopping replay..." << std::endl;
        replay_ctrl->stop(replay_chan);

        std::cout << "Letting device settle..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    else 

    {
        // Replay nsamps, wrapping back to the start of the buffer if nsamps is
        // larger than the buffer size.
        replay_ctrl->config_play(replay_buff_addr, replay_buff_size, replay_chan);
        uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
        int multiple = 200;
        stream_cmd.num_samps = nsamps*multiple;
        std::cout << "Issuing replay command for " << nsamps*multiple << " samps...\n\n" << std::endl;
        stream_cmd.stream_now = true;
        replay_ctrl->issue_stream_cmd(stream_cmd, replay_chan);

        std::cout << "Waiting until replay buffer is clear..." << std::endl;

        const double stream_duration = static_cast<double>(nsamps*multiple) / rate;
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int64_t>(stream_duration * 1000) + 3000)); // Slop factor
        std::cout << "Finished to Push..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }
    
    std::cout << "Done!" << std::endl;

    //delete[] tx_buf_ptr_void;

    return 0;
}