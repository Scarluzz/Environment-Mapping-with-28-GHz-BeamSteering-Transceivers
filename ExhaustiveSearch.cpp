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
#include <numeric> 

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


int set_theta(tlkcore_lib::tlkcore_ptr service, std::string sn_BBox, float gain_dB, int theta ) {

    int phi = 0;

    //if (nbr_channels > 0){
    //service -> set_off_channels(sn_BBox, off_channels, nbr_channels);}

    if (theta < 0){
        theta = std::abs(theta);;
        phi = 180;
        service->change_beam_angle(sn_BBox, gain_dB, theta, phi);
    }
    else {
        phi = 360;
        service->change_beam_angle(sn_BBox, gain_dB, theta, phi);
    }

    printf("[Main] set beam with gain/theta/phi: %.1f/%d/%d\r\n", gain_dB, theta, phi);

   // service -> get_gain_phase_mode_list(sn_BBox, mode);

    return 0; 
    
}

/***********************************************************************
 * Main function
 **********************************************************************/
int UHD_SAFE_MAIN(int argc, char* argv[])
{
    
    // variable definitions
    std::string 	InFilename, OutFilename, dot, args_tx, args_rx, ant_bb, ref, sn_Tx_UD, sn_Rx_UD, sn_Tx_BBox, sn_Rx_BBox; 
    double 			tx_rate, rx_rate, freq_bb, gain_tx_bb, gain_rx_bb, lo_offset, bw, UpsampleFactor; 
    std::ofstream 	OutFile;
    std::ifstream   InputFile;
    uint64_t 		nbr_samps_per_degree;
    int             step, nbr_active_antennas, theta_min_tx, theta_max_tx, theta_min_rx, theta_max_rx;
    size_t          radio_id, radio_chan, replay_id, replay_chan;
    
    // variables with initializations
    std::string 	subdev_tx 			= "A:0 ";
    std::string		subdev_rx_bb 		= "A:0";
    //std::string 	ant_bb 				= "TX/RX";
	float 			seconds_in_future 	= 5.0;


    /*THESE VALUES ARE HARDCODED AND SHOULD BE REDEFINED WHENEVER THE DESIGNED SIGNAL IS DIFFERENT !*/
    double          Nr_Of_Tx_beams      = 8;
    double          Nr_Of_Rx_beams      = 8;
    double          samps_per_packet    = 10000;
    double          Nr_Of_packet_per_degree = 10;
    double          OverSampling        = 0; //Turn to non-zero value to activate Oversampling at Rx...
    double          LengthSignal        = 2210;
    double Max_SSBs_Stored = 64;


    std::string Exp_Name = "Moved_antenna_Setup/";

    OutFilename = Exp_Name + "Exhaustive.dat";

    std::string wire_format("sc16");
    std::string cpu_format_tx("fc32");
    std::string cpu_format_rx("fc32");
    
    
    // setup the program options
    po::options_description desc("Allowed options");
    // clang-format off
    desc.add_options()

	("help", "help message")
	("args-tx", po::value<std::string>(&args_tx)->default_value("addr=192.168.100.55"), "USRP IP address for Tx") 
	("args-rx", po::value<std::string>(&args_rx)->default_value("addr=192.168.100.52"), "USRP IP address for Rx") 
	("ref", po::value<std::string>(&ref)->default_value("external"), "clock reference (internal, external, gpsdo)")
    ("Tx_rate", po::value<double>(&tx_rate)->default_value(1000000), "sample rate of Tx")
	("Rx_rate", po::value<double>(&rx_rate)->default_value(1000000), "sample rate of Rx")

    ("Upsampling", po::value<double>(&UpsampleFactor)->default_value(1 + OverSampling*(3)), "Gain of Tx baseband signal in dB")

	("freq-bb", po::value<double>(&freq_bb)->default_value(4000000000), "Center frequency of Tx and Rx baseband signal in Hz")
	("gain-tx-bb", po::value<double>(&gain_tx_bb)->default_value(5), "Gain of Tx baseband signal in dB")
	("gain-rx-bb", po::value<double>(&gain_rx_bb)->default_value(30), "Gain of Rx baseband signal in dB")
	("nsamps-per-degree", po::value<uint64_t>(&nbr_samps_per_degree)->default_value(samps_per_packet * Nr_Of_packet_per_degree), "Number of samples per Tx/Rx beam direction")
	//("step", po::value<int>(&step)->default_value((45 - (-45))/(Nr_Of_Tx_beams-1)), "Difference in degress between two succesive angles")

    ("sn_Tx_UD", po::value<std::string>(&sn_Tx_UD)->default_value("UD-BD23310064-24"), "serial number of Tx UDBox")
    ("sn_Rx_UD", po::value<std::string>(&sn_Rx_UD)->default_value("UD-BD23310063-24"), "serial number of Rx UDBox")

    ("sn_Tx_BBox", po::value<std::string>(&sn_Tx_BBox)->default_value("D2336L118-28"), "serial number of Tx BBox")
    ("sn_Rx_BBox", po::value<std::string>(&sn_Rx_BBox)->default_value("D2336L117-28"), "serial number of Rx BBox")

    //("OutFile", po::value<std::string>(&OutFilename)->default_value("Exhaustive.dat"), "name of the output file")
    ("InputFile", po::value<std::string>(&InFilename)->default_value("SSB64_and_CSI.dat"), "name of the input file containing the TX samples")   

    ("ant_bb", po::value<std::string>(&ant_bb)->default_value("TX/RX"), "antenna selection")
    ("dot", po::value<std::string>(&dot)->default_value("dot"), "instead of a textual representation, generate a dot graph of the RFNoC connections")

    ("lo-offset", po::value<double>(&lo_offset)->default_value(0), "Offset for frontend LO in Hz (optional)")

    ("radio_id", po::value<size_t>(&radio_id)->default_value(0), "radio block to use (e.g., 0 or 1).")
    ("radio_chan", po::value<size_t>(&radio_chan)->default_value(0), "radio channel to use")
    ("replay_id", po::value<size_t>(&replay_id)->default_value(0), "replay block to use (e.g., 0 or 1)")
    ("replay_chan", po::value<size_t>(&replay_chan)->default_value(0), "replay channel to use") 

    ("theta_min_tx", po::value<int>(&theta_min_tx)->default_value(-45), "Min TX Theta (degree)")
    ("theta_max_tx", po::value<int>(&theta_max_tx)->default_value(45), "Max TX Theta (degree)")
    ("theta_min_rx", po::value<int>(&theta_min_rx)->default_value(-45), "Min RX Theta (degree)")
    ("theta_max_rx", po::value<int>(&theta_max_rx)->default_value(45), "Max RX Theta (degree)")
    ("nantennas", po::value<int>(&nbr_active_antennas)->default_value(16), "Number of active antennas")
    ;







    std::vector<int> tx_angles;
    std::vector<int> rx_angles;

    tx_angles = {-45, -30, -15, -5, 5, 15, 30, 45};
    rx_angles = {-45, -30, -15, -5, 5, 15, 30, 45};




    std::cout << "Display the vectors containing the angles to loop on : \n";

    std::cout << "Tx vector angles : ";
    for (int k = 0; k < tx_angles.size(); k++)
    {
        std::cout << tx_angles[k] <<" ";
    }

    std::cout << "\nRx vector angles : ";
    for (int k = 0; k < rx_angles.size(); k++)
    {
        std::cout << rx_angles[k] <<" ";
    }
    std::cout << std::endl;


    // clang-format on
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    // print the help message
    if (vm.count("help")) {
        std::cout << boost::format("Software to control mmWave Tx and Rx jointly. %s") % desc << std::endl;
        return ~0;
    }
    
    std::string Output_file_path = OutFilename; // Relative path in the current directory
    
    OutFile.open(Output_file_path , std::ofstream::binary);

    if (OutFile.is_open())
    {
		printf("Output file opened correctly. \n");
    }

    else
    {
		printf("OUTPUT FILE NOT OPENED !!! \n");
    }


    if(InFilename == "QPSK.dat")
    {
        LengthSignal = 1000;
    }

    rx_rate = tx_rate + OverSampling*(UpsampleFactor - 1)*tx_rate;

    // ======================================
    // Initialize BBox and UDBox
    // ======================================

    //BBox parameters 
    float target_freq = 28.0;

    //UD Parameter
    int freq_rf_khz = target_freq*1e6;
    int freq_if_khz = 4e6;
    int freq_ud_khz = 24e6; 
    
    //Init TLK services and configuration path
    printf("[Main] Start controlling\r\n");
    // Please keep this pointer to maintain instance of tlkcore.
    tlkcore_lib::tlkcore_ptr service;
    // Make a new tlkcore_lib, you can assign the path to searching tlkcore libraries.
    service = tlkcore_lib::make();
    // Please provide the device config file for lib scanning & init
    const std::string path = "config/device.conf";

    //Scan devices, init all devices and init UD state from configuration file (+add defauld beam state !!! )
    service->scan_init_dev(path);

    //Initilize beams to boresight
    init_beam_config(service, sn_Tx_BBox, sn_Rx_BBox, target_freq);


    //Set UD TX and RX UD frequencies
    service->set_ud_freq(sn_Tx_UD, freq_ud_khz, freq_rf_khz, freq_if_khz);   // ? In the API documentation, it is specified that the bandwidth should also be an input
    service->set_ud_freq(sn_Rx_UD, freq_ud_khz, freq_rf_khz, freq_if_khz);  
    


    // ============================================= ************************************************************************
    // Create and initialize USRP Tx and Rx devices * Create TX USRP Graph and blocks to implement Replay on SSB Signal
    // ============================================= ***********************************************************************/

     /*TX USRP : Graph creation*/
    std::cout << "Creating the RFNoC graph using only Tx USRP at ip : " << args_tx << "..." << std::endl;
    auto graph = uhd::rfnoc::rfnoc_graph::make(args_tx); //Graph principal pour gérer la connexion des blocks...
    
    
    std::cout << boost::format("Creating the USRP-Rx-BB device with: %s...") % args_rx << std::endl;
    uhd::usrp::multi_usrp::sptr usrp_rx_bb = uhd::usrp::multi_usrp::make(args_rx);


    /*Check number of MotherBoards*/
    std::cout << "Number of detected (TX USRP) mother boards : " << graph->get_num_mboards() << std::endl; // Normalement uniquement 1 seul dans mon cas...
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

    
    std::cout << boost::format("Setting subdevice USRP-Rx-BB device to: %s...") % subdev_rx_bb << std::endl;
    usrp_rx_bb->set_rx_subdev_spec(subdev_rx_bb);

    //std::cout << boost::format("Using USRP-Tx Device: %s") % usrp_tx->get_pp_string() << std::endl;

    std::cout << boost::format("Using USRP-Rx-BB Device: %s") % usrp_rx_bb->get_pp_string() << std::endl;


    /************************************************************************
     * Set up streamer to Replay block and commit graph
     ***********************************************************************/

    uhd::device_addr_t streamer_args;
    uhd::stream_args_t stream_args(cpu_format_tx, wire_format);
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

    // Lock mboard clocks
    if (vm.count("ref")) {
        
        // Set clock ref (10MHz from external clock)
    	graph->get_mb_controller()->set_clock_source(ref);
    	usrp_rx_bb->set_clock_source(ref);

        // Set shared PPS reference in both USRP
        graph->get_mb_controller()->set_time_source(ref);
        usrp_rx_bb->set_time_source(ref);

    }

    std::cout << "Synchronize the TX / RX PPS time through their respective Timekeepers !\n";

    uhd::time_spec_t Timer_to_zero = uhd::time_spec_t(0,0);

    auto Tx_mb_timeKper = graph->get_mb_controller()->get_timekeeper(0);


    
    /* Trial to Synch all the timekeepers among rx & tx to erase residual time sampling offset when streaming ? */


    /*size_t numb_timekeepers_tx = graph->get_mb_controller()->get_num_timekeepers();
    size_t numb_timekeepers_rx = usrp_rx_bb->get_mb_controller().get_num_timekeepers();

    std::cout << "Number of Tx Timekeepers : " << numb_timekeepers_tx << ", Number of Rx Timekeepers : " << numb_timekeepers_rx << std::endl;

    for(size_t i = 0; i < numb_timekeepers_rx; i++)
    {
        std::cout << "Currently Synchronizing Timekeepers " << i << " of both rx & tx !\n";

        uhd::rfnoc::mb_controller::timekeeper::sptr tx_timekeeper = graph->get_mb_controller()->get_timekeeper(i);
        uhd::rfnoc::mb_controller::timekeeper::sptr rx_timekeeper = usrp_rx_bb->get_mb_controller().get_timekeeper(i);

        uhd::time_spec_t TimePPS_value = rx_timekeeper->get_time_last_pps();

        while (TimePPS_value == rx_timekeeper->get_time_last_pps())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        tx_timekeeper->set_time_next_pps(Timer_to_zero);
        rx_timekeeper->set_time_next_pps(Timer_to_zero);

        std::this_thread::sleep_for(std::chrono::seconds(2));

    }*/



    /*Delete the following loop after checking is fine... otherwise it mess with the command issuing in the past*/
    /*for (size_t i = 0; i < 50; i++)
    {

        uhd::time_spec_t RxTimePPS = usrp_rx_bb->get_time_last_pps();
        uhd::time_spec_t TxTimePPS = Tx_mb_timeKper->get_time_last_pps();

        std::cout << "Rx USRP last PPS value is : " << RxTimePPS.get_full_secs() << "," << RxTimePPS.get_frac_secs() << "sec.\n";
        std::cout << "Tx USRP last PPS value is : " << TxTimePPS.get_full_secs() << "," << TxTimePPS.get_frac_secs() << "sec.\n";

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }*/
    
    std::cout << "Waiting for next PPS edge before setting the TX/RX timevalues to 0!\n";

    uhd::time_spec_t TimePPS_value = usrp_rx_bb->get_time_last_pps();

    while (TimePPS_value == usrp_rx_bb->get_time_last_pps())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    usrp_rx_bb->set_time_next_pps(Timer_to_zero);
    Tx_mb_timeKper->set_time_next_pps(Timer_to_zero);

    std::cout << "TIME VALUES SET IN BOTH TX and RX...!\n" << "Verify if both USRPs are now synch...\n";


    /*Delete the following loop after checking is fine... otherwise it mess with the command issuing in the past*/
    /*for (size_t i = 0; i < 50; i++)
    {

        uhd::time_spec_t RxTimePPS = usrp_rx_bb->get_time_last_pps();
        uhd::time_spec_t TxTimePPS = Tx_mb_timeKper->get_time_last_pps();

        std::cout << "Rx USRP last PPS value is : " << RxTimePPS.get_full_secs() << "," << RxTimePPS.get_frac_secs() << "sec.\n";
        std::cout << "Tx USRP last PPS value is : " << TxTimePPS.get_full_secs() << "," << TxTimePPS.get_frac_secs() << "sec.\n";

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }*/


    /*std::this_thread::sleep_for(std::chrono::seconds(3));


    for (size_t i = 0; i < 50; i++)
    {

        uhd::time_spec_t executing_time = Tx_mb_timeKper->get_time_last_pps() + uhd::time_spec_t(1,0);

        replay_ctrl->set_command_time(executing_time, 0);
        usrp_rx_bb->set_command_time(executing_time);
    
        uhd::time_spec_t RxTimePPS = usrp_rx_bb->get_time_now();
        uhd::time_spec_t TxTimePPS = Tx_mb_timeKper->get_time_now();

        usrp_rx_bb->clear_command_time();
        replay_ctrl->clear_command_time(0);

        std::cout << "Rx USRP last PPS value is : " << RxTimePPS.get_full_secs() << "," << RxTimePPS.get_frac_secs() << "sec.\n";
        std::cout << "Tx USRP last PPS value is : " << TxTimePPS.get_full_secs() << "," << TxTimePPS.get_frac_secs() << "sec.\n";

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }*/

    /*std::cout << "Waiting for 253 millisecond and check the return uhd timespec fractional value of time elapsed !\n";

    uhd::time_spec_t NowFlag = Tx_mb_timeKper->get_time_now();
    std::this_thread::sleep_for(std::chrono::milliseconds(15));
    uhd::time_spec_t afterFlag = Tx_mb_timeKper->get_time_now();

    uhd::time_spec_t deltafractional = afterFlag - NowFlag;

    std::cout << "Elapsed number of seconds : " << deltafractional.get_full_secs() << ", Elapsed fractionnal sec : " << deltafractional.get_frac_secs() << std::endl;*/



    // Apply any radio arguments provided
    if (vm.count("args-tx")) {
        radio_ctrl->set_tx_tune_args(args_tx, radio_chan);	
    }

    // Set the center frequency
    if (!vm.count("freq-bb")) {
        std::cerr << "Please specify the center frequency with --freq" << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << std::fixed;
    std::cout << "Requesting TX USRP Freq: " << (freq_bb / 1e6) << " MHz..." << std::endl;
    uhd::tune_request_t tune_request(freq_bb);

    if (vm.count("lo-offset")) {
        std::cout << boost::format("Setting TX LO Offset: %f MHz...") % (lo_offset / 1e6) << std::endl;
        tune_request = uhd::tune_request_t(freq_bb, lo_offset);
        tune_request.rf_freq_policy = uhd::tune_request_t::POLICY_MANUAL;
        tune_request.dsp_freq_policy = uhd::tune_request_t::POLICY_MANUAL;
    }

    if (vm.count("int-n")) {
        tune_request.args = uhd::device_addr_t("mode_n=integer");
    }

    std::cout << "Observe the tune request that is about to be send to both TX & RX :\n";

    std::cout << "RF freq policy is " << tune_request.rf_freq_policy << ", dsp freq policy is " << tune_request.dsp_freq_policy << std::endl; 
    std::cout << "RF Freq of the request is " << tune_request.rf_freq << ", DSP freq of the request is " << tune_request.dsp_freq << std::endl;

    std::cout << "RF Freq of the request is " << tune_request.rf_freq << ", DSP freq of the request is " << tune_request.dsp_freq << std::endl;
    std::cout << "Out of curiosity, this is the 'target freq' : " << tune_request.target_freq << "Hz.\n";


    auto tune_req_action = uhd::rfnoc::tune_request_action_info::make(tune_request);
    tune_req_action->tune_request = tune_request;

    uhd::tune_request_t tune_request_bb = tune_request;

    uhd::time_spec_t executing_time = Tx_mb_timeKper->get_time_last_pps() + uhd::time_spec_t(2,0);


    /*Tuning RX & TX frequency at the same instant to avoid LO phase offset...*/
    replay_ctrl->set_command_time(executing_time, 0);
    usrp_rx_bb->set_command_time(executing_time);

    usrp_rx_bb->set_rx_freq(tune_request_bb, 0);
    replay_ctrl->post_output_action(tune_req_action, 0);

    usrp_rx_bb->clear_command_time();
    replay_ctrl->clear_command_time(0);

    std::cout << std::resetiosflags(std::ios::fixed);

    // set the center frequency of the baseband and LO chains
    std::cout << boost::format("USRP-Rx BB Freq: %f MHz...") % (usrp_rx_bb->get_rx_freq(0) / 1e6) << std::endl;
	std::cout << "TX USRP Freq at Radio: " << (radio_ctrl->get_tx_frequency(radio_chan) / 1e6) << " MHz..." << std::endl << std::endl;

    // Set the sample rate TX USRP 
    if (vm.count("Tx_rate")) {
        std::cout << std::fixed;
        std::cout << "Requesting TX Rate: " << (tx_rate / 1e6) << " Msps..." << std::endl;
        if (duc_ctrl) {
                std::cout << "DUC block found." << std::endl;
                duc_ctrl->set_input_rate(tx_rate, duc_chan);
                std::cout << "  Interpolation value is "
                    << duc_ctrl->get_property<int>("interp", duc_chan) << std::endl;
                tx_rate = duc_ctrl->get_input_rate(duc_chan);
        } else {
                tx_rate = radio_ctrl->set_rate(tx_rate);
        }
        std::cout << "Actual TX Rate: " << (tx_rate / 1e6) << " Msps..." << std::endl << std::endl;
        std::cout << std::resetiosflags(std::ios::fixed);

    }

    std::cout << boost::format("Setting USRP-Rx-BB Rx Rate: %f Msps...") % (rx_rate / 1e6) << std::endl;
    usrp_rx_bb->set_rx_rate(rx_rate);
    std::cout << boost::format("Actual USRP-Rx-BB Rx Rate: %f Msps ...") % (usrp_rx_bb->get_rx_rate() / 1e6) << std::endl;
	
    // Set the RF gain
    if (vm.count("gain-tx-bb")) {
        std::cout << std::fixed;

        if (gain_tx_bb > 15)
        {
            gain_tx_bb = 15;
        }

        std::cout << "Requesting TX Gain: " << gain_tx_bb << " dB..." << std::endl;
        radio_ctrl->set_tx_gain(gain_tx_bb, radio_chan);
        std::cout << "Actual TX Gain: " << radio_ctrl->get_tx_gain(radio_chan) << " dB..." << std::endl << std::endl;
        std::cout << std::resetiosflags(std::ios::fixed);
    }

    std::cout << boost::format("Setting USRP-Rx BB Gain: %f dB...") % gain_rx_bb << std::endl;
    usrp_rx_bb->set_rx_gain(gain_rx_bb, 0);
    std::cout << boost::format("Actual USRP-Rx BB Gain: %f dB...") % usrp_rx_bb->get_rx_gain(0) << std::endl;
    

    // Set the analog front-end filter bandwidth
    if (vm.count("bw")) {
        std::cout << std::fixed;
        std::cout << "Requesting TX Bandwidth: " << (bw / 1e6) << " MHz..." << std::endl;
        radio_ctrl->set_tx_bandwidth(bw, radio_chan);
        std::cout << "Actual TX Bandwidth: " << (radio_ctrl->get_tx_bandwidth(radio_chan) / 1e6) << " MHz..." << std::endl << std::endl;
        std::cout << std::resetiosflags(std::ios::fixed);

        //Also set the RX USRP bw here
        usrp_rx_bb->set_rx_bandwidth(bw);

    }
    
    // set the Tx and Rx antenna ports

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

    std::cout << boost::format("Setting USRP Rx antenna ports...") << std::endl;
    usrp_rx_bb->set_rx_antenna(ant_bb, 0);
    std::cout << boost::format("Actual RX USRP Antenna used is : ") << usrp_rx_bb->get_rx_antenna() << std::endl;
    

    // allow for some setup time
    std::this_thread::sleep_for(std::chrono::seconds(1)); 

    
    // Check Ref and LO Lock detect for USRP Tx
    std::vector<std::string> sensor_names;

    sensor_names = radio_ctrl->get_tx_sensor_names(radio_chan);

    if (std::find(sensor_names.begin(), sensor_names.end(), "lo_locked") != sensor_names.end()) {
        uhd::sensor_value_t lo_locked = radio_ctrl->get_tx_sensor("lo_locked", radio_chan);
        std::cout << boost::format("Checking TX USRP : %s ...") % lo_locked.to_pp_string() << std::endl;
        UHD_ASSERT_THROW(lo_locked.to_bool());
    }

    size_t mboard_sensor_idx = 0;

    sensor_names = graph->get_mb_controller()->get_sensor_names();

    if ((ref == "external") and (std::find(sensor_names.begin(), sensor_names.end(), "ref_locked") != sensor_names.end())) {
        uhd::sensor_value_t ref_locked = graph->get_mb_controller()->get_sensor("ref_locked");
        std::cout << boost::format("Checking TX: %s ...") % ref_locked.to_pp_string() << std::endl;
        UHD_ASSERT_THROW(ref_locked.to_bool());
    }

    // Check Ref and LO Lock detect for USRP Rx
    sensor_names = usrp_rx_bb->get_rx_sensor_names(0);
    if (std::find(sensor_names.begin(), sensor_names.end(), "lo_locked") != sensor_names.end()) {
        uhd::sensor_value_t lo_locked = usrp_rx_bb->get_rx_sensor("lo_locked", 0);
        std::cout << boost::format("Checking RX: %s ...") % lo_locked.to_pp_string() << std::endl;
        UHD_ASSERT_THROW(lo_locked.to_bool());
    }
 
    mboard_sensor_idx = 0;
    sensor_names = usrp_rx_bb->get_mboard_sensor_names(mboard_sensor_idx);

    if ((ref == "external")
        and (std::find(sensor_names.begin(), sensor_names.end(), "ref_locked") != sensor_names.end())) {
        uhd::sensor_value_t ref_locked = usrp_rx_bb->get_mboard_sensor("ref_locked", mboard_sensor_idx);
        std::cout << boost::format("Checking RX: %s ...") % ref_locked.to_pp_string() << std::endl;
        UHD_ASSERT_THROW(ref_locked.to_bool());
    }

    /************************************************************************
     * Read the data to replay
     ***********************************************************************/
    // Constants related to the Replay block
    const size_t replay_word_size = replay_ctrl->get_word_size(); // Size of words used by replay block
    const size_t CPU_sample_size = 8; // On utilise fc32 => 8bytes/sample
    replay_ctrl->set_record_type(cpu_format_tx);

    std::cout << "Replay word size : " << replay_word_size << std::endl;

    std::cout << "Memory size : " << replay_ctrl->get_mem_size() << std::endl;


    size_t samples_to_replay(0);
    size_t words_to_replay(0);


    std::ifstream infile(InFilename.c_str(), std::ifstream::binary);
    if (!infile.is_open()) {
        std::cerr << "Could not open specified file" << std::endl;
        return EXIT_FAILURE;
    }

    // Get the file size
    infile.seekg(0, std::ios::end);
	long unsigned int file_size =(long unsigned int)  infile.tellg();
    infile.seekg(0, std::ios::beg);
    std::cout << "File size : " << file_size << " (Bytes...?)" << std::endl;


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

    char* tx_buf_ptr = new char[samples_to_replay*CPU_sample_size];

    std::cout << "Buffer storing the signal is of length : " << samples_to_replay*CPU_sample_size << " Bytes !\n";

    infile.read(tx_buf_ptr, std::streamsize (samples_to_replay * CPU_sample_size));
    infile.close();

    uhd::tx_streamer::buffs_type data_buffer(tx_buf_ptr);

    if(samples_to_replay == 0 || words_to_replay == 0){
	std::cout << "A weird error occured during the files reading..." << std::endl;
	return -1;
    }



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
        std::cout << "Recording... Current fullness : " << replay_ctrl->get_record_fullness(replay_chan) << "..." << std::endl;
        std::cout << "Recording... Current position : " << replay_ctrl->get_record_position() << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "Record fullness: " << replay_ctrl->get_record_fullness(replay_chan) << " bytes" << std::endl << std::endl;

    rf_mode_t modetx = MODE_TX;
    rf_mode_t moderx = MODE_RX;

    //set gain to maximum gain
    float gain_dB_Tx_max = service->get_dynamic_range(sn_Tx_BBox, modetx);

    float gain_dB_Rx_max = service->get_dynamic_range(sn_Rx_BBox, moderx);

    float gain_dB_Tx = gain_dB_Tx_max;
    float gain_dB_Rx = gain_dB_Rx_max;

   
    /*int off_channels[16]; // Adjust size if necessary, based on maximum possible size
    int nbr_channels = 0;

    if (nbr_active_antennas == 4) { 
        int temp[] = {1, 2, 3, 4, 5, 8, 9, 12, 13, 14, 15, 16}; // Keeping only the 4 central channels
        nbr_channels = sizeof(temp) / sizeof(temp[0]);
        memcpy(off_channels, temp, sizeof(temp));
    } else if (nbr_active_antennas == 1) {
        int temp[] = {1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16}; // Keeping only one channel
        nbr_channels = sizeof(temp) / sizeof(temp[0]);
        memcpy(off_channels, temp, sizeof(temp));
    }*/

    //writing parameters in file
    if (OutFile.is_open()) {
    OutFile << boost::format("Rate is %d and freq is %f") % tx_rate % freq_bb;
    OutFile << boost::format("Gain tx is %d and Gain rx is %d") % gain_tx_bb % gain_rx_bb;
    OutFile << boost::format("Number samps per degree is %d ") % nbr_samps_per_degree ;
    OutFile << boost::format("Thetas min and max of tx and rx are %d %d %d %d") % theta_min_tx % theta_max_tx % theta_min_rx %  theta_max_rx;
    OutFile << boost::format("Step is %d and N active antennas are %d") % step % nbr_active_antennas;
    OutFile << boost::format("Length of active packet is %d") % LengthSignal;
    OutFile << boost::format("Over Sampling at Receiver %d %d") % OverSampling % UpsampleFactor;
    }
    
    std::vector<double> RxSampCount;

    // ====================
    // Create Rx streamers
    // ====================
    // create a receive streamer
    std::vector<size_t> channel_nums_rx_bb = {0};
    uhd::stream_args_t stream_args_rx_bb("fc32", "sc16");
    stream_args_rx_bb.channels = channel_nums_rx_bb;
    uhd::rx_streamer::sptr rx_stream = usrp_rx_bb->get_rx_stream(stream_args_rx_bb);
    
    //meta-data will be filled in by recv()
    uhd::rx_metadata_t md;
    
    // allocate a buffer which we re-use for each channel
	size_t spb = rx_stream->get_max_num_samps(); 
    std::vector<std::complex<float>> 	buff_bb(spb);
    std::vector<std::complex<float>*> 	buffs(1);
    buffs[0] = &buff_bb.front();
    
    /*std::cout << "Parameters used by original code.\n";
    std::cout << "Buffer (=est un vecteur) contenant les samples a la taille suivante : " << buff_bb.size() << ", et est stocké à l'addresse : " << &buff_bb << "," << &(buff_bb)  << ", et le premier élément du vecteur est tocké en : " << &(buff_bb[0]) << std::endl;

    std::cout << "vecteur (contenant le pointeur du buffer sample) a donc pour taille : " << buffs.size() << ", et est lui même stocké à l'addresse : " << &buffs << ", et le premier élément du vecteur est tocké en : " << &(buffs[0]) << ", et ce premier élément contient justement " << buffs[0] << " censé être l'endroit où est stocké le premier élément du buffer 364!" << std::endl;
    std::cout << "Voici le contenu du buffer parent : " << buffs[0] << ", il s'agit d'ailleurs de l'addresse donnée à recv() ! : " << &(buffs.front()) << std::endl;*/

    /*My own buffer management will follow afterwards...*/
    size_t TotalRxSamples = nbr_samps_per_degree* UpsampleFactor * (Nr_Of_Tx_beams*Nr_Of_Rx_beams);
    char* rx_buf_ptr = new char[TotalRxSamples * CPU_sample_size];

    /*REPLAY memory configuration */
    uint64_t packetSizeinBytes = samps_per_packet*OTW_sample_size;
    size_t replay_port = 0;

    std::vector<uint64_t> SSB_signal_address(Max_SSBs_Stored);

    for (int i = 0; i < Max_SSBs_Stored; i++)
    {
        SSB_signal_address[i] = i * samps_per_packet * OTW_sample_size;
    }

    int SSB_beam_index = 0;


    /*This Term will IMPACT THE MOST the time needed for the overall sweeping experiment !*/
    double issuing_time_frac = 0.500;
    
    double overtime_Factor = 5;

    double appropriate_timeout = (nbr_samps_per_degree * UpsampleFactor / rx_rate) * overtime_Factor + issuing_time_frac;

    //Streamproperties
    size_t Streaming_samplesNumber = nbr_samps_per_degree;
    size_t Receiving_samplesNumber = Streaming_samplesNumber*UpsampleFactor;

    //Stream Tx/Rx command
    uhd::stream_cmd_t stream_cmd_tx(uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
    stream_cmd_tx.stream_now = false;
    stream_cmd_tx.num_samps = Streaming_samplesNumber;

    uhd::stream_cmd_t stream_cmd_rx(uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
    stream_cmd_rx.stream_now = false;
    stream_cmd_rx.num_samps = Receiving_samplesNumber;

    char* Increasing_Rx_ptr = rx_buf_ptr;    

    /*for (int theta_tx = theta_min_tx; theta_tx <= theta_max_tx  ; theta_tx = theta_tx + step )
    {
        replay_ctrl->config_play(SSB_signal_address[SSB_beam_index], packetSizeinBytes, replay_port);
        std::cout << boost::format("Setting Tx angle to %d ° at time %f") % theta_tx % Tx_mb_timeKper->get_time_now().get_real_secs() << std::endl;
        set_theta(service, sn_Tx_BBox, gain_dB_Tx, theta_tx);

        for(int theta_rx = theta_min_rx; theta_rx <= theta_max_rx  ; theta_rx = theta_rx + step )
        {
            std::cout << boost::format("Setting Rx angle to %d ° at time %f") % theta_rx % Tx_mb_timeKper->get_time_now().get_real_secs() << std::endl;
            set_theta(service, sn_Rx_BBox, gain_dB_Rx, theta_rx);

            //Maybe use integer pps value for time burst starting point : Could this avoid any residual sampling offset if done through get_time_last_pps() ?
            uhd::time_spec_t nextBurst_time = usrp_rx_bb->get_time_now() + uhd::time_spec_t(0, issuing_time_frac);

            stream_cmd_tx.time_spec = nextBurst_time;
            stream_cmd_rx.time_spec = nextBurst_time;

            //Issue the Streaming at both tx/rx end !
            replay_ctrl->issue_stream_cmd(stream_cmd_tx);
            rx_stream->issue_stream_cmd(stream_cmd_rx);

            size_t num_rx_samps = rx_stream->recv(Increasing_Rx_ptr, nbr_samps_per_degree*UpsampleFactor, md, appropriate_timeout);

            //Error Handling 
            if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) break;
            if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE)
            {
                throw std::runtime_error(str(boost::format("Receiver error %s") % md.strerror()));
            }

            std::cout << boost::format(" -- Received %f samples, %d KBytes added to the Rx Buffer !\n") % num_rx_samps % (num_rx_samps*CPU_sample_size/1000);

            Increasing_Rx_ptr = Increasing_Rx_ptr + CPU_sample_size*num_rx_samps;
        }

        if (InFilename != "QPSK.dat")
        {
            SSB_beam_index++;
        }
    }*/


    for (int tx_ind = 0; tx_ind < tx_angles.size(); tx_ind++)
    {
        //replay_ctrl->config_play(SSB_signal_address[tx_ind], packetSizeinBytes, replay_port);

        int currtheta_tx = tx_angles[tx_ind];

        std::cout << boost::format("Setting Tx angle to %d ° at time %f") % currtheta_tx % Tx_mb_timeKper->get_time_now().get_real_secs() << std::endl;
        set_theta(service, sn_Tx_BBox, gain_dB_Tx, currtheta_tx);

        for(int rx_ind = 0; rx_ind < rx_angles.size(); rx_ind++)
        {

            int currtheta_rx = rx_angles[rx_ind];

            replay_ctrl->config_play(SSB_signal_address[SSB_beam_index], packetSizeinBytes, replay_port);

            std::cout << boost::format("Setting Rx angle to %d ° at time %f") % currtheta_rx % Tx_mb_timeKper->get_time_now().get_real_secs() << std::endl;

            set_theta(service, sn_Rx_BBox, gain_dB_Rx, currtheta_rx);

            //Shut down some antennas
            //service->set_off_channels(sn_Rx_BBox, off_channels, nbr_channels);

            /*Maybe use integer pps value for time burst starting point : Could this avoid any residual sampling offset if done through get_time_last_pps() ?*/
            uhd::time_spec_t nextBurst_time = usrp_rx_bb->get_time_now() + uhd::time_spec_t(0, issuing_time_frac);

            stream_cmd_tx.time_spec = nextBurst_time;
            stream_cmd_rx.time_spec = nextBurst_time;

            //Issue the Streaming at both tx/rx end !
            replay_ctrl->issue_stream_cmd(stream_cmd_tx);
            rx_stream->issue_stream_cmd(stream_cmd_rx);

            size_t num_rx_samps = rx_stream->recv(Increasing_Rx_ptr, nbr_samps_per_degree*UpsampleFactor, md, appropriate_timeout);

            //Error Handling 
            if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) break;
            if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE)
            {
                throw std::runtime_error(str(boost::format("Receiver error %s") % md.strerror()));
            }

            std::cout << boost::format(" -- Received %f samples, %d KBytes added to the Rx Buffer !\n") % num_rx_samps % (num_rx_samps*CPU_sample_size/1000);

            Increasing_Rx_ptr = Increasing_Rx_ptr + CPU_sample_size*num_rx_samps;
            SSB_beam_index++;
        }
    }



    OutFile.write((const char*)rx_buf_ptr, TotalRxSamples*CPU_sample_size*sizeof(char)); //Need to save = total_samples x (Bytes for each complex sampleCPU) x size of one char (=1Byte)


    // ======================
    // Closing up everything 
    // ======================

    std::cout << "Halting all commands on tx/rx Streamers...\n";

    uhd::stream_cmd_t stop_cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
    rx_stream->issue_stream_cmd(stop_cmd);
    rx_stream.reset();

    replay_ctrl->stop(replay_port);
    
    // finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;
    OutFile.close();

    return EXIT_SUCCESS;
}
