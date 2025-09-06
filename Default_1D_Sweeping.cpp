#include <tlkcore_lib/tlkcore_lib.hpp>
#include <common_lib.h>
#include <iostream>
#include <vector>
#include <string.h>
#include <chrono>
#include <thread>
#include <array> 


#include <uhd/utils/thread_priority.hpp>
#include <boost/thread.hpp>
#include <uhd/exception.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/static.hpp>
#include <uhd/utils/thread.hpp>
#include <stdint.h>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include <boost/math/special_functions/round.hpp>   
#include <boost/program_options.hpp>
#include <csignal>
#include <string>
#include <fstream>



namespace po = boost::program_options;
using namespace tlkcore;

bool stop_signal_called = false;


/* VERSION DU FICHIER : BRANCHE DEV*/

/***********************************************************************
 * transmit_worker function
 * A function to be used as a boost::thread_group thread for transmitting
 **********************************************************************/
void tx_worker(std::vector<std::complex<float>> data_bb, uhd::tx_streamer::sptr tx_stream)
{

	// allocate a buffer
    size_t spb = tx_stream->get_max_num_samps(); 
    std::vector<std::complex<float>> buff_bb(spb);
    std::vector<std::complex<float>*> buffs(1);
    buffs[0] = &buff_bb.front(); 
	
    // setup the metadata flags
    uhd::tx_metadata_t md;
    md.start_of_burst = true;
    md.end_of_burst   = false;
    md.has_time_spec  = true;
    md.time_spec = uhd::time_spec_t(1.0); // give us 1.0 seconds to fill the tx buffers
    
    size_t index_bb = 0;

    // send data until the signal handler gets called
    while (not stop_signal_called) {
    
        // fill the buffer with the data file
        for (size_t n = 0; n < spb; n++) {
		    buff_bb[n] = data_bb[index_bb];
		    index_bb++;
		    if (index_bb == data_bb.size()){
		    	index_bb = 0;
			}
		}
		
        // send the entire contents of the buffer

        tx_stream->send(buffs, spb, md);


        //Check the format of the complex samples in the tx buffer to be pushed & sent
        //int n = 546;
        //std::cout << n << "th Sample is of size : " << sizeof(buff_bb[n]) << " Bytes [both-IQ] (AFTER ->SEND() FUNC)" << std::endl;

        md.start_of_burst = false;
        md.has_time_spec  = false;
    }

    // send a mini EOB packet
    md.end_of_burst = true;
    tx_stream->send("", 0, md);
}



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
    std::string 	args_tx, args_rx, ref, sn_Tx_UD, sn_Rx_UD, sn_Tx_BBox, sn_Rx_BBox, file_name; 
    double 			rate_tx, rate_rx, freq_bb, gain_tx_bb, gain_rx_bb; 
    std::ofstream 	outfile;
    uint64_t 		nbr_samps_per_degree;
    int             step, nbr_active_antennas, theta_min_tx, theta_max_tx, theta_min_rx, theta_max_rx;
    
//test
    
    // variables with initializations
    std::string 	subdev_tx 			= "A:0 ";
    std::string		subdev_rx_bb 		= "A:0";
    std::string 	ant_bb 				= "TX/RX";
	float 			seconds_in_future 	= 5.0;
    double          Nr_Of_Tx_Beams      = 10;


    
    
    // setup the program options
    po::options_description desc("Allowed options");
    // clang-format off
    desc.add_options()
	("help", "help message")
	("args-tx", po::value<std::string>(&args_tx)->default_value("addr=192.168.100.51"), "USRP IP address for Tx") 
	("args-rx", po::value<std::string>(&args_rx)->default_value("addr=192.168.100.53"), "USRP IP address for Rx")

	("ref", po::value<std::string>(&ref)->default_value("external"), "clock reference (internal, external, gpsdo)")
	("rate-tx", po::value<double>(&rate_tx)->default_value(1000000), "sample rate of Tx")
	("rate-rx", po::value<double>(&rate_rx)->default_value(1000000), "sample rate of Rx")
	("freq-bb", po::value<double>(&freq_bb)->default_value(4000000000), "Center frequency of Tx and Rx baseband signal in Hz")
	("gain-tx-bb", po::value<double>(&gain_tx_bb)->default_value(5), "Gain of Tx baseband signal in dB")
	("gain-rx-bb", po::value<double>(&gain_rx_bb)->default_value(30), "Gain of Rx baseband signal in dB")
	("nsamps-per-degree", po::value<uint64_t>(&nbr_samps_per_degree)->default_value(100000), "Number of samples per Tx/Rx beam direction")
	("step", po::value<int>(&step)->default_value((45 - (-45))/(Nr_Of_Tx_Beams-1)), "Difference in degress between two succesive angles")
        
    ("sn_Tx_UD", po::value<std::string>(&sn_Tx_UD)->default_value("UD-BD23310064-24"), "serial number of Tx UDBox")
    ("sn_Rx_UD", po::value<std::string>(&sn_Rx_UD)->default_value("UD-BD23310063-24"), "serial number of Rx UDBox")

    ("sn_Tx_BBox", po::value<std::string>(&sn_Tx_BBox)->default_value("D2336L118-28"), "serial number of Tx BBox")
    ("sn_Rx_BBox", po::value<std::string>(&sn_Rx_BBox)->default_value("D2336L117-28"), "serial number of Rx BBox")

    ("theta_min_tx", po::value<int>(&theta_min_tx)->default_value(-45), "Min TX Theta (degree)")
    ("theta_max_tx", po::value<int>(&theta_max_tx)->default_value(45), "Max TX Theta (degree)")
    ("theta_min_rx", po::value<int>(&theta_min_rx)->default_value(-45), "Min RX Theta (degree)")
    ("theta_max_rx", po::value<int>(&theta_max_rx)->default_value(45), "Max RX Theta (degree)")
    ("file_name", po::value<std::string>(&file_name)->default_value("Sweep_1D_Default.dat"), "name of the output file")   
    ("nantennas", po::value<int>(&nbr_active_antennas)->default_value(16), "Number of active antennas")
    ;

    // clang-format on
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    // print the help message
    if (vm.count("help")) {
        std::cout << boost::format("Software to control mmWave Tx and Rx jointly. %s") % desc << std::endl;
        return ~0;
    }
    

    //std::string file_path = "/home/amelia/Bureau/TMYTEK_API/data_processing/raw_data/" + file_name; 
    
    std::string file_path = file_name; // Relative path in the current directory

    // Open the output file
   // outfile.open("/home/amelia/Bureau/TMYTEK_API/data_processing/raw_data/joint_test_3.dat", std::ofstream::binary);
   
    
     outfile.open(file_path , std::ofstream::binary);

    if (outfile.is_open()){
		printf("Output file opened correctly. \n"); }
    else{
		printf("OUTPUT FILE NOT OPENED !!! \n"); }



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
     
    // =============================================
    // Create and initialize USRP Tx and Rx devices
    // =============================================
    // Create USRP devices
    std::cout << boost::format("Creating the USRP-Tx device with: %s...") % args_tx << std::endl;
    uhd::usrp::multi_usrp::sptr usrp_tx = uhd::usrp::multi_usrp::make(args_tx);
    std::cout << boost::format("Creating the USRP-Rx-BB device with: %s...") % args_rx << std::endl;
    uhd::usrp::multi_usrp::sptr usrp_rx_bb = uhd::usrp::multi_usrp::make(args_rx);
    
    // always select the subdevice first, the channel mapping affects the other settings
    std::cout << boost::format("Setting subdevice USRP-Tx device to: %s...") % subdev_tx << std::endl;
    std::cout << boost::format("Setting subdevice USRP-Rx-BB device to: %s...") % subdev_rx_bb << std::endl;

    usrp_tx->set_tx_subdev_spec(subdev_tx); 
    usrp_rx_bb->set_rx_subdev_spec(subdev_rx_bb); 
    std::cout << boost::format("Using USRP-Tx Device: %s") % usrp_tx->get_pp_string() << std::endl;
    std::cout << boost::format("Using USRP-Rx-BB Device: %s") % usrp_rx_bb->get_pp_string() << std::endl;

    // Lock mboard clocks
    if (vm.count("ref")) {
    	usrp_tx->set_clock_source(ref);
    	usrp_rx_bb->set_clock_source(ref);
    }
    
    // set the sample rate
    std::cout << boost::format("Setting USRP-Tx Rate: %f Msps...") % (rate_tx / 1e6) << std::endl;
    usrp_tx->set_tx_rate(rate_tx);
    std::cout << boost::format("Actual USRP-Tx Rate: %f Msps...") % (usrp_tx->get_tx_rate() / 1e6) << std::endl;
    std::cout << boost::format("Setting USRP-Rx-BB Rx Rate: %f Msps...") % (rate_rx / 1e6) << std::endl;
    usrp_rx_bb->set_rx_rate(rate_rx);
    std::cout << boost::format("Actual USRP-Rx-BB Rx Rate: %f Msps ...") % (usrp_rx_bb->get_rx_rate() / 1e6) << std::endl;
	
	// set the center frequency of the baseband and LO chains
    std::cout << boost::format("Setting USRP-Tx BB and USRP-Rx BB Freq: %f MHz...") % (freq_bb / 1e6) << std::endl;
    
    uhd::tune_request_t tune_request_bb(freq_bb, 0);

    tune_request_bb.dsp_freq_policy = uhd::tune_request_t::POLICY_MANUAL;
    tune_request_bb.rf_freq_policy = uhd::tune_request_t::POLICY_MANUAL;

    std::cout << "Observe the tune request that is about to be send to both TX & RX :\n";

    std::cout << "RF freq policy is " << tune_request_bb.rf_freq_policy << ", dsp freq policy is " << tune_request_bb.dsp_freq_policy << std::endl; 

    std::cout << "RF Freq of the request is " << tune_request_bb.rf_freq << ", DSP freq of the request is " << tune_request_bb.dsp_freq << std::endl;
    std::cout << "Out of curiosity, this is the 'target freq' : " << tune_request_bb.target_freq << "Hz.\n";


    usrp_tx->set_tx_freq(tune_request_bb, 0);
    usrp_rx_bb->set_rx_freq(tune_request_bb, 0);
    std::cout << boost::format("Actual USRP-Tx BB and USRP-Rx BB Freq: %f MHz and %f MHz...") % (usrp_tx->get_tx_freq(0) / 1e6) % (usrp_rx_bb->get_rx_freq(0) / 1e6) << std::endl;
	
	// set the gains of the baseband and LO chains
    std::cout << boost::format("Setting USRP-Tx BB Gain: %f dB...") % gain_tx_bb << std::endl;

    if (gain_tx_bb > 15) {
        gain_tx_bb = 15;
    }

    usrp_tx->set_tx_gain(gain_tx_bb, 0);
    std::cout << boost::format("Actual USRP-Tx BB Gain: %f dB...") % usrp_tx->get_tx_gain(0) << std::endl;
    std::cout << boost::format("Setting USRP-Rx BB Gain: %f dB...") % gain_rx_bb << std::endl;
    usrp_rx_bb->set_rx_gain(gain_rx_bb, 0);
    std::cout << boost::format("Actual USRP-Rx BB Gain: %f dB...") % usrp_rx_bb->get_rx_gain(0) << std::endl;
    
    // set the Tx and Rx antenna ports
    std::cout << boost::format("Setting USRP Tx and Rx antenna ports...") << std::endl;
    usrp_tx->set_tx_antenna(ant_bb, 0);
    usrp_rx_bb->set_rx_antenna(ant_bb, 0);
    
    // allow for some setup time
    std::this_thread::sleep_for(std::chrono::seconds(1)); 
    

    uhd::time_spec_t Timer_to_zero = uhd::time_spec_t(0,0);

    // Setting timestamp and time source
    std::cout << boost::format("Setting USRP Tx and Rx timestamps to 0...") << std::endl;
    usrp_tx->set_time_source(ref);
    usrp_tx->set_time_unknown_pps(uhd::time_spec_t(0.0));
    usrp_rx_bb->set_time_source(ref);
    usrp_rx_bb->set_time_unknown_pps(uhd::time_spec_t(0.0));
    std::this_thread::sleep_for(std::chrono::seconds(1)); // wait for pps sync pulse

    std::cout << "First : Observe for 5sec if there is a time delay, ro difference in their PPS time !\n";

    /*for (size_t i = 0; i < 50; i++)
    {

        uhd::time_spec_t RxTimePPS = usrp_rx_bb->get_time_last_pps();
        uhd::time_spec_t TxTimePPS = usrp_tx->get_time_last_pps();

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
    usrp_tx->set_time_next_pps(Timer_to_zero);

    std::this_thread::sleep_for(std::chrono::seconds(1));


    std::cout << "TIME VALUES SET IN BOTH TX and RX...!\n" << "Verify if both USRPs are now synch...";


    /*for (size_t i = 0; i < 50; i++)
    {

        uhd::time_spec_t RxTimePPS = usrp_rx_bb->get_time_now();
        uhd::time_spec_t TxTimePPS = usrp_tx->get_time_now();

        std::cout << "Rx USRP last PPS value is : " << RxTimePPS.get_full_secs() << "," << RxTimePPS.get_frac_secs() << "sec.\n";
        std::cout << "Tx USRP last PPS value is : " << TxTimePPS.get_full_secs() << "," << TxTimePPS.get_frac_secs() << "sec.\n";

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }*/
    
    // Check Ref and LO Lock detect for USRP Tx
    std::vector<std::string> sensor_names;
    sensor_names = usrp_tx->get_tx_sensor_names(0);
    if (std::find(sensor_names.begin(), sensor_names.end(), "lo_locked")
        != sensor_names.end()) {
        uhd::sensor_value_t lo_locked = usrp_tx->get_tx_sensor("lo_locked", 0);
        std::cout << boost::format("Checking TX: %s ...") % lo_locked.to_pp_string() << std::endl;
        UHD_ASSERT_THROW(lo_locked.to_bool());
    }
    size_t mboard_sensor_idx = 0;
    sensor_names = usrp_tx->get_mboard_sensor_names(mboard_sensor_idx);
    if ((ref == "external")
        and (std::find(sensor_names.begin(), sensor_names.end(), "ref_locked")
                != sensor_names.end())) {
        uhd::sensor_value_t ref_locked =
            usrp_tx->get_mboard_sensor("ref_locked", mboard_sensor_idx);
        std::cout << boost::format("Checking TX: %s ...") % ref_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(ref_locked.to_bool());
    }

    // Check Ref and LO Lock detect for USRP Rx
    sensor_names = usrp_rx_bb->get_rx_sensor_names(0);
    if (std::find(sensor_names.begin(), sensor_names.end(), "lo_locked")
        != sensor_names.end()) {
        uhd::sensor_value_t lo_locked = usrp_rx_bb->get_rx_sensor("lo_locked", 0);
        std::cout << boost::format("Checking RX: %s ...") % lo_locked.to_pp_string() << std::endl;
        UHD_ASSERT_THROW(lo_locked.to_bool());
    }
 
    mboard_sensor_idx = 0;
    sensor_names = usrp_rx_bb->get_mboard_sensor_names(mboard_sensor_idx);
    if ((ref == "external")
        and (std::find(sensor_names.begin(), sensor_names.end(), "ref_locked")
                != sensor_names.end())) {
        uhd::sensor_value_t ref_locked =
            usrp_rx_bb->get_mboard_sensor("ref_locked", mboard_sensor_idx);
        std::cout << boost::format("Checking RX: %s ...") % ref_locked.to_pp_string()
                  << std::endl;
        UHD_ASSERT_THROW(ref_locked.to_bool());
    }


    /*Checking number of Motherboards, Daughterboards(=subdev, Tx or Rx channels... ). Might be useful for rfNoC*/
    size_t num_mboardsTX = usrp_tx->get_num_mboards();
    std::cout << "Number of mBoards on Tx USRP : " << num_mboardsTX << std::endl;

    size_t num_mboardsRX = usrp_rx_bb->get_num_mboards();
    std::cout << "Number of mBoards on Rx USRP : " << num_mboardsRX << std::endl << std::endl;


    size_t num_rx_channels = usrp_rx_bb->get_rx_num_channels();
    std::cout << "Number of Receive Channels : " << num_rx_channels << std::endl;

    std::string rx_name = usrp_rx_bb->get_rx_subdev_name(num_rx_channels - 1);
    std::cout << "Daughterboard RX ch" << num_rx_channels - 1 << ": " << rx_name << std::endl;


    size_t num_tx_channels = usrp_tx->get_tx_num_channels();
    std::cout << "Number of Transmit Channels : " << num_tx_channels << std::endl;
    
    std::string tx_name = usrp_tx->get_tx_subdev_name(num_tx_channels - 1);
    std::cout << "Daughterboard TX ch" << num_tx_channels - 1 << ": " << tx_name << std::endl;

    // ================================================
    // Create signals to transmit and UHD Tx streamers
    // ================================================
    
    // Generate baseband data to transmit
    std::vector<std::complex<float>> data_bb(10000);
    srand (1);
    for (size_t i = 0; i < 1000; i++){
        data_bb[i] = 0.5*((2*(rand() % 2) -1) + (2*(rand() % 2) -1)*1j) ;

    }
    for (size_t i = 1000; i < data_bb.size(); i++){
        data_bb[i] = 0.0;
    }





     // Generate baseband data to transmit
//   std::vector<std::complex<float>> data_bb(10000);
//     srand (1);
//    for (size_t i = 0; i < 250; i++){
//         data_bb[i] = (0.5) + (0.5)*1j ;
//     }
    
//      for (size_t i = 250; i < 500; i++){
//         data_bb[i] = (-0.5) + (0.5)*1j ;
//     } 
    
//     for (size_t i = 500; i < 750; i++){
//         data_bb[i] = (0.5) + (-0.5)*1j ;
//     } 
    
    
//     for (size_t i = 750; i < 1000; i++){
//         data_bb[i] = (-0.5) + (-0.5)*1j ;
//     } 
    
    
//     for (size_t i = 1000; i < data_bb.size(); i++){
//         data_bb[i] = 0.0;
//     }
 
  
    // create a transmit streamer for USRP-Tx
    std::vector<size_t> channel_nums_tx = {0};
    uhd::stream_args_t stream_args_tx("fc32", "sc16");
    stream_args_tx.channels = channel_nums_tx;
    uhd::tx_streamer::sptr stream_tx = usrp_tx->get_tx_stream(stream_args_tx);
    
    
    // =========================================================
    // start USRP-Tx worker thread 
    // =========================================================
    std::cout << boost::format("Starting USRP-Tx thread...") << std::endl;
    boost::thread_group tx_thread;
    tx_thread.create_thread(boost::bind(&tx_worker, data_bb, stream_tx));

   
 
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

    std::cout << "\n\n buffs variable is of size : " << buffs.size() << "!\n\n";
    
    //the first call to recv() will block this many seconds before receiving
    double timeout = seconds_in_future + 0.1; //timeout 
    
    uhd::time_spec_t RxTimePPS = usrp_rx_bb->get_time_now();
    uhd::time_spec_t TxTimePPS = usrp_tx->get_time_now();

    std::cout << "Rx USRP last PPS value is : " << RxTimePPS.get_full_secs() << "," << RxTimePPS.get_frac_secs() << "sec.\n";
    std::cout << "Tx USRP last PPS value is : " << TxTimePPS.get_full_secs() << "," << TxTimePPS.get_frac_secs() << "sec.\n";
    
    //setup streaming
	std::cout << boost::format("Begin streaming , %f seconds in the future...")  % seconds_in_future << std::endl;
	uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
	stream_cmd.stream_now = false;
	stream_cmd.time_spec = uhd::time_spec_t(seconds_in_future);
	rx_stream->issue_stream_cmd(stream_cmd);
	
    
    // ========================================================
	// Start looping over all AiP directions at Tx and Rx side
	// =====set_theta_channels===================================================


    //int theta_min_tx = -45;
    //int theta_max_tx = 45;
    //int theta_min_rx = -45;
    //int theta_max_rx = 45;

    //gains

    rf_mode_t modetx = MODE_TX;
    rf_mode_t moderx = MODE_RX;

    //set gain to maximum gain
    float gain_dB_Tx_max = service->get_dynamic_range(sn_Tx_BBox, modetx);

    float gain_dB_Rx_max = service->get_dynamic_range(sn_Rx_BBox, moderx);

    float gain_dB_Tx = gain_dB_Tx_max;
    float gain_dB_Rx = gain_dB_Rx_max;

   
    int off_channels[16]; // Adjust size if necessary, based on maximum possible size
    int nbr_channels = 0;

    if (nbr_active_antennas == 4) { 
        int temp[] = {1, 2, 3, 4, 5, 8, 9, 12, 13, 14, 15, 16}; // Keeping only the 4 central channels
        nbr_channels = sizeof(temp) / sizeof(temp[0]);
        memcpy(off_channels, temp, sizeof(temp));
    } else if (nbr_active_antennas == 1) {
        int temp[] = {1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16}; // Keeping only one channel
        nbr_channels = sizeof(temp) / sizeof(temp[0]);
        memcpy(off_channels, temp, sizeof(temp));
    }

    //Set RX
    //mode = MODE_RX;
    //int theta = 0;
    //set_theta(service, sn_Rx_BBox, gain_dB_Rx, theta); 

    //Shut down some antennas
    //service->set_off_channels(sn_Rx_BBox, off_channels, nbr_channels);
    //Sweeping

    int theta_tx = theta_min_tx;
    int theta_rx;

    //Shut down some antennas

        //writing parameters in file
     if (outfile.is_open()) {
        outfile << boost::format("Rate is %d and freq is %f") % rate_tx % freq_bb;
        outfile << boost::format("Gain tx is %d and Gain rx is %d") % gain_tx_bb % gain_rx_bb;
        outfile << boost::format("Number samps per degree is %d ") % nbr_samps_per_degree ;
        outfile << boost::format("Thetas min and max of tx and rx are %d %d %d %d") % theta_min_tx % theta_max_tx % theta_min_rx %  theta_max_rx;
        outfile << boost::format("Step is %d and N active antennas are %d") % step % nbr_active_antennas;
     }


    while(theta_tx <= theta_max_tx){  

        std::cout << boost::format("Setting Tx angle to %d ° at time %f") % theta_tx % usrp_tx->get_time_now().get_real_secs() << std::endl;
        set_theta(service, sn_Tx_BBox, gain_dB_Tx, theta_tx);

        //service->set_off_channels(sn_Tx_BBox, off_channels, nbr_channels);

        theta_rx = theta_min_rx;

        while(theta_rx <= theta_max_rx){

            std::cout << boost::format("Setting Rx angle to %d ° at time %f") % theta_rx % usrp_rx_bb->get_time_now().get_real_secs() << std::endl;
            set_theta(service, sn_Rx_BBox, gain_dB_Rx, theta_rx);

            float time_now = usrp_rx_bb->get_time_now().get_real_secs() ;  

            if (outfile.is_open()) {
                outfile << std::endl << "Angle Tx data" << std::endl ;
                outfile << boost::format("%d at time %f") % theta_tx % time_now;
                outfile << std::endl << "Angle Rx data" << std::endl ;
                outfile << boost::format("%d at time %f") % theta_rx % time_now;
                outfile << std::endl;				
            }
            
            // Receive "nbr_samps_per_degree" samples
            if (outfile.is_open()) {
                outfile << std::endl << "USRP data" << std::endl ;
            }

            size_t num_acc_samps = 0; //number of accumulated samples
            while(num_acc_samps < nbr_samps_per_degree){
                //receive a single packet
                size_t num_rx_samps = rx_stream->recv(&buffs.front(), buffs.size(), md, timeout, true);

                //use a small timeout for subsequent packets
                timeout = 0.1;
                
                //handle the error code
                if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) break;
                if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE){
                    throw std::runtime_error(str(boost::format(
                        "Receiver error %s"
                    ) % md.strerror()));
                }
                
                if (outfile.is_open()) {
                    outfile.write((const char*)&buffs.front(), num_rx_samps*sizeof(std::complex<float>));
                }
                
                num_acc_samps += num_rx_samps;

            }
            std::cout << boost::format("  -- Received %f samples") % num_acc_samps <<  std::endl;

            if (outfile.is_open()) {
                outfile << std::endl;
            }

            theta_rx = theta_rx + step;
      
            }
            
        theta_tx = theta_tx + step;
    }

    


    // ======================
    // Closing up everything 
    // ======================

    uhd::stream_cmd_t stop_cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
    rx_stream->issue_stream_cmd(stop_cmd);
    rx_stream.reset();

    
    // Stopping all transmitter threads
    stop_signal_called = true;
    
    // finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;

    outfile.close();
    
    return EXIT_SUCCESS;
}
