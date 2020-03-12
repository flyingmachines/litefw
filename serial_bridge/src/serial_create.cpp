#include "serial_create.h"

serial_create::serial_create(const std::string &portName, int baudRate) : 
        _portName(portName), _baudRate(baudRate) {}



void serial_create::Create(boost::asio::io_service &ios) {
        try {

            _serialPort.reset(new serialboost::SerialPort(ios,  _portName));  
	
			_serialPort->openport(_baudRate);

			//boost::thread subthread(boost::bind(&serialboost::SerialPort::testfunc, _serialPort));	

			boost::thread offboardsubthread(boost::bind(&serialboost::SerialPort::sendoffboardcommands, _serialPort));

			boost::thread udpsubthread(boost::bind(&serialboost::SerialPort::udpqgc, _serialPort));
				
			//subthread.detach();	

			offboardsubthread.detach();	

			udpsubthread.detach();	

        } catch (const std::exception &e) {

            std::cout << e.what() << std::endl;     

        }
}

void serial_create::Run(unsigned int numThreads){

    if(OnRun){
        OnRun(_ioservice);
    }

    boost::thread_group workerThreads;

    for (unsigned int i=0; i < ((numThreads == (unsigned int)-1) ? (boost::thread::hardware_concurrency()) : numThreads); ++i){
        workerThreads.create_thread(boost::bind(&serial_create::WorkerThread, this, boost::ref(_ioservice)));
    }

    workerThreads.join_all();
}

void serial_create::WorkerThread(boost::asio::io_service &ios) {
    if (OnWorkerThreadStart)
        OnWorkerThreadStart(ios);
 
    while (true) {
        try
        {
            boost::system::error_code ec;

            ios.run(ec);
            if (ec && OnWorkerThreadError)
                OnWorkerThreadError(ios, ec);
            break;
        }
        catch(const std::exception &ex) {
            if (OnWorkerThreadException)
                OnWorkerThreadException(ios, ex);
        }
    }
 
    if (OnWorkerThreadStop)
        OnWorkerThreadStop(ios);
}

void serial_create::AddCtrlCHandling() {
    boost::asio::signal_set sig_set(_ioservice, SIGTERM, SIGINT);
    sig_set.async_wait(boost::bind(
        &boost::asio::io_service::stop, boost::ref(_ioservice)));
}

int main(int argc, char *argv[]) {
    
    std::string portName, file;
    
    int baudRate;
    
    boost::program_options::options_description desc("Options");
    
    desc.add_options()
        ("help,h", "help")
        ("port,p", boost::program_options::value<std::string>(
            &portName)->required(), "port name (required)")
        ("baud,b", boost::program_options::value<int>(
            &baudRate)->required(), "baud rate (required)")
        ("file,f", boost::program_options::value<std::string>(
            &file), "file to save to")
        ;
    
    boost::program_options::variables_map vm;
    
    boost::program_options::store(
        boost::program_options::parse_command_line(
            argc, argv, desc), vm);

    if (vm.empty() || vm.count("help")) {
    
        std::cout << desc << "\n";
    
        return -1;
    }
    
    boost::program_options::notify(vm);

    const boost::shared_ptr<serial_create> sp(new serial_create(
        portName, baudRate));
    
    //sp->OnWorkerThreadError;
    //sp->OnWorkerThreadException;

    sp->OnRun = boost::bind(&serial_create::Create, sp, _1);
    sp->Run();
         
    return 0;
}
