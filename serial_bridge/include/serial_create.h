//#include "Executor.h"
#include "SerialPort.h"
#include <boost/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/program_options.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/vector.hpp>

class serial_create : private boost::noncopyable, 
    public boost::enable_shared_from_this<serial_create> {

    boost::shared_ptr<serialboost::SerialPort> _serialPort;

    std::string _portName;

    unsigned int _baudRate;

protected:

	boost::asio::io_service _ioservice;

	void WorkerThread(boost::asio::io_service &io_service);

public:

    serial_create(const std::string &portName, int baudRate);
   
	void Create(boost::asio::io_service &ios); 

	boost::function<void (boost::asio::io_service &)> OnWorkerThreadStart;

	boost::function<void (boost::asio::io_service &)> OnWorkerThreadStop;

	boost::function<void (boost::asio::io_service &, boost::system::error_code)> OnWorkerThreadError;

	boost::function<void (boost::asio::io_service &, const std::exception &)> OnWorkerThreadException;

	boost::function<void (boost::asio::io_service &)> OnRun;

	boost::asio::io_service &GetIOService() {return _ioservice;}

	void AddCtrlCHandling();

	void Run(unsigned int numThreads = -1);
};
