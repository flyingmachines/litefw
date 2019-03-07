#include <boost/asio.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>

namespace serialboost {

	class Executor : private boost::noncopyable{

	protected:

		boost::asio::io_service _ioservice;

		void WorkerThread(boost::asio::io_service &io_service);

	public:
		boost::function<void (boost::asio::io_service &)> OnWorkerThreadStart;

		boost::function<void (boost::asio::io_service &)> OnWorkerThreadStop;

		boost::function<void (boost::asio::io_service &, boost::system::error_code)> OnWorkerThreadError;

		boost::function<void (boost::asio::io_service &, const std::exception &)> OnWorkerThreadException;

		boost::function<void (boost::asio::io_service &)> OnRun;

		boost::asio::io_service &GetIOService() {return _ioservice;}

		void AddCtrlCHandling();

		void Run(unsigned int numThreads = -1);

	};
}
