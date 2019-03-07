#include "Executor.h"

void serialboost::Executor::Run(unsigned int numThreads){

	if(OnRun){
		OnRun(_ioservice);
	}

	boost::thread_group workerThreads;

	for (unsigned int i=0; i < ((numThreads == (unsigned int)-1) ? (boost::thread::hardware_concurrency()) : numThreads); ++i){
		workerThreads.create_thread(boost::bind(&Executor::WorkerThread, this, boost::ref(_ioservice)));
	}

	workerThreads.join_all();
}

void serialboost::Executor::WorkerThread(boost::asio::io_service &ios) {
    if (OnWorkerThreadStart)
        OnWorkerThreadStart(ios);
 
    while (true) {
        try
        {
            boost::system::error_code ec;
			//boost::function<void(const boost::system::error_code&)> handler;

			//boost::asio::deadline_timer t(_ioservice, boost::posix_time::seconds(5));
			
			//t.async_wait(handler);

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

void serialboost::Executor::AddCtrlCHandling() {
    boost::asio::signal_set sig_set(_ioservice, SIGTERM, SIGINT);
    sig_set.async_wait(boost::bind(
        &boost::asio::io_service::stop, boost::ref(_ioservice)));
}
