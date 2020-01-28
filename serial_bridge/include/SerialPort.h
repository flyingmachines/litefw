#include <boost/asio.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/function.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/iostreams/device/file.hpp>
#include <boost/iostreams/stream.hpp>
#include <boost/lexical_cast.hpp>
#include <ostream>
#include <sys/time.h>
#include <boost/chrono.hpp>
#include <boost/bind.hpp>
#include "common/mavlink.h"
#include "mavlink_types.h"
#include "zhelpers.hpp"

#define NELEMS(x)  (sizeof(x) / sizeof((x)[0]))

namespace serialboost {
    typedef boost::tuple<
        boost::asio::serial_port_base::character_size,
        boost::asio::serial_port_base::parity,
        boost::asio::serial_port_base::stop_bits> SerialParams;
 
    static const SerialParams SP_8N1 = boost::make_tuple(
        8,
        boost::asio::serial_port_base::parity::none,
        boost::asio::serial_port_base::stop_bits::one);
 
    static const SerialParams SP_7E1 = boost::make_tuple(
        7,
        boost::asio::serial_port_base::parity::even,
        boost::asio::serial_port_base::stop_bits::one);
	typedef boost::posix_time::ptime Time;
	typedef boost::posix_time::time_duration TimeDuration;

    class SerialPort : private boost::noncopyable, 
        public boost::enable_shared_from_this<SerialPort>  {
        
        boost::asio::serial_port _serialPort;
        
        bool _isOpen;

		mavlink_message_t _msg;

		mavlink_status_t _status;
        
        boost::system::error_code _errorCode;
        
        boost::mutex _errorCodeMutex;
        
        std::vector<unsigned char> _readBuffer;
        
        boost::function<void (boost::asio::io_service &, 
            const std::vector<unsigned char> &, size_t)> _onRead;
 
        std::vector<unsigned char> _writeQueue, _writeBuffer;
        
        boost::mutex _writeQueueMutex, _writeBufferMutex;

        boost::system::error_code Flush();

		zmq::context_t _context;

		zmq::socket_t _publisher;

		zmq::context_t _contextsub; /*This has to be first else throws weird zmq_errort that bad address*/
	
		zmq::socket_t _subscriber;

		//boost::posix_time::ptime started_;// = boost::chrono::system_clock::now()
		
		//boost::posix_time::ptime ended_;

		//boost::iostreams::stream_buffer<boost::iostreams::file_sink> buf;
        
        void SetErrorCode(const boost::system::error_code &ec);
        
        void ReadBegin();
        
        void ReadComplete(const boost::system::error_code &ec, size_t bytesTransferred);
        
        void WriteBegin();
        
		void WriteToPixhawk();        
		
		void WriteComplete(const boost::system::error_code &ec);

		void handle_message(mavlink_message_t *msg);
		
		void handle_message_heartbeat(mavlink_message_t *msg);

		void handle_message_attitude(mavlink_message_t *msg);

        void handle_message_mission_item(mavlink_message_t *msg);

        void handle_message_mission_item_reached(mavlink_message_t *msg);

        void handle_message_mission_count(mavlink_message_t *msg);
		
		//void testfunc();


    public:

        SerialPort(boost::asio::io_service &ioService, const std::string &portName);
        
        ~SerialPort();
        
        void openport(unsigned int baudRate,
            SerialParams serialParams = SP_8N1,
            boost::asio::serial_port_base::flow_control flowControl =
                boost::asio::serial_port_base::flow_control(
                    boost::asio::serial_port_base::flow_control::none));

        void Close();
        
        void Write(const unsigned char *buffer, size_t bufferLength);
        
        void Write(const std::vector<unsigned char> &buffer);
        
        void Write(const std::string &buffer);	

		void testfunc();
    };
};





