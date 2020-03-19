#include "SerialPort.h"


serialboost::SerialPort::SerialPort(boost::asio::io_service &ioService, 
    const std::string &portName) : 
    _serialPort(ioService, portName), 
    _isOpen(false), 
    _gcstodrone(true),
    //_context(1), 
    //_publisher(_context, ZMQ_PUB), 
    _contextsub(1), 
    _subscriber(_contextsub, ZMQ_SUB),
    _started(boost::posix_time::microsec_clock::local_time()),
    _current(boost::posix_time::microsec_clock::local_time()),
    _sock(socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP))/*buf("rollnew.txt")*/
    {

    strcpy(target_ip, "192.168.1.6");

    _readBuffer.resize(128);

    memset(&_buf, 0, sizeof(_buf));

    memset(&locAddr, 0, sizeof(locAddr));
	locAddr.sin_family = AF_INET;
	locAddr.sin_addr.s_addr = INADDR_ANY;
	locAddr.sin_port = htons(14551);

	if (-1 == bind(_sock,(struct sockaddr *)&locAddr, sizeof(struct sockaddr)))
    {
		perror("Error Bind Failed");
		close(_sock);
		exit(EXIT_FAILURE);
    }

    #if (defined __QNX__) | (defined __QNXNTO__)
		if (fcntl(_sock, F_SETFL, O_NONBLOCK | FASYNC) < 0)
	#else
		if (fcntl(_sock, F_SETFL, O_NONBLOCK | O_ASYNC) < 0)
	#endif

	{
		fprintf(stderr, "error setting nonblocking: %s\n", strerror(errno));
		close(_sock);
		exit(EXIT_FAILURE);
 	}

    memset(&gcAddr, 0, sizeof(gcAddr));
	gcAddr.sin_family = AF_INET;
	gcAddr.sin_addr.s_addr = inet_addr(target_ip);
	gcAddr.sin_port = htons(14550);
	//_publisher.bind("tcp://127.0.0.1:5563");
    
    }

 
boost::system::error_code serialboost::SerialPort::Flush() {
    boost::system::error_code ec;

    const bool isFlushed =! ::tcflush(_serialPort.native(), TCIOFLUSH);
    if (!isFlushed)
        ec = boost::system::error_code(errno, 
            boost::asio::error::get_system_category());

    return ec;
}

void serialboost::SerialPort::SetErrorCode(
    const boost::system::error_code &ec) {
    if (ec) {
        boost::mutex::scoped_lock lock(_errorCodeMutex);
        _errorCode = ec;
    }
}

void serialboost::SerialPort::ReadBegin() {
	//started_ = boost::posix_time::microsec_clock::universal_time();
    _serialPort.async_read_some(boost::asio::buffer(_readBuffer),
        boost::bind(&SerialPort::ReadComplete, shared_from_this(),
        boost::asio::placeholders::error,
        boost::asio::placeholders::bytes_transferred));
}

void serialboost::SerialPort::ReadComplete(
    const boost::system::error_code &ec, size_t bytesTransferred) {
    if (!ec) {
        if (bytesTransferred > 0){
				
			for(int i=0; i < bytesTransferred; i++){
				if(mavlink_parse_char(MAVLINK_COMM_0,boost::cref(_readBuffer[i]),&_msg,&_status))
					{
     		//		//std::cout << "Received message with ID" << _msg.msgid << _msg.compid << _msg.sysid << std::endl;
   					handle_message(&_msg);
   					}
			}
		}
        
		ReadBegin();  // queue another read
    
	} else {
        Close();
        SetErrorCode(ec);
    }
}

void serialboost::SerialPort::handle_message(mavlink_message_t *msg)
 	{
 		switch (msg->msgid){
		
 		case MAVLINK_MSG_ID_HEARTBEAT:
			handle_message_heartbeat(msg);
 			break;

 		case MAVLINK_MSG_ID_SYS_STATUS:
 			handle_message_status(msg);
 			break;

 		case MAVLINK_MSG_ID_ATTITUDE:
			handle_message_attitude(msg);
			break;
        
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
        	handle_message_id_param_request_list(msg);
        	break;

        case MAVLINK_MSG_ID_PARAM_VALUE:
        	handle_message_param_value(msg);
        	break;

        case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
        	handle_message_param_request_read(msg);
        	break;

        case MAVLINK_MSG_ID_PARAM_SET:
        	handle_message_param_set(msg);
        	break;

        case MAVLINK_MSG_ID_MISSION_REQUEST_INT:
        	handle_message_id_mission_request_int(msg);
        	break;

        case MAVLINK_MSG_ID_MISSION_ITEM_INT:
        	handle_message_id_mission_item_int(msg);
        	break;

        case MAVLINK_MSG_ID_MISSION_ITEM_REACHED:
            handle_message_mission_item_reached(msg);
            break;

        case MAVLINK_MSG_ID_MISSION_COUNT:
            handle_message_mission_count(msg);
            break;

        case MAVLINK_MSG_ID_MISSION_ITEM:
            handle_message_mission_item(msg);
            break;

        case MAVLINK_MSG_ID_MISSION_ACK:
        	handle_message_id_mission_ack(msg);
        	break;

        case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
        	handle_message_mission_request_list(msg);
        	break;

        case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
        	handle_message_set_current_mission_id(msg);
        	break;

        case MAVLINK_MSG_ID_MISSION_CURRENT:
        	handle_message_mission_current(msg);
        	break;

        case MAVLINK_MSG_ID_STATUSTEXT:
        	handle_message_statustext(msg);
        	break;

        case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
        	handle_message_mission_clear_all(msg);
        	break;

		//case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
		//	handle_message_lpos_ned(msg);
		//	break;

		//case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
		//	handle_message_lpos_ned_target(msg);
		//  break;
 		default:
			break;
 		}
 	}

void serialboost::SerialPort::handle_message_param_set(mavlink_message_t *msg)
	{

		uint8_t buffer[128];
		size_t len = mavlink_msg_to_send_buffer(buffer, msg);
		std::vector<unsigned char> vec(buffer, buffer + len);
		Write(&vec[0], vec.size());

	}

void serialboost::SerialPort::handle_message_param_request_read(mavlink_message_t *msg)
	{		

		uint8_t buffer[128];
		size_t len = mavlink_msg_to_send_buffer(buffer, msg);
		std::vector<unsigned char> vec(buffer, buffer + len);
		Write(&vec[0], vec.size());

	}

void serialboost::SerialPort::handle_message_id_param_request_list(mavlink_message_t *msg)
	{

		uint8_t buffer[128];
		size_t len = mavlink_msg_to_send_buffer(buffer, msg);	
		std::vector<unsigned char> vec(buffer, buffer + len);
		Write(&vec[0], vec.size());

	}

void serialboost::SerialPort::handle_message_param_value(mavlink_message_t *msg)
	{

		memset(&_buf, 0, sizeof(_buf));
		uint16_t len = mavlink_msg_to_send_buffer(_buf, msg);
		int bytes_sent = sendto(_sock, _buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));

	}

void serialboost::SerialPort::handle_message_id_mission_request_int(mavlink_message_t *msg)
	{

		if(_gcstodrone){
			/*When GCS to Drone is active mission request int goes from drone to gcs*/
			memset(&_buf, 0, sizeof(_buf));
			uint16_t len = mavlink_msg_to_send_buffer(_buf, msg);
			int bytes_sent = sendto(_sock, _buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
		
		}else{

			uint8_t buffer[128];
			size_t len = mavlink_msg_to_send_buffer(buffer, msg);	
			std::vector<unsigned char> vec(buffer, buffer + len);
			Write(&vec[0], vec.size());

		}
	
	}

void serialboost::SerialPort::handle_message_id_mission_item_int(mavlink_message_t *msg)
	{

		if(_gcstodrone){
			
			uint8_t buffer[128];
			size_t len = mavlink_msg_to_send_buffer(buffer, msg);	
			std::vector<unsigned char> vec(buffer, buffer + len);
			Write(&vec[0], vec.size());
		
		}else{
			
			memset(&_buf, 0, sizeof(_buf));
			uint16_t len = mavlink_msg_to_send_buffer(_buf, msg);
			int bytes_sent = sendto(_sock, _buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));

		}
	}

void serialboost::SerialPort::handle_message_mission_item(mavlink_message_t *msg)
    {
        	mavlink_mission_item_t mission;
        	mavlink_msg_mission_item_decode(msg, &mission);
        	std::cout << unsigned(mission.seq) << " "<< mission.x << " "<< mission.y << std::endl;
    }

void serialboost::SerialPort::handle_message_id_mission_ack(mavlink_message_t *msg)
	{
		if(_gcstodrone){

			memset(&_buf, 0, sizeof(_buf));
			uint16_t len = mavlink_msg_to_send_buffer(_buf, msg);
			int bytes_sent = sendto(_sock, _buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));

		}else{

			uint8_t buffer[128];
			size_t len = mavlink_msg_to_send_buffer(buffer, msg);	
			std::vector<unsigned char> vec(buffer, buffer + len);
			Write(&vec[0], vec.size());

		}
	}

void serialboost::SerialPort::handle_message_mission_count(mavlink_message_t *msg)
    {
		if(_gcstodrone){
			/*This is the mission count being sent from gcs to drone*/
			uint8_t buffer[128];
			size_t len = mavlink_msg_to_send_buffer(buffer, msg);	
			std::vector<unsigned char> vec(buffer, buffer + len);
			Write(&vec[0], vec.size());
    	
    	}else{
    		/*if _gcstodrone is false that means mission count is sent from drone to gcs*/
    		memset(&_buf, 0, sizeof(_buf));
			uint16_t len = mavlink_msg_to_send_buffer(_buf, msg);
			int bytes_sent = sendto(_sock, _buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));

    	}
    }

void serialboost::SerialPort::handle_message_mission_item_reached(mavlink_message_t *msg)
    {
        
    	memset(&_buf, 0, sizeof(_buf));
		uint16_t len = mavlink_msg_to_send_buffer(_buf, msg);
		int bytes_sent = sendto(_sock, _buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));
        //mavlink_mission_item_reached_t missionreached;
        //mavlink_msg_mission_item_reached_decode(msg, &missionreached);

        //std::cout << unsigned(missionreached.seq) << std::endl;
    }

void serialboost::SerialPort::handle_message_mission_request_list(mavlink_message_t *msg)
	{

		_gcstodrone = false;
		uint8_t buffer[128];
		size_t len = mavlink_msg_to_send_buffer(buffer, msg);
		std::vector<unsigned char> vec(buffer, buffer + len);
		Write(&vec[0], vec.size());

	}

void serialboost::SerialPort::handle_message_set_current_mission_id(mavlink_message_t *msg)
	{
		
		uint8_t buffer[128];
		size_t len = mavlink_msg_to_send_buffer(buffer, msg);
		std::vector<unsigned char> vec(buffer, buffer + len);
		Write(&vec[0], vec.size());		

	}

void serialboost::SerialPort::handle_message_mission_current(mavlink_message_t *msg)
	{

		memset(&_buf, 0, sizeof(_buf));
		uint16_t len = mavlink_msg_to_send_buffer(_buf, msg);
		int bytes_sent = sendto(_sock, _buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));

	}

void serialboost::SerialPort::handle_message_statustext(mavlink_message_t *msg)
	{

		memset(&_buf, 0, sizeof(_buf));
		uint16_t len = mavlink_msg_to_send_buffer(_buf, msg);
		int bytes_sent = sendto(_sock, _buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));

	}

void serialboost::SerialPort::handle_message_mission_clear_all(mavlink_message_t *msg)
	{

		_gcstodrone = true;
		uint8_t buffer[128];
		size_t len = mavlink_msg_to_send_buffer(buffer, msg);
		std::vector<unsigned char> vec(buffer, buffer + len);
		Write(&vec[0], vec.size());		

	}

void serialboost::SerialPort::handle_message_heartbeat(mavlink_message_t *msg)
 	{
		
	 		
		mavlink_heartbeat_t hb;
 		mavlink_msg_heartbeat_decode(msg, &hb);

 		/*Sending data to QGC*/
		memset(&_buf, 0, sizeof(_buf));
		uint16_t len = mavlink_msg_to_send_buffer(_buf, msg);
		int bytes_sent = sendto(_sock, _buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));

 		std::cout << "Received Heartbeat Message" << std::endl;
 	}

void serialboost::SerialPort::handle_message_attitude(mavlink_message_t *msg)
 	{
		
		mavlink_attitude_t at;
		mavlink_msg_attitude_decode(msg, &at);
		
		zmq::message_t message(20);
		float roll = at.roll;
		float yaw = at.yaw;
		float pitch = at.pitch;		
		
		snprintf((char *)message.data(), 20, "%0.3f %0.3f %0.3f", roll, yaw ,pitch);
		//_publisher.send(message);
 	}

void serialboost::SerialPort::handle_message_status(mavlink_message_t *msg)
	{
		// /mavlink_sys_status_t ss;

		memset(&_buf, 0, sizeof(_buf));
		uint16_t len = mavlink_msg_to_send_buffer(_buf, msg);
		int bytes_sent = sendto(_sock, _buf, len, 0, (struct sockaddr*)&gcAddr, sizeof(struct sockaddr_in));

	}

void serialboost::SerialPort::WriteToPixhawk(){
		mavlink_message_t msg;
		uint8_t buffer[128];
		
		mavlink_msg_command_long_pack(1, 1, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, 1, 1, 0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

		size_t len = mavlink_msg_to_send_buffer(buffer, &msg);	

		std::vector<unsigned char> vec(buffer, buffer + len);
		
		Write(&vec[0], vec.size());

		//std::cout << "here" << std::endl;
	}

void serialboost::SerialPort::WriteToPixhawkOffboardSetpoint(uint32_t ms, float x, float y, float z, float vx, float vy, float vz, float afx, float afy, float afz, float yaw, float yaw_rate ){
        mavlink_message_t msg;
        uint8_t buffer[128];
        // uint16_t typemask = 0b0010110111000111;
        uint16_t typemask = 0b0010110111111000;

        mavlink_msg_set_position_target_local_ned_pack(1, 1, &msg, ms, 1, 1, 1, typemask, x, y, z, vx, vy, vz, afx, afy, afz, yaw, yaw_rate);

        size_t len = mavlink_msg_to_send_buffer(buffer, &msg);

        std::vector<unsigned char> vec(buffer, buffer + len);

        Write(&vec[0], vec.size());

}

void serialboost::SerialPort::Write(const unsigned char *buffer, 
    size_t bufferLength) {
    {
        boost::mutex::scoped_lock lock(_writeQueueMutex);
        _writeQueue.insert(_writeQueue.end(), buffer, 
            buffer+bufferLength);
    }
    _serialPort.get_io_service().post(boost::bind(
        &SerialPort::WriteBegin, shared_from_this()));
	
}

void serialboost::SerialPort::WriteBegin() {
	 		//std::cout << "here also" << std::endl;
    boost::mutex::scoped_lock writeBufferlock(_writeBufferMutex);
    if (_writeBuffer.size() != 0)
        return;  // a write is in progress, so don't start another

    boost::mutex::scoped_lock writeQueuelock(_writeQueueMutex);
    if (_writeQueue.size() == 0)
        return;  // nothing to write
 
    // allocate a larger buffer if needed
    const std::vector<unsigned char>::size_type writeQueueSize = 
        _writeQueue.size();
    if (writeQueueSize > _writeBuffer.size()) 
        _writeBuffer.resize(writeQueueSize);
 
    // copy the queued bytes to the write buffer, 
    // and clear the queued bytes
    std::copy(_writeQueue.begin(), _writeQueue.end(), 
        _writeBuffer.begin());
    _writeQueue.clear();
 	

	//std::cout << "def getting here" << std::endl;

    boost::asio::async_write(_serialPort, 
        boost::asio::buffer(_writeBuffer, writeQueueSize),
        boost::bind(&SerialPort::WriteComplete, shared_from_this(), 
        boost::asio::placeholders::error));
}

void serialboost::SerialPort::WriteComplete(
    const boost::system::error_code &ec) {
    if (!ec) {
        {
            boost::mutex::scoped_lock lock(_writeBufferMutex);
            _writeBuffer.clear();
        }
    } else {
        Close();
        SetErrorCode(ec);
    }
}

void serialboost::SerialPort::openport(unsigned int baudRate,
    serialboost::SerialParams serialParams,
    boost::asio::serial_port_base::flow_control flowControl) {
        //_onRead = onRead;
        _serialPort.set_option(
            boost::asio::serial_port_base::baud_rate(baudRate));
        _serialPort.set_option(serialParams.get<0>());
        _serialPort.set_option(serialParams.get<1>());
        _serialPort.set_option(serialParams.get<2>());
        _serialPort.set_option(flowControl);
 
        const boost::system::error_code ec = Flush();
        if (ec)
            SetErrorCode(ec);
 
        _isOpen = true;
 		
		_serialPort.get_io_service().post(boost::bind(
            &SerialPort::ReadBegin, shared_from_this()));
}

serialboost::SerialPort::~SerialPort() {
    Close();
}

void serialboost::SerialPort::Close() {
    if (_isOpen) {
        _isOpen = false;
        boost::system::error_code ec;
        _serialPort.cancel(ec);
        SetErrorCode(ec);
        _serialPort.close(ec);
        SetErrorCode(ec);
    }
}

void serialboost::SerialPort::Write(
    const std::vector<unsigned char> &buffer) {
    Write(&buffer[0], buffer.size());
}

void serialboost::SerialPort::Write(const std::string &buffer) {
    Write(reinterpret_cast<const unsigned char *>(buffer.c_str()), 
        buffer.size());
}

void serialboost::SerialPort::testfunc()
{
	//boost::posix_time::seconds worktime(3);	
	//while(1){

	_subscriber.connect("tcp://127.0.0.1:5556");
	_subscriber.setsockopt( ZMQ_SUBSCRIBE, "", 0);
	
	//std::cout << "priinting some" << std::endl;	

	try{
		while(true){
	//for(int up_nbr = 0; up_nbr < 100; up_nbr++){		
		
		zmq::message_t update;

		_subscriber.recv(&update);

		int zipcode, temperature, relhumidity;
		std::istringstream iss(static_cast<char *>(update.data()));
		iss >> zipcode >> temperature >> relhumidity;

		std::cout << "ZTR" << zipcode << " " << temperature << " " << relhumidity << " " << std::endl;
	
		WriteToPixhawk();

			}
		}
	 catch (std::exception &e) {

	 	std::cout << "Error occurred " << std::endl;
	 }		
	
}

void serialboost::SerialPort::sendoffboardcommands()
{
    int count = 0;
    uint32_t millis = 0;
    float vx = 0.0;
    float vy = 0.0;
    float vz = 0.0;

    try{
        while(true){

            std::cout << "offboard mode" << std::endl;
            _current = boost::posix_time::microsec_clock::local_time();
            boost::posix_time::time_duration timelapsed = _current - _started;
            millis = timelapsed.total_milliseconds();

            WriteToPixhawkOffboardSetpoint(millis, 0.0, 0.0, 0.1, vx, vy, vz, 0.0, 0.0, 0.0, 0.0, 0.0);
            usleep(100000);

            }
        }
    catch (std::exception &e){
        std::cout << "Error occured " << std::endl;
    }
}

void serialboost::SerialPort::recvudpqgc()
{
    /*Receive Data From QGC's UDP Port*/
    int recsize;
    int nbytes;
    mavlink_message_t msgqgc;
    mavlink_status_t statusqgc;
    
    while(true){

    	FD_ZERO(&rfs);
    	FD_SET(_sock, &rfs);

        memset(&_buf, 0, sizeof(_buf));

        std::cout << "getting here" << std::endl;
        recsize = select(_sock+1, &rfs, NULL, NULL, NULL);

        if (recsize >= 1){

        	nbytes = recvfrom(_sock, (void *)_buf, BUFFER_LENGTH, 0, (struct sockaddr *)&gcAddr, &_fromlen);
            
        	if(nbytes > 0){
        		for (int k = 0; k < nbytes; k++)
					{
						if (mavlink_parse_char(MAVLINK_COMM_1, _buf[k], &msgqgc, &statusqgc))
						{
							if(msgqgc.msgid == MAVLINK_MSG_ID_MISSION_COUNT){

								_gcstodrone = true;
							
							}

							handle_message(&msgqgc);
						}
					}
        	}

            std::cout << nbytes<<std::endl;
        
        }
        
        else {
            
            std::cout << "no data"<<std::endl;
        
        }

    }

}