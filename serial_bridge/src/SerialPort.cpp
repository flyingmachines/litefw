#include "SerialPort.h"

serialboost::SerialPort::SerialPort(boost::asio::io_service &ioService, 
    const std::string &portName) : 
    _serialPort(ioService, portName), 
    _isOpen(false), 
    _start(false), 
    _contextsub(1), 
    _subscriber(_contextsub, ZMQ_SUB), 
    _xned(0.0), 
    _yned(0.0), 
    _zned(0.0), 
    _cnt(0), 
    _yawip(0.0)
    //_ms(-100.0),
    /*buf("rollnew.txt")*/{
    _readBuffer.resize(128);
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
            //_onRead(boost::ref(_serialPort.get_io_service()), 
            //    boost::cref(_readBuffer), bytesTransferred);
				
			for(int i=0; i < bytesTransferred; i++){
				if(mavlink_parse_char(MAVLINK_COMM_0,boost::cref(_readBuffer[i]),&_msg,&_status))
					{
     				//std::cout << "Received message with ID" << _msg.msgid << _msg.compid << _msg.sysid << std::endl;
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

 		case MAVLINK_MSG_ID_ATTITUDE:
			handle_message_attitude(msg);
			break;

		//case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
			//handle_message_lpos_ned(msg);
			//break;

		//case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
		//	handle_message_lpos_ned_target(msg);
		//  break;
 		default:
			break;
 		}
 	}

void serialboost::SerialPort::handle_message_heartbeat(mavlink_message_t *msg)
 	{
		
	 		
		mavlink_heartbeat_t hb;
 		mavlink_msg_heartbeat_decode(msg, &hb);

 		std::cout << "Received Heartbeat Message" << std::endl;
 	}

/*void serialboost::SerialPort::handle_message_lpos_ned(mavlink_message_t *msg)
{
	mavlink_local_position_ned_t lpos;
	mavlink_msg_local_position_ned_decode(msg, &lpos);

	std::cout<<"\nReceived LOCAL_POSITION_NED MESSAGE"<<std::endl;
	

	float pos_x,pos_y,pos_z;

	pos_x = lpos.x;
	pos_y = lpos.y;
	pos_z = lpos.z;

	//printf("\n Received position : [x=%f y=%f z=%f]\n",,pos_x,pos_y,pos_z);

	Add respective mavlink libraries and case messages and add publisher.send at the end of each handle_message callback function

}
*/


void serialboost::SerialPort::WriteToPixhawk(){
		mavlink_message_t msg;
		
		uint8_t buffer[128];
		
		//float q[4] = {1.0,0.0,0.0,0.0};
		//mavlink_msg_att_pos_mocap_pack(1, 1, &msg, 0, q , 0.3f, 0.6f, -0.5f);

		mavlink_msg_att_pos_mocap_pack(1, 1, &msg, 0, q, _xned, _yned, _zned);


		size_t len = mavlink_msg_to_send_buffer(buffer, &msg);	

		std::vector<unsigned char> vec(buffer, buffer + len);
		
		Write(&vec[0], vec.size());

		//std::cout << "here" << std::endl;
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
 		
 		//std::cout << "getting here " << std::endl;
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

void serialboost::SerialPort::handle_message_attitude(mavlink_message_t *msg)
 	{
		
		mavlink_attitude_t at;
		mavlink_msg_attitude_decode(msg, &at);
		
		//zmq::message_t message(20);
		//float roll = at.roll;
		//float yaw = at.yaw;
		//float dt = 0.05;
		//float pitch = at.pitch;		

		//std::cout << roll << std::endl;

		// if (_admittancecnt == 5){

		// 	_errk = -_ms*dt*dt*(yaw + 2.0*_tauk1 + _tauk2) - (2.0f*dt*dt*_ks - 8.0f)*_errk1 -(4.0f + dt*dt*_ks - 2.0f*_ds*dt)*_errk2;
		// 	_errk = _errk / (4.0f + 2.0f*_ds*dt + dt*dt*_ks);
			
		// 	if(abs(_errk - yaw) > 0.3){
		// 		_errk = yaw;
		// 	}

		// 	_errk2 = _errk1;
		// 	_errk1 = _errk;
		// 	_tauk2 = _tauk1;
		// 	_tauk1 = yaw;

		// 	_admittancecnt = 0;
		// 	_yawip = _errk;

		// 	//std::cout << _yawip << " " << yaw <<  std::endl;
		// }

		// _admittancecnt++;
		
		// //snprintf((char *)message.data(), 20, "%0.3f %0.3f %0.3f", roll, yaw ,pitch);
		// //_publisher.send(message);
 	}

void serialboost::SerialPort::bridge_subscribe()
{
	_subscriber.connect("tcp://127.0.0.1:3885");
    _subscriber.setsockopt(ZMQ_SUBSCRIBE,"",0);
    
    try {
        while(1)
        {
            zmq::message_t pose_msgR;

            _subscriber.recv(&pose_msgR);  

            float quart_w, quart_x, quart_y, quart_z, x, y, z;
            
            std::istringstream iss(static_cast<char *>(pose_msgR.data()));

            
            iss>>quart_w>>quart_x>>quart_y>>quart_z>>x>>y>>z; 

            q[0] = quart_w; 
            q[1] = quart_x;
            q[2] = quart_y;
            q[3] = quart_z;
            _xned = x;
            _yned = y;
            _zned = z; 

            if(_cnt == 100){
            std::cout<<"\n Orientation :\n"<<"\t"<<q[0]<<"\n \t"<<q[1]<<"\n \t"<<q[2]<<"\n \t"<<q[3]<<std::endl;
            std::cout<<"\n Position :\n"<<"\t"<<_xned<<"\n \t"<<_yned<<"\n \t"<<_zned<<std::endl; 
            _cnt = 0;
            }

            _cnt++;
            
            WriteToPixhawk();

        }
       
    }
    catch(std::exception &e){
    	std::cout<<"Error Occured!! \n";
   } 
}

void serialboost::SerialPort::trajecmod()
{
	mavlink_message_t msg;
	uint8_t buffer[128];
	uint16_t spmask = 0b0000100111111000;
	//uint16_t spmask = 0b0000110111000111;
	//std::string data("test.csv");
	//std::ifstream in(data.c_str());
	
	//while(!in.is_open()) return 1;

	//typedef boost::tokenizer<boost::escaped_list_separator<char> > Token;
	//std::vector<std::string> vec;
	//std::string line;
	//io::CSVReader<5> in("traj2.csv");
	//in.read_header(io::ignore_extra_column, "t", "x", "y", "z", "r");
	//float t, x, y, z, r;
	//_yawned = -1.5f;
	float xv = 0.0;
	float yv = 0.0;
	float yawoffboard = -1.5f;
	float xtraveled = 0.0;
	float ytraveled = 0.0;

	while(true){
		if(_start){

		xv = 0.0f;
		yv = -3.0f;
		yawoffboard = _yawip;

		}
		mavlink_msg_set_position_target_local_ned_pack(1, 1, &msg, 0, 0, 0, 1, spmask, xv, yv, -0.85f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, yawoffboard, 0.0f);
		size_t len = mavlink_msg_to_send_buffer(buffer, &msg);
		std::vector<unsigned char> vec(buffer, buffer+len);
		Write(&vec[0], vec.size());
		usleep(50000);
	}	
}

void serialboost::SerialPort::starttraj()
{
	int strinp;
	
	std::cout << "Do you want to start trajectory. Enter start" << std::endl;

	std::cin >> strinp;

	_start = strinp;
}
