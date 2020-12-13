#include "SerialPort.h"
#include <math.h>


serialboost::SerialPort::SerialPort(boost::asio::io_service &ioService, 
    const std::string &portName) : 
    _serialPort(ioService, portName), 
    _isOpen(false), 
    _initz(false),
    _offb(false),
    _visrecv(false),
    _timeinit(false),
    _within0_5(false),
    _stopmotors(false),
    _zned(0.0),
    _vzned(0.0),
    _xned(0.0),
    _yned(0.0),
    _xvned(0.0),
    _yvned(0.0),
    _xobjned(0.0),
    _yobjned(0.0),
    _vertd(0.0),
    _zsp(0.0),
    _vsum(0.0),
    _uz(0.0),
    _ux(0.0),
    _uy(0.0),
    _phi(0.0),
    _theta(0.0),
    _psi(0.0),
    _uzscale(0.6),
    _uxscale(0.0),
    _uyscale(0.0),
    _lidarprev(0.0),
    _xd(0.0),
    _yd(0.0),
    _xd1(0.0),
    _yd1(0.0),
    _xd2(0.0),
    _yd2(0.0),
    _xdf(0.0),
    _xdf1(0.0),   
    _xdf2(0.0),
    _ydf(0.0),
    _ydf1(0.0),
    _ydf2(0.0),
    _cnt(0),
    _cntsub(0),
    _cntldr(0),
    _cntlog(0),
    _context(1), 
    _publisher(_context, ZMQ_PUB), 
    _contextsub(1), 
    _subscriber(_contextsub, ZMQ_SUB),
    _ctxtvis(1),
    _subscribevis(_ctxtvis, ZMQ_SUB),
    _started(boost::posix_time::microsec_clock::local_time()),
    _current(boost::posix_time::microsec_clock::local_time()),
    _current1(boost::posix_time::microsec_clock::local_time()),
    _previous(boost::posix_time::microsec_clock::local_time()),
    _offbrecv(boost::posix_time::microsec_clock::local_time()),
    _offbrecvcurr(boost::posix_time::microsec_clock::local_time()),
    _visionupdate(boost::posix_time::microsec_clock::local_time()),
    _buflog("log1.txt"){
    _readBuffer.resize(256);
    //_landBuffer.resize(15);
	_publisher.bind("tcp://127.0.0.1:5563");
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
        
        //Flush();
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

        case MAVLINK_MSG_ID_MISSION_ITEM_REACHED:
            handle_message_mission_item_reached(msg);
            break;

        case MAVLINK_MSG_ID_MISSION_COUNT:
            handle_message_mission_count(msg);
            break;
        case MAVLINK_MSG_ID_MISSION_ITEM:
            handle_message_mission_item(msg);
            break;

		case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
			handle_message_lpos_ned(msg);
			break;

        case MAVLINK_MSG_ID_DISTANCE_SENSOR:
            handle_message_lidar(msg);
            break;

		//case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
		//	handle_message_lpos_ned_target(msg);
		//  break;
 		default:
			break;
 		}
 	}

void serialboost::SerialPort::handle_message_lidar(mavlink_message_t *msg)
    {
        mavlink_distance_sensor_t dst;
        mavlink_msg_distance_sensor_decode(msg, &dst);

        int above0_5 = 0;
        int sum = 0;
        float lidard = dst.current_distance * 0.01f;
        float err = fabsf(lidard - _lidarprev);
        _lidarprev = lidard;

        if (err > 0.6f)
        {
            lidard = _lidarprev;
            /* code */
        }

        if (lidard < 0.5f)
        {
            lidard = 0.5f;
            /* code */
        }else if(lidard > 3.0f){

            lidard = 3.0f;
        }

        _vertd = fabsf(lidard) * cos(_theta) * cos(_phi);

        //std::cout << "The vertd is " << _vertd << std::endl; 

        if(_vertd < 0.6f || lidard > 2.8f){

            above0_5 = 0;
        
        }else{
        
            above0_5 = 1;
        
        }

        //I dont think visrecv is needed here
        if(_offb){
            if(_landBuffer.size() < 12){
            
                _landBuffer.push_back(above0_5);
        
            }else{

                _landBuffer.erase(_landBuffer.begin());
                _landBuffer.push_back(above0_5);

                sum = accumulate(_landBuffer.begin(), _landBuffer.end(), 0);

                if(sum < 4){

                    _within0_5 = true;

                }else{

                    _within0_5 = false;

                }
            }

           // std::cout << "The sum is " << sum << std::endl; 
        }else {

           _landBuffer.clear();
           _within0_5 = false;
        }

    }
void serialboost::SerialPort::handle_message_lpos_ned(mavlink_message_t *msg)
	{
		mavlink_local_position_ned_t lpos;
		mavlink_msg_local_position_ned_decode(msg, &lpos);
		_zned = lpos.z;
		_vzned = lpos.vz;
        uint32_t millilapsed = 0;

        //LPOSZ STATE MACHINE
        if(!_offb){

            _zsp = _zned;
            _initz = false;

        }else if(_offb && !_initz){

            _initz = true;

        }else{

            _initz = true;
        }

        //std::cout << lpos.z << std::endl;

		//---Simple P-PID Controller----//
 
		float ez = _zsp - _zned;
		float evz = 0.0 - _vzned;
		
        if(!_stopmotors){

	    _uz = -2.0*ez - 2.0*evz + 9.81;
        
        }
        //_uz = 9.81 - 1.5*(0.2 - _vzned);
        //_uz = 9.81 - 1.0*tanh(4*(evz + ez));
        
       
        // remove visrecv from the conditions
        //if (_offb && !_timeinit && !_stopmotors)
        if (_offb && !_timeinit)
        {
            _offbrecv = boost::posix_time::microsec_clock::local_time();
            _timeinit = true;
        
        }else if(_offb && _visrecv && !_stopmotors){
            
            _offbrecvcurr = boost::posix_time::microsec_clock::local_time();
            boost::posix_time::time_duration timelapsed = _offbrecvcurr - _offbrecv;
            millilapsed = timelapsed.total_milliseconds();
            
	    //std::cout << millilapsed << std::endl;
            if (unsigned(millilapsed) < 2000)
            {
                _uz = -2.0*ez - 2.0*evz + 9.81;
            
            }else if(unsigned(millilapsed) > 2000){
                
                _uz = 9.81 - 1.5*(0.2 - _vzned);

               if(unsigned(millilapsed) > 3000 && _within0_5){
                    //STOP MOTORS HERE
                    _stopmotors = true;
                   // _uz = 9.81 - 1.5*(0.0 - _vzned);
                    _uz = 0.0;
               }

            }
        }

        // }else if(_offb && !_visrecv && _within0_5){

        //     _stopmotors = true;
        //     _uz = 0;
        //     //_uz = 9.81 - 1.5*(0.0 - _vzned);



        // }else if(_offb && !_visrecv && !_stopmotors){

        //     _uz = -2.0*ez - 2.0*evz + 9.81;


        // }
        else if(!_offb){

            _timeinit = false;
            _stopmotors = false;
            millilapsed = 0;
        }else{}

        
        _uzscale = (0.62 * _uz)/9.81;


        zmq::message_t message(100);
        float xn = lpos.x;
        float vxn = lpos.vx;
        float yn = lpos.y;
        float vyn = lpos.vy;
        _xned = xn;
        _yned = yn;
        _xvned = vxn;
        _yvned = vyn;
        
        if(_offb && _visrecv){
            
            //boost::shared_lock<boost::shared_mutex> lock(_sendMpcMutex);

            if(_cnt == 30){
		        std::cout << lpos.x << " " << lpos.y << std::endl;
                std::cout << "Publishing data" << " " << vyn << std::endl;
                _cnt = 0;
            }
            _cnt++;
            snprintf((char *)message.data(), 100, "%0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f", xn, vxn, yn, vyn, _xobjned, _yobjned, _xdf, _ydf, _vertd);
            _publisher.send(message);
        }   

	}

void serialboost::SerialPort::handle_message_mission_item(mavlink_message_t *msg)
    {
        mavlink_mission_item_t mission;
        mavlink_msg_mission_item_decode(msg, &mission);

        std::cout << unsigned(mission.seq) << " "<< mission.x << " "<< mission.y << std::endl;
    }

void serialboost::SerialPort::handle_message_mission_count(mavlink_message_t *msg)
    {
        mavlink_mission_count_t missioncount;
        mavlink_msg_mission_count_decode(msg, &missioncount);

        //std::cout << unsigned(missioncount.count) << std::endl;
    }

void serialboost::SerialPort::handle_message_mission_item_reached(mavlink_message_t *msg)
    {
        mavlink_mission_item_reached_t missionreached;
        mavlink_msg_mission_item_reached_decode(msg, &missionreached);

        //std::cout << unsigned(missionreached.seq) << std::endl;
    }
void serialboost::SerialPort::handle_message_heartbeat(mavlink_message_t *msg)
 	{
		
	 		
		mavlink_heartbeat_t hb;
 		mavlink_msg_heartbeat_decode(msg, &hb);

 		_current1 = boost::posix_time::microsec_clock::local_time();
        boost::posix_time::time_duration timelapsed = _current1 - _previous;
        uint32_t millis = timelapsed.total_milliseconds();

        if (unsigned(hb.custom_mode) == 393216)
        {
            _offb = true;
            //std::cout << "offboard" << std::endl;
        }else{
            _offb = false;
        }

 		std::cout << "Received Heartbeat Message" << " " << millis << " " << unsigned(hb.base_mode) << unsigned(hb.custom_mode) << " " << unsigned(_offb) << std::endl;
 		_previous = _current1;
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
        //uint16_t typemask = 0b0010110111111000;
		uint16_t typemask = 0b0000110000111111;
        
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

void serialboost::SerialPort::handle_message_attitude(mavlink_message_t *msg)
 	{
		
		mavlink_attitude_t at;
		mavlink_msg_attitude_decode(msg, &at);

        _phi = at.roll;
        _theta = at.pitch;
        _psi = at.yaw;

        //std::cout << "Roll is " << at.roll << std::endl;
 	}


void serialboost::SerialPort::testfunc()
{
	//boost::posix_time::seconds worktime(3);	
	//while(1){

	_subscriber.connect("tcp://127.0.0.1:5556");
	_subscriber.setsockopt( ZMQ_SUBSCRIBE, "", 0);
    //uint32_t millis = 0;
	
	//std::cout << "priinting some" << std::endl;	

	try{
		while(true){
	//for(int up_nbr = 0; up_nbr < 100; up_nbr++){		
		
		zmq::message_t update;

		_subscriber.recv(&update);

		float uxs, uys;
		std::istringstream iss(static_cast<char *>(update.data()));
		iss >> uxs >> uys;

        //_current = boost::posix_time::microsec_clock::local_time();
        //boost::posix_time::time_duration timelapsed = _current - _started;
        //millis = timelapsed.total_milliseconds();

        if(_cntsub == 30){
            std::cout << "Publishing ctrl " << uxs << std::endl;
            _cntsub = 0;
        }

        _cntsub++;

        _uxscale = uxs;
        _uyscale = uys;

        
        //WriteToPixhawkOffboardSetpoint(millis, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ux, 0.0, _uz, 0.0, 0.0);
		//WriteToPixhawk();

			}
		}
	 catch (std::exception &e) {

	 	std::cout << "Error occurred " << std::endl;
	 }		
	
}

void serialboost::SerialPort::receivevision()
{
    _subscribevis.connect("tcp://127.0.0.1:6000");
    _subscribevis.setsockopt(ZMQ_SUBSCRIBE, "", 0);

    //std::cout << "gettin here " << std::endl;

    try{
        while(true){

            boost::lock_guard<boost::shared_mutex> lock(_sendMpcMutex);

            //std::cout << "gettin here loop" << std::endl;

            zmq::message_t update;
            _subscribevis.recv(&update);

            //_visionupdate = boost::posix_time::microsec_clock::local_time();

            _visrecv = true;

            float ucam, vcam;
            float xnorm, ynorm, r2;
            float xnew, ynew;
            float beta;
            float xvirtual, yvirtual;
            float alpha = 2/0.016;
            float k = 0.5*0.5;
            std::istringstream iss(static_cast<char *>(update.data()));
            iss >> ucam >> vcam;

            //std::cout << "u,v are " << ucam << "," << vcam << std::endl;
            //-------------Convert from pixels to meters------------//
            r2 = pow((ucam-u0)/principalx, 2) + pow((vcam-v0)/principaly, 2);

            //std::cout << "r2 is " << r2 <<" " << u0 << " " << v0 << " " << kdu << " " << principalx << " " << principaly << std::endl;

            xnorm = (ucam - u0)*(1.0 + kdu*r2)/principalx;
            ynorm = (vcam - v0)*(1.0 + kdu*r2)/principaly;

            xnew = -ynorm;
            ynew = xnorm;

            // theta is pitch, phi is roll
            beta = 1.0f / (sin(_theta)*xnew + cos(_theta)*sin(_phi)*ynew + cos(_theta)*cos(_phi));

            xvirtual = beta * (cos(_theta) * xnew + sin(_theta) * sin(_phi) * ynew + sin(_theta) * cos(_phi));
            yvirtual = beta * (cos(_phi) * ynew - sin(_phi));

            xvirtual = _vertd * xvirtual;
            yvirtual = _vertd * yvirtual;

            _xobjned = _xned + cos(_psi) * xvirtual - sin(_psi) * yvirtual;
            _yobjned = _yned + sin(_psi) * xvirtual + cos(_psi) * yvirtual;
            _xd = _xobjned;
            _yd = _yobjned;

            _xdf = (1.0f/(alpha*alpha*k + alpha + 1.0))*(alpha*_xd - alpha*_xd2 - _xdf1*(2.0 - 2.0*alpha*alpha*k) - _xdf2*(alpha*alpha*k - alpha + 1.0));
            _xd2 = _xd1;
            _xd1 = _xd;
            _xdf2 = _xdf1;
            _xdf1 = _xdf;

            _ydf = (1.0f/(alpha*alpha*k + alpha + 1.0))*(alpha*_yd - alpha*_yd2 - _ydf1*(2.0 - 2.0*alpha*alpha*k) - _ydf2*(alpha*alpha*k - alpha + 1.0));
            _yd2 = _yd1;
            _yd1 = _yd;
            _ydf2 = _ydf1;
            _ydf1 = _ydf;

            //------------------------------------------//
            if (_xdf > 2.0f)
            {
                _xdf = 2.0f;
            
            }else if (_xdf < -2.0f){

                _xdf = -2.0f;
            }else{}

            if (_ydf > 2.0f)
            {
                _ydf = 2.0f;
            
            }else if (_ydf < -2.0f){

                _ydf = -2.0f;
            }else{}

            //_xobjvelprev = _xobjvel;
            //_yobjvelprev = _yobjvel;
            //std::cout << "x,y are " << _xobjned << "," << _yobjned << std::endl;


        }

    }
    catch (std::exception &e) {

        std::cout << "Error occurred" << std::endl;
    }
}

void serialboost::SerialPort::sendoffboardcommands()
{
    int count = 0;
    uint32_t millis = 0;
    uint32_t millistimeout = 0;
    //float vx = 0.0;
    //float vy = 0.0;
    //float vz = 0.0;
    bool flag = false;

    try{
        while(true){

            usleep(10000);

            _current = boost::posix_time::microsec_clock::local_time();
            boost::posix_time::time_duration timelapsed = _current - _started;
            millis = timelapsed.total_milliseconds();

            // boost::posix_time::time_duration timeout = _current - _visionupdate;
            // millistimeout = timeout.total_milliseconds();

            
   //          if(_offb){
   //              if (unsigned(millistimeout) > 100)
   //                  {
   //                      _visrecv = false;
			// //std::cout << "object NOT in frame" << std::endl;
            
   //                  }else{

   //                      _visrecv = true;
			// //std::cout << "object in frame" << std::endl;

   //                  }

   //              if (!_visrecv && !_within0_5)
   //                  {
   //                      float ex = _xobjned - _xned;
   //                      float ey = _yobjned - _yned;

   //                      if(fabsf(ex) < 0.1 && fabsf(ey) < 0.1){

   //                              _visrecv = true;

   //                      }else {

   //                              float ang = atan2(ey, ex);
   //                              _uxscale = 0.06 * cos(ang);
   //                              _uyscale = 0.06 * sin(ang);

   //                              if (_uxscale > 0.06f){

   //                                  _uxscale = 0.06f;
        
   //                              }else if(_uxscale < -0.06f){

   //                                  _uxscale = -0.06f;

   //                              }else{}

   //                              if (_uyscale > 0.06f){

   //                                  _uyscale = 0.06f;
        
   //                              }else if(_uyscale < -0.06f){

   //                                  _uyscale = -0.06f;

   //                              }else{}
   //                      }

   //                  }
   //          }

            if (_offb && !flag)
            {
                count++;
            }

            if(_stopmotors){

            _uxscale = 0.0;
            _uyscale = 0.0;
            _uzscale = 0.0;

            }

            //std::cout << "getting" << std::endl;

            if(_offb){

                if (_cntlog == 5)
                {
                    std::ostream out(&_buflog);
                    out << millis << "," << _xobjned << "," << _yobjned << "," << _xdf << "," << _ydf << "," << _xned << "," <<  _yned << "," << _xvned << "," << _yvned << "," << _uxscale << "," << _uyscale << "\n";
                    _cntlog = 0;

                } 
                _cntlog++;
            }


            if(_offb && count < 400 && !flag){
                
                WriteToPixhawkOffboardSetpoint(millis, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, _uxscale, _uyscale, -_uzscale, 0.0, 0.0);
            }else if(_offb && count > 400){
                WriteToPixhawkOffboardSetpoint(millis, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, _uxscale, _uyscale, -_uzscale, 0.0, 0.0);
                flag = true;

            }else{
               WriteToPixhawkOffboardSetpoint(millis, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, -_uzscale, 0.0, 0.0); 
            }
            // if(count == 200){
            //     std::cout <<  millis << std::endl;
            //     }
            }
        }
    catch (std::exception &e){
        std::cout << "Error occured " << std::endl;
    }
}

