#include "zhelpers.hpp"

int main () {
    //  Prepare our context and subscriber
    zmq::context_t context(1);
    zmq::socket_t subscriber (context, ZMQ_SUB);
    subscriber.connect("tcp://127.0.0.1:5563");
    subscriber.setsockopt( ZMQ_SUBSCRIBE, "", 0);

    while (1) {
 
		//  Read envelope with address
		//std::string address = s_recv (subscriber);
		//  Read message contents
		//std::string contents = s_recv (subscriber);
		
		zmq::message_t update;

		subscriber.recv(&update);
		
		float roll, yaw, pitch;
		std::istringstream iss(static_cast<char *>(update.data()));
		iss >> roll >> yaw >> pitch;
		//float roll = ::atof(contents.c_str());
		
		std::cout << "RPY" << roll << " " << yaw << " " << pitch << " " << std::endl;	
       // std::cout << "[" << address << "] " << roll << std::endl;
    }
    return 0;
}
