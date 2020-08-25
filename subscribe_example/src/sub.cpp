#include "zhelpers.hpp"

int main () {
    //  Prepare our context and subscriber
    zmq::context_t context(1);
    zmq::socket_t subscriber (context, ZMQ_SUB);
    subscriber.connect("tcp://127.0.0.1:6000");
    subscriber.setsockopt( ZMQ_SUBSCRIBE, "", 0);

    while (1) {
 
		//  Read envelope with address
		//std::string address = s_recv (subscriber);
		//  Read message contents
		//std::string contents = s_recv (subscriber);
		
		zmq::message_t update;

		subscriber.recv(&update);
		
		float bb_center_x, bb_center_y;
		std::istringstream iss(static_cast<char *>(update.data()));
		iss >> bb_center_x >> bb_center_y;
		//float roll = ::atof(contents.c_str());
		
		std::cout << "Center X: " << bb_center_x << ", " << "Center Y: " << bb_center_y << std::endl;	
       // std::cout << "[" << address << "] " << roll << std::endl;
    }
    return 0;
}
