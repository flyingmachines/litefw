#include <azmq/socket.hpp>
#include <boost/asio.hpp>
#include <array>
#include <iostream>
#include <thread>
#include <chrono>


namespace asio = boost::asio;

int main(int argc, char** argv){

asio::io_service ios;
azmq::pub_socket publisher(ios);

publisher.bind("tcp://192.168.1.8:5563");

std::array<char, 4> buf = {'y','o','l',0};
for (;;){
	std::cout << "Sending" << std::endl;
	
	publisher.send(asio::buffer(buf));
	
	std::this_thread::sleep_for(std::chrono::seconds(1));
}


}
