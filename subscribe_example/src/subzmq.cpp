#include <azmq/socket.hpp>
#include <boost/asio.hpp>
#include <array>
#include <iostream>
#include <thread>
#include <chrono>


namespace asio = boost::asio;

int main(int argc, char** argv){

asio::io_service ios;
azmq::sub_socket subscriber(ios);

subscriber.connect("tcp://192.168.1.8:5563");
subscriber.set_option(azmq::socket::subscribe("NASDAQ"));

std::array<char, 4> buf;
for (;;){
	//std::cout << "Sending" << std::endl;
	
	auto size = subscriber.receive(asio::buffer(buf));
	
	std::cout << buf.at(1) << std::endl;	
	//std::this_thread::sleep_for(std::chrono::seconds(1));
}


}
