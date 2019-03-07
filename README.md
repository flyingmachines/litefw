# litefw
A light framework for high-level and fcu communication

Boost ASIO and ZeroMQ libraries are utilized to create a communication framework to enable deployment on Single Board Computers including
low-power embedded boards like Raspberry Pi Zero and Intel Edison.

Reference for low-level ASIO Framework

[1] https://objectcomputing.com/resources/publications/mnb/multi-platform-serial-interfacing-using-boost-a-gps-sensor-and-opendds-part-i


ZMQ publish/subscribe is generally utilized to communicate between different modules in the framework. For e.g. - One module could be running an Object Tracking pipeline and the other module could be the serial bridge. In this example, the object tracking pipeline could be publishing the current and desired features and the serial bridge could be subscribing to the data to generate low-level actuator signals for the flight control unit (FCU).

This pipeline comes very handy as it requires minimum setup time and for its ease of use. 


Requirements : 


```
1) Install ZMQ - http://zeromq.org/intro:get-the-software

2) sudo apt-get install libzmq3-dev

3) sudo apt-get install libboost-all-dev

```
