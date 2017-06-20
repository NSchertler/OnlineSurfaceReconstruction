#include "DavidTurntableConnection.h"

#include <iostream>
#include <thread>

#include <boost/bind.hpp>

using namespace osr;
using namespace osr::gui;
using namespace osr::gui::loaders;

DavidTurntableConnection::DavidTurntableConnection()
	: turntablePort(nullptr), _status(Invalid)
{
}

DavidTurntableConnection::~DavidTurntableConnection()
{
	if (turntablePort)
	{
		turntablePort->close();
		delete turntablePort;
	}
}

bool DavidTurntableConnection::openConnection()
{
	boost::system::error_code ec;

	turntablePort = new boost::asio::serial_port(io);
	turntablePort->open("COM3", ec); //TODO: Generalize

	if (ec)
	{
		std::cout << "Error opening COM port to turntable: " << ec.message() << std::endl;
		return false;
	}

	_status = Stopped;

	turntablePort->set_option(boost::asio::serial_port::baud_rate(9600));
	turntablePort->set_option(boost::asio::serial_port_base::character_size(8));
	turntablePort->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
	turntablePort->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
	turntablePort->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

	stepsPerDegree = 0;

	std::string message = "steps_per_degree\n";
	turntablePort->write_some(boost::asio::buffer(message.c_str(), message.size()));

	boost::asio::async_read_until(*turntablePort, response, '\n', boost::bind(&DavidTurntableConnection::handleResponse, this, boost::placeholders::_1, boost::placeholders::_2));

	std::this_thread::sleep_for(std::chrono::milliseconds(200)); //wait for incoming messages

	while (io.poll())
		std::this_thread::sleep_for(std::chrono::milliseconds(200));

	return stepsPerDegree > 0;
}

double DavidTurntableConnection::move(double targetDegrees)
{
	if (_status == Invalid)
		return 0;

	if (targetDegrees == 0)
		return 0;

	while (targetDegrees < 180)
		targetDegrees += 360;
	while (targetDegrees > 180)
		targetDegrees -= 360;

	_status = Running;

	int steps = (int)std::round(targetDegrees * stepsPerDegree);

	std::string message = "move " + std::to_string(steps) + "\n";
	turntablePort->write_some(boost::asio::buffer(message.c_str(), message.size()));
	while (status() == Running)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
		while (io.poll())
			std::this_thread::sleep_for(std::chrono::milliseconds(200));
	}

	return steps / stepsPerDegree;
}

void DavidTurntableConnection::handleResponse(const boost::system::error_code & error, std::size_t bytes_transferred)
{
	std::istream responseStream(&response);
	std::string category;
	while (responseStream >> category)
	{
		if (category == "steps_per_degree")
			responseStream >> stepsPerDegree;
		else if (category == "status")
		{
			std::string statusName;
			responseStream >> statusName;
			if (statusName == "running")
				_status = Running;
			else if (statusName == "stopped")
				_status = Stopped;
		}
	}

	boost::asio::async_read_until(*turntablePort, response, '\n', boost::bind(&DavidTurntableConnection::handleResponse, this, boost::placeholders::_1, boost::placeholders::_2));
}
