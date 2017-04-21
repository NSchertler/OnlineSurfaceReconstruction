#pragma once

#include <boost/asio.hpp>

class DavidTurntableConnection
{
public:
	enum Status
	{
		Invalid,
		Stopped,
		Running,
	};

	DavidTurntableConnection();

	~DavidTurntableConnection();

	bool openConnection();

	Status status() const { return _status; }

	double move(double targetDegrees);

private:

	void handleResponse(const boost::system::error_code& error, std::size_t bytes_transferred);


	boost::asio::io_service io;
	boost::asio::serial_port* turntablePort;

	boost::asio::streambuf response;

	double stepsPerDegree;

	Status _status;
};