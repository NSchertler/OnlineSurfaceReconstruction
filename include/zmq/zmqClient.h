#ifndef _ZMQ_SERVER_H
#define _ZMQ_SERVER_H

#include <zmq.hpp>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

class zmqClient
{
private:
	zmqClient();
	~zmqClient();

public:
	static zmqClient* getInstance();
	void connect();
	void send(std::vector<Eigen::Affine3f> m);

private:
	static zmqClient* m_instance;

	zmq::context_t* m_context;
	zmq::socket_t* m_socket;
};

#endif