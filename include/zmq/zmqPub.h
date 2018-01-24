#ifndef _SINGLETON_PUB_H
#define _SINGLETON_PUB_H


#include "zmq.hpp"
#ifndef _WIN32
#include <unistd.h>
#else
#include <windows.h>
#define sleep(n)    Sleep(n)
#endif

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <iostream>

class zmqPub
{
	private:
	zmqPub();
	~zmqPub();

public:
	static zmqPub* getInstance();

	void connect();

	void send(std::string topic, float msg);

	void send(std::string topic, std::vector<Eigen::Affine3f> matrixs);

	void send(std::string topic);	// the trigger for unity to read ply

	void send(std::string topic, float msg[]);

private:
	//  Sends string as 0MQ string, as multipart non-terminal
	bool s_sendmore(std::string string);

	//  Convert string to 0MQ string and send to socket
	bool s_send(std::string string);

	bool s_send(const float & f);

	bool s_send(float f[], int len);

private:
	static zmqPub* m_instance;
	zmq::socket_t* m_socket;
	zmq::context_t* m_context;
};

#endif