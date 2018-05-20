#include "zmq\zmqPub.h"



zmqPub::zmqPub()
{
	m_context = new zmq::context_t(1);
	m_socket = new zmq::socket_t(*m_context, ZMQ_PUB);
}


zmqPub::~zmqPub()
{
}

zmqPub* zmqPub::getInstance()
{
	if (m_instance == NULL)
		m_instance = new zmqPub;
	return m_instance;
}

bool zmqPub::s_sendmore(std::string string)
{
	zmq::message_t message(string.size());
	memcpy(message.data(), string.data(), string.size());

	bool rc = m_socket->send(message, ZMQ_SNDMORE);
	return (rc);
}

bool zmqPub::s_send(std::string string)
{
	zmq::message_t message(string.size());
	memcpy(message.data(), string.data(), string.size());

	bool rc = m_socket->send(message);
	return (rc);
}

bool zmqPub::s_send(const float & f)
{
	zmq::message_t message(4);
	memcpy(message.data(), &f, 4);

	bool rc = m_socket->send(message);
	return (rc);
}

bool zmqPub::s_send(float f[], int len)
{
	zmq::message_t message(len * 4);
	memcpy(message.data(), &f[0], len * 4);

	bool rc = m_socket->send(message);

// 	zmq::message_t reply;
// 	m_socket->recv(&reply);

	return (rc);
}

void zmqPub::connect()
{
	m_socket->bind("tcp://*:5563");
	std::cout << "connecting\n";
}

void zmqPub::send(std::string topic, float msg)
{
	s_sendmore(topic);
	s_send(msg);
}

void zmqPub::send(std::string topic, std::vector<Eigen::Affine3f> matrixs)
{
	s_sendmore(topic);
	int len = atoi(topic.substr(1).c_str());
	//if (topic.substr(1) == "64") {
		float* fmtx = new float[len];
		for (int i = 0; i < matrixs.size(); i++) {
			memcpy(&fmtx[i * 16], matrixs[i].data(), 4 * 16);
		}
		s_send(fmtx, len);
		delete[] fmtx;
		fmtx = NULL;
	//}
}

void zmqPub::send(std::string topic)
{
	s_sendmore(topic);
	if (topic.substr(1) == "01") {
		float cmd[1];
		cmd[0] = 1.0f;
		s_send(cmd, 1);
		std::cout << "sending" << topic << "\n";
	}
}


void zmqPub::send(std::string topic, float msg[])
{
	s_sendmore(topic);
	s_send(msg, 4);
}

zmqPub* zmqPub::m_instance = NULL;
