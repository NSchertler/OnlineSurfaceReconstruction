#include "zmq\zmqPub.h"



zmqPub::zmqPub()
{
	m_context = new zmq::context_t(1);
	m_socket = new zmq::socket_t(*m_context, ZMQ_PUB);
	isConnect = false;
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

bool zmqPub::s_send(byte b[], int len)
{
	zmq::message_t message(len);
	memcpy(message.data(), &b, len);

	bool rc = m_socket->send(message);
	return (rc);
}

void zmqPub::connect()
{
	if (!isConnect) {
		isConnect = true;
		m_socket->bind("tcp://*:5563");
		std::cout << "connecting\n";
	}	
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

void zmqPub::send(std::string topic, byte* msg, int len)
{
	s_sendmore(topic);
	s_send(msg, len);
}

void zmqPub::send(std::string topic, const std::string& path)
{
	s_sendmore(topic + std::to_string(path.size()));
	s_send(std::string(path));
}

zmqPub* zmqPub::m_instance = NULL;
