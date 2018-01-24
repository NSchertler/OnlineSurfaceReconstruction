#include "zmq\zmqClient.h"



zmqClient::zmqClient()
{
	//  Prepare our context and socket
	m_context = new zmq::context_t(1);
	m_socket = new zmq::socket_t(*m_context, ZMQ_REQ);
}


zmqClient::~zmqClient()
{
	delete m_context;
	delete m_socket;
}

zmqClient* zmqClient::getInstance()
{
	if (m_instance == NULL)
		m_instance = new zmqClient;
	return m_instance;
}

void zmqClient::connect()
{
	std::cout << "Connecting to hello world server" << std::endl;
	m_socket->connect("tcp://localhost:5555"); 
}


void zmqClient::send(std::vector<Eigen::Affine3f> m)
{
	zmq::message_t msg(sizeof(float) * 16 * m.size());
	float* f = new float(16 * m.size());
	float* fp = f;
	for (int i = 0; i < m.size(); i++) {
		memcpy(fp, m[i].data(), sizeof(float) * 16);
		fp = fp + 16;
	}
	memcpy(msg.data(), f, sizeof(float) * 16 * m.size());
	m_socket->send(msg);

	//  Get the reply.
	//zmq::message_t reply;
	//m_socket->recv(&reply);
	//std::cout << "Received" << std::endl;
	//delete f;
	//f = NULL;
}

void zmqClient::send()
{
	zmq::message_t msg(5);
	memcpy(msg.data(),"hello", 5);
	m_socket->send(msg);

	zmq::message_t reply;
	m_socket->recv(&reply);
	std::cout << "Received" << std::endl;
}

zmqClient* zmqClient::m_instance=NULL;
