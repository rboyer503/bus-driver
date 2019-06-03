/*
 * Socket.h
 *
 *  Created on: Jan 13, 2015
 *      Author: rboyer
 */

#ifndef SOCKET_H_
#define SOCKET_H_

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#define RECV_BUFFER_SIZE 100


class SocketMgr;

class Socket
{
	SocketMgr * m_pSocketMgr;
	int m_port;
	int m_listenfd;
	int m_connfd;
	volatile bool m_accepting;
	boost::thread m_thread;
	char m_recvBuffer[RECV_BUFFER_SIZE];

public:
	Socket(SocketMgr * pSocketMgr, int port);
	~Socket();

	bool IsAccepting() const { return m_accepting; }
	bool IsConnected() const { return (m_connfd >= 0); }

	bool EstablishListener();
	bool AcceptConnection();
	bool TransmitSizedMessage(unsigned char * pRawData, int size);
	char * ReceiveCommand();
	bool Shutdown();
	void Close();

private:
	void AcceptWorker();
};

#endif /* SOCKET_H_ */
