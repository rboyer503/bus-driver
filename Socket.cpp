/*
 * Socket.cpp
 *
 *  Created on: Jan 13, 2015
 *      Author: rboyer
 */

#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include "Socket.h"
#include "SocketMgr.h"

using namespace std;


Socket::Socket(SocketMgr * pSocketMgr, int port) :
	m_pSocketMgr(pSocketMgr), m_port(port),
	m_listenfd(-1), m_connfd(-1),
	m_accepting(false)
{
}

Socket::~Socket()
{
	// Close active connection if present.
	Close();

	// Close listen socket if open.
	if (m_listenfd != -1)
	{
		close(m_listenfd);
		m_listenfd = -1;
	}

	//cout << "Socket destroyed." << endl;
}

bool Socket::EstablishListener()
{
	// Establish listening socket, bound to specified port.
	if ( (m_listenfd = socket(AF_INET, SOCK_STREAM, 0)) < 0 )
	{
		cerr << "Error: Could not create socket." << endl;
		return false;
	}

	struct sockaddr_in serverAddr;
	serverAddr.sin_family = AF_INET;
	serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	serverAddr.sin_port = htons(m_port);

	if (bind(m_listenfd, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0)
	{
		cerr << "Error: Could not bind to port " << m_port << "." << endl;
		return false;
	}

	if (listen(m_listenfd, 1) < 0)
	{
		cerr << "Error: Listen failure." << endl;
		return false;
	}

	//cout << "Socket initialized - listening on port " << m_port << "." << endl;
	return true;
}

bool Socket::AcceptConnection()
{
	if (m_accepting || IsConnected())
		return false;

	m_accepting = true;
	m_thread = boost::thread(&Socket::AcceptWorker, this);
	return true;
}

bool Socket::TransmitSizedMessage(unsigned char * pRawData, int size)
{
	if (!IsConnected())
		return false;

	// Send length of frame to client.
	if (send(m_connfd, &size, sizeof(size), MSG_NOSIGNAL) < 0)
	{
		//cerr << "Error: Frame length send failed." << endl;
		cout << "Remote client disconnected." << endl;
		return false;
	}

	// Now send raw frame data.
	while (size)
	{
		int bytesSent = send(m_connfd, pRawData, size, MSG_NOSIGNAL);
		if (bytesSent <= 0)
		{
			//cerr << "Error: Frame send failed with error code " << bytesSent << "." << endl;
			cout << "Remote client disconnected (error code=" << bytesSent << ")." << endl;
			break;
		}

		pRawData += bytesSent;
		size -= bytesSent;
	}

	return (size == 0);
}

char * Socket::ReceiveCommand()
{
	int recvSize = recv(m_connfd, m_recvBuffer, sizeof(m_recvBuffer)-1, 0);
	if (recvSize <= 0)
		return NULL;

	m_recvBuffer[recvSize] = 0;
	return m_recvBuffer;
}

bool Socket::Shutdown()
{
	bool forceShutdown = false;

	// Check if accept worker is currently active.
	{
		boost::mutex::scoped_lock lock(m_pSocketMgr->m_acceptMutex);
		if (m_accepting)
		{
			cout << "Socket cancelling accept..." << endl;

			// Shutdown listen socket to cancel accept.
			m_accepting = false;
			forceShutdown = true;
			shutdown(m_listenfd, SHUT_RDWR);
		}
	}

	// Close active connection if present.
	if (m_connfd != -1)
	{
		shutdown(m_connfd, SHUT_RDWR);
	}

	// If this is a forced shutdown, wait for worker thread to finish.
	if (forceShutdown)
	{
		m_thread.join();
		close(m_listenfd);
		m_listenfd = -1;
		return false;
	}

	return true;
}

void Socket::Close()
{
	if (m_connfd != -1)
	{
		close(m_connfd);
		m_connfd = -1;
	}
}

void Socket::AcceptWorker()
{
	// Block until connection is established.
	m_connfd = accept(m_listenfd, (struct sockaddr *)NULL, NULL);

	// Update flags and notify main thread.
	{
		boost::mutex::scoped_lock lock(m_pSocketMgr->m_acceptMutex);
		m_accepting = false;
		m_pSocketMgr->m_condition.notify_one();
	}

	cout << "Socket connection accept thread exited." << endl;
}
