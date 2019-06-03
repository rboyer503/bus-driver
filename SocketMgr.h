/*
 * SocketMgr.h
 *
 *  Created on: Dec 22, 2014
 *      Author: rboyer
 */

#ifndef SOCKETMGR_H_
#define SOCKETMGR_H_

#include "Socket.h"

#define SM_MONITOR_PORT 5000
#define SM_COMMAND_PORT 5001


class BusMgr;

class SocketMgr
{
	friend class Socket;

	BusMgr * m_owner;

	Socket * m_pSocketMon;
	Socket * m_pSocketCmd;

	boost::mutex m_acceptMutex;
	boost::condition_variable m_condition;

	bool m_connected;
	bool m_reading;

	boost::thread m_thread;

public:
	SocketMgr(BusMgr * owner);
	~SocketMgr();

	bool IsConnected() const { return m_connected; }

	bool Initialize();
	bool WaitForConnection();
	bool StartReadingCommands();
	bool ReleaseConnection();
	void Close();

	bool SendFrame(unsigned char * pRawData, int size);

private:
	void ReadCommandsWorker();

};

#endif /* SOCKETMGR_H_ */
