/*
 * SocketMgr.h
 *
 *  Created on: Dec 22, 2014
 *      Author: rboyer
 */

#ifndef SOCKETMGR_H_
#define SOCKETMGR_H_

#include <memory>
#include <vector>

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
	boost::mutex m_monitorMutex;

	bool m_connected;
	bool m_reading;
	bool m_monitoring;

	boost::thread m_thread;
	boost::thread m_monitorThread;

	std::unique_ptr<std::vector<unsigned char> > m_pCurrBuffer;
	int m_droppedFrames = 0;

public:
	SocketMgr(BusMgr * owner);
	~SocketMgr();

	bool IsConnected() const { return m_connected; }
	int GetDroppedFrames() const { return m_droppedFrames; }

	bool Initialize();
	bool WaitForConnection();
	void StartReadingCommands();
	void StartMonitorThread();
	bool ReleaseConnection();
	void Close();

	bool SendFrame(std::unique_ptr<std::vector<unsigned char> > pBuf);

private:
	void ReadCommandsWorker();
	void MonitorWorker();

};

#endif /* SOCKETMGR_H_ */
