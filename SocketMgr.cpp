/*
 * SocketMgr.cpp
 *
 *  Created on: Dec 22, 2014
 *      Author: rboyer
 */

#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstdlib>
#include "SocketMgr.h"

#include "BusMgr.h"

using namespace std;


SocketMgr::SocketMgr(BusMgr * owner) :
	m_owner(owner), m_pSocketMon(NULL), m_pSocketCmd(NULL),
	m_connected(false), m_reading(false), m_monitoring(false)
{
}

SocketMgr::~SocketMgr()
{
	Close();

	if (m_thread.joinable())
		m_thread.join();

	if (m_monitorThread.joinable())
		m_monitorThread.join();

	cout << "Socket manager destroyed." << endl;
}

bool SocketMgr::Initialize()
{
	m_pSocketMon = new Socket(this, SM_MONITOR_PORT);
	if (!m_pSocketMon->EstablishListener())
	{
		delete m_pSocketMon;
		m_pSocketMon = NULL;
		return false;
	}
	cout << "Monitor socket listening..." << endl;

	m_pSocketCmd = new Socket(this, SM_COMMAND_PORT);
	if (!m_pSocketCmd->EstablishListener())
	{
		delete m_pSocketMon;
		m_pSocketMon = NULL;
		delete m_pSocketCmd;
		m_pSocketCmd = NULL;
		return false;
	}
	cout << "Command socket listening..." << endl;

	cout << "Socket manager initialized successfully." << endl;
	return true;
}

bool SocketMgr::WaitForConnection()
{
	// Start worker threads to accept a connection for each socket.
	if (!m_pSocketMon->AcceptConnection())
	{
		cerr << "Error: Monitor socket already connected." << endl;
		return false;
	}
	if (!m_pSocketCmd->AcceptConnection())
	{
		m_pSocketMon->Close();
		cerr << "Error: Command socket already connected." << endl;
		return false;
	}

	// Block until both sockets have complete connection accept processing.
	{
		boost::mutex::scoped_lock lock(m_acceptMutex);
		try
		{
			while (m_pSocketMon->IsAccepting() || m_pSocketCmd->IsAccepting())
				m_condition.wait(lock);
		}
		catch (boost::thread_interrupted&)
		{
			cout << "Interrupted while waiting for connection - shutting down server..." << endl;
			m_owner->SetInterrupted();
		}
	}

	if (!m_pSocketMon->IsConnected() || !m_pSocketCmd->IsConnected() || m_owner->IsInterrupted())
	{
		// At least one socket accept failed.
		// Close both to get to a known state.
		m_pSocketMon->Close();
		m_pSocketCmd->Close();
		return false;
	}

	cout << "Socket manager accepted new connection." << endl;
	m_connected = true;
	m_droppedFrames = 0;
	return true;
}

void SocketMgr::StartReadingCommands()
{
	m_reading = true;
	m_thread = boost::thread(&SocketMgr::ReadCommandsWorker, this);
}

void SocketMgr::StartMonitorThread()
{
	m_monitoring = true;
	m_monitorThread = boost::thread(&SocketMgr::MonitorWorker, this);
}

bool SocketMgr::ReleaseConnection()
{
	bool ret = true;

	// Shut down sockets to give opportunity for command reading thread to exit.
	if (m_pSocketMon)
		ret = ret && m_pSocketMon->Shutdown();

	if (m_pSocketCmd)
		ret = ret && m_pSocketCmd->Shutdown();

	if (m_thread.joinable())
		m_thread.join();

	if (m_monitorThread.joinable())
		m_monitorThread.join();

	// Close actual sockets.
	if (m_pSocketMon)
		m_pSocketMon->Close();

	if (m_pSocketCmd)
		m_pSocketCmd->Close();

	cout << "Socket manager released connection." << endl;
	return ret;
}

void SocketMgr::Close()
{
	// Destroy sockets in preparation to program termination.
	delete m_pSocketMon;
	m_pSocketMon = NULL;
	cout << "Monitor socket released..." << endl;

	delete m_pSocketCmd;
	m_pSocketCmd = NULL;
	cout << "Command socket released..." << endl;
}

bool SocketMgr::SendFrame(unique_ptr<vector<unsigned char> > pBuf)
{
	if (!m_monitoring)
		return false;

	{
		boost::mutex::scoped_lock lock(m_monitorMutex);
		if (!m_pCurrBuffer)
			m_pCurrBuffer = std::move(pBuf);
		else
			++m_droppedFrames;
	}

	return true;
}

void SocketMgr::ReadCommandsWorker()
{
	// Wait for client commands on command socket and handle them.
	char * recvBuffer;
	while ( (recvBuffer = m_pSocketCmd->ReceiveCommand()) != NULL )
	{
		if (strcmp(recvBuffer, "mode") == 0)
			m_owner->UpdateIPM();
		else if (strcmp(recvBuffer, "status") == 0)
			m_owner->OutputStatus();
		else if (strcmp(recvBuffer, "config") == 0)
			m_owner->OutputConfig();
		else if (strcmp(recvBuffer, "page") == 0)
			m_owner->UpdatePage();
		else if (strcmp(recvBuffer, "param1 up") == 0)
			m_owner->UpdateParam(1, true);
		else if (strcmp(recvBuffer, "param1 down") == 0)
			m_owner->UpdateParam(1, false);
		else if (strcmp(recvBuffer, "param2 up") == 0)
			m_owner->UpdateParam(2, true);
		else if (strcmp(recvBuffer, "param2 down") == 0)
			m_owner->UpdateParam(2, false);
		else if (strcmp(recvBuffer, "debug") == 0)
			m_owner->DebugCommand();
		else if (strncmp(recvBuffer, "servo ", 6) == 0)
		{
			if (!m_owner->GetLaneAssist())
			{
				int value = atoi(&recvBuffer[6]);

				// Joystick sends value from -32768 to 32767 - scale to 126 to 190
				value = (value / 1024) + ControlMgr::cDefServo;

				m_owner->SetServo(value);
			}
		}
		else if (strncmp(recvBuffer, "motor ", 6) == 0)
		{
			int value = atoi(&recvBuffer[6]);

			// Joystick sends value from -32768 to 32767 - scale to 10.0 to -10.0 for acceleration value.
			m_owner->SetAcceleration(-value / 3276.8f);
		}
		else if (strcmp(recvBuffer, "laneassist on") == 0)
		{
			m_owner->SetLaneAssist(true);
		}
		else if (strcmp(recvBuffer, "laneassist off") == 0)
		{
			m_owner->SetLaneAssist(false);
		}
		else if (strcmp(recvBuffer, "shiftleft") == 0)
		{
			m_owner->SwitchLane(LEFT_LANE);
		}
		else if (strcmp(recvBuffer, "shiftright") == 0)
		{
			m_owner->SwitchLane(RIGHT_LANE);
		}
	}

	cout << "Socket manager command reader thread exited." << endl;
}

void SocketMgr::MonitorWorker()
{
	// Transmit frames to client for monitoring as they become available.
	while (true)
	{
		boost::this_thread::sleep(boost::posix_time::milliseconds(5));

		unique_ptr<vector<unsigned char> > pBuf;

		{
			boost::mutex::scoped_lock lock(m_monitorMutex);
			if (!m_pCurrBuffer)
				continue;
			else
				pBuf = std::move(m_pCurrBuffer);
		}

		// Delegate to the monitor socket.
		if (!m_pSocketMon->TransmitSizedMessage(&(*pBuf)[0], pBuf->size()))
			break;
	}

	m_monitoring = false;
	cout << "Socket manager monitor thread exited." << endl;
}
