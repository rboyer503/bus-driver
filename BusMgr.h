/*
 * BusMgr.h
 *
 *  Created on: Jan 15, 2015
 *      Author: rboyer
 */

#ifndef BUSMGR_H_
#define BUSMGR_H_

#include <algorithm>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "ControlMgr.h"
#include "LaneTransform.h"
#include "Constants.h"

#define FRAME_SKIP 2
#define FRAME_BACKLOG_MIN -5

#define STATUS_SUPPRESS_DELAY 10
#define STATUS_HISTORY_BUCKETS 10


enum eBDErrorCode
{
	EC_NONE,
	EC_LANETRANSFAIL,
	EC_LISTENFAIL,
	EC_ACCEPTFAIL,
	EC_READCMDFAIL,
	EC_CAPTUREOPENFAIL,
	EC_CAPTUREGRABFAIL,
	EC_SERIALIZEFAIL,
	EC_SENDFAIL,
	EC_RELEASEFAIL,
	EC_INTERRUPT
};

enum eBDImageProcMode
{
	IPM_NONE,
	IPM_GRAY,
	IPM_BLUR,
	IPM_LANEASSIST,
	IPM_MAX
};

enum eBDImageProcStage
{
	IPS_GRAY,
	IPS_BLUR,
	IPS_LANEASSIST,
	IPS_SENT,
	IPS_TOTAL,
	IPS_MAX
};

enum eBDParamPage
{
	PP_BLUR,
	PP_GRADIENTTHRESHOLD,
	PP_MAX
};

struct Status
{
	unsigned char suppressDelay;
	int numFrames;
	int numDroppedFrames;
	int currProcessUs[IPS_MAX];
	int totalProcessUs[IPS_MAX];
	int maxProcessUs[IPS_MAX];

public:
	Status() : suppressDelay(STATUS_SUPPRESS_DELAY), numFrames(0), numDroppedFrames(0)
	{
		for (int i = 0; i < IPS_MAX; ++i)
		{
			currProcessUs[i] = totalProcessUs[i] = maxProcessUs[i] = 0;
		}
	}

	bool IsSuppressed() const { return suppressDelay; }

	bool SuppressionProcessing()
	{
		// Return true only first time supression delay expires.
		if (suppressDelay)
		{
			--suppressDelay;
			return (!suppressDelay);
		}
		return false;
	}

};

struct Config
{
	unsigned char kernelSize;
	unsigned char gradientThreshold;
	Config() :
		kernelSize(5), gradientThreshold(20)
	{}
};


class SocketMgr;


class BusMgr
{
	static const char * const c_imageProcModeNames[];
	static const char * const c_imageProcStageNames[];
	static const cv::Vec2i c_defaultRange[MAX_LANES];

	eBDErrorCode m_errorCode;
	eBDImageProcMode m_ipm;
	SocketMgr * m_pSocketMgr;
	Status m_status;
	Config m_config;
	eBDParamPage m_paramPage;
	boost::thread m_thread;
	volatile bool m_running;
	bool m_interrupted;
	boost::posix_time::ptime m_startTime;
	boost::posix_time::time_duration m_diff;
	ControlMgr * m_pCtrlMgr;
	LaneTransform * m_pLaneTransform;
	LaneInfo m_lockedLanes[MAX_LANES];
	cv::Vec2i m_searchRange[MAX_LANES];
	bool m_debugTrigger;
	boost::mutex m_accelMutex;
	float m_acceleration;
	float m_speed;

public:
	BusMgr();
	~BusMgr();

	eBDErrorCode GetErrorCode() const { return m_errorCode; }
	bool IsRunning() const { return m_running; }
	bool IsInterrupted() const { return m_interrupted; }
	void SetInterrupted() { m_interrupted = true; }

	bool Initialize();
	void Terminate();

	void UpdateIPM();
	void OutputStatus();
	void OutputConfig();
	void UpdatePage();
	void UpdateParam(int param, bool up);
	void DebugCommand();

	void SetSpeed(int speed) { m_pCtrlMgr->SetSpeed(speed); }
	void SetAcceleration(float accel)
	{
		boost::mutex::scoped_lock lock(m_accelMutex);
		m_acceleration = accel;
	}
	void ApplyAcceleration();
	void SetReverse(bool reverse) { m_pCtrlMgr->SetReverse(reverse); }
	void SetServo(int servo) { m_pCtrlMgr->SetServo(servo); }
	void SetLaneAssist(bool enable) { m_pCtrlMgr->SetLaneAssist(enable); }
	bool GetLaneAssist() const { return m_pCtrlMgr->GetLaneAssist(); }

private:
	void WorkerFunc();
	bool ProcessFrame(cv::Mat & frame);
	int LaneAssistComputeServo(cv::Mat & frame);
	void DisplayCurrentParamPage();

};

#endif /* BUSMGR_H_ */
