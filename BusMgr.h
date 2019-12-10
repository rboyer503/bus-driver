/*
 * BusMgr.h
 *
 *  Created on: Jan 15, 2015
 *      Author: rboyer
 */

#ifndef BUSMGR_H_
#define BUSMGR_H_

#include <algorithm>
#include <limits>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "ControlMgr.h"
#include "LaneTransform.h"

#define STATUS_SUPPRESS_DELAY 10


enum eBDErrorCode
{
	EC_NONE,
	EC_LANETRANSFAIL,
	EC_LISTENFAIL,
	EC_ACCEPTFAIL,
	EC_CAPTUREOPENFAIL,
	EC_CAPTUREGRABFAIL,
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
	IPM_DEBUG,
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
		kernelSize(5), gradientThreshold(40)
	{}
};

struct FDRecord
{
	cv::Mat frame;
	int target[MAX_LANES];
	int searchStart[MAX_LANES];
	int servo;
	int lastServo;
	int lastAngle;
};

struct LaneAssistStats
{
	int minAngleDiff;
	int maxAngleDiff;

	LaneAssistStats() :
		minAngleDiff(std::numeric_limits<int>::max()), maxAngleDiff(std::numeric_limits<int>::min())
	{}
};


class SocketMgr;


class BusMgr
{
	static const char * const c_imageProcModeNames[];
	static const char * const c_imageProcStageNames[];
	static const cv::Vec2i c_defaultRange[MAX_LANES];
	static const int c_centerX[MAX_LANES];
	static constexpr int c_maxFDRecords = 150;

	enum eEdgeSearchState
	{
		ESS_SEARCH_THRES,
		ESS_SEARCH_MAX_POS,
		ESS_SEARCH_MAX_NEG,
		ESS_MAX
	};

	eBDErrorCode m_errorCode = EC_NONE;
	SocketMgr * m_pSocketMgr;
	ControlMgr * m_pCtrlMgr;
	LaneTransform * m_pLaneTransform;
	boost::thread m_thread;
	volatile bool m_running = false;
	bool m_interrupted = false;
	eBDImageProcMode m_ipm = IPM_LANEASSIST;
	Status m_status;
	Config m_config;
	eBDParamPage m_paramPage = PP_BLUR;
	bool m_debugTrigger = false;
	float m_speed = 0.0f;
	boost::mutex m_accelMutex;
	float m_acceleration = 0.0f;
	int m_maxSpeed = 1000;
	int m_actualMaxSpeed = 1000;
	boost::mutex m_laneStateMutex;
	LaneInfo m_lockedLanes[MAX_LANES];
	cv::Vec2i m_searchRange[MAX_LANES];
	boost::posix_time::ptime m_startTime;
	boost::posix_time::time_duration m_diff;
	int m_lastServo = 0;
	cv::Mat m_frameResize;
	cv::Mat m_frameGray;
	cv::Mat m_frameROI;
	cv::Mat m_frameFilter;
	cv::Mat m_frameDouble;
	cv::Mat m_frameGradient;
	cv::Mat m_frameGradientVert;
	FDRecord m_FDRecords[c_maxFDRecords];
	int m_currFDRIndex = 0;
	bool m_FDRFull = false;
	int m_selectedFDRIndex = 0;
	bool m_updateFDR = true;
	bool m_renderLanes = true;
	LaneAssistStats m_laStats;

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
	void ToggleRenderLanes() { m_renderLanes = !m_renderLanes; }

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
	void SetAutoPilot(bool enable) { m_pCtrlMgr->SetAutoPilot(enable); }
	bool GetAutoPilot() const { return m_pCtrlMgr->GetAutoPilot(); }
	void SwitchLane(eLane toLane);

	void PrevFDR()
	{
		int maxIndex = (m_FDRFull ? c_maxFDRecords - 1 : m_currFDRIndex - 1);

		if (--m_selectedFDRIndex < 0)
			m_selectedFDRIndex = maxIndex;

		m_updateFDR = true;
	}
	void NextFDR()
	{
		int maxIndex = (m_FDRFull ? c_maxFDRecords - 1 : m_currFDRIndex - 1);

		if (++m_selectedFDRIndex > maxIndex)
			m_selectedFDRIndex = 0;

		m_updateFDR = true;
	}

private:
	void WorkerFunc();
	bool ProcessFrame(cv::Mat & frame);
	int LaneAssistComputeServo(cv::Mat & frame);
	cv::Mat * ProcessDebugFrame();
	void DisplayCurrentParamPage();
	int TranslateXTargetToServo(eLane lane, int xTarget) const;
	void TrimSearchRange(cv::Vec2i & searchRange) const;

};

#endif /* BUSMGR_H_ */
