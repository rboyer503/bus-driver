/*
 * BusMgr.h
 *
 *  Created on: Jan 15, 2015
 *      Author: rboyer
 */

#ifndef BUSMGR_H_
#define BUSMGR_H_

#include <opencv2/opencv.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#define MAX_BUFFER_SIZE (320 * 240 + 100)
#define NUM_LANES 6
#define MAX_LINE_WEIGHT 50
#define FRAME_SKIP 2
#define FRAME_BACKLOG_MIN -5

#define STATUS_SUPPRESS_DELAY 10
#define STATUS_HISTORY_BUCKETS 10


enum eBDErrorCode
{
	EC_NONE,
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
	IPM_CANNY,
	IPM_ROAD,
	IPM_HOUGH,
	IPM_MAX
};

enum eBDImageProcStage
{
	IPS_GRAY,
	IPS_BLUR,
	IPS_CANNY,
	IPS_ROAD,
	IPS_HOUGH,
	IPS_POST,
	IPS_SENT,
	IPS_TOTAL,
	IPS_MAX
};

enum eBDParamPage
{
	PP_BLUR,
	PP_CANNYTHRESHOLD,
	PP_HOUGHGRANULARITY,
	PP_HOUGHTHRESHOLD,
	PP_HOUGHLINEPARAMS,
	PP_LANETHRESHOLD,
	PP_MAX
};

struct Lane
{
	cv::Vec4i line;
	float m; // Slope
	unsigned char weight; // Importance

	Lane() : m(0.0f), weight(0)
	{}
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
	unsigned char cannyThresholdLow;
	float cannyThresholdFactor;
	unsigned char houghRho;
	unsigned char houghTheta; // In degrees
	unsigned char houghThreshold;
	unsigned char houghMinLineLength;
	unsigned char houghMaxLineGap;
	unsigned char laneStitchThreshold;
	unsigned char laneSegregationThreshold;
	Config() :
		kernelSize(5), cannyThresholdLow(100), cannyThresholdFactor(2.0),
		houghRho(2), houghTheta(2), houghThreshold(25),
		houghMinLineLength(1), houghMaxLineGap(50),
		laneStitchThreshold(20), laneSegregationThreshold(20)
	{}
};

struct LaneCandidate
{
	cv::Vec4i line;
	float m, b;
	float mStart, bStart;
	float mEnd, bEnd;

	LaneCandidate() :
		m(0.0), b(0.0),
		mStart(0.0), bStart(0.0),
		mEnd(0.0), bEnd(0.0)
	{}
};


class SocketMgr;

class BusMgr
{
	static const char * const c_imageProcModeNames[];
	static const char * const c_imageProcStageNames[];
	eBDErrorCode m_errorCode;
	eBDImageProcMode m_ipm;
	SocketMgr * m_pSocketMgr;
	Lane m_lanes[NUM_LANES];
	Status m_status;
	Config m_config;
	eBDParamPage m_paramPage;
	boost::thread m_thread;
	volatile bool m_running;
	bool m_interrupted;
	boost::posix_time::ptime m_startTime;
	boost::posix_time::time_duration m_diff;

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

private:
	void WorkerFunc();
	bool ProcessFrame(cv::Mat & frame);
	void BuildLaneCandidates(std::vector<cv::Vec4i> & lines, int yBase, std::vector<LaneCandidate *> & laneCandidates);
	void DisplayCurrentParamPage();

};

#endif /* BUSMGR_H_ */
