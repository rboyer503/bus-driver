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

//#define MAX_BUFFER_SIZE (320 * 240 + 100)
//#define NUM_LANES 6
//#define MAX_LINE_WEIGHT 50
#define FRAME_SKIP 2
#define FRAME_BACKLOG_MIN -5

#define STATUS_SUPPRESS_DELAY 10
#define STATUS_HISTORY_BUCKETS 10

#define ROI_WIDTH 160
#define ROI_HEIGHT 60

#define VOTE_ARRAY_WIDTH 370
#define PACKED_VOTE_BUFFER_SIZE 64000

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

enum eLane
{
	LEFT_LANE,
	RIGHT_LANE,
	MAX_LANES
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

struct LaneState
{
	static constexpr int cSlopeHistSize = 10;
	static constexpr int cSlopeHistMin = 5;

	int xVals[ROI_HEIGHT];
	int slopeHist[cSlopeHistSize];
	int slopeHistIndex;
	bool slopeHistFull;
	bool started, searching;
	cv::Vec2i firstEdge, lastEdge;
	float firstSlope, lastSlope;

	LaneState() :
		slopeHistIndex(0), slopeHistFull(false), started(false), searching(false), firstSlope(0.0f), lastSlope(0.0f)
	{
		std::fill_n(xVals, ROI_HEIGHT, 0);
		lastEdge[0] = firstEdge[0] = -1;
	}

	int getSlopeHistCount() const { return (slopeHistFull ? cSlopeHistSize : slopeHistIndex); }
	bool isValidFirstSlope() const { return (firstEdge[0] >= 0); }
	bool isValidLastSlope() const { return (lastEdge[0] >= 0); }

	bool addSlopeHistEntry(int slope)
	{
		slopeHist[slopeHistIndex++] = slope;
		if (slopeHistIndex == cSlopeHistSize)
		{
			slopeHistIndex = 0;

			if (!slopeHistFull)
			{
				slopeHistFull = true;
				firstSlope = static_cast<float>(std::accumulate(slopeHist, slopeHist + cSlopeHistSize, 0)) / cSlopeHistSize;
				return true;
			}
		}

		return false;
	}

	void startSearch(int y)
	{
		searching = true;

		int ySearch = y + 1;
		while ( (ySearch < ROI_HEIGHT) && !xVals[ySearch] )
			++ySearch;

		lastEdge[0] = xVals[ySearch + 1];
		lastEdge[1] = ySearch;

		int slopeHistCount = getSlopeHistCount();
		lastSlope = static_cast<float>(std::accumulate(slopeHist, slopeHist + slopeHistCount, 0)) / slopeHistCount;
	}

	void finalize()
	{
		if (!slopeHistFull)
		{
			// May need to set firstSlope.
			// Check if any adequate slope measurement exists yet.
			// If not, reset firstEdge since it's of no use without a viable slope.
			if (lastEdge[0] >= 0)
			{
				firstSlope = lastSlope;
			}
			else
			{
				firstEdge[0] = -1;
			}
		}
	}

	void debugOutput() const
	{
		std::cout << "    firstEdge: " << firstEdge[0] << ", " << firstEdge[1] << "; slope: " << firstSlope << std::endl;
		std::cout << "    lastEdge: " << lastEdge[0] << ", " << lastEdge[1] << "; slope: " << lastSlope << std::endl;
	}

	int calcXTarget(int targetY, bool debug = false) const
	{
		if (xVals[targetY])
		{
			// Simple case: edge was present at target scanline.
			if (debug)
				std::cout << "  Trivial case" << std::endl;

			return xVals[targetY];
		}
		else
		{
			// Search for nearest edges above and below target scanline.
			int yTop;
			for (yTop = (targetY - 1); yTop >= 0; --yTop)
				if (xVals[yTop])
					break;

			int yBottom;
			for (yBottom = (targetY + 1); yBottom < ROI_HEIGHT; ++yBottom)
				if (xVals[yBottom])
					break;

			if ( (yTop >= 0) && (yBottom < ROI_HEIGHT) )
			{
				if (debug)
					std::cout << "  Interpolate case" << std::endl;

				// Found edges - interpolate for target scanline.
				float m = static_cast<float>(xVals[yBottom] - xVals[yTop]) / (yBottom - yTop);
				return xVals[yTop] + static_cast<int>((targetY - yTop) * m);
			}
			else if (yBottom < ROI_HEIGHT)
			{
				if (debug)
					std::cout << "  Slope up case" << std::endl;

				// Cannot interpolate, resort to slope estimation from shift history.
				// Follow slope up from last edge detected (assuming slope estimate was acquired).
				if (lastEdge[0] >= 0)
					return static_cast<int>( (lastEdge[1] - targetY) * lastSlope ) + lastEdge[0];
			}
			else if (yTop >= 0)
			{
				if (debug)
					std::cout << "  Slope down case" << std::endl;

				// Cannot interpolate, resort to slope estimation from shift history.
				// Follow slope down from first edge detected (assuming slope estimate was acquired).
				if (firstEdge[0] >= 0)
					return static_cast<int>( (firstEdge[1] - targetY) * firstSlope ) + firstEdge[0];
			}
		}

		return 0;
	}
};


class SocketMgr;


class BusMgr
{
	static const char * const c_imageProcModeNames[];
	static const char * const c_imageProcStageNames[];

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
	bool m_debugTrigger;
	int m_voteIndex[ROI_HEIGHT][VOTE_ARRAY_WIDTH];
	short m_packedVoteArray[PACKED_VOTE_BUFFER_SIZE];

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
	void SetReverse(bool reverse) { m_pCtrlMgr->SetReverse(reverse); }
	void SetServo(int servo) { m_pCtrlMgr->SetServo(servo); }
	void SetLaneAssist(bool enable) { m_pCtrlMgr->SetLaneAssist(enable); }
	bool GetLaneAssist() const { return m_pCtrlMgr->GetLaneAssist(); }

private:
	void WorkerFunc();
	bool ProcessFrame(cv::Mat & frame);
	int LaneAssistComputeServo(cv::Mat & frame);
	void DisplayCurrentParamPage();
	bool LoadVoteArray();

};

#endif /* BUSMGR_H_ */
