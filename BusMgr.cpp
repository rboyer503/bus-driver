/*
 * BusMgr.cpp
 *
 *  Created on: Jan 15, 2015
 *      Author: rboyer
 */
#include <limits>
#include <fstream>
#include <vector>

#include "BusMgr.h"
#include "SocketMgr.h"
#include "VideoCaptureMgr.h"
#include "ControlMgr.h"

#define ENABLE_SOCKET_MGR 1

using namespace std;
using namespace cv;


// Global variables and macros for profiling.
boost::posix_time::ptime g_start;
boost::posix_time::time_duration g_diff;

#define PROFILE_START g_start = boost::posix_time::microsec_clock::local_time();

#define PROFILE_LOG(tag) \
	do { \
		g_diff = boost::posix_time::microsec_clock::local_time() - g_start; \
		cout << "Profile (" << #tag << "): " << g_diff.total_microseconds() << " us" << endl; \
	} while (0)

#define PROFILE_DIFF (boost::posix_time::microsec_clock::local_time() - g_start).total_microseconds()


const char * const BusMgr::c_imageProcModeNames[] = {"None", "Gray", "Blur", "Lane Assist"};
const char * const BusMgr::c_imageProcStageNames[] = {"Gray", "Blur", "Lane Assist", "Send", "Total"};


BusMgr::BusMgr() :
	m_errorCode(EC_NONE), m_ipm(IPM_LANEASSIST), m_paramPage(PP_BLUR), m_running(false), m_interrupted(false), m_debugTrigger(false)
{
	m_pSocketMgr = new SocketMgr(this);
	m_pCtrlMgr = new ControlMgr();
}

BusMgr::~BusMgr()
{
	delete m_pSocketMgr;
	delete m_pCtrlMgr;

	if (m_thread.joinable())
		m_thread.join();
}

bool BusMgr::Initialize()
{
	if (!LoadVoteArray())
	{
		return false;
	}

	// Initialize monitor and command sockets.
#ifdef ENABLE_SOCKET_MGR
	if (!m_pSocketMgr->Initialize())
	{
		m_errorCode = EC_LISTENFAIL;
		return false;
	}
#endif

	m_running = true;
	m_thread = boost::thread(&BusMgr::WorkerFunc, this);
	return true;
}

void BusMgr::Terminate()
{
	m_thread.interrupt();
	m_pCtrlMgr->Terminate();
}

void BusMgr::UpdateIPM()
{
	m_ipm = (eBDImageProcMode)(((int)m_ipm + 1) % IPM_MAX);
	if (m_ipm == IPM_NONE)
		m_ipm = IPM_GRAY;
	cout << "Image processing mode: " << c_imageProcModeNames[m_ipm] << endl;
}

void BusMgr::OutputStatus()
{
	cout << endl << "Statistics" << endl;

	cout << "  Total Frames=" << m_status.numFrames;
	cout << "\tDelayed Frames=" << m_status.numDroppedFrames;
	cout << "\tAverage FPS=" << (m_status.numFrames / m_diff.total_seconds()) << endl;

	cout << "  Processing times:" << endl;
	for (int i = IPS_GRAY; i < IPS_MAX; ++i)
	{
		cout << "    " << c_imageProcStageNames[i] <<
				": curr=" << m_status.currProcessUs[i] <<
				", avg=" << (m_status.totalProcessUs[i] / m_status.numFrames) <<
				", max=" << m_status.maxProcessUs[i] << endl;
	}
}

void BusMgr::OutputConfig()
{
	cout << endl << "Configuration" << endl;
	cout << "  Image Processing Mode=" << c_imageProcModeNames[m_ipm] << endl;
	cout << "  Current Parameter Page=" << m_paramPage << endl;
	cout << "  Kernel Size=" << (int)m_config.kernelSize << endl;
	cout << "  Gradient Threshold=" << (int)m_config.gradientThreshold << endl;
}

void BusMgr::UpdatePage()
{
	m_paramPage = (eBDParamPage)((m_paramPage + 1) % PP_MAX);
	DisplayCurrentParamPage();
}

void BusMgr::UpdateParam(int param, bool up)
{
	switch (m_paramPage)
	{
	case PP_BLUR:
		if ( up && (m_config.kernelSize < 15) )
			m_config.kernelSize += 2;
		else if (!up && (m_config.kernelSize > 1))
			m_config.kernelSize -= 2;
		break;
	case PP_GRADIENTTHRESHOLD:
		if ( up && (m_config.gradientThreshold < 100) )
			m_config.gradientThreshold++;
		else if (!up && (m_config.gradientThreshold > 1))
			m_config.gradientThreshold--;
		break;
	}
}

void BusMgr::DebugCommand()
{
	m_debugTrigger = true;
}

void BusMgr::WorkerFunc()
{
	// Main loop - each iteration handles a client connection.
	// Exit only in response to an error condition or interruption (user cancellation request).
	while (1)
	{
		// Accept an incoming connection.
#ifdef ENABLE_SOCKET_MGR
		if (!m_pSocketMgr->WaitForConnection())
		{
			if (m_interrupted)
				m_errorCode = EC_INTERRUPT;
			else
				m_errorCode = EC_ACCEPTFAIL;
			break;
		}

		// Start accepting commands from client.
		if (!m_pSocketMgr->StartReadingCommands())
		{
			m_errorCode = EC_READCMDFAIL;
			break;
		}
#endif

		// Initialize video.
		VideoCaptureMgr vcMgr(this);
		if (!vcMgr.Initialize())
		{
			cerr << "Error: Failed to open video capture." << endl;
			m_errorCode = EC_CAPTUREOPENFAIL;
			break;
		}

		m_status = Status();

		DisplayCurrentParamPage();

		// Continually transmit frames to client.
		Mat * frame;
		int nextFrame = FRAME_SKIP;
		while (1)
		{
			//PROFILE_START;

			// Retrieve frame.
			do
			{
				int skippedFrames;
				if ( (skippedFrames = vcMgr.GetLatest(frame)) < 0 )
				{
					if (m_interrupted)
					{
						cout << "Interrupted while waiting for frame - shutting down server..." << endl;
						m_errorCode = EC_INTERRUPT;
					}
					else
					{
						cerr << "Error: Failed to read a frame." << endl;
						m_errorCode = EC_CAPTUREGRABFAIL;
					}
					break;
				}
				nextFrame -= skippedFrames + 1;
				//PROFILE_LOG(read);
			} while (nextFrame > 0);
			if (m_errorCode)
				break;
			nextFrame += FRAME_SKIP;
			if (nextFrame < FRAME_BACKLOG_MIN)
				nextFrame = FRAME_BACKLOG_MIN;

			if (m_status.SuppressionProcessing())
				m_startTime = boost::posix_time::microsec_clock::local_time();

			//PROFILE_LOG(read);

			if (!m_status.IsSuppressed())
			{
				if (nextFrame != FRAME_SKIP)
				{
					//cout << "Frame processing delayed, nextFrame = " << nextFrame << endl;
					m_status.numDroppedFrames++;
				}
				m_status.numFrames++;
			}

			// Process frame.
			if (!ProcessFrame(*frame))
			{
				// Suppress error for send failure - just wait for another connection.
				if (m_errorCode == EC_SENDFAIL)
					m_errorCode = EC_NONE;
				break;
			}

			m_diff = boost::posix_time::microsec_clock::local_time() - m_startTime;

			try
			{
				boost::this_thread::interruption_point();
			}
			catch (boost::thread_interrupted&)
			{
				cout << "Interrupted after processing frame - shutting down server..." << endl;
				m_errorCode = EC_INTERRUPT;
				break;
			}
		} // end video streaming loop

		// Clean up connection.
#ifdef ENABLE_SOCKET_MGR
		if (!m_pSocketMgr->ReleaseConnection())
		{
			cerr << "Error: Connection release failed." << endl;
			m_errorCode = EC_RELEASEFAIL;
		}
#endif

		// Exit program if any error was reported.
		if (m_errorCode)
			break;
	} // end connection handling loop

	m_running = false;
}

bool BusMgr::ProcessFrame(Mat & frame)
{
	PROFILE_START;

	Mat * pFrameDisplay = NULL;
	Mat frameResize;
	Mat frameGray;
	Mat frameROI;
	Mat frameFilter;
	eBDImageProcMode ipm = m_ipm;
	int processUs[IPS_MAX];
	memset(processUs, 0, sizeof(processUs));
	int servo = ControlMgr::cDefServo;

	if (ipm == IPM_NONE)
	{
		pFrameDisplay = &frame;
	}
	else
	{
		// Convert to grayscale image, flip, and focus to ROI.
		resize(frame, frameResize, Size(), 0.5, 0.5, INTER_NEAREST);
		cvtColor(frameResize, frameGray, CV_BGR2GRAY);
		flip(frameGray, frameGray, -1);
		frameROI = frameGray(Rect(0, 48, ROI_WIDTH, ROI_HEIGHT));

		processUs[IPS_GRAY] = PROFILE_DIFF;
		PROFILE_START;

		if (ipm == IPM_GRAY)
		{
			pFrameDisplay = &frameROI;
		}
		else
		{
			// Apply gaussian blur.
			GaussianBlur(frameROI, frameFilter, Size(m_config.kernelSize, m_config.kernelSize), 0, 0);

			processUs[IPS_BLUR] = PROFILE_DIFF;
			PROFILE_START;

			if (ipm == IPM_BLUR)
			{
				pFrameDisplay = &frameFilter;
			}
			else
			{
				// Run lane assist algorithm to calculate servo position.
				servo = LaneAssistComputeServo(frameFilter);

				processUs[IPS_LANEASSIST] = PROFILE_DIFF;
				PROFILE_START;

				pFrameDisplay = &frameFilter;
			}
		}
	}

	if (m_pCtrlMgr->GetLaneAssist())
	{
		m_pCtrlMgr->SetServo(servo);
	}

	// Encode for wifi transmission.
	vector<uchar> buf;
	imencode(".bmp", *pFrameDisplay, buf);

	// Transmit to client.
#ifdef ENABLE_SOCKET_MGR
	if (!m_pSocketMgr->SendFrame(&buf[0], buf.size()))
	{
		// Client probably disconnected - exit streaming loop and wait for a new connection.
		m_errorCode = EC_SENDFAIL;
		return false;
	}
#endif

	processUs[IPS_SENT] = PROFILE_DIFF;

	// Update status.
	if (!m_status.IsSuppressed())
	{
		for (int i = IPS_GRAY; i < IPS_MAX; ++i)
		{
			if (processUs[i])
			{
				if (i < IPS_TOTAL)
					processUs[IPS_TOTAL] += processUs[i];
				m_status.currProcessUs[i] = processUs[i];
				m_status.totalProcessUs[i] += processUs[i];
				if (processUs[i] > m_status.maxProcessUs[i])
					m_status.maxProcessUs[i] = processUs[i];
			}
			else
			{
				m_status.currProcessUs[i] = 0;
			}
		}
	}

	return true;
}

int BusMgr::LaneAssistComputeServo(cv::Mat & frame)
{
	bool debugOutput = false;
	if (m_debugTrigger)
	{
		m_debugTrigger = false;

		// DEBUG CODE: Output debug data for this frame.
		debugOutput = true;
		cout << endl << "Debug output" << endl;
	}

	// Perform edge detection using simple linear filter.
	// Left lane and right lane edges handled independently.
	// Sweep right to left for left lane edges, and vice versa for right lane edges.
	// Apply suppression after an edge is detected as a simplistic "edge thinning".

	const int filter[] = {2, 1, 0, -1, -2}; // Not directly used due to optimizations.
	const int edgeBuffer = 3; // Ignore a few pixels on far left and far right edges of image.
	const int conditionalEdgeBuffer = 10; // Ignore more pixels on far left for right lane and far right for left lane.
	const int suppressCount = 10; // Skip several pixels after an edge is detected.

	uchar * pRow;
	int gradient;

	// Negative and positive edge maps for left and right lanes respectively.
	// Edge maps are built so that edges resulting from neg->pos gradients can be culled.
	// (Neg->pos gradients can occur from shadows, etc., and should be ignored.  Actual lane markers should be brighter than road color, not darker.)
	uchar edgeMapNeg[ROI_HEIGHT][ROI_WIDTH] = {};
	uchar edgeMapPos[ROI_HEIGHT][ROI_WIDTH] = {};

	// Actual edge coordinates after neg-->pos gradient culling - separate bins for left and right lanes.
	// Reserve enough for room to avoid ever needing to resize.
	vector<Vec2i> leftEdges;
	vector<Vec2i> rightEdges;
	leftEdges.reserve(900);
	rightEdges.reserve(900);

	// Mark left and right X position for sweeping kernel for linear filter.
	int xLeft = edgeBuffer;
	int xRight = frame.cols - edgeBuffer - (sizeof(filter) / sizeof(filter[0]));
	if (debugOutput)
		cout << "  xLeft=" << xLeft << ", xRight=" << xRight << endl;

	// Build edge maps.
	for (int y = 0; y < frame.rows - 1; ++y)
	{
		pRow = frame.ptr(y);

		for (int x = xRight - conditionalEdgeBuffer; x >= xLeft; --x)
		{
			gradient = (pRow[x] << 1) +
					   (pRow[x + 1]) -
					   (pRow[x + 3]) -
					   (pRow[x + 4] << 1);

			if (gradient >= m_config.gradientThreshold)
			{
				//leftEdges.push_back(Vec2i(x + 2, y));
				edgeMapNeg[y][x + 2] = 255;
				//pRow[x + 2] = 255;
				x -= suppressCount;
			}
		}

		for (int x = xLeft + conditionalEdgeBuffer; x <= xRight; ++x)
		{
			gradient = -(pRow[x] << 1) -
					   (pRow[x + 1]) +
					   (pRow[x + 3]) +
					   (pRow[x + 4] << 1);

			if (gradient >= m_config.gradientThreshold)
			{
				//rightEdges.push_back(Vec2i(x + 2, y));
				edgeMapPos[y][x + 2] = 255;
				//pRow[x + 2] = 192;
				x += suppressCount;
			}
		}
	}

	// Left lane edge detection with culling of neg->pos gradients.
	for (int y = 0; y < ROI_HEIGHT; ++y)
	{
		for (int x = (xLeft + 2); x <= (xRight + 2 - conditionalEdgeBuffer); ++x)
		{
			if (edgeMapNeg[y][x])
			{
				// Perform culling if necessary.
				int i;
				for (i = 1; i <= 10; ++i)
				{
					if (edgeMapPos[y][x + i])
					{
						edgeMapPos[y][x + i] = 0;
						break;
					}
				}

				if (i > 10)
				{
					leftEdges.push_back(Vec2i(x, y));
				}
			}
		}
	}

	// Right lane edge detection - culling was already complete.
	for (int y = 0; y < ROI_HEIGHT; ++y)
	{
		for (int x = (xLeft + 2 + conditionalEdgeBuffer); x <= (xRight + 2); ++x)
		{
			if (edgeMapPos[y][x])
			{
				rightEdges.push_back(Vec2i(x, y));
			}
		}
	}

	// Show detected edges on image for diagnostics purposes.
	for (Vec2i & edge : leftEdges)
	{
		frame.at<uchar>(edge[1], edge[0], 0) = 255;
	}

	for (Vec2i & edge : rightEdges)
	{
		frame.at<uchar>(edge[1], edge[0], 0) = 192;
	}

	// Apply lane voting.
	short * pVote;

	// Offset range: left lane (100 -> -25) => (85 => 210)
	//              right lane (60 -> 185) => (125 => 0)
	int bestOffset = 0;
	int bestLaneId = 0;
	int maxVotes = 0;
	int debugMaxVotes = 0;

	// Lane voting to find best left lane.
	for (int offset = 85; offset <= 210; ++offset)
	{
		short laneVoteTable[31 * 31] = {};
		debugMaxVotes = 0;

		for (Vec2i & edge : leftEdges)
		{
			int index = m_voteIndex[edge[1]][edge[0] + offset];
			if (index)
			{
				pVote = m_packedVoteArray + index;
				while (*pVote)
					++laneVoteTable[*pVote++];
			}

			index = m_voteIndex[edge[1]][edge[0] + offset - 1];
			if (index)
			{
				pVote = m_packedVoteArray + index;
				while (*pVote)
					++laneVoteTable[*pVote++];
			}

			index = m_voteIndex[edge[1]][edge[0] + offset + 1];
			if (index)
			{
				pVote = m_packedVoteArray + index;
				while (*pVote)
					++laneVoteTable[*pVote++];
			}
		}

		for (int i = 0; i < (31 * 31); ++i)
		{
			if (laneVoteTable[i] > maxVotes)
			{
				maxVotes = laneVoteTable[i];
				bestOffset = offset;
				bestLaneId = i;

				//if (debugOutput)
				//	cout << "    New maxVotes=" << maxVotes << ", bestOffset=" << bestOffset << ", bestLaneId=" << bestLaneId << endl;
			}

			if (debugOutput)
			{
				if (laneVoteTable[i] > debugMaxVotes)
					debugMaxVotes = laneVoteTable[i];
			}
		}

		if (debugOutput)
			cout << "    Debug x=" << (185 - offset) << ", maxVotes=" << debugMaxVotes << endl;
	}

	if (debugOutput)
		cout << "  bestOffset=" << bestOffset << ", bestLaneId=" << bestLaneId << ", maxVotes=" << maxVotes << endl;

	int leftTarget = ROI_WIDTH;
	if (bestOffset)
	{
		leftTarget = 185 - bestOffset;
	}

	// Lane voting to find best right lane.
	bestOffset = 0;
	bestLaneId = 0;
	maxVotes = 0;

	for (int offset = 125; offset >= 0; --offset)
	{
		short laneVoteTable[31 * 31] = {};
		debugMaxVotes = 0;

		for (Vec2i & edge : rightEdges)
		{
			int index = m_voteIndex[edge[1]][edge[0] + offset];
			if (index)
			{
				pVote = m_packedVoteArray + index;
				while (*pVote)
					++laneVoteTable[*pVote++];
			}

			index = m_voteIndex[edge[1]][edge[0] + offset - 1];
			if (index)
			{
				pVote = m_packedVoteArray + index;
				while (*pVote)
					++laneVoteTable[*pVote++];
			}

			index = m_voteIndex[edge[1]][edge[0] + offset + 1];
			if (index)
			{
				pVote = m_packedVoteArray + index;
				while (*pVote)
					++laneVoteTable[*pVote++];
			}
		}

		for (int i = 0; i < (31 * 31); ++i)
		{
			if (laneVoteTable[i] > maxVotes)
			{
				maxVotes = laneVoteTable[i];
				bestOffset = offset;
				bestLaneId = i;

				//if (debugOutput)
				//	cout << "    New maxVotes=" << maxVotes << ", bestOffset=" << bestOffset << ", bestLaneId=" << bestLaneId << endl;
			}

			if (debugOutput)
			{
				if (laneVoteTable[i] > debugMaxVotes)
					debugMaxVotes = laneVoteTable[i];
			}
		}

		if (debugOutput)
			cout << "    Debug x=" << (185 - offset) << ", maxVotes=" << debugMaxVotes << endl;
	}

	if (debugOutput)
		cout << "  bestOffset=" << bestOffset << ", bestLaneId=" << bestLaneId << ", maxVotes=" << maxVotes << endl;

	int rightTarget = 0;
	if (bestOffset)
	{
		rightTarget = 185 - bestOffset;
	}

	// Adapt target X values to servo position.
	const int leftLaneCenterX = 26;
	const int rightLaneCenterX = 149;
	const float offsetToServoFactor = 3.0f;

	if (leftTarget < ROI_WIDTH)
	{
		if (debugOutput)
			cout << "    leftX=" << leftTarget;

		// Correlate to servo value.
		leftTarget = ControlMgr::cDefServo + static_cast<int>( (leftTarget - leftLaneCenterX) / offsetToServoFactor );

		if (debugOutput)
			cout << ", leftServo=" << leftTarget << endl;
	}

	if (rightTarget)
	{
		if (debugOutput)
			cout << "    rightX=" << rightTarget;

		// Correlate to servo value.
		rightTarget = ControlMgr::cDefServo + static_cast<int>( (rightTarget - rightLaneCenterX) / offsetToServoFactor );

		if (debugOutput)
			cout << ", rightServo=" << rightTarget << endl;
	}

	int servo = ControlMgr::cDefServo;
	const int targetDiffThreshold = 15; // If left and right lanes disagree strongly, ignore the one furthest from default (straight) position.

	if (leftTarget || rightTarget)
	{
		if (leftTarget && rightTarget)
		{
			if ( abs(leftTarget - rightTarget) > targetDiffThreshold )
			{
				if ( abs(leftTarget - ControlMgr::cDefServo) > abs(rightTarget - ControlMgr::cDefServo) )
					servo = rightTarget;
				else
					servo = leftTarget;
			}
			else
				servo = (leftTarget + rightTarget) / 2;
		}
		else if (leftTarget)
			servo = leftTarget;
		else
			servo = rightTarget;

		if ( servo < (ControlMgr::cDefServo - ControlMgr::cServoRange) )
			servo = ControlMgr::cDefServo - ControlMgr::cServoRange;
		else if ( servo > (ControlMgr::cDefServo + ControlMgr::cServoRange) )
			servo = ControlMgr::cDefServo + ControlMgr::cServoRange;
	}

	if (debugOutput)
		cout << "  Target: " << leftTarget << ", " << rightTarget << "; Servo: " << servo << endl;

	return servo;
}

void BusMgr::DisplayCurrentParamPage()
{
	cout << "Current parameter page: " << m_paramPage << endl;
	switch (m_paramPage)
	{
	case PP_BLUR:
		cout << "  1) Kernel Size" << endl;
		break;
	case PP_GRADIENTTHRESHOLD:
		cout << "  1) Gradient Threshold" << endl;
		break;
	}
}

bool BusMgr::LoadVoteArray()
{
	ifstream indexFile("index_file.txt");
	if (indexFile.is_open())
	{
		for (int y = 0; y < ROI_HEIGHT; ++y)
			for (int x = 0; x < VOTE_ARRAY_WIDTH; ++x)
				indexFile >> m_voteIndex[y][x];
		indexFile.close();
	}
	else
	{
		cerr << "ERROR: Cannot open index file." << endl;
		return false;
	}

	ifstream voteArrayFile("vote_array_file.txt");
	if (voteArrayFile.is_open())
	{
		for (int i = 0; i < PACKED_VOTE_BUFFER_SIZE; ++i)
			voteArrayFile >> m_packedVoteArray[i];
		voteArrayFile.close();
	}
	else
	{
		cerr << "ERROR: Cannot open vote array file." << endl;
		return false;
	}

	return true;
}
