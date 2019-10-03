/*
 * BusMgr.cpp
 *
 *  Created on: Jan 15, 2015
 *      Author: rboyer
 */
#include <limits>

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
	m_errorCode(EC_NONE), m_ipm(IPM_BLUR), m_paramPage(PP_BLUR), m_running(false), m_interrupted(false), m_debugTrigger(false)
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
		cvtColor(frame, frameGray, CV_BGR2GRAY);
		flip(frameGray, frameGray, -1);
		frameROI = frameGray(Rect(0, 96, ROI_WIDTH, ROI_HEIGHT));

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

				// Run lane assist algorithm to calculate servo position.
				servo = LaneAssistComputeServo(frameFilter);

				processUs[IPS_LANEASSIST] = PROFILE_DIFF;
				PROFILE_START;
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

	// Simplified lane detection algorithm.
	LaneState laneState[MAX_LANES];

	const int filter[] = {2, 1, 0, -1, -2};
	const int edgeBuffer = 3;
	const int contThreshold = 20; // Continuation threshold: if x value diverges more than this in one scanline, treat as different line.

	if (debugOutput)
		cout << "  Left lane processing" << endl;

	for (int y = (frame.rows - 1); y > 0; --y)
	{
		uchar * pROIRow = frame.ptr(y);

		// Search for left lane markings.
		int gradient;
		int x = (frame.cols >> 1) - m_config.kernelSize;
		do
		{
			gradient = (pROIRow[x] * filter[0]) +
					   (pROIRow[x + 1] * filter[1]) +
					   (pROIRow[x + 3] * filter[3]) +
					   (pROIRow[x + 4] * filter[4]);
		}
		while ( (gradient < m_config.gradientThreshold) && (--x > edgeBuffer) );

		// Found something exceeding threshold on this scanline?
		LaneState * currState = &laneState[LEFT_LANE];
		if (gradient >= m_config.gradientThreshold)
		{
			if (!currState->started)
			{
				// First edge.
				currState->started = true;
				currState->xVals[y] = x;

				if (currState->firstEdge[0] < 0)
				{
					currState->firstEdge[0] = x;
					currState->firstEdge[1] = y;

					if (debugOutput)
						cout << "    firstEdge: " << x << ", " << y << endl;
				}
			}
			else if (!currState->searching)
			{
				// Edge potentially connected to previous edge.
				if ( (currState->xVals[y + 1] - x) < contThreshold )
				{
					// Not too far left.
					currState->xVals[y] = x;

					if ( (x - currState->xVals[y + 1]) < contThreshold )
					{
						// Not jumping to the right either - OK to include in shift history.
						if ( currState->addSlopeHistEntry(x - currState->xVals[y + 1]) )
						{
							if (debugOutput)
								cout << "    firstSlope (full): " << currState->firstSlope << endl;
						}
					}
				}
			}
			else
			{
				// Searching for the start of a new edge.
				int expectX = static_cast<int>( (currState->lastEdge[1] - y) * currState->lastSlope ) + currState->lastEdge[0];
				if ( (expectX - x) < contThreshold )
				{
					// Found it - not too far left, although might be jumping to right.
					currState->xVals[y] = x;
					currState->searching = false;
				}
			}
		}
		else if (currState->started && !currState->searching)
		{
			int slopeHistCount = currState->getSlopeHistCount();
			if (slopeHistCount < LaneState::cSlopeHistMin)
			{
				// Not enough shift history for reasonable estimate of slope - just accept next edge.
				currState->started = false;
			}
			else
			{
				// Estimate slope to better know where to expect the next edge.
				currState->startSearch(y);

				if (debugOutput)
					cout << "    lastEdge: " << currState->lastEdge[0] << ", " << currState->lastEdge[1] << "; slope: " << currState->lastSlope << endl;
			}
		}
	}

	if (debugOutput)
		cout << "  Right lane processing" << endl;

	for (int y = (frame.rows - 1); y > 0; --y)
	{
		uchar * pROIRow = frame.ptr(y);

		// Search for right lane markings.
		int gradient;
		int x = (frame.cols >> 1);
		do
		{
			gradient = (pROIRow[x] * filter[4]) +
					   (pROIRow[x + 1] * filter[3]) +
					   (pROIRow[x + 3] * filter[1]) +
					   (pROIRow[x + 4] * filter[0]);
		}
		while ( (gradient < m_config.gradientThreshold) && (++x <= (320 - m_config.kernelSize - edgeBuffer)) );

		// Found something exceeding threshold on this scanline?
		LaneState * currState = &laneState[RIGHT_LANE];
		if (gradient >= m_config.gradientThreshold)
		{
			if (!currState->started)
			{
				// First edge.
				currState->started = true;
				currState->xVals[y] = x;

				if (currState->firstEdge[0] < 0)
				{
					currState->firstEdge[0] = x;
					currState->firstEdge[1] = y;

					if (debugOutput)
						cout << "    firstEdge: " << x << ", " << y << endl;
				}
			}
			else if (!currState->searching)
			{
				// Edge potentially connected to previous edge.
				if ( (x - currState->xVals[y + 1]) < contThreshold )
				{
					// Not too far right.
					currState->xVals[y] = x;

					if ( (currState->xVals[y + 1] - x) < contThreshold )
					{
						// Not jumping to the left either - OK to include in shift history.
						if ( currState->addSlopeHistEntry(x - currState->xVals[y + 1]) )
						{
							if (debugOutput)
								cout << "    firstSlope (full): " << currState->firstSlope << endl;
						}
					}
				}
			}
			else
			{
				// Searching for the start of a new edge.
				int expectX = static_cast<int>( (currState->lastEdge[1] - y) * currState->lastSlope ) + currState->lastEdge[0];
				if ( (x - expectX) < contThreshold )
				{
					// Found it - not too far right, although might be jumping to left.
					currState->xVals[y] = x;
					currState->searching = false;
				}
			}
		}
		else if (currState->started && !currState->searching)
		{
			int slopeHistCount = currState->getSlopeHistCount();
			if (slopeHistCount < LaneState::cSlopeHistMin)
			{
				// Not enough shift history for reasonable estimate of slope - just accept next edge.
				currState->started = false;
			}
			else
			{
				// Estimate slope to better know where to expect the next edge.
				currState->startSearch(y);

				if (debugOutput)
					cout << "    lastEdge: " << currState->lastEdge[0] << ", " << currState->lastEdge[1] << "; slope: " << currState->lastSlope << endl;
			}
		}
	}

	for (LaneState & state : laneState)
		state.finalize();

	if (debugOutput)
	{
		cout << "  Left lane summary" << endl;
		laneState[LEFT_LANE].debugOutput();
		cout << "  Right lane summary" << endl;
		laneState[RIGHT_LANE].debugOutput();
	}

	const int targetScanline = 90;
	const int leftLaneCenterX = 52;
	const int rightLaneCenterX = 298;
	const float offsetToServoFactor = 6.0f;

	int leftTarget = laneState[LEFT_LANE].calcXTarget(targetScanline, debugOutput);
	if (leftTarget)
	{
		if (debugOutput)
			cout << "    leftX=" << leftTarget;

		// Correlate to servo value.
		leftTarget = ControlMgr::cDefServo + static_cast<int>( (leftTarget - leftLaneCenterX) / offsetToServoFactor );

		if (debugOutput)
			cout << ", leftServo=" << leftTarget << endl;
	}

	int rightTarget = laneState[RIGHT_LANE].calcXTarget(targetScanline, debugOutput);
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

	if (leftTarget || rightTarget)
	{
		if (leftTarget && rightTarget)
			servo = (leftTarget + rightTarget) / 2;
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
