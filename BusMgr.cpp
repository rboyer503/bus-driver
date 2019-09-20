/*
 * BusMgr.cpp
 *
 *  Created on: Jan 15, 2015
 *      Author: rboyer
 */
#include <algorithm>
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

bool g_outputLines = false;

#define PROFILE_START g_start = boost::posix_time::microsec_clock::local_time();

#define PROFILE_LOG(tag) \
	do { \
		g_diff = boost::posix_time::microsec_clock::local_time() - g_start; \
		cout << "Profile (" << #tag << "): " << g_diff.total_microseconds() << " us" << endl; \
	} while (0)

#define PROFILE_DIFF (boost::posix_time::microsec_clock::local_time() - g_start).total_microseconds()


bool lineSortFunction(Vec4i i, Vec4i j) { return (i[1] < j[1]); }


const char * const BusMgr::c_imageProcModeNames[] = {"None", "Gray", "Blur", "Canny", "Road", "Hough"};
const char * const BusMgr::c_imageProcStageNames[] = {"Gray", "Blur", "Canny", "Road", "Hough", "Post", "Send", "Total"};

BusMgr::BusMgr() :
	m_errorCode(EC_NONE), m_ipm(IPM_BLUR), m_paramPage(PP_BLUR), m_running(false), m_interrupted(false)
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
	cout << "  Current lanes:" << endl;
	for (int laneIndex = 0; laneIndex < NUM_LANES; ++laneIndex)
	{
		if (m_lanes[laneIndex].weight > 0)
		{
			cout << "    " << laneIndex << ": weight=" << (int)m_lanes[laneIndex].weight <<
					", m=" << m_lanes[laneIndex].m << ", xBase=" << m_lanes[laneIndex].line[2] << endl;
		}
	}
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
	cout << "  Canny Low Threshold=" << (int)m_config.cannyThresholdLow;
	cout << "\tCanny Threshold Ratio=" << m_config.cannyThresholdFactor << endl;
	cout << "  Hough Rho=" << (int)m_config.houghRho;
	cout << "\tHough Theta=" << (int)m_config.houghTheta;
	cout << "\tHough Threshold=" << (int)m_config.houghThreshold << endl;
	cout << "  Hough Min Line Length=" << (int)m_config.houghMinLineLength;
	cout << "\tHough Max Line Gap=" << (int)m_config.houghMaxLineGap << endl;
	cout << "  Lane Stitch Threshold=" << (int)m_config.laneStitchThreshold;
	cout << "\tLane Segregation Threshold=" << (int)m_config.laneSegregationThreshold << endl;
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
	case PP_CANNYTHRESHOLD:
		if (param == 1)
		{
			if ( up && (m_config.cannyThresholdLow < 200) )
				m_config.cannyThresholdLow += 5;
			else if (!up && (m_config.cannyThresholdLow > 20))
				m_config.cannyThresholdLow -= 5;
		}
		else if (param == 2)
		{
			if ( up && (m_config.cannyThresholdFactor < 3.5) )
				m_config.cannyThresholdFactor += 0.1;
			else if (!up && (m_config.cannyThresholdFactor > 1.5))
				m_config.cannyThresholdFactor -= 0.1;
		}
		break;
	case PP_HOUGHGRANULARITY:
		if (param == 1)
		{
			if ( up && (m_config.houghRho < 100) )
				m_config.houghRho++;
			else if (!up && (m_config.houghRho > 1))
				m_config.houghRho--;
		}
		else if (param == 2)
		{
			if ( up && (m_config.houghTheta < 100) )
				m_config.houghTheta++;
			else if (!up && (m_config.houghTheta > 1))
				m_config.houghTheta--;
		}
		break;
	case PP_HOUGHTHRESHOLD:
		if ( up && (m_config.houghThreshold < 200) )
			m_config.houghThreshold++;
		else if (!up && (m_config.houghThreshold > 2))
			m_config.houghThreshold--;
		break;
	case PP_HOUGHLINEPARAMS:
		if (param == 1)
		{
			if ( up && (m_config.houghMinLineLength < 100) )
				m_config.houghMinLineLength++;
			else if (!up && (m_config.houghMinLineLength > 0))
				m_config.houghMinLineLength--;
		}
		else if (param == 2)
		{
			if ( up && (m_config.houghMaxLineGap < 200) )
				m_config.houghMaxLineGap++;
			else if (!up && (m_config.houghMaxLineGap > 1))
				m_config.houghMaxLineGap--;
		}
		break;
	case PP_LANETHRESHOLD:
		if (param == 1)
		{
			if ( up && (m_config.laneStitchThreshold < 100) )
				m_config.laneStitchThreshold++;
			else if (!up && (m_config.laneStitchThreshold > 1))
				m_config.laneStitchThreshold--;
		}
		else if (param == 2)
		{
			if ( up && (m_config.laneSegregationThreshold < 100) )
				m_config.laneSegregationThreshold++;
			else if (!up && (m_config.laneSegregationThreshold > 1))
				m_config.laneSegregationThreshold--;
		}
		break;
	}
}

void BusMgr::DebugCommand()
{
	g_outputLines = true;
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

		// Clear lane data from last client.
		for (int i = 0; i < NUM_LANES; ++i)
			m_lanes[i] = Lane();
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

	const int cRoadEdgeBuffer = 5;

	Mat * pFrameDisplay = NULL;
	Mat frameROI;
	Mat frameGray;
	Mat frameFilter;
	Mat frameCanny;
	Mat frameRoad;
	Mat frameHough;
	eBDImageProcMode ipm = m_ipm;
	int processUs[IPS_MAX];
	memset(processUs, 0, sizeof(processUs));
	int servo = 160;

	if (ipm == IPM_NONE)
	{
		pFrameDisplay = &frame;
	}
	else
	{
		// Process
		cvtColor(frame, frameGray, CV_BGR2GRAY);
                flip(frameGray, frameGray, -1);
		//PROFILE_LOG(gray);
		processUs[IPS_GRAY] = PROFILE_DIFF;
		PROFILE_START;

		//frameROI = frameGray(Rect(0, 144, 320, 96));
		frameROI = frameGray(Rect(0, 96, 320, 120));
		//frameROI = frameGray(Rect(0, 144, 320, 70));

		if (ipm == IPM_GRAY)
		{
			pFrameDisplay = &frameGray;
		}
		else
		{
			//bilateralFilter(frameROI, frameFilter, 3, 3, 3);
			GaussianBlur(frameROI, frameFilter, Size(m_config.kernelSize, m_config.kernelSize), 0, 0);
			//PROFILE_LOG(blur);
			processUs[IPS_BLUR] = PROFILE_DIFF;
			PROFILE_START;

			if (ipm == IPM_BLUR)
			{
				pFrameDisplay = &frameFilter;

				// Testing simplified, custom lane detection algorithm.
				const int edgeBuffer = 3;
				const int contThreshold = 10; // Continuation threshold: if x value diverges more than this in one scanline, treat as different line.

				enum eLane { LEFT_LANE, RIGHT_LANE, MAX_LANES };

				vector<Vec4i> lines[MAX_LANES];
				Vec4i currLine[MAX_LANES];
				currLine[LEFT_LANE][0] = currLine[RIGHT_LANE][0] = -1;
				const int filter[] = {2, 1, 0, -1, -2};

				for (int y = (frameFilter.rows - 1); y > 0; --y)
				{
					uchar * pROIRow = frameFilter.ptr(y);

					int x;
					int gradient;

					// Search for left lane markings.
					x = (frameFilter.cols >> 1) - m_config.kernelSize;
					do
					{
						gradient = (pROIRow[x] * filter[0]) +
								   (pROIRow[x + 1] * filter[1]) +
								   (pROIRow[x + 3] * filter[3]) +
								   (pROIRow[x + 4] * filter[4]);
					}
					while ( (gradient < m_config.laneSegregationThreshold) && (--x > edgeBuffer) );

					// -- Found something exceeding threshold on this scanline?
					if (gradient >= m_config.laneSegregationThreshold)
					{
						// Extension of current line?
						if (currLine[LEFT_LANE][0] >= 0)
						{
							if ( abs(currLine[LEFT_LANE][2] - x) < contThreshold )
							{
								currLine[LEFT_LANE][2] = x;
								currLine[LEFT_LANE][3] = y;
							}
							else
							{
								// Record current line.
								if (abs(currLine[LEFT_LANE][3] - currLine[LEFT_LANE][1]) >= m_config.houghMinLineLength)
									lines[LEFT_LANE].push_back(currLine[LEFT_LANE]);
								currLine[LEFT_LANE][0] = -1;
							}
						}
						else
						{
							// New line:
							currLine[LEFT_LANE][0] = currLine[LEFT_LANE][2] = x;
							currLine[LEFT_LANE][1] = currLine[LEFT_LANE][3] = y;
						}
					}
					else if (currLine[LEFT_LANE][0] >= 0)
					{
						// Record current line.
						if (abs(currLine[LEFT_LANE][3] - currLine[LEFT_LANE][1]) >= m_config.houghMinLineLength)
							lines[LEFT_LANE].push_back(currLine[LEFT_LANE]);
						currLine[LEFT_LANE][0] = -1;
					}

					// Search for right lane markings.
					x = (frameFilter.cols >> 1);
					do
					{
						gradient = (pROIRow[x] * filter[4]) +
								   (pROIRow[x + 1] * filter[3]) +
								   (pROIRow[x + 3] * filter[1]) +
								   (pROIRow[x + 4] * filter[0]);
					}
					while ( (gradient < m_config.laneSegregationThreshold) && (++x <= (320 - m_config.kernelSize - edgeBuffer)) );

					// -- Found something exceeding threshold on this scanline?
					if (gradient >= m_config.laneSegregationThreshold)
					{
						// Extension of current line?
						if (currLine[RIGHT_LANE][0] >= 0)
						{
							if ( abs(currLine[RIGHT_LANE][2] - x) < contThreshold )
							{
								currLine[RIGHT_LANE][2] = x;
								currLine[RIGHT_LANE][3] = y;
							}
							else
							{
								// Record current line.
								if (abs(currLine[RIGHT_LANE][3] - currLine[RIGHT_LANE][1]) >= m_config.houghMinLineLength)
									lines[RIGHT_LANE].push_back(currLine[RIGHT_LANE]);
								currLine[RIGHT_LANE][0] = -1;
							}
						}
						else
						{
							// New line:
							currLine[RIGHT_LANE][0] = currLine[RIGHT_LANE][2] = x;
							currLine[RIGHT_LANE][1] = currLine[RIGHT_LANE][3] = y;
						}
					}
					else if (currLine[RIGHT_LANE][0] >= 0)
					{
						// Record current line.
						if (abs(currLine[RIGHT_LANE][3] - currLine[RIGHT_LANE][1]) >= m_config.houghMinLineLength)
							lines[RIGHT_LANE].push_back(currLine[RIGHT_LANE]);
						currLine[RIGHT_LANE][0] = -1;
					}
				}

				// Pick up last lines.
				if (currLine[LEFT_LANE][0] >= 0)
				{
					// Record current line.
					if (abs(currLine[LEFT_LANE][3] - currLine[LEFT_LANE][1]) >= m_config.houghMinLineLength)
						lines[LEFT_LANE].push_back(currLine[LEFT_LANE]);
				}
				if (currLine[RIGHT_LANE][0] >= 0)
				{
					// Record current line.
					if (abs(currLine[RIGHT_LANE][3] - currLine[RIGHT_LANE][1]) >= m_config.houghMinLineLength)
						lines[RIGHT_LANE].push_back(currLine[RIGHT_LANE]);
				}

				// DEBUG CODE: Output raw line data after stats.
				bool outputLines = false;
				if (g_outputLines)
				{
					g_outputLines = false;
					outputLines = true;
					cout << endl;
				}

				static int intensity[3] = {150, 125, 100};
				int colorIndex = 0;

				int leftTarget = 0;
				int rightTarget = 0;

				bool tracking = false;
				bool targetFound = false;
				float m, b;
				float mTarget, bTarget;
				const int targetScanline = 60;
				const int leftLaneCenterX = 70;
				const int rightLaneCenterX = 270;
				const float offsetToServoFactor = 3.0f;

				for (size_t i = 0; i < lines[LEFT_LANE].size(); ++i)
				{
					Vec4i l = lines[LEFT_LANE][i];

					// Pre-process lines to avoid inf/-inf slope.
					if (l[0] == l[2])
						l[0]++;

					// y = m * x + b
					// x = (y - b) / m
					// where m = (y2 - y1) / (x2 - x1) and b = y1 - m * x1.
					if (!tracking)
					{
						tracking = true;
						m = ((float)(l[3] - l[1])) / (l[2] - l[0]);
						b = l[1] - (m * l[0]);
						colorIndex = 0;

						if ( !targetFound && (l[3] < targetScanline) )
						{
							mTarget = m;
							bTarget = b;
							targetFound = true;
						}
					}
					else
					{
						if ( !targetFound && (l[1] < targetScanline) )
						{
							mTarget = m;
							bTarget = b;
							targetFound = true;
						}

						int xExpect = (int)( (l[1] - b) / m );
						if ( (xExpect - l[0]) < (int)m_config.laneStitchThreshold )
						{
							// Line not unexpectedly too far left - treat as lane continuation.
							m = ((float)(l[3] - l[1])) / (l[2] - l[0]);
							b = l[1] - (m * l[0]);
							colorIndex = 0;

							if ( !targetFound && (l[3] < targetScanline) )
							{
								mTarget = m;
								bTarget = b;
								targetFound = true;
							}
						}
						else
							colorIndex = 2;
					}

					line(frameFilter, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(intensity[colorIndex], intensity[colorIndex], intensity[colorIndex]), 2);

					if (outputLines)
					{
						cout << "Line " << i << ": (" << l[0] << ", " << l[1] << ") (" <<
								l[2] << ", " << l[3] << ")" << endl;
					}
				}

				if (targetFound)
				{
					float offset = ( static_cast<int>( (targetScanline - bTarget) / mTarget ) - leftLaneCenterX ) / offsetToServoFactor;
					leftTarget = ControlMgr::cDefServo + static_cast<int>(offset);

					if (outputLines)
						cout << "Left lane target: " << static_cast<int>( (targetScanline - bTarget) / mTarget ) << endl;
				}

				tracking = targetFound = false;
				for (size_t i = 0; i < lines[RIGHT_LANE].size(); ++i)
				{
					Vec4i l = lines[RIGHT_LANE][i];

					// Pre-process lines to avoid inf/-inf slope.
					if (l[0] == l[2])
						l[0]++;

					// y = m * x + b
					// x = (y - b) / m
					// where m = (y2 - y1) / (x2 - x1) and b = y1 - m * x1.
					if (!tracking)
					{
						tracking = true;
						m = ((float)(l[3] - l[1])) / (l[2] - l[0]);
						b = l[1] - (m * l[0]);
						colorIndex = 0;

						if ( !targetFound && (l[3] < targetScanline) )
						{
							mTarget = m;
							bTarget = b;
							targetFound = true;
						}
					}
					else
					{
						if ( !targetFound && (l[1] < targetScanline) )
						{
							mTarget = m;
							bTarget = b;
							targetFound = true;
						}

						int xExpect = (int)( (l[1] - b) / m );
						if ( (l[0] - xExpect) < (int)m_config.laneStitchThreshold )
						{
							// Line not unexpectedly too far right - treat as lane continuation.
							m = ((float)(l[3] - l[1])) / (l[2] - l[0]);
							b = l[1] - (m * l[0]);
							colorIndex = 0;

							if ( !targetFound && (l[3] < targetScanline) )
							{
								mTarget = m;
								bTarget = b;
								targetFound = true;
							}
						}
						else
							colorIndex = 2;
					}

					line(frameFilter, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(intensity[colorIndex], intensity[colorIndex], intensity[colorIndex]), 2);

					if (outputLines)
					{
						cout << "Line " << i << ": (" << l[0] << ", " << l[1] << ") (" <<
								l[2] << ", " << l[3] << ")" << endl;
					}
				}

				if (targetFound)
				{
					float offset = ( static_cast<int>( (targetScanline - bTarget) / mTarget ) - rightLaneCenterX ) / offsetToServoFactor;
					rightTarget = ControlMgr::cDefServo + static_cast<int>(offset);

					if (outputLines)
						cout << "Right lane target: " << static_cast<int>( (targetScanline - bTarget) / mTarget ) << endl;
				}

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

				if (outputLines)
				{
					cout << "Target: " << leftTarget << ", " << rightTarget << "; Servo: " << servo << endl;
				}

				processUs[IPS_CANNY] = PROFILE_DIFF;
				PROFILE_START;
			}
			else
			{
				Canny(frameFilter, frameCanny,
					  (double)m_config.cannyThresholdLow,
					  (double)(m_config.cannyThresholdLow * m_config.cannyThresholdFactor));
				//PROFILE_LOG(canny);
				processUs[IPS_CANNY] = PROFILE_DIFF;
				PROFILE_START;

				if (ipm == IPM_CANNY)
				{
					pFrameDisplay = &frameCanny;
				}
				else
				{
					frameRoad = Mat::zeros(frameCanny.size(), CV_8UC1);

					int y;
					for (y = (frameCanny.rows - 1); y > 0; --y)
					{
						uchar * pCannyRow = frameCanny.ptr(y);
						uchar * pOutRow = frameRoad.ptr(y);

						int x1 = frameCanny.cols >> 1;
						while ( (x1 < (frameCanny.cols - 1)) && (!pCannyRow[x1]) )
							++x1;
						int x2 = frameCanny.cols >> 1;
						while ( (x2 > 0) && (!pCannyRow[x2]) )
							--x2;

						if ( (x1 - x2) < 10)
							continue;

						// Note: Ignore edges detected near edge of image
						if (x1 < (frameCanny.cols - 1 - cRoadEdgeBuffer))
						{
							pOutRow[x1] = 255;
						}

						if (x2 > cRoadEdgeBuffer)
						{
							pOutRow[x2] = 255;
						}
					}

					//PROFILE_LOG(road);
					processUs[IPS_ROAD] = PROFILE_DIFF;
					PROFILE_START;

					if (ipm == IPM_ROAD)
					{
						pFrameDisplay = &frameRoad;
					}
					else
					{
						pFrameDisplay = &frameHough;

						frameGray.copyTo(frameHough);
						int offset = frameHough.rows - frameRoad.rows;

						// Use probabilistic hough transform to get raw lines from binary road image.
						vector<Vec4i> lines;
						HoughLinesP(frameRoad, lines,
									m_config.houghRho, (m_config.houghTheta * (CV_PI / 180)),
									m_config.houghThreshold, m_config.houghMinLineLength,
									m_config.houghMaxLineGap);
						//PROFILE_LOG(hough);
						processUs[IPS_HOUGH] = PROFILE_DIFF;
						PROFILE_START;

						// DEBUG CODE: Output raw line data after stats.
						bool outputLines = false;
						if (g_outputLines)
						{
							g_outputLines = false;
							outputLines = true;
							cout << endl;
						}

						int colorIndex = 0;
						for (size_t i = 0; i < lines.size(); ++i)
						{
							static int intensity[3] =   {150, 125, 100};

							Vec4i l = lines[i];
							line(frameHough, Point(l[0], l[1] + offset), Point(l[2], l[3] + offset), Scalar(intensity[colorIndex], intensity[colorIndex], intensity[colorIndex]), 2);
							colorIndex = (colorIndex + 1) % 3;

							if (outputLines)
							{
								cout << "Line " << i << ": (" << l[0] << ", " << l[1] << ") (" <<
										l[2] << ", " << l[3] << ")" << endl;
							}
						}

						// Post-processing of lines.
						// This involves adapting raw line data into lane candidates, and then maintaining
						// a model of the relevant lanes.

						// First, get lane candidates from the raw lines.
						vector<LaneCandidate *> laneCandidates;
						BuildLaneCandidates(lines, (frameRoad.rows - 1), laneCandidates);

						vector<LaneCandidate *>::iterator iter;

						// DEBUG CODE: Output lane candidates after stats.
						for (iter = laneCandidates.begin(); iter != laneCandidates.end(); iter++)
						{
							Vec4i l = (*iter)->line;
							//line(frameHough, Point(l[0], l[1] + offset), Point(l[2], l[3] + offset), Scalar(75, 75, 75), 3);

							if (outputLines)
							{
								cout << "Lane Candidate: (" << l[0] << ", " << l[1] << ") (" <<
										l[2] << ", " << l[3] << ") m=" << (*iter)->m << endl;
							}
						}

						// Associate lane candidates with existing lanes, or create new ones.
						for (iter = laneCandidates.begin(); iter != laneCandidates.end(); iter++)
						{
							bool found = false;
							for (int laneIndex = 0; laneIndex < NUM_LANES; ++laneIndex)
							{
								if (m_lanes[laneIndex].weight > 0)
								{
									// Found a pre-existing lane.
									// Same as new one?
									if (abs((*iter)->line[2] - m_lanes[laneIndex].line[2]) < (int)m_config.laneSegregationThreshold)
									{
										// Yes - adjust pre-existing data.
										m_lanes[laneIndex].line = (*iter)->line;
										m_lanes[laneIndex].m = (*iter)->m;
										m_lanes[laneIndex].weight += 3;
										found = true;
										break;
									}
								}
							}

							if (!found)
							{
								for (int laneIndex = 0; laneIndex < NUM_LANES; ++laneIndex)
								{
									if (m_lanes[laneIndex].weight == 0)
									{
										m_lanes[laneIndex].line = (*iter)->line;
										m_lanes[laneIndex].m = (*iter)->m;
										m_lanes[laneIndex].weight = 3;
										cout << "New lane detected: m=" << m_lanes[laneIndex].m << ", xBase=" << m_lanes[laneIndex].line[2] << endl;
										break;
									}
								}
							}

							// Free lane candidate.
							delete (*iter);
						}

						// Perform lane weight adjustment.
						int xMid = frameRoad.cols >> 1;
						int leftLane = -1;
						int leftDistance = numeric_limits<int>::min();
						int rightLane = -1;
						int rightDistance = numeric_limits<int>::max();
						for (int laneIndex = 0; laneIndex < NUM_LANES; ++laneIndex)
						{
							if (m_lanes[laneIndex].weight > 0)
							{
								// Normal lane decay.
								if (m_lanes[laneIndex].weight > 1)
									m_lanes[laneIndex].weight -= 2;
								else
									m_lanes[laneIndex].weight--;

								// Lane weight cap.
								if (m_lanes[laneIndex].weight > MAX_LINE_WEIGHT)
									m_lanes[laneIndex].weight = MAX_LINE_WEIGHT;

								// Lane distance processing.
								// Find closest left and right lane.
								int distance = m_lanes[laneIndex].line[2] - xMid;
								if (distance < 0)
								{
									// Left lane.
									if (distance > leftDistance)
									{
										leftDistance = distance;
										leftLane = laneIndex;
									}
								}
								else
								{
									// Right lane.
									if (distance < rightDistance)
									{
										rightDistance = distance;
										rightLane = laneIndex;
									}
								}
							}
						}

						// Increase weight of closest lanes.
						if (leftLane != -1)
							if (m_lanes[leftLane].weight)
								m_lanes[leftLane].weight++;
						if (rightLane != -1)
							if (m_lanes[rightLane].weight)
								m_lanes[rightLane].weight++;

						// Identify "best" lanes (i.e.: those with highest weight).
						int bestLanes[2];
						bestLanes[0] = -1;
						bestLanes[1] = -1;

						for (int laneIndex = 0; laneIndex < NUM_LANES; ++laneIndex)
						{
							if (m_lanes[laneIndex].weight > 0)
							{
								if ( (bestLanes[0] == -1) || (m_lanes[laneIndex].weight >= m_lanes[bestLanes[0]].weight) )
								{
									bestLanes[1] = bestLanes[0];
									bestLanes[0] = laneIndex;
								}
								else if ( (bestLanes[1] == -1) || (m_lanes[laneIndex].weight >= m_lanes[bestLanes[1]].weight) )
								{
									bestLanes[1] = laneIndex;
								}
							}
						}

						float leftOffset = 0.0f;
						int leftTarget = 0;
						float rightOffset = 0.0f;
						int rightTarget = 0;

						// Display best lanes.
						for (int bestIndex = 0; bestIndex < 2; ++bestIndex)
						{
							if (bestLanes[bestIndex] != -1)
							{
								Lane lane = m_lanes[bestLanes[bestIndex]];
								if (lane.weight)
								{
									line(frameHough, Point(lane.line[0], lane.line[1] + offset), Point(lane.line[2], lane.line[3] + offset), Scalar(200, 200, 200), 3);

									if (lane.line[2] < 160)
									{
										// Left lane.
										leftOffset = (lane.line[0] - 135) / 3.0f;
										leftTarget = 160 + static_cast<int>(leftOffset);
									}
									else
									{
										// Right lane.
										rightOffset = (lane.line[0] - 226) / 3.0f;
										rightTarget = 160 + static_cast<int>(rightOffset);
									}
								}
							}
						}

						if (leftTarget || rightTarget)
						{
							if (leftTarget && rightTarget)
								servo = (leftTarget + rightTarget) / 2;
							else if (leftTarget)
								servo = leftTarget;
							else
								servo = rightTarget;

							if (servo < 130)
								servo = 130;
							else if (servo > 190)
								servo = 190;
						}

						//PROFILE_LOG(post);
						processUs[IPS_POST] = PROFILE_DIFF;
						PROFILE_START;
					}
				}
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

	//PROFILE_LOG(serial);

	// Transmit to client.
#ifdef ENABLE_SOCKET_MGR
	if (!m_pSocketMgr->SendFrame(&buf[0], buf.size()))
	{
		// Client probably disconnected - exit streaming loop and wait for a new connection.
		m_errorCode = EC_SENDFAIL;
		return false;
	}
#endif

	//PROFILE_LOG(sent);
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

void BusMgr::BuildLaneCandidates(std::vector<cv::Vec4i> & lines, int yBase, std::vector<LaneCandidate *> & laneCandidates)
{
	vector<LaneCandidate *>::iterator iter;

	// Sort lines.
	for (size_t i = 0; i < lines.size(); ++i)
	{
		if (lines[i][1] > lines[i][3])
		{
			swap(lines[i][0], lines[i][2]);
			swap(lines[i][1], lines[i][3]);
		}
	}
	sort(lines.begin(), lines.end(), lineSortFunction);

	// Stitch lines into lane candidates.
	for (size_t i = 0; i < lines.size(); ++i)
	{
		Vec4i l = lines[i];

		// Pre-process lines to avoid inf/-inf slope.
		if (l[0] == l[2])
			l[0]--;

		// y = m * x + b
		// x = (y - b) / m
		// where m = (y2 - y1) / (x2 - x1) and b = y1 - m * x1.
		float m = ((float)(l[3] - l[1])) / (l[2] - l[0]);

		// Skip lines that are too horizontal.
		if ( (m >= -0.25f) && (m <= 0.25f) )
			continue;

		float b = l[1] - (m * l[0]);

		// Check whether this is an extension of an existing lane candidate.
		for (iter = laneCandidates.begin(); iter != laneCandidates.end(); iter++)
		{
			// Compare current line with this lane candidate.
			int xExpect = (int)( (l[1] - (*iter)->b) / (*iter)->m );
			if (abs(xExpect - l[0]) < (int)m_config.laneStitchThreshold)
			{
				// Update existing line candidate.
				if (l[3] > (*iter)->line[3])
				{
					// Move lane candidate end point to end of new line segment.
					(*iter)->line[2] = l[2];
					(*iter)->line[3] = l[3];
					(*iter)->mEnd = m;
					(*iter)->bEnd = b;

					// Reevaluate lane candidate.
					Vec4i lTemp = (*iter)->line;
					if (lTemp[0] == lTemp[2])
					{
						lTemp[0]--;
						(*iter)->line[0]--;
					}
					(*iter)->m = ((float)(lTemp[3] - lTemp[1])) / (lTemp[2] - lTemp[0]);
					(*iter)->b = lTemp[1] - ((*iter)->m * lTemp[0]);
				}
				break;
			}
		}

		if (iter == laneCandidates.end())
		{
			// No existing lane candidate for this line - build new one.
			LaneCandidate * pLane = new LaneCandidate();
			pLane->m = pLane->mStart = pLane->mEnd = m;
			pLane->b = pLane->bStart = pLane->bEnd = b;
			pLane->line = l;
			laneCandidates.push_back(pLane);
		}
	}

	// All lane candidates found.
	// Now extend each lane candidate to reach from top to bottom of ROI.
	for (iter = laneCandidates.begin(); iter != laneCandidates.end(); iter++)
	{
		Vec4i l;
		l[0] = (int)( -(*iter)->bStart / (*iter)->mStart );
		l[1] = 0;
		l[2] = (int)( (yBase - (*iter)->bEnd) / (*iter)->mEnd );
		l[3] = yBase;
		if (l[0] == l[2])
			l[0]--;
		(*iter)->m = ((float)(l[3] - l[1])) / (l[2] - l[0]);
		(*iter)->line = l;
	}
}

void BusMgr::DisplayCurrentParamPage()
{
	cout << "Current parameter page: " << m_paramPage << endl;
	switch (m_paramPage)
	{
	case PP_BLUR:
		cout << "  1) Kernel Size" << endl;
		break;
	case PP_CANNYTHRESHOLD:
		cout << "  1) Canny Low Threshold" << endl;
		cout << "  2) Canny Threshold Ratio" << endl;
		break;
	case PP_HOUGHGRANULARITY:
		cout << "  1) Hough Rho" << endl;
		cout << "  2) Hough Theta" << endl;
		break;
	case PP_HOUGHTHRESHOLD:
		cout << "  1) Hough Threshold" << endl;
		break;
	case PP_HOUGHLINEPARAMS:
		cout << "  1) Hough Min Line Length" << endl;
		cout << "  2) Hough Max Line Gap" << endl;
		break;
	case PP_LANETHRESHOLD:
		cout << "  1) Lane Stitch Threshold" << endl;
		cout << "  2) Lane Segregation Theshold" << endl;
		break;
	}
}
