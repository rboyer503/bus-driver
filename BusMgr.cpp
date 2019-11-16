/*
 * BusMgr.cpp
 *
 *  Created on: Jan 15, 2015
 *      Author: rboyer
 */
#include <cstdlib>
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


template <typename T>
T clamp(const T& val, const T& lower, const T& upper)
{
	return max(lower, min(val, upper));
}


const char * const BusMgr::c_imageProcModeNames[] = {"None", "Gray", "Blur", "Lane Assist", "FDR"};
const char * const BusMgr::c_imageProcStageNames[] = {"Gray", "Blur", "Lane Assist", "Send", "Total"};
const cv::Vec2i BusMgr::c_defaultRange[MAX_LANES] = { {ROI_WIDTH / 2, -26}, {ROI_WIDTH / 2, ROI_WIDTH + 26} };


BusMgr::BusMgr() :
	m_errorCode(EC_NONE), m_ipm(IPM_LANEASSIST), m_paramPage(PP_BLUR), m_running(false), m_interrupted(false), m_lastServo(0), m_debugTrigger(false),
	m_acceleration(0.0f), m_speed(0.0f)
{
	m_pSocketMgr = new SocketMgr(this);
	m_pCtrlMgr = new ControlMgr();
	m_pLaneTransform = new LaneTransform();

	for (int i = 0; i < MAX_LANES; ++i)
		m_searchRange[i] = c_defaultRange[i];
}

BusMgr::~BusMgr()
{
	delete m_pSocketMgr;
	delete m_pCtrlMgr;
	delete m_pLaneTransform;

	if (m_thread.joinable())
		m_thread.join();
}

bool BusMgr::Initialize()
{
	if (!m_pLaneTransform->Load())
	{
		m_errorCode = EC_LANETRANSFAIL;
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
		if ( up && (m_config.gradientThreshold < 1000) )
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

void BusMgr::ApplyAcceleration()
{
	const float resistanceFactor = 0.01f;
	float resistance = -m_speed * resistanceFactor;
	float effectiveAccel;
	{
		boost::mutex::scoped_lock lock(m_accelMutex);
		effectiveAccel = m_acceleration + resistance;
	}

	m_speed += effectiveAccel;

	int speed = abs(static_cast<int>(m_speed));
	if (speed > 1000)
		speed = 1000;
	else if (speed < 10)
		speed = 0;

	SetReverse(m_speed < 0.0f);
	SetSpeed(speed);
}

void BusMgr::SwitchLane(eLane toLane)
{
	// TODO: Need mutex?
	// TODO: Revisit implementation due to lane assist algorithm modifications.
	if (toLane == LEFT_LANE)
	{
		if ( m_lockedLanes[LEFT_LANE].isActive() && (m_lockedLanes[LEFT_LANE].xTarget >= 20) )
		{
			m_searchRange[RIGHT_LANE][0] = m_lockedLanes[LEFT_LANE].xTarget - 20;
			m_lastServo = TranslateXTargetToServo(RIGHT_LANE, m_searchRange[RIGHT_LANE][0] + 10);

			cout << "  DEBUG: Lane shift left; new right lane search start: " << m_searchRange[RIGHT_LANE][0] << endl;
		}
	}
	else
	{
		if ( m_lockedLanes[RIGHT_LANE].isActive() && (m_lockedLanes[RIGHT_LANE].xTarget <= 140) )
		{
			m_searchRange[LEFT_LANE][0] = m_lockedLanes[RIGHT_LANE].xTarget + 20;
			m_lastServo = TranslateXTargetToServo(LEFT_LANE, m_searchRange[LEFT_LANE][0] - 10);

			cout << "  DEBUG: Lane shift right; new left lane search start: " << m_searchRange[LEFT_LANE][0] << endl;
		}
	}
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

		// Start accepting commands from client and create monitor worker thread.
		m_pSocketMgr->StartReadingCommands();
		m_pSocketMgr->StartMonitorThread();
#endif

		// Initialize video.
		VideoCaptureMgr vcMgr(this);
		if (!vcMgr.Initialize())
		{
			cerr << "Error: Failed to open video capture." << endl;
			m_errorCode = EC_CAPTUREOPENFAIL;
			break;
		}

		system("v4l2-ctl --set-ctrl=contrast=100");
		system("v4l2-ctl --set-ctrl=brightness=90");

		m_status = Status();

		DisplayCurrentParamPage();

		// Continually transmit frames to client.
		Mat * frame;
		int nextFrame = FRAME_SKIP;
		while (1)
		{
			//PROFILE_START;

			// Retrieve frame.
			int skippedFrames;
			do
			{
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

					// Not possible to "catch up" on backlog when running full speed - just move on.
					if (FRAME_SKIP == 1)
						nextFrame = 1;
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
	else if (ipm == IPM_DEBUG)
	{
		pFrameDisplay = ProcessDebugFrame();
	}
	else
	{
		// Convert to grayscale image, flip, and focus to ROI.
		resize(frame, frameResize, Size(), 0.5, 0.5, INTER_NEAREST);
		cvtColor(frameResize, frameGray, CV_BGR2GRAY);
		flip(frameGray, frameGray, -1);
		frameROI = frameGray(Rect(0, 72, ROI_WIDTH, ROI_HEIGHT));

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
				static bool laneAssistActive = false;
				if (m_pCtrlMgr->GetLaneAssist())
				{
					if (!laneAssistActive)
					{
						laneAssistActive = true;

						// Lane assist just activated - reset search ranges.
						for (int i = 0; i < MAX_LANES; ++i)
							m_searchRange[i] = c_defaultRange[i];
						m_lastServo = 0;
					}
				}
				else
					laneAssistActive = false;

				// Run lane assist algorithm to calculate servo position.
				servo = LaneAssistComputeServo(frameFilter);

				if (m_pCtrlMgr->GetLaneAssist())
				{
					m_pCtrlMgr->SetServo(servo);
				}

				processUs[IPS_LANEASSIST] = PROFILE_DIFF;
				PROFILE_START;

				pFrameDisplay = &frameFilter;
			}
		}
	}

	// Encode for wifi transmission.
	unique_ptr<vector<uchar> > pBuf = std::make_unique<vector<uchar> >();
	imencode(".bmp", *pFrameDisplay, *pBuf);

	// Transmit to client.
#ifdef ENABLE_SOCKET_MGR
	if (!m_pSocketMgr->SendFrame(std::move(pBuf)))
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

	// Perform edge detection using simple linear filters.
	// Lane markings are characterized by a positive gradient quickly followed by a negative gradient.
	const int filter[] = {2, 1, 0, -1, -2}; // Not directly used due to optimizations.
	//const int onLaneCountdownInit = 30; // Once positive gradient found, search for corresponding negative gradient for this many steps.
	const int stateDuration[ESS_MAX] = {30, 40, 30, 40};

	// Actual edge coordinates; separate bins for left and right lanes.
	// Reserve enough for room to avoid ever needing to resize.
	vector<Vec3i> edges[MAX_LANES];
	//vector<Vec3i> edges[RIGHT_LANE];
	edges[LEFT_LANE].reserve(450); // TODO: Tune this.
	edges[RIGHT_LANE].reserve(450);

	int laneWidthMap[MAX_LANES][ROI_WIDTH][ROI_HEIGHT] = {0};

	// Mark left and right X position for sweeping kernel for linear filter.
	int xLeft = 0;
	int xRight = frame.cols - (sizeof(filter) / sizeof(filter[0]));

	// Edge detection implementation.
	uchar * pRow;
	int gradient;
	eEdgeSearchState ess;
	int stateCountdown;
	int laneStart, laneEnd;
	int maxGradient;
	bool laneFound;

	for (int y = 0; y < frame.rows - 1; ++y)
	{
		pRow = frame.ptr(y);
		ess = ESS_SEARCH_POS_THRES;
		laneFound = false;

		for (int x = xLeft; x <= xRight; ++x)
		{
			gradient = (pRow[x] << 1) +
					   (pRow[x + 1]) -
					   (pRow[x + 3]) -
					   (pRow[x + 4] << 1);

			switch (ess)
			{
			case ESS_SEARCH_POS_THRES:
				if (-gradient >= m_config.gradientThreshold)
				{
					// Minimal positive gradient threshold crossed.
					ess = ESS_SEARCH_MAX_POS;
					stateCountdown = min(stateDuration[ess], (xRight - x));
					laneStart = x;
					maxGradient = gradient;
				}
				break;

			case ESS_SEARCH_MAX_POS:
				if (-gradient >= maxGradient)
				{
					// Found larger positive gradient.
					laneStart = x;
					maxGradient = gradient;
				}

				if ( (--stateCountdown == 0) || (gradient >= m_config.gradientThreshold) )
				{
					// State expired or negative gradient threshold found.
					ess = ESS_SEARCH_NEG_THRES;
					stateCountdown = stateDuration[ess];
					x = laneStart;
				}
				break;

			case ESS_SEARCH_NEG_THRES:
				if (gradient >= m_config.gradientThreshold)
				{
					// Minimal negative gradient threshold crossed.
					ess = ESS_SEARCH_MAX_NEG;
					stateCountdown = min(stateDuration[ess], (xRight - x));
					laneEnd = x;
					maxGradient = gradient;
					laneFound = true;
				}
				else if (--stateCountdown == 0)
				{
					// No negative gradient found.
					ess = ESS_SEARCH_POS_THRES;
				}
				break;

			case ESS_SEARCH_MAX_NEG:
				if (gradient >= maxGradient)
				{
					// Found larger negative gradient.
					laneEnd = x;
					maxGradient = gradient;
				}

				if ( (--stateCountdown == 0) || (-gradient >= m_config.gradientThreshold) )
				{
					// State expired or positive gradient threshold found.
					ess = ESS_SEARCH_POS_THRES;
					//edges[LEFT_LANE].push_back(Vec3i(laneEnd + 2, y, 0));
					//edges[RIGHT_LANE].push_back(Vec3i(laneStart + 2, y, 0));
					laneWidthMap[LEFT_LANE][laneEnd + 2][y] = (laneEnd - laneStart);
					laneWidthMap[RIGHT_LANE][laneStart + 2][y] = (laneEnd - laneStart);
					laneFound = false;
				}
				break;
			}
		}

		if (laneFound)
		{
			//edges[LEFT_LANE].push_back(Vec3i(laneEnd + 2, y, 0));
			//edges[RIGHT_LANE].push_back(Vec3i(laneStart + 2, y, 0));
			laneWidthMap[LEFT_LANE][laneEnd + 2][y] = (laneEnd - laneStart);
			laneWidthMap[RIGHT_LANE][laneStart + 2][y] = (laneEnd - laneStart);
			laneFound = false;
		}
	}

	// Mark top and bottom y position for sweeping kernel for vertical linear filter.
	const int yTop = 10; // Ignore top 1/3 of image.
	int yBottom = frame.rows - (sizeof(filter) / sizeof(filter[0]));

	// Edge detection (vertical gradients).
	const int angleThreshold = 30;
	static int lastAngle = 0;
	//if (abs(lastAngle) >= angleThreshold)
	{
		for (int x = 0; x < frame.cols - 1; ++x)
		{
			ess = ESS_SEARCH_NEG_THRES;
			laneFound = false;

			for (int y = yBottom; y >= yTop; --y)
			{
				// Searching for negative gradient (swapped for vertical edge detection).
				gradient = (frame.data[ (y * frame.step) + x ] << 1) +
						   (frame.data[ ((y + 1) * frame.step) + x ]) -
						   (frame.data[ ((y + 3) * frame.step) + x ]) -
						   (frame.data[ ((y + 4) * frame.step) + x ] << 1);

				switch (ess)
				{
				case ESS_SEARCH_NEG_THRES:
					if (gradient >= m_config.gradientThreshold)
					{
						// Minimal negative gradient threshold crossed.
						ess = ESS_SEARCH_MAX_NEG;
						stateCountdown = min(stateDuration[ess], (y - yTop));
						laneStart = y;
						maxGradient = gradient;
					}
					break;

				case ESS_SEARCH_MAX_NEG:
					if (gradient >= maxGradient)
					{
						// Found larger negative gradient.
						laneStart = y;
						maxGradient = gradient;
					}

					if ( (--stateCountdown == 0) || (-gradient >= m_config.gradientThreshold) )
					{
						// State expired or positive gradient threshold found.
						ess = ESS_SEARCH_POS_THRES;
						stateCountdown = stateDuration[ess];
						y = laneStart;
					}
					break;

				case ESS_SEARCH_POS_THRES:
					if (-gradient >= m_config.gradientThreshold)
					{
						// Minimal positive gradient threshold crossed.
						ess = ESS_SEARCH_MAX_POS;
						stateCountdown = min(stateDuration[ess], (y - yTop));
						laneEnd = y;
						maxGradient = gradient;
						laneFound = true;
					}
					else if (--stateCountdown == 0)
					{
						// No positive gradient found.
						ess = ESS_SEARCH_NEG_THRES;
					}
					break;

				case ESS_SEARCH_MAX_POS:
					if (-gradient >= maxGradient)
					{
						// Found larger positive gradient.
						laneEnd = y;
						maxGradient = gradient;
					}

					if ( (--stateCountdown == 0) || (gradient >= m_config.gradientThreshold) )
					{
						// State expired or negative gradient threshold found.
						ess = ESS_SEARCH_NEG_THRES;

						// Neg -> pos gradient found - track edge in either left or right lane bins.
						// Lane selection is based on previous frame's angle.
						/*
						if (lastAngle > 0)
							edges[LEFT_LANE].push_back(Vec3i(x, laneStart + 2, 0));
						else
							edges[RIGHT_LANE].push_back(Vec3i(x, laneStart + 2, 0));
						*/

						if (lastAngle > angleThreshold)
							laneWidthMap[LEFT_LANE][x][laneStart + 2] = (laneStart - laneEnd);
						else if (lastAngle < -angleThreshold)
							laneWidthMap[RIGHT_LANE][x][laneStart + 2] = (laneStart - laneEnd);
						else
						{
							laneWidthMap[LEFT_LANE][x][laneStart + 2] = (laneStart - laneEnd);
							laneWidthMap[RIGHT_LANE][x][laneStart + 2] = (laneStart - laneEnd);
						}

						laneFound = false;
					}
					break;
				}
			}

			if (laneFound)
			{
				// Neg -> pos gradient found - track edge in either left or right lane bins.
				// Lane selection is based on previous frame's angle.
				/*
				if (lastAngle > 0)
					edges[LEFT_LANE].push_back(Vec3i(x, laneStart + 2, 0));
				else
					edges[RIGHT_LANE].push_back(Vec3i(x, laneStart + 2, 0));
				*/

				if (lastAngle > angleThreshold)
					laneWidthMap[LEFT_LANE][x][laneStart + 2] = (laneStart - laneEnd);
				else if (lastAngle < -angleThreshold)
					laneWidthMap[RIGHT_LANE][x][laneStart + 2] = (laneStart - laneEnd);
				else
				{
					laneWidthMap[LEFT_LANE][x][laneStart + 2] = (laneStart - laneEnd);
					laneWidthMap[RIGHT_LANE][x][laneStart + 2] = (laneStart - laneEnd);
				}

				laneFound = false;
			}
		}
	}

	// Post-processing to establish "coherency factor" for edges.
	// Edge coherency refers to the existence of neighboring edges with similar lane widths.
	// Actual lane markings should typically have high coherency, whereas noise resulting from (e.g.) glare should have low coherency.
	const int laneWidthDiffThreshold = 2;
	int currLaneWidth;
	int coherency;
	for (int x = 1; x < ROI_WIDTH - 1; ++x)
	{
		for (int y = 1; y < ROI_HEIGHT - 1; ++y)
		{
			for (int lane = LEFT_LANE; lane < MAX_LANES; ++lane)
			{
				if ( (currLaneWidth = laneWidthMap[lane][x][y]) != 0 )
				{
					coherency = 0;
					if (abs(laneWidthMap[lane][x-1][y-1] - currLaneWidth) < laneWidthDiffThreshold)
						++coherency;
					if (abs(laneWidthMap[lane][x][y-1] - currLaneWidth) < laneWidthDiffThreshold)
						++coherency;
					if (abs(laneWidthMap[lane][x+1][y-1] - currLaneWidth) < laneWidthDiffThreshold)
						++coherency;
					if (abs(laneWidthMap[lane][x-1][y] - currLaneWidth) < laneWidthDiffThreshold)
						++coherency;
					if (abs(laneWidthMap[lane][x+1][y] - currLaneWidth) < laneWidthDiffThreshold)
						++coherency;
					if (abs(laneWidthMap[lane][x-1][y+1] - currLaneWidth) < laneWidthDiffThreshold)
						++coherency;
					if (abs(laneWidthMap[lane][x][y+1] - currLaneWidth) < laneWidthDiffThreshold)
						++coherency;
					if (abs(laneWidthMap[lane][x+1][y+1] - currLaneWidth) < laneWidthDiffThreshold)
						++coherency;

					edges[lane].push_back(Vec3i(x, y, coherency));
				}
			}
		}
	}


	// Show detected edges on image for diagnostics purposes.
	for (Vec3i & edge : edges[LEFT_LANE])
		frame.at<uchar>(edge[1], edge[0], 0) = 0; //128;

	for (Vec3i & edge : edges[RIGHT_LANE])
		frame.at<uchar>(edge[1], edge[0], 0) = 0; //192;

	// Perform lane transform and adapt target X values for each lane to target servo positions.
	const int searchBuffer = 40; // A little under half lane width.
	int leftTarget = 0;
	int totalAngle = 0;
	int activeCount = 0;
	if (m_pLaneTransform->LaneSearch(edges[LEFT_LANE], LEFT_LANE, m_searchRange[LEFT_LANE], m_lockedLanes[LEFT_LANE], debugOutput))
	{
		leftTarget = TranslateXTargetToServo(LEFT_LANE, m_lockedLanes[LEFT_LANE].xTarget);

		if (m_renderLanes)
			m_pLaneTransform->RenderLane(frame, m_lockedLanes[LEFT_LANE]);

		m_searchRange[LEFT_LANE] = Vec2i(m_lockedLanes[LEFT_LANE].xTarget + searchBuffer, m_lockedLanes[LEFT_LANE].xTarget - searchBuffer);
		TrimSearchRange(LEFT_LANE, m_searchRange[LEFT_LANE]);

		totalAngle += m_lockedLanes[LEFT_LANE].angle;
		++activeCount;
	}

	int rightTarget = 0;
	if (m_pLaneTransform->LaneSearch(edges[RIGHT_LANE], RIGHT_LANE, m_searchRange[RIGHT_LANE], m_lockedLanes[RIGHT_LANE], debugOutput))
	{
		rightTarget = TranslateXTargetToServo(RIGHT_LANE, m_lockedLanes[RIGHT_LANE].xTarget);

		if (m_renderLanes)
			m_pLaneTransform->RenderLane(frame, m_lockedLanes[RIGHT_LANE]);

		m_searchRange[RIGHT_LANE] = Vec2i(m_lockedLanes[RIGHT_LANE].xTarget - searchBuffer, m_lockedLanes[RIGHT_LANE].xTarget + searchBuffer);
		TrimSearchRange(RIGHT_LANE, m_searchRange[RIGHT_LANE]);

		totalAngle += m_lockedLanes[RIGHT_LANE].angle;
		++activeCount;
	}

	if (activeCount)
	{
		lastAngle = totalAngle / activeCount;

		if (activeCount == 1)
		{
			if (m_lockedLanes[LEFT_LANE].isActive())
			{
				m_searchRange[RIGHT_LANE][0] = m_searchRange[LEFT_LANE][0];
				m_searchRange[RIGHT_LANE][1] = c_defaultRange[RIGHT_LANE][1];
				TrimSearchRange(RIGHT_LANE, m_searchRange[RIGHT_LANE]);
			}
			else
			{
				m_searchRange[LEFT_LANE][0] = m_searchRange[RIGHT_LANE][0];
				m_searchRange[LEFT_LANE][1] = c_defaultRange[LEFT_LANE][1];
				TrimSearchRange(LEFT_LANE, m_searchRange[LEFT_LANE]);
			}
		}

		//m_config.gradientThreshold = maxGradient >> 2;
	}
	else
		lastAngle = 0;

	static int servo = ControlMgr::cDefServo;
	FDRecord & currFDR = m_FDRecords[m_currFDRIndex];
	currFDR.lastServo = m_lastServo;

	if (leftTarget || rightTarget)
	{
		if (leftTarget && rightTarget)
			servo = (leftTarget + rightTarget) / 2;
		else if (leftTarget)
			servo = leftTarget;
		else
			servo = rightTarget;

		m_lastServo = servo;
	}

	if (debugOutput)
	{
		cout << "  Search start left: " << m_searchRange[LEFT_LANE][0] << ", right: " << m_searchRange[RIGHT_LANE][0] << endl;
		//cout << "  Gradient threshold: " << static_cast<int>(m_config.gradientThreshold) << endl;
		cout << "  Target: " << leftTarget << ", " << rightTarget << "; Servo: " << servo << endl;
	}

	currFDR.frame = frame;
	currFDR.target[LEFT_LANE] = leftTarget;
	currFDR.target[RIGHT_LANE] = rightTarget;
	currFDR.searchStart[LEFT_LANE] = m_searchRange[LEFT_LANE][0];
	currFDR.searchStart[RIGHT_LANE] = m_searchRange[RIGHT_LANE][0];
	currFDR.servo = servo;
	currFDR.lastAngle = lastAngle;
	//currFDR.gradientThreshold = m_config.gradientThreshold;
	m_selectedFDRIndex = m_currFDRIndex++;
	if (m_currFDRIndex == c_maxFDRecords)
	{
		m_currFDRIndex = 0;
		m_FDRFull = true;
	}

	return servo;
}

Mat * BusMgr::ProcessDebugFrame()
{
	FDRecord & currFDR = m_FDRecords[m_selectedFDRIndex];
	if (m_updateFDR)
	{
		m_updateFDR = false;

		cout << "FDR #" << m_selectedFDRIndex << ":" << endl;
		cout << "  Left/right target = " << currFDR.target[LEFT_LANE] << "/" << currFDR.target[RIGHT_LANE] << endl;
		cout << "  Left/right search start = " << currFDR.searchStart[LEFT_LANE] << "/" << currFDR.searchStart[RIGHT_LANE] << endl;
		cout << "  Servo/last servo = " << currFDR.servo << "/" << currFDR.lastServo << endl;
		cout << "  Last angle = " << currFDR.lastAngle << endl;
		//cout << "  Gradient threshold = " << static_cast<int>(currFDR.gradientThreshold) << endl;
	}

	return &(currFDR.frame);
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

int BusMgr::TranslateXTargetToServo(eLane lane, int xTarget) const
{
	const int centerX[MAX_LANES] = { 40, 140 };

	// TODO: Try making servo factor adaptive (inverse relationship with speed).
	const float offsetToServoFactor = 2.0f; // 3.0f;

	int servo = ControlMgr::cDefServo + static_cast<int>( (xTarget - centerX[lane]) / offsetToServoFactor );

	if ( servo < (ControlMgr::cDefServo - ControlMgr::cServoRange) )
		servo = ControlMgr::cDefServo - ControlMgr::cServoRange;
	else if ( servo > (ControlMgr::cDefServo + ControlMgr::cServoRange) )
		servo = ControlMgr::cDefServo + ControlMgr::cServoRange;

	return servo;
}

void BusMgr::TrimSearchRange(eLane lane, cv::Vec2i & searchRange) const
{
	/*
	if (lane == LEFT_LANE)
	{
		searchRange[0] = clamp(searchRange[0], 0, ROI_WIDTH - 1);
		searchRange[1] = clamp(searchRange[1], c_defaultRange[LEFT_LANE][1], ROI_WIDTH - 1);
	}
	else
	{
		searchRange[0] = clamp(searchRange[0], 0, ROI_WIDTH - 1);
		searchRange[1] = clamp(searchRange[1], 0, c_defaultRange[RIGHT_LANE][1]);
	}
	*/

	searchRange[0] = clamp(searchRange[0], -26, ROI_WIDTH + 26);
	searchRange[1] = clamp(searchRange[1], -26, ROI_WIDTH + 26);

}
