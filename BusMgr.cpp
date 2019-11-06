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

	// Perform edge detection using simple linear filter.
	// Left lane and right lane edges handled independently.
	// Sweep right to left for left lane edges, and vice versa for right lane edges.
	// Apply suppression after an edge is detected as a simplistic "edge thinning".

	const int filter[] = {2, 1, 0, -1, -2}; // Not directly used due to optimizations.
	const int edgeBuffer = 3; // Ignore a few pixels on far left and far right edges of image.
	const int suppressCount = 10; // Skip several pixels after an edge is detected.
	const int startRow = 0;

	uchar * pRow;
	int gradient;

	// Negative and positive edge maps for left and right lanes respectively.
	// Edge maps are built so that edges resulting from neg->pos gradients can be culled.
	// (Neg->pos gradients can occur from shadows, etc., and should be ignored.  Actual lane markers should be brighter than road color, not darker.)
	uchar edgeMapNeg[ROI_HEIGHT][ROI_WIDTH] = {};
	uchar edgeMapPos[ROI_HEIGHT][ROI_WIDTH] = {};
	uchar edgeMapVert[ROI_HEIGHT][ROI_WIDTH] = {};

	// Actual edge coordinates after neg-->pos gradient culling - separate bins for left and right lanes.
	// Reserve enough for room to avoid ever needing to resize.
	vector<Vec2i> leftEdges;
	vector<Vec2i> rightEdges;
	leftEdges.reserve(450);
	rightEdges.reserve(450);

	// Mark left and right X position for sweeping kernel for linear filter.
	int xLeft = edgeBuffer;
	int xRight = frame.cols - edgeBuffer - (sizeof(filter) / sizeof(filter[0]));
	if (debugOutput)
		cout << "  xLeft=" << xLeft << ", xRight=" << xRight << endl;

	// Build edge maps.
	for (int y = startRow; y < frame.rows - 1; ++y)
	{
		pRow = frame.ptr(y);

		for (int x = xRight; x >= xLeft; --x)
		{
			gradient = (pRow[x] << 1) +
					   (pRow[x + 1]) -
					   (pRow[x + 3]) -
					   (pRow[x + 4] << 1);

			if (gradient >= m_config.gradientThreshold)
			{
				edgeMapNeg[y][x + 2] = 255;
				x -= suppressCount;
			}
		}

		for (int x = xLeft; x <= xRight; ++x)
		{
			gradient = -(pRow[x] << 1) -
					   (pRow[x + 1]) +
					   (pRow[x + 3]) +
					   (pRow[x + 4] << 1);

			if (gradient >= m_config.gradientThreshold)
			{
				edgeMapPos[y][x + 2] = 255;
				x += suppressCount;
			}
		}
	}

	// Mark top and bottom y position for sweeping kernel for vertical linear filter.
	const int yTop = 10 + edgeBuffer; // Ignore top 1/3 of image.
	int yBottom = frame.rows - edgeBuffer - (sizeof(filter) / sizeof(filter[0]));

	// Build vertical edge map.
	for (int x = 0; x < frame.cols - 1; ++x)
	{
		for (int y = yBottom; y >= yTop; --y)
		{
			gradient = (frame.data[ (y * frame.step) + x ] << 1) +
					   (frame.data[ ((y + 1) * frame.step) + x ]) -
					   (frame.data[ ((y + 3) * frame.step) + x ]) -
					   (frame.data[ ((y + 4) * frame.step) + x ] << 1);

			if (gradient >= m_config.gradientThreshold)
			{
				edgeMapVert[y + 2][x] = 255;
				y -= suppressCount;
			}
		}
	}

	// Left lane edge detection with culling of neg->pos gradients.
	for (int y = startRow; y < ROI_HEIGHT; ++y)
	{
		for (int x = (xLeft + 3); x <= (xRight + 1); ++x)  // Narrow by one to mitigate "compression" of edges along far left and right.
		{
			if (edgeMapNeg[y][x])
			{
				// Perform culling if necessary.
				int maxOffset = min(10, (xRight + 1 - x));
				int i;
				for (i = 1; i <= maxOffset; ++i)
				{
					if (edgeMapPos[y][x + i])
					{
						edgeMapPos[y][x + i] = 0;

						for (int j = 0; j <= i; ++j)
							edgeMapVert[y][x + j] = 0;

						break;
					}
				}

				if (i > maxOffset)
				{
					leftEdges.push_back(Vec2i(x, y));
				}
			}
		}
	}

	// Right lane edge detection - culling was already complete.
	for (int y = startRow; y < ROI_HEIGHT; ++y)
	{
		for (int x = (xLeft + 3); x <= (xRight + 1); ++x)
		{
			if (edgeMapPos[y][x])
			{
				rightEdges.push_back(Vec2i(x, y));
			}
		}
	}

	// Add vertical gradient edges to both lane edge vectors.
	for (int x = 0; x < frame.cols - 1; ++x)
	{
		for (int y = yBottom + 1; y >= yTop + 3; --y)
		{
			if (edgeMapVert[y][x])
			{
				leftEdges.push_back(Vec2i(x, y));
				rightEdges.push_back(Vec2i(x, y));
			}
		}
	}

	// Show detected edges on image for diagnostics purposes.
	// Bounce between left and right edges.
	//static int diagDisplayCount = 0;
	//if (++diagDisplayCount == 10)
	//	diagDisplayCount = 0;

	//if (diagDisplayCount < 5)
	//{
		for (Vec2i & edge : leftEdges)
		{
			frame.at<uchar>(edge[1], edge[0], 0) = 128;
		}
	//}
	//else
	//{
		for (Vec2i & edge : rightEdges)
		{
			frame.at<uchar>(edge[1], edge[0], 0) = 192;
		}
	//}

	// Perform lane transform and adapt target X values for each lane to target servo positions.
	const int maxServoDelta = 25;
	const int searchBuffer = 60;

	int leftTarget = 0;
	LaneInfo leftLaneInfo;
	//int leftAngle = 0;
	if (m_pLaneTransform->LaneSearch(leftEdges, LEFT_LANE, m_searchRange[LEFT_LANE], leftLaneInfo, debugOutput))
	{
		leftTarget = TranslateXTargetToServo(LEFT_LANE, leftLaneInfo.xTarget);
		m_pLaneTransform->RenderLane(frame, leftLaneInfo);

		if ( m_lastServo && (abs(m_lastServo - leftTarget) > maxServoDelta) )
		{
			if (debugOutput)
				cout << "  DEBUG: maxServoDelta exceeded! lastServo=" << m_lastServo << ", leftTarget=" << leftTarget << endl;

			leftTarget = 0;
			m_lockedLanes[LEFT_LANE].deactivate();
		}
		else
		{
			m_searchRange[LEFT_LANE][0] = leftLaneInfo.xTarget + searchBuffer;
			if (m_searchRange[LEFT_LANE][0] >= ROI_WIDTH)
				m_searchRange[LEFT_LANE][0] = ROI_WIDTH - 1;

			m_lockedLanes[LEFT_LANE] = leftLaneInfo;
		}

		//leftAngle = m_pLaneTransform->GetLaneAngle(leftLaneInfo.laneId);
	}
	else
		m_lockedLanes[LEFT_LANE].deactivate();

	int rightTarget = 0;
	LaneInfo rightLaneInfo;
	//int rightAngle = 0;
	if (m_pLaneTransform->LaneSearch(rightEdges, RIGHT_LANE, m_searchRange[RIGHT_LANE], rightLaneInfo, debugOutput))
	{
		rightTarget = TranslateXTargetToServo(RIGHT_LANE, rightLaneInfo.xTarget);
		m_pLaneTransform->RenderLane(frame, rightLaneInfo);

		if ( m_lastServo && (abs(m_lastServo - rightTarget) > maxServoDelta) )
		{
			if (debugOutput)
				cout << "  DEBUG: maxServoDelta exceeded! lastServo=" << m_lastServo << ", rightTarget=" << rightTarget << endl;

			rightTarget = 0;
			m_lockedLanes[RIGHT_LANE].deactivate();
		}
		else
		{
			m_searchRange[RIGHT_LANE][0] = rightLaneInfo.xTarget - searchBuffer;
			if (m_searchRange[RIGHT_LANE][0] < 0)
				m_searchRange[RIGHT_LANE][0] = 0;

			m_lockedLanes[RIGHT_LANE] = rightLaneInfo;
		}

		//rightAngle = m_pLaneTransform->GetLaneAngle(rightLaneInfo.laneId);
	}
	else
		m_lockedLanes[RIGHT_LANE].deactivate();

	/*
	if ( abs(leftAngle - rightAngle) >= 96 )
	{
		if (leftLaneInfo.votes > rightLaneInfo.votes)
			rightTarget = 0;
		else
			leftTarget = 0;
	}
	*/

	/*
	static int lastServo = 0;
	if (leftTarget && rightTarget)
	{
		//const int closeFactor = 20;
		//const int closeCutoff = (ROI_WIDTH >> 1) + 10;
		//int leftCloserX, rightCloserX;
		if (leftLaneInfo.xTarget >= rightLaneInfo.xTarget)
		{
			if (lastServo)
			{
				if (debugOutput)
					cout << "  Suppressed lane; lastServo=" << lastServo << ", leftTarget=" << leftTarget << ", rightTarget=" << rightTarget << endl;

				if ( abs(leftTarget - lastServo) > abs(rightTarget - lastServo) )
					leftTarget = 0;
				else
					rightTarget = 0;
			}
			/*
			// Probably picking up two sides of one lane - decide whether we're to the right or left of it.
			leftCloserX = leftLaneInfo.xTarget - (m_pLaneTransform->GetLaneSlope(leftLaneInfo.laneId) * closeFactor);
			rightCloserX = rightLaneInfo.xTarget - (m_pLaneTransform->GetLaneSlope(rightLaneInfo.laneId) * closeFactor);

			if ( ((leftCloserX + rightCloserX) / 2) < closeCutoff )
			{
				// Bus is to the right, so left lane is correct - get rid of right lane.
				rightTarget = 0;
			}
			else
			{
				// Bus is to the left...
				leftTarget = 0;
			}

			if (debugOutput)
				cout << "  Suppressed lane; leftCloserX=" << leftCloserX << ", rightCloserX=" << rightCloserX << endl;
		}
	}
	*/

	static int servo = ControlMgr::cDefServo;
	//const int targetDiffThreshold = 10; // If left and right lanes disagree strongly, ignore the one furthest from default (straight) position.
	//int expectTarget = ControlMgr::cDefServo + ( (leftAngle + rightAngle) / 4 );

	FDRecord & currFDR = m_FDRecords[m_currFDRIndex];
	currFDR.lastServo = m_lastServo;

	if (leftTarget || rightTarget)
	{
		if (leftTarget && rightTarget)
		{
			/*
			if ( abs(leftTarget - rightTarget) > targetDiffThreshold )
			{
				if ( abs(leftTarget - expectTarget) > abs(rightTarget - expectTarget) )
					servo = rightTarget;
				else
					servo = leftTarget;
			}
			else
			*/

			servo = (leftTarget + rightTarget) / 2;
		}
		else if (leftTarget)
			servo = leftTarget;
		else
			servo = rightTarget;

		m_lastServo = servo;

		if ( servo < (ControlMgr::cDefServo - ControlMgr::cServoRange) )
			servo = ControlMgr::cDefServo - ControlMgr::cServoRange;
		else if ( servo > (ControlMgr::cDefServo + ControlMgr::cServoRange) )
			servo = ControlMgr::cDefServo + ControlMgr::cServoRange;
	}
	//else
		//lastServo = 0;

	if (debugOutput)
	{
		cout << "  Search start left: " << m_searchRange[LEFT_LANE][0] << ", right: " << m_searchRange[RIGHT_LANE][0] << endl;
		cout << "  Target: " << leftTarget << ", " << rightTarget << "; Servo: " << servo << endl;
	}

	currFDR.frame = frame;
	currFDR.target[LEFT_LANE] = leftTarget;
	currFDR.target[RIGHT_LANE] = rightTarget;
	currFDR.searchStart[LEFT_LANE] = m_searchRange[LEFT_LANE][0];
	currFDR.searchStart[RIGHT_LANE] = m_searchRange[RIGHT_LANE][0];
	currFDR.servo = servo;
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
	const int centerX[MAX_LANES] = { 31, 143 };

	// TODO: Try making servo factor adaptive (inverse relationship with speed).
	const float offsetToServoFactor = 2.0f; // 3.0f;

	return ControlMgr::cDefServo + static_cast<int>( (xTarget - centerX[lane]) / offsetToServoFactor );
}
