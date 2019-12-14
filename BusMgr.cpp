/*
 * BusMgr.cpp
 *
 *  Created on: Jan 15, 2015
 *      Author: rboyer
 */
#include <cstdlib>
#include <fstream>
#include <vector>

#include "BusMgr.h"
#include "SocketMgr.h"
#include "VideoCaptureMgr.h"
#include "ControlMgr.h"

#define ENABLE_SOCKET_MGR 1

#define FRAME_SKIP 1
#define FRAME_BACKLOG_MIN -5

#define ROI_TOP 72
#define ROI_WIDTH 160
#define ROI_HEIGHT 30

#define DEF_KERNEL_SIZE 5
#define DEF_GRADIENT_THRESHOLD 40
#define DEF_RESISTANCE_FACTOR 0.01f
#define DEF_SEARCH_BUFFER 50
#define DEF_LANE_SWITCH_XOFFSET 10
#define DEF_HYSTERESIS_DURATION 10
#define DEF_EDGE_MAP_ANGLE_THRESHOLD 30
#define DEF_LANE_ANGLE_DIFF_MIN 0
#define DEF_LANE_ANGLE_DIFF_MAX 100
#define DEF_AUTO_PILOT_SPEED_CAP 800
#define DEF_AUTO_PILOT_SPEED_CAP_FACTOR 8
#define DEF_XOFFSET_SERVO_FACTOR 3.5f

// Parameters for lane transform.
#define DEF_ANGLE_DEVIATION_MAX 30
#define DEF_ANGLE_LIMIT 60
#define DEF_LANE_VOTE_THRESHOLD 20


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
const int BusMgr::c_centerX[MAX_LANES] = { 30, 141 };


BusMgr::BusMgr() :
		m_config(Config(DEF_KERNEL_SIZE, DEF_GRADIENT_THRESHOLD, DEF_RESISTANCE_FACTOR, DEF_SEARCH_BUFFER, DEF_LANE_SWITCH_XOFFSET, DEF_HYSTERESIS_DURATION,
					    DEF_EDGE_MAP_ANGLE_THRESHOLD, DEF_LANE_ANGLE_DIFF_MIN, DEF_LANE_ANGLE_DIFF_MAX, DEF_AUTO_PILOT_SPEED_CAP, DEF_AUTO_PILOT_SPEED_CAP_FACTOR,
					    DEF_XOFFSET_SERVO_FACTOR, DEF_ANGLE_DEVIATION_MAX, DEF_ANGLE_LIMIT, DEF_LANE_VOTE_THRESHOLD))
{
	m_pSocketMgr = new SocketMgr(this);
	m_pCtrlMgr = new ControlMgr();
	m_pLaneTransform = new LaneTransform();

	m_frameDouble = Mat(ROI_HEIGHT, ROI_WIDTH, CV_32SC1);
	m_frameGradient = Mat(ROI_HEIGHT, ROI_WIDTH, CV_32SC1, Scalar(0));
	m_frameGradientVert = Mat(ROI_HEIGHT, ROI_WIDTH, CV_32SC1, Scalar(0));

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
	cout << "  Resistance Factor=" << m_config.resistanceFactor << endl;
	cout << "  Search Buffer=" << m_config.searchBuffer << endl;
	cout << "  Lane Switch X-Offset=" << m_config.laneSwitchXOffset << endl;
	cout << "  Hysteresis Duration=" << m_config.hysteresisDuration << endl;
	cout << "  Edge Map Angle Threshold=" << m_config.edgeMapAngleThreshold << endl;
	cout << "  Lane Angle Diff Min/Max=" << m_config.laneAngleDiffMin << "/" << m_config.laneAngleDiffMax << endl;
	cout << "  Auto-pilot Speed Cap/Factor=" << m_config.autoPilotSpeedCap << "/" << m_config.autoPilotSpeedCapFactor << endl;
	cout << "  X-Offset Servo Factor=" << m_config.xOffsetServoFactor << endl;
	cout << "  Angle Deviation Max=" << m_config.angleDeviationMax << endl;
	cout << "  Angle Limit=" << m_config.angleLimit << endl;
	cout << "  Lane Vote Threshold=" << m_config.laneVoteThreshold << endl;
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
	case PP_RESISTANCEFACTOR:
		if ( up && (m_config.resistanceFactor < 0.5f) )
			m_config.resistanceFactor += 0.05f;
		else if (!up && (m_config.resistanceFactor > 0.0f))
			m_config.resistanceFactor -= 0.05f;
		break;
	case PP_SEARCHBUFFER:
		if ( up && (m_config.searchBuffer < 100) )
			++m_config.searchBuffer;
		else if (!up && (m_config.searchBuffer > 1))
			--m_config.searchBuffer;
		break;
	case PP_LANESWITCHXOFFSET:
		if ( up && (m_config.laneSwitchXOffset < 50) )
			++m_config.laneSwitchXOffset;
		else if (!up && (m_config.laneSwitchXOffset > 1))
			--m_config.laneSwitchXOffset;
		break;
	case PP_HYSTERESISDURATION:
		if ( up && (m_config.hysteresisDuration < 50) )
			++m_config.hysteresisDuration;
		else if (!up && (m_config.hysteresisDuration > 1))
			--m_config.hysteresisDuration;
		break;
	case PP_EDGEMAPANGLETHRESHOLD:
		if ( up && (m_config.edgeMapAngleThreshold < 90) )
			++m_config.edgeMapAngleThreshold;
		else if (!up && (m_config.edgeMapAngleThreshold > 1))
			--m_config.edgeMapAngleThreshold;
		break;
	case PP_LANEANGLEDIFF:
		if (param == 1)
		{
			if ( up && (m_config.laneAngleDiffMin < 90) )
				++m_config.laneAngleDiffMin;
			else if (!up && (m_config.laneAngleDiffMin > -90))
				--m_config.laneAngleDiffMin;
		}
		else
		{
			if ( up && (m_config.laneAngleDiffMax < 180) )
				++m_config.laneAngleDiffMax;
			else if (!up && (m_config.laneAngleDiffMax > 0))
				--m_config.laneAngleDiffMax;
		}
		break;
	case PP_AUTOPILOTSPEED:
		if (param == 1)
		{
			if ( up && (m_config.autoPilotSpeedCap < 1000) )
				m_config.autoPilotSpeedCap += 10;
			else if (!up && (m_config.autoPilotSpeedCap > 100))
				m_config.autoPilotSpeedCap -= 10;
		}
		else
		{
			if ( up && (m_config.autoPilotSpeedCapFactor < 30) )
				++m_config.autoPilotSpeedCapFactor;
			else if (!up && (m_config.autoPilotSpeedCapFactor > 0))
				--m_config.autoPilotSpeedCapFactor;
		}
		break;
	case PP_XOFFSETSERVOFACTOR:
		if ( up && (m_config.xOffsetServoFactor < 5.0f) )
			m_config.xOffsetServoFactor += 0.1f;
		else if (!up && (m_config.xOffsetServoFactor > 1.0f))
			m_config.xOffsetServoFactor -= 0.1f;
		break;
	case PP_ANGLEDEVIATIONMAX:
		if ( up && (m_config.angleDeviationMax < 90) )
			++m_config.angleDeviationMax;
		else if (!up && (m_config.angleDeviationMax > 1))
			--m_config.angleDeviationMax;
		break;
	case PP_ANGLELIMIT:
		if ( up && (m_config.angleLimit < 90) )
			++m_config.angleLimit;
		else if (!up && (m_config.angleLimit > 1))
			--m_config.angleLimit;
		break;
	case PP_LANEVOTETHRESHOLD:
		if ( up && (m_config.laneVoteThreshold < 100) )
			++m_config.laneVoteThreshold;
		else if (!up && (m_config.laneVoteThreshold > 1))
			--m_config.laneVoteThreshold;
		break;
	}
}

void BusMgr::DebugCommand()
{
	m_debugTrigger = true;
}

void BusMgr::ApplyAcceleration()
{
	float resistance = -m_speed * m_config.resistanceFactor;
	float effectiveAccel;
	int targetMaxSpeed;
	{
		boost::mutex::scoped_lock lock(m_accelMutex);
		effectiveAccel = m_acceleration + resistance;
		targetMaxSpeed = m_maxSpeed;
	}

	if ( targetMaxSpeed > (m_actualMaxSpeed + 2) )
		m_actualMaxSpeed += 2;
	else if ( targetMaxSpeed < (m_actualMaxSpeed - 2) )
		m_actualMaxSpeed -= 2;
	else
		m_actualMaxSpeed = targetMaxSpeed;

	m_speed += effectiveAccel;

	int speed = abs(static_cast<int>(m_speed));
	if (speed > m_actualMaxSpeed)
		speed = m_actualMaxSpeed;
	else if (speed < 10)
		speed = 0;

	SetReverse(m_speed < 0.0f);
	SetSpeed(speed);
}

void BusMgr::AdjustServo()
{
	//static int count = 0;
	//if (++count == 2)
	//{
	//	count = 0;
		if (m_pCtrlMgr->GetLaneAssist())
		{
			if (m_servoTarget > m_servoActual )
				++m_servoActual;
			else if (m_servoTarget < m_servoActual)
				--m_servoActual;

			m_pCtrlMgr->SetServo(m_servoActual);
		}
	//}
}

void BusMgr::SwitchLane(eLane toLane)
{
	boost::mutex::scoped_lock lock(m_laneStateMutex);

	if (toLane == LEFT_LANE)
	{
		if ( m_lockedLanes[LEFT_LANE].isActive() && (m_lockedLanes[LEFT_LANE].xTarget >= m_config.laneSwitchXOffset) )
		{
			m_searchRange[RIGHT_LANE] = Vec2i(m_lockedLanes[LEFT_LANE].xTarget - m_config.searchBuffer, m_lockedLanes[LEFT_LANE].xTarget + m_config.searchBuffer);
			TrimSearchRange(m_searchRange[RIGHT_LANE]);
			m_searchRange[LEFT_LANE] = Vec2i(m_lockedLanes[LEFT_LANE].xTarget - m_config.searchBuffer, m_lockedLanes[LEFT_LANE].xTarget - (m_config.searchBuffer * 3));
			TrimSearchRange(m_searchRange[LEFT_LANE]);
			cout << "  DEBUG: Lane shift left." << endl;
		}
	}
	else
	{
		if ( m_lockedLanes[RIGHT_LANE].isActive() && (m_lockedLanes[RIGHT_LANE].xTarget <= (ROI_WIDTH - m_config.laneSwitchXOffset)) )
		{
			m_searchRange[LEFT_LANE] = Vec2i(m_lockedLanes[RIGHT_LANE].xTarget + m_config.searchBuffer, m_lockedLanes[RIGHT_LANE].xTarget - m_config.searchBuffer);
			TrimSearchRange(m_searchRange[LEFT_LANE]);
			m_searchRange[RIGHT_LANE] = Vec2i(m_lockedLanes[RIGHT_LANE].xTarget + m_config.searchBuffer, m_lockedLanes[RIGHT_LANE].xTarget + (m_config.searchBuffer * 3));
			TrimSearchRange(m_searchRange[RIGHT_LANE]);
			cout << "  DEBUG: Lane shift right." << endl;
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

	static bool laneAssistActive = false;
	static bool autoPilotActive = false;

	Mat * pFrameDisplay = NULL;
	eBDImageProcMode ipm = m_ipm;
	int processUs[IPS_MAX];
	memset(processUs, 0, sizeof(processUs));

	if (ipm == IPM_NONE)
	{
		pFrameDisplay = &frame;
		laneAssistActive = false;
	}
	else if (ipm == IPM_DEBUG)
	{
		pFrameDisplay = ProcessDebugFrame();
		laneAssistActive = false;
	}
	else
	{
		// Convert to downsampled, grayscale image with correct orientation and focus to ROI.
		resize(frame, m_frameResize, Size(), 0.5, 0.5, INTER_NEAREST);
		cvtColor(m_frameResize, m_frameGray, CV_BGR2GRAY);
		flip(m_frameGray, m_frameGray, -1);
		m_frameROI = m_frameGray(Rect(0, ROI_TOP, ROI_WIDTH, ROI_HEIGHT));

		processUs[IPS_GRAY] = PROFILE_DIFF;
		PROFILE_START;

		if (ipm == IPM_GRAY)
		{
			pFrameDisplay = &m_frameROI;
			laneAssistActive = false;
		}
		else
		{
			// Apply gaussian blur.
			GaussianBlur(m_frameROI, m_frameFilter, Size(m_config.kernelSize, m_config.kernelSize), 0, 0);

			processUs[IPS_BLUR] = PROFILE_DIFF;
			PROFILE_START;

			if (ipm == IPM_BLUR)
			{
				pFrameDisplay = &m_frameFilter;
				laneAssistActive = false;
			}
			else
			{

				if (m_pCtrlMgr->GetLaneAssist())
				{
					if (!laneAssistActive)
					{
						laneAssistActive = true;

						// Lane assist just activated - reset search ranges.
						boost::mutex::scoped_lock lock(m_laneStateMutex);
						for (int i = 0; i < MAX_LANES; ++i)
							m_searchRange[i] = c_defaultRange[i];
						m_lastServo = 0;
					}
				}
				else
					laneAssistActive = false;

				if (m_pCtrlMgr->GetAutoPilot())
				{
					if (!autoPilotActive)
					{
						autoPilotActive = true;

						// Auto-pilot just activated - initialize acceleration.
						SetAcceleration(8.0f);
					}
				}
				else
				{
					if (autoPilotActive)
					{
						autoPilotActive = false;

						// Auto-pilot just disabled - drift to stop.
						SetAcceleration(0.0f);
					}
				}

				// Run lane assist algorithm to calculate servo position.
				int servo = LaneAssistComputeServo(m_frameFilter);

				if (m_pCtrlMgr->GetLaneAssist())
				{
					m_servoTarget = servo;
					//m_pCtrlMgr->SetServo(servo);
				}

				processUs[IPS_LANEASSIST] = PROFILE_DIFF;
				PROFILE_START;

				if (m_debugMode)
					pFrameDisplay = &m_frameFilter;
				else
				{
					flip(frame, frame, -1);
					pFrameDisplay = &frame;
				}
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

	// Calculate gradient maps, kernel = {-2, -1, 0, 1, 2}.
	// -- First precalculate frame * 2.
	frame.convertTo(m_frameDouble, CV_32SC1, 2);

	// -- Mark left and right X position for sweeping kernel for linear filter.
	const int linearFilterSize = 5;
	const int xLeft = 0;
	const int xRight = frame.cols - linearFilterSize;

	// -- Calculate horizontal gradients.
	uchar * pRow;
	int * pRowDouble;
	int * pRowGradient;
	for (int y = 0; y < m_frameGradient.rows; ++y)
	{
		pRow = frame.ptr(y);
		pRowDouble = m_frameDouble.ptr<int>(y);
		pRowGradient = m_frameGradient.ptr<int>(y);

		for (int x = xLeft; x <= xRight; ++x)
		{
			pRowGradient[x + 2] = -pRowDouble[x] -
								   pRow[x + 1] +
								   pRow[x + 3] +
								   pRowDouble[x + 4];
		}
	}

	// -- Mark top and bottom y position for sweeping kernel for vertical linear filter.
	const int yTop = 0;
	int yBottom = frame.rows - linearFilterSize;

	// -- Calculate vertical gradients.
	uchar * pRow2;
	int * pRowDouble2;
	for (int y = yTop; y <= yBottom; ++y)
	{
		pRowDouble = m_frameDouble.ptr<int>(y);
		pRow = frame.ptr(y + 1);
		pRow2 = frame.ptr(y + 3);
		pRowDouble2 = m_frameDouble.ptr<int>(y + 4);
		pRowGradient = m_frameGradientVert.ptr<int>(y + 2);

		for (int x = 0; x < m_frameGradientVert.cols; ++x)
		{
			pRowGradient[x] = pRowDouble[x] +
							  pRow[x] -
							  pRow2[x] -
							  pRowDouble2[x];
		}
	}

	// Perform edge detection based on gradient map.
	// Search for gradients that exceed threshold; short hysteresis phase is implemented to find maximum gradient in neighborhood.
	// Positive gradients are for the right lane, negative for the left.  (Assuming dark road and lighter lane markings/road edges.)

	// Actual edge coordinates; separate bins for left and right lanes.
	// Reserve enough for room to avoid ever needing to resize.
	vector<Vec3i> edges[MAX_LANES];
	edges[LEFT_LANE].reserve(450); // TODO: Tune this.
	edges[RIGHT_LANE].reserve(450);

	uchar edgeMap[MAX_LANES][ROI_WIDTH][ROI_HEIGHT] = {0};

	// FSM-based edge detection implementation.
	int gradient;
	eEdgeSearchState ess;
	int stateCountdown;
	int edgePos;
	int maxGradient;

	for (int y = 0; y < frame.rows; ++y)
	{
		pRowGradient = m_frameGradient.ptr<int>(y);
		ess = ESS_SEARCH_THRES;

		for (int x = xLeft; x <= xRight; ++x)
		{
			gradient = pRowGradient[x + 2];

			switch (ess)
			{
			case ESS_SEARCH_THRES:
				if (gradient >= m_config.gradientThreshold)
				{
					// Minimal positive gradient threshold crossed.
					ess = ESS_SEARCH_MAX_POS;
					stateCountdown = min(m_config.hysteresisDuration, (xRight - x));
					edgePos = x;
					maxGradient = gradient;
				}
				else if (-gradient >= m_config.gradientThreshold)
				{
					// Minimal negative gradient threshold crossed.
					ess = ESS_SEARCH_MAX_NEG;
					stateCountdown = min(m_config.hysteresisDuration, (xRight - x));
					edgePos = x;
					maxGradient = -gradient;
				}
				break;

			case ESS_SEARCH_MAX_POS:
				if (gradient >= maxGradient)
				{
					// Found larger positive gradient.
					edgePos = x;
					maxGradient = gradient;
				}

				if (-gradient >= m_config.gradientThreshold)
				{
					// Negative gradient threshold found.
					edgeMap[RIGHT_LANE][edgePos + 2][y] = 1;

					ess = ESS_SEARCH_MAX_NEG;
					stateCountdown = min(m_config.hysteresisDuration, (xRight - x));
					edgePos = x;
					maxGradient = -gradient;
				}
				else if (--stateCountdown == 0)
				{
					// State expired.
					edgeMap[RIGHT_LANE][edgePos + 2][y] = 1;

					ess = ESS_SEARCH_THRES;
				}
				break;

			case ESS_SEARCH_MAX_NEG:
				if (-gradient >= maxGradient)
				{
					// Found larger negative gradient.
					edgePos = x;
					maxGradient = -gradient;
				}

				if (gradient >= m_config.gradientThreshold)
				{
					// Positive gradient threshold found.
					edgeMap[LEFT_LANE][edgePos + 2][y] = 1;

					ess = ESS_SEARCH_MAX_POS;
					stateCountdown = min(m_config.hysteresisDuration, (xRight - x));
					edgePos = x;
					maxGradient = gradient;
				}
				else if (--stateCountdown == 0)
				{
					// State expired.
					edgeMap[LEFT_LANE][edgePos + 2][y] = 1;

					ess = ESS_SEARCH_THRES;
				}
				break;

			default:
				break;
			}
		}
	}

	// FSM-based edge detection implementation for vertical gradients.
	static int lastAngle = 0;
	bool edgeFound;
	for (int x = 0; x < frame.cols; ++x)
	{
		ess = ESS_SEARCH_THRES;
		edgeFound = false;

		for (int y = yBottom; y >= yTop; --y)
		{
			gradient = m_frameGradientVert.at<int>((y + 2), x);

			switch (ess)
			{
			case ESS_SEARCH_THRES:
				if (gradient >= m_config.gradientThreshold)
				{
					// Minimal positive gradient threshold crossed.
					ess = ESS_SEARCH_MAX_POS;
					stateCountdown = min(m_config.hysteresisDuration, (y - yTop));
					edgePos = y;
					maxGradient = gradient;
				}
				break;

			case ESS_SEARCH_MAX_POS:
				if (gradient >= maxGradient)
				{
					// Found larger positive gradient.
					edgePos = y;
					maxGradient = gradient;
				}

				if ( (--stateCountdown == 0) || (-gradient >= m_config.gradientThreshold) )
				{
					edgeFound = true;
				}
				break;
			}

			if (edgeFound)
			{
				if (lastAngle > m_config.edgeMapAngleThreshold)
					edgeMap[LEFT_LANE][x][edgePos + 2] = 1;
				else if (lastAngle < -m_config.edgeMapAngleThreshold)
					edgeMap[RIGHT_LANE][x][edgePos + 2] = 1;
				else
					edgeMap[LEFT_LANE][x][edgePos + 2] = edgeMap[RIGHT_LANE][x][edgePos + 2] = 1;
				break;
			}
		}
	}

	// Post-processing to establish "coherency factor" for edges.
	// Edge coherency refers to the existence of neighboring edges.
	// Actual lane markings/road edges should typically have high coherency, whereas noise resulting from (e.g.) glare should have low coherency.
	int coherency;
	for (int lane = LEFT_LANE; lane < MAX_LANES; ++lane)
	{
		for (int x = 1; x < ROI_WIDTH - 1; ++x)
		{
			for (int y = 1; y < ROI_HEIGHT - 1; ++y)
			{
				if (edgeMap[lane][x][y])
				{
					coherency = 0;
					if (edgeMap[lane][x-1][y-1])
						++coherency;
					if (edgeMap[lane][x][y-1])
						++coherency;
					if (edgeMap[lane][x+1][y-1])
						++coherency;
					if (edgeMap[lane][x-1][y])
						++coherency;
					if (edgeMap[lane][x+1][y])
						++coherency;
					if (edgeMap[lane][x-1][y+1])
						++coherency;
					if (edgeMap[lane][x][y+1])
						++coherency;
					if (edgeMap[lane][x+1][y+1])
						++coherency;

					if (coherency > 2)
						coherency = 2;
					edges[lane].push_back(Vec3i(x, y, coherency));
				}
			}
		}
	}

	// Show detected edges on image for diagnostics purposes.
	for (Vec3i & edge : edges[LEFT_LANE])
		frame.at<uchar>(edge[1], edge[0], 0) = 0;

	for (Vec3i & edge : edges[RIGHT_LANE])
		frame.at<uchar>(edge[1], edge[0], 0) = 80;

	// Perform lane transform and adapt target X values for each lane to target servo positions.
	int targetServo[MAX_LANES] = {0};
	{
		boost::mutex::scoped_lock lock(m_laneStateMutex);

		// Run lane search, but first track which lane(s) were already locked and their angles.
		bool prevActive[MAX_LANES];
		int prevAngle[MAX_LANES];
		for (int lane = LEFT_LANE; lane < MAX_LANES; ++lane)
		{
			prevActive[lane] = m_lockedLanes[lane].isActive();
			prevAngle[lane] = m_lockedLanes[lane].angle;

			m_pLaneTransform->LaneSearch(edges[lane], static_cast<eLane>(lane), m_searchRange[lane], m_lockedLanes[lane], m_config.angleDeviationMax,
										 m_config.angleLimit, m_config.laneVoteThreshold, debugOutput);
		}

		// If both lanes detected, compare their angles to sanitize.
		// If angle difference is outside the normal range, deactivate one or both lanes.
		// Prefer to deactivate newly detected lane.
		// If both lanes were already established, deactivate the one that has undergone the larger angle change.
		if (m_lockedLanes[LEFT_LANE].isActive() && m_lockedLanes[RIGHT_LANE].isActive())
		{
			int angleDiff = m_lockedLanes[LEFT_LANE].angle - m_lockedLanes[RIGHT_LANE].angle;
			if ( (angleDiff < m_config.laneAngleDiffMin) || (angleDiff > m_config.laneAngleDiffMax) )
			{
				if (!prevActive[LEFT_LANE])
					m_lockedLanes[LEFT_LANE].deactivate();

				if (!prevActive[RIGHT_LANE])
					m_lockedLanes[RIGHT_LANE].deactivate();

				if (prevActive[LEFT_LANE] && prevActive[RIGHT_LANE])
				{
					if ( abs(m_lockedLanes[LEFT_LANE].angle - prevAngle[LEFT_LANE]) > abs(m_lockedLanes[RIGHT_LANE].angle - prevAngle[RIGHT_LANE]) )
						m_lockedLanes[LEFT_LANE].deactivate();
					else
						m_lockedLanes[RIGHT_LANE].deactivate();
				}
			}
		}

		// For active lane(s), calculate appropriate servo target, optionally render the lane, and adjust search range.
		int searchBuffer = m_config.searchBuffer;
		int totalAngle = 0;
		int activeCount = 0;
		for (int lane = LEFT_LANE; lane < MAX_LANES; ++lane)
		{
			if (m_lockedLanes[lane].isActive())
			{
				targetServo[lane] = TranslateXTargetToServo(static_cast<eLane>(lane), m_lockedLanes[lane].xTarget);

				if (m_renderLanes)
					m_pLaneTransform->RenderLane(frame, m_lockedLanes[lane]);

				m_searchRange[lane] = Vec2i(m_lockedLanes[lane].xTarget + searchBuffer, m_lockedLanes[lane].xTarget - searchBuffer);
				TrimSearchRange(m_searchRange[lane]);

				totalAngle += m_lockedLanes[lane].angle;
				++activeCount;
			}

			// Swap search buffer "polarity" for right lane.
			searchBuffer *= -1;
		}

		// Miscellaneous post-processing.
		if (activeCount)
		{
			// Hold onto average lane angle for next frame processing.
			lastAngle = totalAngle / activeCount;

			if (activeCount == 1)
			{
				// One lane is active; set reasonable search range for the other lane.
				if (m_lockedLanes[LEFT_LANE].isActive())
				{
					m_searchRange[RIGHT_LANE][0] = m_searchRange[LEFT_LANE][0];
					m_searchRange[RIGHT_LANE][1] = c_defaultRange[RIGHT_LANE][1];
					TrimSearchRange(m_searchRange[RIGHT_LANE]);
				}
				else
				{
					m_searchRange[LEFT_LANE][0] = m_searchRange[RIGHT_LANE][0];
					m_searchRange[LEFT_LANE][1] = c_defaultRange[LEFT_LANE][1];
					TrimSearchRange(m_searchRange[LEFT_LANE]);
				}
			}
			else
			{
				// Track some useful stats.
				int angleDiff = m_lockedLanes[LEFT_LANE].angle - m_lockedLanes[RIGHT_LANE].angle;
				if (angleDiff > m_laStats.maxAngleDiff)
					m_laStats.maxAngleDiff = angleDiff;
				else if (angleDiff < m_laStats.minAngleDiff)
					m_laStats.minAngleDiff = angleDiff;
			}
		}
		else
			lastAngle = 0;
	} // End critical section.

	// Grab last servo value for FDR record before we overwrite it.
	FDRecord & currFDR = m_FDRecords[m_currFDRIndex];
	currFDR.lastServo = m_lastServo;

	// Calculate weighted servo value based on input from active lane(s).
	static int servo = ControlMgr::cDefServo;
	if (targetServo[LEFT_LANE] || targetServo[RIGHT_LANE])
	{
		if (targetServo[LEFT_LANE] && targetServo[RIGHT_LANE])
			servo = (targetServo[LEFT_LANE] + targetServo[RIGHT_LANE]) / 2;
		else if (targetServo[LEFT_LANE])
			servo = targetServo[LEFT_LANE];
		else
			servo = targetServo[RIGHT_LANE];

		m_lastServo = servo;
	}

	// Limit max speed for auto-pilot based on servo value (i.e.: go slow around tight turns).
	{
		boost::mutex::scoped_lock lock(m_accelMutex);
		m_maxSpeed = m_config.autoPilotSpeedCap - (abs(servo - ControlMgr::cDefServo) * m_config.autoPilotSpeedCapFactor);
	}

	if (debugOutput)
	{
		cout << "  Search start left: " << m_searchRange[LEFT_LANE][0] << ", right: " << m_searchRange[RIGHT_LANE][0] << endl;
		cout << "  Target: " << targetServo[LEFT_LANE] << ", " << targetServo[RIGHT_LANE] << "; Servo: " << servo << endl;
		cout << "  Min/max angle diff: " << m_laStats.minAngleDiff << ", " << m_laStats.maxAngleDiff << endl;
	}

	// Clone current frame and collect FDR record.
	currFDR.frame = frame.clone();
	currFDR.target[LEFT_LANE] = targetServo[LEFT_LANE];
	currFDR.target[RIGHT_LANE] = targetServo[RIGHT_LANE];
	currFDR.searchStart[LEFT_LANE] = m_searchRange[LEFT_LANE][0];
	currFDR.searchStart[RIGHT_LANE] = m_searchRange[RIGHT_LANE][0];
	currFDR.servo = servo;
	currFDR.lastAngle = lastAngle;
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
	case PP_RESISTANCEFACTOR:
		cout << "  1) Resistance Factor" << endl;
		break;
	case PP_SEARCHBUFFER:
		cout << "  1) Search Buffer" << endl;
		break;
	case PP_LANESWITCHXOFFSET:
		cout << "  1) Lane Switch X-Offset" << endl;
		break;
	case PP_HYSTERESISDURATION:
		cout << "  1) Hysteresis Duration" << endl;
		break;
	case PP_EDGEMAPANGLETHRESHOLD:
		cout << "  1) Edge Map Angle Threshold" << endl;
		break;
	case PP_LANEANGLEDIFF:
		cout << "  1) Lane Angle Diff Min" << endl;
		cout << "  2) Lane Angle Diff Max" << endl;
		break;
	case PP_AUTOPILOTSPEED:
		cout << "  1) Auto-pilot Speed Cap" << endl;
		cout << "  2) Auto-pilot Speed Cap Factor" << endl;
		break;
	case PP_XOFFSETSERVOFACTOR:
		cout << "  1) X-Offset Servo Factor" << endl;
		break;
	case PP_ANGLEDEVIATIONMAX:
		cout << "  1) Angle Deviation Max" << endl;
		break;
	case PP_ANGLELIMIT:
		cout << "  1) Angle Limit" << endl;
		break;
	case PP_LANEVOTETHRESHOLD:
		cout << "  1) Lane Vote Threshold" << endl;
		break;
	}
}

int BusMgr::TranslateXTargetToServo(eLane lane, int xTarget) const
{
	int servo = ControlMgr::cDefServo + static_cast<int>( (xTarget - c_centerX[lane]) / m_config.xOffsetServoFactor );
	return clamp(servo, (ControlMgr::cDefServo - ControlMgr::cServoRange), (ControlMgr::cDefServo + ControlMgr::cServoRange));
}

void BusMgr::TrimSearchRange(cv::Vec2i & searchRange) const
{
	searchRange[0] = clamp(searchRange[0], -26, ROI_WIDTH + 26);
	searchRange[1] = clamp(searchRange[1], -26, ROI_WIDTH + 26);
}
