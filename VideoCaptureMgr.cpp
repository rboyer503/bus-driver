/*
 * VideoCaptureMgr.cpp
 *
 *  Created on: Jan 8, 2015
 *      Author: rboyer
 */

#include <iostream>
#include "VideoCaptureMgr.h"

#include "BusMgr.h"

using namespace std;
using namespace cv;


/*
// Global variables and macros for profiling.
boost::posix_time::ptime g_start2;
boost::posix_time::time_duration g_diff2;

#define PROFILE_START \
	do { \
		g_start2 = boost::posix_time::microsec_clock::local_time(); \
		cout << "Profile (RESET)" << endl; \
	} while (0)

#define PROFILE_LOG(tag) \
	do { \
		g_diff2 = boost::posix_time::microsec_clock::local_time() - g_start2; \
		cout << "Profile (" << #tag << "): " << g_diff2.total_microseconds() << " us" << endl; \
	} while (0)
*/


VideoCaptureMgr::VideoCaptureMgr(BusMgr * owner) :
	m_owner(owner), m_fd(-1), m_capturing(false),
	m_pCurrImage(NULL), m_currIndex(-1)
{
}

VideoCaptureMgr::~VideoCaptureMgr()
{
	if (m_capturing)
	{
		m_thread.interrupt();
		m_thread.join();
	}

	if (m_fd != -1)
		v4l2_close(m_fd);

	cout << "Video capture manager released." << endl;
}

bool VideoCaptureMgr::Initialize()
{
	// Open device for non-blocking operation.
	m_fd = v4l2_open("/dev/video0", O_RDWR | O_NONBLOCK);
	if (m_fd == -1)
	{
		cout << "Error opening video device" << endl;
		return false;
	}
	FD_ZERO(&m_fds);
	FD_SET(m_fd, &m_fds);

	// Set up image format.
	if (!SetImageFormat())
		return false;

	// Configure frame rate.
	/*if (!SetFrameRate(10))
		return false;*/

	// Initialize necessary video buffers.
	if (!AllocateVideoMemory())
		return false;

	// Kick off capture thread.
	m_capturing = true;
	m_thread = boost::thread(&VideoCaptureMgr::DoCapture, this);
	return true;
}

int VideoCaptureMgr::GetLatest(cv::Mat *& image)
{
	//PROFILE_START;

	if (!m_capturing)
		return -1;

	int numDroppedFrames;

	// Free up current image and put buffer on the free queue.
	if (m_currIndex != -1)
	{
		delete m_pCurrImage;
		m_pCurrImage = NULL;

		boost::mutex::scoped_lock lock(m_freeQueueMutex);
		m_freeQueue.push(m_currIndex);
	}

	//PROFILE_LOG(FREE1);

	// Block until a new frame is available and then grab it.
	// Also, flush any extra, older queued frames.
	{
		boost::mutex::scoped_lock lock(m_readyQueueMutex);
		try
		{
			while (m_readyQueue.size() == 0)
				m_condition.wait(lock);
			//PROFILE_LOG(READY);
		}
		catch (boost::thread_interrupted&)
		{
			m_owner->SetInterrupted();
			return -1;
		}

		numDroppedFrames = m_readyQueue.size() - 1;

		{
			boost::mutex::scoped_lock lockFree(m_freeQueueMutex);
			while (m_readyQueue.size() > 1)
			{
				m_freeQueue.push(m_readyQueue.front());
				m_readyQueue.pop();
			}
			//PROFILE_LOG(FREE2);
		}

		m_currIndex = m_readyQueue.front();
		m_readyQueue.pop();
	}

	// Build new Mat from latest returned buffer.
	image = m_pCurrImage = new Mat(Size(320, 240), CV_8UC3, m_buffers[m_currIndex].start, Mat::AUTO_STEP);
	//PROFILE_LOG(DONE);
	return numDroppedFrames;
}

bool VideoCaptureMgr::SetImageFormat()
{
	struct v4l2_format fmt = {};
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width = 320;
	fmt.fmt.pix.height = 240;
	fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_BGR24; //V4L2_PIX_FMT_YUYV;
	fmt.fmt.pix.field = V4L2_FIELD_NONE;
	if (-1 == xioctl(m_fd, VIDIOC_S_FMT, &fmt))
	{
		cout << "Error setting image format" << endl;
		return false;
	}
	if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_BGR24)
	{
		cout << "Requested pixel format rejected" << endl;
		return false;
	}

	return true;
}

bool VideoCaptureMgr::SetFrameRate(int fps)
{
	// TODO: Test for capability first.
	// This will not work for SB101D camera because it only supports 30 fps.
	struct v4l2_streamparm streamparm;
	memset (&streamparm, 0, sizeof (streamparm));
	streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	struct v4l2_fract * pFract = &streamparm.parm.capture.timeperframe;
	pFract->numerator = 1;
	pFract->denominator = fps;
	if ( -1 == xioctl(m_fd, VIDIOC_S_PARM, &streamparm))
	{
		cout << "Error setting image format" << endl;
		return false;
	}

	return true;
}
bool VideoCaptureMgr::AllocateVideoMemory()
{
	// Request buffers from driver.
	struct v4l2_requestbuffers req = {};
    req.count = NUM_BUFFERS;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if ( (-1 == xioctl(m_fd, VIDIOC_REQBUFS, &req)) || (req.count != NUM_BUFFERS) )
	{
		cout << "Error requesting buffers" << endl;
		return false;
	}

    // Map each buffer into memory and track them in m_buffers.
    for (size_t i = 0; i < NUM_BUFFERS; ++i)
    {
    	struct v4l2_buffer buf = {};
    	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (-1 == xioctl(m_fd, VIDIOC_QUERYBUF, &buf))
        {
        	cout << "Error querying buffers" << endl;
        	return false;
        }

        m_buffers[i].length = buf.length;
        m_buffers[i].start = v4l2_mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
        						  	   MAP_SHARED, m_fd, buf.m.offset);

        if (MAP_FAILED == m_buffers[i].start)
        {
        	cout << "Error mapping buffers" << endl;
        	return false;
        }
    }

    return true;
}

void VideoCaptureMgr::DeAllocateVideoMemory()
{
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	xioctl(m_fd, VIDIOC_STREAMOFF, &type);
	for (size_t i = 0; i < NUM_BUFFERS; ++i)
		v4l2_munmap(m_buffers[i].start, m_buffers[i].length);
}

void VideoCaptureMgr::DoCapture()
{
	// Enqueue all buffers that we've allocated.
	for (int i = 0; i < NUM_BUFFERS; ++i)
	{
		struct v4l2_buffer buf = {};
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = i;
		if (-1 == xioctl(m_fd, VIDIOC_QBUF, &buf))
		{
			cout << "Error enqueueing buffers" << endl;
			m_capturing = false;
			return;
		}
	}

	// Activate the video stream.
	enum v4l2_buf_type bufType = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (-1 == xioctl(m_fd, VIDIOC_STREAMON, &bufType))
	{
		cout << "Error activating stream" << endl;
		m_capturing = false;
		return;
	}

	// Main loop for processing incoming frames.
	struct timeval tv = {};
	do
	{
		// Wait for a frame to be available.
		int r;
		do
		{
			tv.tv_sec = 2;
			tv.tv_usec = 0;
			r = select(m_fd + 1, &m_fds, NULL, NULL, &tv);
		} while ( (r == -1) && (errno == EINTR) );
		if (r == -1)
		{
			cout << "Error or timeout during select" << endl;
			m_capturing = false;
			return;
		}

		// Dequeue the buffer containing the frame.
		struct v4l2_buffer buf = {};
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		do
		{
			r = xioctl(m_fd, VIDIOC_DQBUF, &buf);

			// Give main thread an opportunity to shut this thread down.
			try
			{
				boost::this_thread::interruption_point();
			}
			catch (boost::thread_interrupted&)
			{
				cout << "Interrupted after dequeueing frame - shutting down video capture manager..." << endl;
				DeAllocateVideoMemory();
				m_capturing = false;
				return;
			}
		} while (EAGAIN == errno);
		if (r == -1)
		{
			cout << "Error dequeueing buffer" << endl;
			m_capturing = false;
			return;
		}

		// Free any returned queue elements.
		queue<int> localQueue;

		// Critical section begin
		{
			boost::mutex::scoped_lock lock(m_freeQueueMutex);
			if (!m_freeQueue.empty())
			{
				swap(m_freeQueue, localQueue);
			}
		}
		// Critical section end

		int index;
		while (!localQueue.empty())
		{
			index = localQueue.front();
			localQueue.pop();
			if (!ReEnqueue(index))
			{
				m_capturing = false;
				return;
			}
		}

		// Add to the ready queue and notify main thread.
		// Dequeue oldest record if necessary.
		index = -1;

		// Critical section begin
		{
			boost::mutex::scoped_lock lock(m_readyQueueMutex);
			if (m_readyQueue.size() == (NUM_BUFFERS - MIN_QUEUE_HEADSPACE))
			{
				index = m_readyQueue.front();
				m_readyQueue.pop();
			}
			m_readyQueue.push(buf.index);
			m_condition.notify_one();
		}
		// Critical section end

		if (index != -1)
		{
			if (!ReEnqueue(index))
			{
				m_capturing = false;
				return;
			}
		}

	} while (true);
}

bool VideoCaptureMgr::ReEnqueue(int index)
{
	struct v4l2_buffer buf = {};
	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;
	buf.index = index;
	if (-1 == xioctl(m_fd, VIDIOC_QBUF, &buf))
	{
		cout << "Error re-enqueueing buffer" << endl;
		return false;
	}

	return true;
}

int VideoCaptureMgr::xioctl(int fd, int request, void * arg)
{
	// Helper function to ignore EINTR.
	int r;

	do
	{
		r = v4l2_ioctl(fd, request, arg);
	} while (-1 == r && EINTR == errno);

	return r;
}
