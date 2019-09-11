/*
 * ControlMgr.h
 *
 *  Created on: Aug 20, 2019
 *      Author: rboyer
 */

#ifndef CONTROLMGR_H_
#define CONTROLMGR_H_

#include <iostream>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <pigpio.h>

#define GPIO_SERVO 13 	// 1.5ms = straight, 1ms = left, 2ms = right; freq = 100Hz
#define GPIO_MOTOR 18 	// 0% = stop, 100% = full on; freq = 100Hz
#define GPIO_DIR 24 	// 1 = forward, 0 = reverse


enum eCtrlErrorCode
{
	CEC_NONE,
	CEC_INITFAIL,
	CEC_WRITEFAIL,
	CEC_PWMFAIL,
	CEC_INTERRUPT
};


class ControlMgr {
	boost::thread m_thread;
	eCtrlErrorCode m_errorCode;
	int m_speed;
	bool m_reverse;
	int m_servo;
	bool m_laneAssist;
	bool m_update;

public:
	ControlMgr();
	virtual ~ControlMgr();

	eCtrlErrorCode GetErrorCode() const { return m_errorCode; }
	void Terminate();

	void SetSpeed(int speed)
	{
		if (speed != m_speed)
		{
			//std::cout << "DEBUG: SetSpeed(" << speed << ")" << std::endl;
			m_speed = speed;
			m_update = true;
		}
	}

	void SetReverse(bool reverse)
	{
		if (reverse != m_reverse)
		{
			//std::cout << "DEBUG: SetReverse(" << reverse << ")" << std::endl;
			m_reverse = reverse;
			m_update = true;
		}
	}

	void SetServo(int servo)
	{
		if (servo != m_servo)
		{
			//std::cout << "DEBUG: SetServo(" << servo << ")" << std::endl;
			m_servo = servo;
			m_update = true;
		}
	}

	void SetLaneAssist(bool enable)
	{
		std::cout << "DEBUG: SetLaneAssist(" << enable << ")" << std::endl;
		m_laneAssist = enable;
	}

	bool GetLaneAssist() const { return m_laneAssist; }

private:
	void WorkerFunc();

};

#endif /* CONTROLMGR_H_ */
