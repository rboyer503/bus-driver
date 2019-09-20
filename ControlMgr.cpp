/*
 * ControlMgr.cpp
 *
 *  Created on: Aug 20, 2019
 *      Author: rboyer
 */

#include "ControlMgr.h"

using namespace std;


ControlMgr::ControlMgr() :
	m_errorCode(CEC_NONE), m_speed(0), m_reverse(false), m_servo(cDefServo), m_laneAssist(false), m_update(true)
{
	m_thread = boost::thread(&ControlMgr::WorkerFunc, this);
}

ControlMgr::~ControlMgr() {
	if (m_thread.joinable())
		m_thread.join();
}

void ControlMgr::Terminate()
{
	m_thread.interrupt();
}

void ControlMgr::WorkerFunc()
{
	int result = gpioInitialise();
	if (result < 0)
	{
		cout << "Error: gpioInitialise() failed." << endl;
		m_errorCode = CEC_INITFAIL;
		return;
	}

	result = gpioSetPWMfrequency(GPIO_SERVO, 100);
	if (result < 0)
	{
		cout << "Error: gpioSetPWMfrequency() failed, err=" << result << endl;
		m_errorCode = CEC_INITFAIL;
		gpioTerminate();
		return;
	}

	result = gpioSetPWMfrequency(GPIO_MOTOR, 100);
	if (result < 0)
	{
		cout << "Error: gpioSetPWMfrequency() failed, err=" << result << endl;
		m_errorCode = CEC_INITFAIL;
		gpioTerminate();
		return;
	}

	result = gpioSetPWMrange(GPIO_SERVO, 1000);
	if (result < 0)
	{
		cout << "Error: gpioSetPWMrange() failed, err=" << result << endl;
		m_errorCode = CEC_INITFAIL;
		gpioTerminate();
		return;
	}

	result = gpioSetPWMrange(GPIO_MOTOR, 1000);
	if (result < 0)
	{
		cout << "Error: gpioSetPWMrange() failed, err=" << result << endl;
		m_errorCode = CEC_INITFAIL;
		gpioTerminate();
		return;
	}

	while (1)
	{
		if (m_update)
		{
			m_update = false;

			result = gpioWrite(GPIO_DIR, !m_reverse);
			if (result < 0)
			{
				cout << "Error: gpioWrite() failed, err=" << result << endl;
				m_errorCode = CEC_WRITEFAIL;
				gpioTerminate();
				return;
			}

			result = gpioPWM(GPIO_MOTOR, m_speed);
			if (result < 0)
			{
				cout << "Error: gpioPWM() failed, err=" << result << endl;
				m_errorCode = CEC_PWMFAIL;
				gpioTerminate();
				return;
			}

			result = gpioPWM(GPIO_SERVO, m_servo);
			if (result < 0)
			{
				cout << "Error: gpioPWM() failed, err=" << result << endl;
				m_errorCode = CEC_PWMFAIL;
				gpioTerminate();
				return;
			}

			cout << "DEBUG: " << (m_reverse ? "-" : "") << m_speed << "; " << m_servo << endl;
		}

		try
		{
			boost::this_thread::interruption_point();
		}
		catch (boost::thread_interrupted&)
		{
			cout << "Interrupted - shutting down pigpio..." << endl;
			m_errorCode = CEC_INTERRUPT;
			break;
		}

		gpioDelay(5000);
	}

	gpioTerminate();
}

