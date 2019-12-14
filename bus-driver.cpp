/*
 * Copyright (C) -.
 *
 * Author: Rob Boyer
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the PG_ORGANIZATION nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY	THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS-IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <termios.h>

#include "BusMgr.h"


void config_canonical_mode(bool enable)
{
	// Get current STDIN terminal attributes and update it to enable or disable canonical mode.
	struct termios ttystate;
	tcgetattr(STDIN_FILENO, &ttystate);

	if (enable)
	{
		ttystate.c_lflag |= ICANON;
		ttystate.c_lflag |= ECHO;
		ttystate.c_lflag &= ~ECHONL;
	}
	else
	{
		ttystate.c_lflag &= ~ICANON;
		ttystate.c_cc[VMIN] = 1; // Wait for 1 character minimum
		ttystate.c_lflag &= ~ECHO;
		ttystate.c_lflag |= ECHONL;
	}
	tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);
}

bool kbhit()
{
	// Use select on STDIN to check if a character is ready.
    struct timeval tv;
    fd_set fds;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);
    return (FD_ISSET(STDIN_FILENO, &fds) != 0);
}

int main(void)
{
	// Initialize bus manager object.
	BusMgr busMgr;
	if (!busMgr.Initialize())
		return busMgr.GetErrorCode();

	// Disable canonical mode so that keyboard input can be received character by character.
	config_canonical_mode(false);

	// Keep checking for cancellation request until bus manager thread has been joined.
	int speed = 0;
	int servo = ControlMgr::cDefServo;
	bool reverse = false;

	while (busMgr.IsRunning())
	{
		if (kbhit())
		{
			// Trigger termination logic once user presses 'q'.
			char c;
			if ( (c = fgetc(stdin)) == 'q' )
				busMgr.Terminate();
			else if (c == 's')
				busMgr.OutputStatus();
			else if (c == 'c')
				busMgr.OutputConfig();
			else if (c == 'm')
				busMgr.UpdateIPM();
			else if (c == 'p')
				busMgr.UpdatePage();
			else if (c == 'd')
				busMgr.DebugCommand();
			else if (c == '[')
				busMgr.UpdateParam(1, false);
			else if (c == ']')
				busMgr.UpdateParam(1, true);
			else if (c == '{')
				busMgr.UpdateParam(2, false);
			else if (c == '}')
				busMgr.UpdateParam(2, true);
			else if (c == '0')
				busMgr.SetSpeed(0);
			else if (c == '+')
			{
				speed += 50;
				if (speed >= 1000) speed = 1000;
				busMgr.SetSpeed(speed);
			}
			else if (c == '-')
			{
				speed -= 50;
				if (speed < 0) speed = 0;
				busMgr.SetSpeed(speed);
			}
			else if (c == '1')
			{
				servo -= 1;
				if (servo <= 100) servo = 100;
				busMgr.SetServo(servo);
			}
			else if (c == '2')
			{
				servo += 1;
				if (servo >= 200) servo = 200;
				busMgr.SetServo(servo);
			}
			else if (c == 'r')
			{
				busMgr.SetReverse(true);
			}
			else if (c == 'f')
			{
				busMgr.SetReverse(false);
			}
			else if (c == '<')
			{
				busMgr.PrevFDR();
			}
			else if (c == '>')
			{
				busMgr.NextFDR();
			}
			else if (c == 'l')
			{
				busMgr.ToggleRenderLanes();
			}
		}

		busMgr.ApplyAcceleration();
		busMgr.AdjustServo();

		boost::this_thread::sleep(boost::posix_time::milliseconds(5));
	}

	// Clean up.
	config_canonical_mode(true);
	return busMgr.GetErrorCode();
}
