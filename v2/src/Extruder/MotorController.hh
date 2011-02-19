/*
 * Copyright 2010 by Adam Mayer <adam@makerbot.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */


#ifndef MOTOR_CONTROLLER_HH_
#define MOTOR_CONTROLLER_HH_

#include "Timeout.hh"

class MotorController {
public:
	void update();
	void setSpeed(int speed);
	void setRPMSpeed(uint32_t speed);
	void setDDASpeed(uint32_t speed);
	void setDir(bool dir);
	void setOn(bool on);
	void pause();
	static MotorController& getController() { return motor_controller; }
	static void runMotorSlice() { getController().update(); }
	// Reset to board-on state
	void reset();
private:
	MotorController();
	bool set_with_rpm;
	bool direction;
	bool on;
	int speed;
	uint32_t rpm;
	bool paused;
	static MotorController motor_controller;
};

#endif // MOTOR_CONTROLLER_HH_
