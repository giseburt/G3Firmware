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

#include "MotorController.hh"
#include "ExtruderBoard.hh"
#include "EepromMap.hh"

MotorController MotorController::motor_controller;

MotorController::MotorController() {
	reset();
}

void MotorController::reset() {
	direction = true;
	paused = false;
	on = false;
	speed = 0;
	set_with_rpm = false;
}
void MotorController::update() {
	ExtruderBoard& board = ExtruderBoard::getBoard();
	if (!set_with_rpm) {
		int new_speed = (!paused&&on)?(direction?speed:-speed):0;
		board.setMotorSpeed(new_speed);
	} else {
#ifdef DEFAULT_EXTERNAL_STEPPER
		board.setMotorSpeedRPM(rpm, direction);
		board.setMotorOn(!paused && on);
#else
		board.setMotorSpeedRPM((!paused&&on) ? rpm : 0, direction);
#endif
	}

}

void MotorController::setSpeed(int speed_in) {
	speed = speed_in;
	set_with_rpm = false;
}

void MotorController::setRPMSpeed(uint32_t speed_in) {
	rpm = speed_in;
	set_with_rpm = true;
}

void MotorController::setDDASpeed(uint32_t dda_interval) {
	rpm = dda_interval;
	set_with_rpm = true;
}

void MotorController::pause() {
	paused = !paused;
	//ExtruderBoard::getBoard().indicateError(paused?1:0);

}

void MotorController::setDir(bool dir_in) {
	direction = dir_in;
}

void MotorController::setOn(bool on_in) {
	ExtruderBoard& board = ExtruderBoard::getBoard();
	on = on_in;
}
