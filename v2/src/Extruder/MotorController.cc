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
	rpm_or_dda = 0;
	speed_set_as = SET_AS_PWM;
}
void MotorController::update() {
	ExtruderBoard& board = ExtruderBoard::getBoard();
	if (speed_set_as == SET_AS_PWM) {
		int new_speed = (!paused&&on)?(direction?speed:-speed):0;
		board.setMotorSpeed(new_speed);
	} else {
#ifdef DEFAULT_EXTERNAL_STEPPER
		board.setMotorSpeedRPM(rpm_or_dda, direction, (speed_set_as == SET_AS_RPM));
		board.setMotorOn(!paused && on);
#else
		board.setMotorSpeedRPM((!paused&&on) ? rpm_or_dda : 0, direction);
#endif
	}

}

void MotorController::setSpeed(int speed_in) {
	speed = speed_in;
	speed_set_as = SET_AS_PWM;
}

void MotorController::setRPMSpeed(uint32_t speed_in) {
	rpm_or_dda = speed_in;
	speed_set_as = SET_AS_RPM;
}

void MotorController::setDDASpeed(uint32_t dda_interval) {
	rpm_or_dda = dda_interval;
	speed_set_as = SET_AS_DDA;
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
