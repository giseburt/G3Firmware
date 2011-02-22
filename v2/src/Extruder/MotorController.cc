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
	rpm_or_dda1 = 0;
	dda2 = 0;
	steps = 0;
	speed_set_as = SET_AS_PWM;
	changes_sent = true;
}
void MotorController::update() {
	if (changes_sent)
		return;
	ExtruderBoard& board = ExtruderBoard::getBoard();
	if (speed_set_as == SET_AS_PWM) {
		int new_speed = (!paused&&on)?(direction?speed:-speed):0;
		board.setMotorSpeed(new_speed);
	} else {
#ifdef DEFAULT_EXTERNAL_STEPPER
		if (speed_set_as == SET_AS_RPM) {
			//board.setMotorSpeedRPM(rpm_or_dda1, direction);
			// for RPM, we get a start or stop command seperate
			//board.setMotorOn(!paused && on);
		}
		else
		{	
			board.setMotorSpeedDDA(rpm_or_dda1, dda2, steps, direction, on);
			steps = 0;
		}
#else
		board.setMotorSpeedRPM((!paused&&on) ? rpm_or_dda : 0, direction);
#endif
	}
	
	changes_sent = true;

}

void MotorController::setSpeed(int speed_in) {
	speed = speed_in;
	speed_set_as = SET_AS_PWM;
}

void MotorController::setRPMSpeed(uint32_t speed_in) {
	rpm_or_dda1 = speed_in;
	speed_set_as = SET_AS_RPM;
	changes_sent = false;
}

void MotorController::setDDASpeed(uint32_t dda_interval1, uint32_t dda_interval2, uint32_t steps_in) {
	rpm_or_dda1 = dda_interval1;
	dda2 = dda_interval2;
	steps = steps_in;
	speed_set_as = SET_AS_DDA;
	changes_sent = false;
}

void MotorController::pause() {
	paused = !paused;
	changes_sent = false;
	//ExtruderBoard::getBoard().indicateError(paused?1:0);

}

void MotorController::setDir(bool dir_in) {
	direction = dir_in;
	changes_sent = false;
}

void MotorController::setOn(bool on_in) {
	on = on_in;
	changes_sent = false;
}
