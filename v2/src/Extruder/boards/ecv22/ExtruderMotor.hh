/*
 * Copyright 2010 by Adam Mayer	 <adam@makerbot.com>
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

#ifndef BOARDS_ECV22_EXTRUDER_MOTOR_HH_
#define BOARDS_ECV22_EXTRUDER_MOTOR_HH_

#include <stdint.h>

void initExtruderMotor();

void setStepperMode(bool mode, bool external = false);

// 0 = stop
// + = forward direction
// - = negative direction
// Valid range: -255 through 255
void setExtruderMotor(int16_t speed);
void setExtruderMotorRPM(uint32_t micros, bool direction);
void setExtruderMotorDDA(uint32_t dda1, uint32_t dda2, uint32_t steps, bool direction, bool on);
#ifdef DEFAULT_EXTERNAL_STEPPER
void setExtruderMotorOn(bool on);
#endif

#endif // BOARDS_ECV22_EXTRUDER_MOTOR_HH_
