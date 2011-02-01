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

#define __STDC_LIMIT_MACROS
#include "Steppers.hh"
#include <stdint.h>

namespace steppers {

class Axis {
public:
	Axis() : interface(0) {
		reset();
	}

	Axis(StepperInterface& stepper_interface) :
		interface(&stepper_interface) {
		reset();
	}

	/// Set target coordinate and compute delta
	void setTarget(const int32_t target_in, bool relative) {
		target = target_in;
		if (relative) {
			delta = target;
		} else {
			delta = target - position;
		}
		
		unscaled_delta = delta;
		
		delta = (delta * scale) / 10000;
		
		direction = true;
		if (delta != 0) {
			if (interface != 0)
				interface->setEnabled(true);
		}
		if (delta < 0) {
			delta = -delta;
			direction = false;
		}
	}

	/// Set homing mode
	void setHoming(const bool direction_in) {
		direction = direction_in;
		if (interface != 0)
			interface->setEnabled(true);
		delta = 1;
	}

	/// Define current position as the given value
	void definePosition(const int32_t position_in) {
		position = position_in;
	}

	/// Set the scale value, 10000 = 100%
	void setScale(const int32_t scale_in) {
		scale = scale_in;
		
		// should we do anything here with position, delta, etc?
	}
	
	/// Enable/disable stepper
	void enableStepper(bool enable) {
		if (interface != 0)
			interface->setEnabled(enable);
	}

	/// Reset to initial state
	void reset() {
		position = 0;
		minimum = 0;
		maximum = 0;
		target = 0;
		counter = 0;
		delta = 0;
		unscaled_delta = 0;
		scale = 10000;
	}
	
	inline bool atTarget() {
		if (position == target)
			return true;
		return false;
	}
	
	// return if we took a step
	bool doInterrupt(const int32_t intervals) {
		if (atTarget())
			return false;
		
		counter += delta;
		if (counter >= 0) {
			if (interface != 0)
				interface->setDirection(direction);
			counter -= intervals;
			if (direction) {
				if (interface != 0 && !interface->isAtMaximum()) interface->step(true);
				position++;
			} else {
				if (interface != 0 && !interface->isAtMinimum()) interface->step(true);
				position--;
			}
			if (interface != 0)
				interface->step(false);

			return true;
		}
		return false;
	}

	// Return true if still homing; false if done.
	bool doHoming(const int32_t intervals) {
		if (delta == 0 || interface == 0) return false;
		counter += delta;
		if (counter >= 0) {
			interface->setDirection(direction);
			counter -= intervals;
			if (direction) {
				if (!interface->isAtMaximum()) {
					interface->step(true);
				} else {
					return false;
				}
				position++;
			} else {
				if (!interface->isAtMinimum()) {
					interface->step(true);
				} else {
					return false;
				}
				position--;
			}
			interface->step(false);
		}
		return true;
	}

	StepperInterface* interface;
	/// Current position on this axis, in steps
	volatile int32_t position;
	/// Minimum position, in steps
	int32_t minimum;
	/// Maximum position, in steps
	int32_t maximum;
	/// Target position, in steps
	volatile int32_t target;
	/// Step counter; represents the proportion of a
	/// step so far passed.  When the counter hits
	/// zero, a step is taken.
	volatile int32_t counter;
	/// Amount to increment counter per tick
	volatile int32_t delta;
	/// The delta before scaling
	volatile int32_t unscaled_delta;
	/// True for positive, false for negative
	volatile bool direction;
	/// Scale of the axis
	volatile int32_t scale;
};

	
volatile bool is_running;
int32_t intervals;
volatile int32_t intervals_remaining;
#define AXIS_COUNT STEPPER_COUNT+1
// The index of the feedrate axis:
#define FEEDRATE_AXIS STEPPER_COUNT
Axis axes[AXIS_COUNT]; // add a virtal axis for feedrate
// To keep from over stepping for the feed rate, we might scale it
volatile int8_t feedrate_scale;
volatile int32_t ticks_left, ticks_per_step;
volatile bool is_homing;

bool isRunning() {
	return is_running || is_homing;
}

//public:
void init(Motherboard& motherboard) {
	is_running = false;
	for (int i = 0; i < STEPPER_COUNT; i++) {
		axes[i] = Axis(motherboard.getStepperInterface(i));
	}
	// add virual interface for feedrate
	axes[FEEDRATE_AXIS] = Axis();
	
	feedrate_scale=1;
	// FIXME! This needs to come from a command!
}

void abort() {
	is_running = false;
	is_homing = false;
}

/// Define current position as given point
void definePosition(const Point& position) {
	for (int i = 0; i < STEPPER_COUNT; i++) {
		axes[i].definePosition(position[i]);
	}
}

/// Get current position
const Point getPosition() {
#if STEPPER_COUNT > 3
	return Point(axes[0].position,axes[1].position,axes[2].position,axes[3].position,axes[4].position);
#else
	return Point(axes[0].position,axes[1].position,axes[2].position);
#endif
}

bool holdZ = false;

void setHoldZ(bool holdZ_in) {
	holdZ = holdZ_in;
}

void setTarget(const Point& target, int32_t dda_interval) {
	int32_t max_delta = 0;
	for (int i = 0; i < STEPPER_COUNT; i++) {
		axes[i].setTarget(target[i], false);
		const int32_t delta = axes[i].delta;
		// Only shut z axis on inactivity
		if (i == 2 && !holdZ) axes[i].enableStepper(delta != 0);
		else if (delta != 0) axes[i].enableStepper(true);
		if (delta > max_delta) {
			max_delta = delta;
		}
	}
	
	// if the feedrate has never been set, use the incoming one
	if (axes[FEEDRATE_AXIS].position == 0)
		axes[FEEDRATE_AXIS].position = dda_interval;
	
	// undo scaling before setting position
	axes[FEEDRATE_AXIS].position = axes[FEEDRATE_AXIS].position * feedrate_scale;
	feedrate_scale = 1;
	
	// To disable interpolation, uncomment this:
	//axes[FEEDRATE_AXIS].position = dda_interval;
	
	if (max_delta == 0) {
		axes[FEEDRATE_AXIS].position = dda_interval;
		intervals_remaining = 0;
		is_running = false;
		return;
	}
	
	// Keep the feedrate scaling from requiring too many steps
	axes[FEEDRATE_AXIS].setTarget(dda_interval, false);
	if (axes[FEEDRATE_AXIS].delta > max_delta*3) {
		feedrate_scale = axes[FEEDRATE_AXIS].delta / max_delta;
		axes[FEEDRATE_AXIS].position = axes[FEEDRATE_AXIS].position / feedrate_scale;
		axes[FEEDRATE_AXIS].target = axes[FEEDRATE_AXIS].target / feedrate_scale;
		axes[FEEDRATE_AXIS].delta = axes[FEEDRATE_AXIS].delta / feedrate_scale;
	}

	if (axes[FEEDRATE_AXIS].delta > max_delta)
		max_delta = axes[FEEDRATE_AXIS].delta;
	
	// compute number of intervals for this move
	intervals = max_delta;
	intervals_remaining = intervals;
	const int32_t negative_half_interval = -intervals / 2;
	for (int i = 0; i < AXIS_COUNT; i++) {
		axes[i].counter = negative_half_interval;
	}
	
	ticks_per_step = ticks_left = (axes[FEEDRATE_AXIS].position * feedrate_scale) / INTERVAL_IN_MICROSECONDS;
	
	is_running = true;
}

void setTargetNew(const Point& target, int32_t us, uint8_t relative) {
	int32_t max_delta = 0;
	for (int i = 0; i < STEPPER_COUNT; i++) {
		axes[i].setTarget(target[i], (relative & (1 << i)) != 0);
		// Only shut z axis on inactivity
		const int32_t delta = axes[i].delta;
		if (i == 2 && !holdZ) {
			axes[i].enableStepper(delta != 0);
		} else if (delta != 0) {
			axes[i].enableStepper(true);
		}
		if (delta > max_delta) {
			max_delta = delta;
		}
	}
	
	// we can't do interpolation here, and we're not going to try
	axes[FEEDRATE_AXIS].position = us/max_delta;
	axes[FEEDRATE_AXIS].setTarget(us/max_delta, false);
	
	// compute number of intervals for this move
	intervals = max_delta;
	intervals_remaining = intervals;
	const int32_t negative_half_interval = -intervals / 2;
	for (int i = 0; i < AXIS_COUNT; i++) {
		axes[i].counter = negative_half_interval;
	}
	
	ticks_per_step = ticks_left = axes[FEEDRATE_AXIS].position / INTERVAL_IN_MICROSECONDS;
	
	is_running = true;
}

/// Start homing
void startHoming(const bool maximums, const uint8_t axes_enabled, const uint32_t us_per_step) {
	intervals_remaining = INT32_MAX;
	intervals = us_per_step / INTERVAL_IN_MICROSECONDS;
	const int32_t negative_half_interval = -intervals / 2;
	for (int i = 0; i < STEPPER_COUNT; i++) {
		axes[i].counter = negative_half_interval;
		if ((axes_enabled & (1<<i)) != 0) {
			axes[i].setHoming(maximums);
		} else {
			axes[i].delta = 0;
		}
	}
	is_homing = true;
}

/// Enable/disable the given axis.
void enableAxis(uint8_t which, bool enable) {
	if (which < STEPPER_COUNT) {
		axes[which].enableStepper(enable);
	}
}
	
// Set the scale factor for the axis
void setAxisScale(uint8_t which, int32_t scale) {
	if (which < AXIS_COUNT) {
		axes[which].setScale(scale);
	}
}

bool doInterrupt() {
	if (is_running) {
		if (ticks_left-- == 0) {
			if (intervals_remaining-- == 0) {
				is_running = false;
			} else {
				bool took_step = false;
				for (int i = 0; i < STEPPER_COUNT; i++) {
					took_step |= axes[i].doInterrupt(intervals);
				}
				bool feedrate_stepped = axes[FEEDRATE_AXIS].doInterrupt(intervals);
				// only if it stepped and the feedrate has changed
				if (took_step && feedrate_stepped) {
					ticks_per_step = (axes[FEEDRATE_AXIS].position * feedrate_scale) / INTERVAL_IN_MICROSECONDS;
				}
				ticks_left = ticks_per_step;
			}
		}
		return is_running;
	} else if (is_homing) {
		is_homing = false;
		for (int i = 0; i < STEPPER_COUNT; i++) {
			bool still_homing = axes[i].doHoming(intervals);
			is_homing = still_homing || is_homing;
		}
		return is_homing;
	}
	return false;
}

}
