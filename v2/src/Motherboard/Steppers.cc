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
	void setTarget(const int32_t target_in, const bool relative) {
		target = target_in;
		if (relative) {
			delta = target;
		} else {
			delta = target - position;
		}
		
		unscaled_delta = delta;
		
		delta = (delta * scale) / 10000;
		
		old_direction = direction;
		
		direction = true;
		if (delta != 0) {
			if (interface != 0)
				interface->setEnabled(true);
		}
		if (delta < 0) {
			delta = -delta;
			direction = false;
		}
		
		first_decel_step = delta;
		total_delta = delta;
		resolved = false;
	}
	
	// return the difference from set speed and deceleration limit
	// accounting for current speed
	// 0 means accelerating, no change in speed, or the change is < than the deceleration limit
	void setSpeed(const int32_t us_per_step) {
		set_speed = us_per_step;
		
		// are we being asked to go faster than we steps for?
		if ((set_speed > 0) /* set speed not stopped  */ 
			&& (set_speed < ((speed || min_speed) - (max_accel * delta))) /* add will accel more than max per step */
			)
		{
			// set the set_speed to what we can achieve in the time given
			set_speed = ((speed || min_speed) - (max_accel * delta));
		}
#if 0		
		uint32_t deceleration_overage = 0;
		// are we decelerating?
		if ((speed > max_decel) /* we are moving fast enough that we can't just stop */ 
				&& ((set_speed == 0) /*and will stop*/ || (set_speed > (speed + max_decel)) /* or slow down fast enough */))
		{
			// Formula: (set speed or min_speed) - speed
			return ((set_speed == 0)?min_speed:set_speed) - speed
		}
		return 0;
#endif
	}
	
	void setFutureTargetAndSpeed(const int32_t target_in, const bool relative, const int32_t us_per_step) {
		int32_t temp_delta = 0;
		if (relative) {
			temp_delta = target_in;
		} else {
			temp_delta = target_in - target;
		}
		
		temp_delta = (temp_delta * scale) / 10000;
		
		bool new_direction = true;
		if (temp_delta < 0) {
			temp_delta = -temp_delta;
			new_direction = false;
		}
		
		// if we aren't moving at the end of the current move, we're done here
		if (set_speed == 0)
		{
			resolved = true;
			return;
		}
		else
		// if we switch directions, or are set to not move, then we stop at the end of this movement
		if (temp_delta == 0 || direction != new_direction) {
			// figure backwards when to start decelerating
			int32_t steps_to_stop = ((min_speed - set_speed) / max_decel);
			first_decel_step = total_delta - steps_to_stop;
		}
		else
		// otherwise, we figure out if we have enough steps in this next move to stop
		{
			int32_t steps_to_stop = (min_speed - min(set_speed, us_per_step) / max_decel);
			if (steps_to_stop > temp_delta) {
				if (steps_to_stop > (total_delta + delta)) {
					total_delta += temp_delta;
					resolved = false;
					return;
				} else {
					first_decel_step = steps_to_stop - total_delta - delta;
					resolved = true;
				}
			}
			// 
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
		total_delta = 0;
		unscaled_delta = 0;
		scale = 10000;
		max_accel = 0;
		max_decel = 0;
		direction = old_direction = true;
		min_speed = 1000000; // one step per second?!
		set_speed = 0; // 0 == stopped
		speed = 0; // 0 == stopped
		first_decel_step = 0;
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
	/// For keeping track of reversals
	volatile bool old_direction;
	/// Scale of the axis
	volatile int32_t scale;
	/// Do we have enough time to stop?
	volatile bool resolved;
	/// The step at which we start decelerating, might be == delta
	volatile int32_t first_decel_step;
	/// Total movement, including future moves, until "Resolved"
	volatile int32_t total_delta;
	/// Maximum acceleration rate
	int32_t max_accel;
	/// Maximum deceleration rate
	int32_t max_decel;
	/// Minimum step rate, in µseconds/step. Only for acceleration start.
	int32_t min_speed;
	/// Current _set_ speed, in µseconds/step
	volatile int32_t set_speed;
	/// Current movement speed, in µseconds/step
	volatile int32_t speed;
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
	axes[FEEDRATE_AXIS].position = dda_interval;
	
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
	int32_t total_us = max_delta * dda_interval;
	const int32_t negative_half_interval = -intervals / 2;
	for (int i = 0; i < AXIS_COUNT; i++) {
		axes[i].counter = negative_half_interval;
		axes[i].set_speed = total_us / axes[i].delta; // <- this will round, that's ok
	}
	
	ticks_per_step = ticks_left = (axes[FEEDRATE_AXIS].position * feedrate_scale) / INTERVAL_IN_MICROSECONDS;
	
	is_running = true;
}

void setTargetNew(const Point& target, int32_t us, uint8_t relative) {
	for (int i = 0; i < AXIS_COUNT; i++) {
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
	
uint8_t setFutureTarget(const Point& target, int32_t dda_interval)
{
	int resolved = 0;
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
