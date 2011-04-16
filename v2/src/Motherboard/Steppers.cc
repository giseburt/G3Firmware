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
#include <math.h>

#define TICKS_PER_SECOND (1000000.0 / (double)INTERVAL_IN_MICROSECONDS)
#define ACCELERATE_TICKS_PER_SECOND 125.0
#define TICKS_PER_ACCELERATE_TICK (TICKS_PER_SECOND/ACCELERATE_TICKS_PER_SECOND)
	

namespace steppers {

// The following three functions borrowed from the grbl project:
// https://github.com/simen/grbl/blob/master/planner.c
	
// Calculates the distance (not time) it takes to accelerate from initial_rate to target_rate using the 
// given acceleration:
inline double estimate_acceleration_distance(double initial_rate, double target_rate, double acceleration) {
	return((target_rate*target_rate-initial_rate*initial_rate)/(2L*acceleration));
}

// Calculates the maximum allowable speed at this point when you must be able to reach target_velocity using the 
// acceleration within the allotted distance.
inline double max_allowable_speed(double acceleration, double target_velocity, double distance) {
	return(sqrt(target_velocity*target_velocity-2*acceleration*distance));
}

	
// This function gives you the point at which you must start braking (at the rate of -acceleration) if 
// you started at speed initial_rate and accelerated until this point and want to end at the final_rate after
// a total travel of distance. This can be used to compute the intersection point between acceleration and
// deceleration in the cases where the trapezoid has no plateau (i.e. never reaches maximum speed)

/*                        + <- some maximum rate we don't care about
                         /|\
                        / | \                    
                       /  |  + <- final_rate     
                      /   |  |                   
	 initial_rate -> +----+--+                   
                          ^  ^                   
                          |  |                   
						  intersection_distance  distance
 */

// MODIFIED from grbl source: We use a seperate deceleration rate, so the math was changed accordingly
// Assumes deceleration is a *posititve* number
	
inline double intersection_distance(double initial_rate, double final_rate, double acceleration, double deceleration, double distance) {
	return(
		   (2*deceleration*distance-initial_rate*initial_rate+final_rate*final_rate)/
		   (2*(acceleration+deceleration))
	      );
}

// ### END of from grbl
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
		last_accel_step = delta;
		total_delta = delta;
		future_delta = 0;
		resolved = false;
		
		if (direction != old_direction || delta == 0)
			speed = 0;
	}
	
	void setSpeed(const int32_t us_per_step) {
		set_speed = 1000000.0/(double)us_per_step;
				
		// are we being asked to go faster than we have time for?
		if ((set_speed > 0) /* set speed not stopped  */
			&& (set_speed > (speed || min_speed)) // we are accelerating
			)
		{
			if (speed == 0)
				speed = min_speed;
			last_accel_step = estimate_acceleration_distance(speed, set_speed, max_accel);
			if (last_accel_step > delta) {
				// set the set_speed to what we can achieve in the time given
				set_speed = max_allowable_speed(max_accel, set_speed-speed, delta);
				last_accel_step = delta;
			}
		} else {
			last_accel_step = 0;
		}

		
		// figure a (soft) point to start slowing down, in case we don't get any future points
		// Example case: jogging from the RepG control panel
		int32_t steps_to_stop = estimate_acceleration_distance(min_speed, set_speed, max_decel);
		first_decel_step = delta - steps_to_stop;
		
		if (first_decel_step < last_accel_step) {
			double distance_to_intersection = intersection_distance(speed || min_speed, min_speed, max_accel, max_decel, delta);
			last_accel_step = ceil(distance_to_intersection);
			first_decel_step = floor(distance_to_intersection);
		}
	}
	
	
	void setFutureTarget(const int32_t target_in, const bool relative) {
		future_delta = 0;
		
		// if we aren't moving at the end of the current move, we're done here
		if (set_speed == 0)
		{
			resolved = true;
		}

		int32_t future_delta = 0;
		if (relative) {
			future_delta = target_in;
		} else {
			future_delta = target_in - target;
		}
		
		future_delta = (future_delta * scale) / 10000;
		
		bool new_direction = true;
		if (future_delta < 0) {
			future_delta = -future_delta;
			new_direction = false;
		}
				
		// if we switch directions, or are set to not move, then we stop at the end of this movement
		if (future_delta == 0 || direction != new_direction) {
			// figure backwards when to start decelerating
			int32_t steps_to_stop = estimate_acceleration_distance(set_speed, speed, max_decel);
			first_decel_step = total_delta - steps_to_stop;
			
			if (first_decel_step < last_accel_step) {
				double distance_to_intersection = delta - intersection_distance(speed, set_speed, max_accel, max_decel, total_delta);
				last_accel_step = ceil(distance_to_intersection);
				first_decel_step = floor(distance_to_intersection);
			}
			resolved = true;
		} else {
			total_delta += future_delta;
		}
	}
	
	void setFutureSpeed(const int32_t us_per_step) {
		double future_set_speed = 1000000.0/(double)us_per_step;

		int32_t steps_to_stop = estimate_acceleration_distance(min_speed, future_set_speed, max_decel);
		
		if (steps_to_stop > total_delta) {
			resolved = false;
		} else {
			first_decel_step = total_delta - steps_to_stop;
			if (first_decel_step < last_accel_step) {
				double distance_to_intersection = intersection_distance(min_speed, set_speed, max_accel, max_decel, total_delta);
				last_accel_step = ceil(distance_to_intersection);
				first_decel_step = floor(distance_to_intersection);
			}
			resolved = true;
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
		future_delta = 0;
		unscaled_delta = 0;
		scale = 10000;
		max_accel = 100000.0;
		max_decel = 2500.0;
		min_speed = 94;
		direction = old_direction = true;
		set_speed = 0; // 0 == stopped
		speed = 0; // 0 == stopped
		first_decel_step = 0;
		last_accel_step = 0;
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
	/// The step at which we stop accelerating, might be == delta
	volatile int32_t last_accel_step;
	/// Future movement
	volatile int32_t future_delta;
	/// Total movement, including future moves, until "Resolved"
	volatile int32_t total_delta;
	/// Maximum acceleration rate, in steps/s^2
	double max_accel;
	/// Maximum deceleration rate, ins steps/s^2
	double max_decel;
	/// Minimum speed
	double min_speed;
	/// Current _set_ speed, in steps/s
	volatile double set_speed;
	/// Current movement speed, in steps/s
	volatile double speed;
	
};

	
volatile bool is_running;
int32_t intervals;
volatile int32_t current_interval;
#define AXIS_COUNT STEPPER_COUNT
Axis axes[AXIS_COUNT]; // add a virtal axis for feedrate
volatile int8_t resolved;
#define ALL_AXIS_RESOLVED 0x1F
// To keep from over stepping for the feed rate, we might scale it
volatile int8_t feedrate_scale;
// Keep track of how many ticks are left in the current step, and how long between steps
volatile int32_t ticks_left, ticks_per_step, accelerate_ticks_left;
// Speed is in steps/second
volatile double speed;
volatile bool is_homing;
	
// These mirror those of the axes, but are overall
volatile int32_t last_accel_step = 0;
volatile int32_t first_decel_step = 0;
volatile double accel_rate;
volatile double decel_rate;
volatile double min_speed;

bool isRunning() {
	return is_running || is_homing;
}

//public:
void init(Motherboard& motherboard) {
	is_running = false;
	for (int i = 0; i < STEPPER_COUNT; i++) {
		axes[i] = Axis(motherboard.getStepperInterface(i));
	}
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
	last_accel_step = 0;
	int master_axis = 0;
	for (int i = 0; i < STEPPER_COUNT; i++) {
		axes[i].setTarget(target[i], false);
		const int32_t delta = axes[i].delta;
		// Only shut z axis on inactivity
		if (i == 2 && !holdZ) axes[i].enableStepper(delta != 0);
		else if (delta != 0) axes[i].enableStepper(true);
		if (delta > max_delta) {
			max_delta = delta;
			first_decel_step = delta;
			// grab the ending speed of the primary axis as the speed
			speed = axes[i].speed;
			min_speed = axes[i].min_speed;
			master_axis = i;
		}
	}
	
	resolved = 0;
	
	// compute number of intervals for this move
	intervals = max_delta;
	current_interval = 0;
	int32_t total_us = max_delta * dda_interval;
	const int32_t negative_half_interval = -intervals / 2;
	for (int i = 0; i < AXIS_COUNT; i++) {
		axes[i].counter = negative_half_interval;
		if (i < STEPPER_COUNT) {
			if (axes[i].delta == 0) {
				resolved |= 1 << i;
				continue;
			}
			axes[i].setSpeed(total_us / axes[i].delta); // <- this will round, that's ok
			resolved |= axes[i].resolved << i;
#if 0
			if (axes[i].last_accel_step > last_accel_step) {
				last_accel_step = axes[i].last_accel_step;
				accel_rate = axes[i].max_accel / ACCELERATE_TICKS_PER_SECOND;
			}
			if (axes[i].first_decel_step < first_decel_step) {
				first_decel_step = axes[i].first_decel_step;
				decel_rate = axes[i].max_decel / ACCELERATE_TICKS_PER_SECOND;
			}			
#endif
		}
	}

#if 1
	last_accel_step = axes[master_axis].last_accel_step;
	first_decel_step = axes[master_axis].first_decel_step;
	accel_rate = axes[master_axis].max_accel / ACCELERATE_TICKS_PER_SECOND;
	decel_rate = axes[master_axis].max_decel / ACCELERATE_TICKS_PER_SECOND;
#endif		
		
	if (speed < min_speed)
		speed = min_speed; // minimum speed
	ticks_per_step = ticks_left = (TICKS_PER_SECOND / speed);
	accelerate_ticks_left = TICKS_PER_ACCELERATE_TICK;
	
	is_running = true;
}

void setTargetNew(const Point& target, int32_t us, uint8_t relative) {
	int32_t max_delta = 0;
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
			// grab the ending speed of the primary axis as the speed
			speed = axes[i].speed;
			min_speed = axes[i].min_speed;
		}
	}
		
	// compute number of intervals for this move
	intervals = max_delta;
	current_interval = 0;
	const int32_t negative_half_interval = -intervals / 2;
	for (int i = 0; i < AXIS_COUNT; i++) {
		axes[i].counter = negative_half_interval;
		if (i < STEPPER_COUNT) {
			if (axes[i].delta == 0) {
				resolved |= 1 << i;
				continue;
			}

			axes[i].setSpeed(us / axes[i].delta); // <- this will round, that's ok
			resolved |= axes[i].resolved << i;

			if (axes[i].last_accel_step > last_accel_step) {
				last_accel_step = axes[i].last_accel_step;
				accel_rate = axes[i].max_accel / ACCELERATE_TICKS_PER_SECOND;
			}
			if (axes[i].first_decel_step < first_decel_step) {
				first_decel_step = axes[i].first_decel_step;
				decel_rate = axes[i].max_decel / ACCELERATE_TICKS_PER_SECOND;
			}			
		}
	}
	
	if (speed < min_speed)
		speed = min_speed; // minimum speed
	ticks_per_step = ticks_left = (TICKS_PER_SECOND / speed);
	accelerate_ticks_left = TICKS_PER_ACCELERATE_TICK;
	
	is_running = true;
}

	
	
bool setFutureTarget(const Point& target, int32_t dda_interval)
{
	int32_t max_delta = 0;
	for (int i = 0; i < STEPPER_COUNT; i++) {
		axes[i].setFutureTarget(target[i], false);
		const int32_t delta = axes[i].future_delta;
		if (delta > max_delta) {
			max_delta = delta;
		}
	}

	int32_t total_us = max_delta * dda_interval;
	for (int i = 0; i < STEPPER_COUNT; i++) {
		if (!axes[i].resolved)
			axes[i].setFutureSpeed(total_us / axes[i].delta); // <- this will round, that's ok

		resolved |= axes[i].resolved << i;

		if (axes[i].last_accel_step > last_accel_step) {
			last_accel_step = axes[i].last_accel_step;
			accel_rate = axes[i].max_accel / ACCELERATE_TICKS_PER_SECOND;
		}
		if (axes[i].first_decel_step < first_decel_step) {
			first_decel_step = axes[i].first_decel_step;
			decel_rate = axes[i].max_decel / ACCELERATE_TICKS_PER_SECOND;
		}
	}
	
	return resolved == ALL_AXIS_RESOLVED;
}


/// Start homing
void startHoming(const bool maximums, const uint8_t axes_enabled, const uint32_t us_per_step) {
	current_interval = 0;
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

		if (accelerate_ticks_left-- == 0) {
			if (current_interval <= last_accel_step) {
				speed += accel_rate;
			}
			else if (current_interval >= first_decel_step) {
				speed -= decel_rate;
			}
			
			ticks_per_step = (TICKS_PER_SECOND / speed);
			accelerate_ticks_left = TICKS_PER_ACCELERATE_TICK;
		}
		
		if (ticks_left-- == 0) {
			if (++current_interval == intervals) {
				is_running = false;
				double max_delta_speed = 0;
				// first, max_delta_speed is just max_delta
				for (int i = 0; i < STEPPER_COUNT; i++) {
					if (max_delta_speed < axes[i].delta) {
						max_delta_speed = axes[i].delta;
					}
				}
				// now we re-use it to include the speed
				max_delta_speed *= speed;
				for (int i = 0; i < STEPPER_COUNT; i++) {
					if (axes[i].delta == 0 || speed == 0) {
						axes[i].speed = 0;
						continue;
					}
					axes[i].speed = max_delta_speed / axes[i].delta;
					if ((axes[i].speed - axes[i].max_decel) < min_speed)
						axes[i].speed = 0;
				}				
			} else {
				bool took_step = false;
				for (int i = 0; i < STEPPER_COUNT; i++) {
					took_step |= axes[i].doInterrupt(intervals);
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
