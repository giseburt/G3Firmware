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
#include "StepperAxis.hh"
#include "Planner.hh"
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <avr/interrupt.h> // sei()

namespace steppers {

volatile bool is_running;
int32_t intervals;
volatile int32_t intervals_remaining;

struct feedrate_element {
	uint32_t rate; // interval value of the feedrate axis
	uint32_t steps;     // number of steps of the master axis to change
	uint32_t target;
};
feedrate_element feedrate_elements[4]; // one extra in case of planning emergency
volatile int32_t feedrate_steps_remaining;
volatile int32_t feedrate;
volatile int32_t feedrate_target; // convenient storage to save lookup time
volatile int8_t  feedrate_dirty; // indicates if the feedrate_inverted needs recalculated
volatile int32_t feedrate_inverted;
volatile int32_t feedrate_changerate;
volatile int32_t acceleration_tick_counter;
volatile uint8_t current_feedrate_index;

volatile int32_t timer_counter;
StepperAxis axes[STEPPER_COUNT];
volatile bool is_homing;

Pin stepperTimingDebugPin = STEPPER_TIMER_DEBUG;

bool holdZ = false;

planner::Block *current_block;

bool isRunning() {
	return is_running || is_homing || !planner::isBufferEmpty();
}

//public:
void init(Motherboard& motherboard) {
	is_running = false;
	is_homing = false;
	for (int i = 0; i < STEPPER_COUNT; i++) {
		axes[i] = StepperAxis(motherboard.getStepperInterface(i));
	}
	timer_counter = 0;

	current_block = NULL;
	
	for (int i = 0; i < 3; i++) {
		feedrate_elements[i] = feedrate_element();
		feedrate_elements[i].rate = 0;
		feedrate_elements[i].target = 0;
		feedrate_elements[i].steps = 0;
	}
	
	feedrate_steps_remaining = 0;
	feedrate = 0;
	feedrate_inverted = 0;
	feedrate_dirty = 1;
	acceleration_tick_counter = 0;
	current_feedrate_index = 0;
	
	stepperTimingDebugPin.setDirection(true);
	stepperTimingDebugPin.setValue(false);
}

void abort() {
	is_running = false;
	is_homing = false;
	timer_counter = 0;
	current_block = NULL;
	feedrate_steps_remaining = 0;
	feedrate = 0;
	feedrate_inverted = 0;
	feedrate_dirty = 1;
	acceleration_tick_counter = 0;
	current_feedrate_index = 0;
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
	#if STEPPER_COUNT > 4
		return Point(axes[0].position,axes[1].position,axes[2].position,axes[3].position,axes[4].position);
	#else
		return Point(axes[0].position,axes[1].position,axes[2].position,axes[3].position,0);
	#endif
#else
	return Point(axes[0].position,axes[1].position,axes[2].position);
#endif
}

void setHoldZ(bool holdZ_in) {
	holdZ = holdZ_in;
}

inline void prepareFeedrateIntervals() {
	// if (current_feedrate_index > 2)
	// 	return;
	feedrate_steps_remaining  = feedrate_elements[current_feedrate_index].steps;
	feedrate_changerate       = feedrate_elements[current_feedrate_index].rate;
	feedrate_target           = feedrate_elements[current_feedrate_index].target;
	// feedrate_dirty = 1;
	// acceleration_tick_counter = 0;
}

inline void recalcFeedrate() {
	if (feedrate == 0)
		return; // SHRIEK!
	feedrate_inverted = 1000000/feedrate;

	feedrate_dirty = 0;
}

uint32_t getCurrentStep() {
	return intervals - intervals_remaining;
}

// step_return contains the next block's nominal_rate
uint32_t getCurrentFeedrateAndStep(uint32_t &step_return, uint32_t &steps_to_calc) {
	uint32_t next_nominal_rate = step_return;
	
	// We need 
	
	// when we get here, we need to get a currentBlockChanged soon...
	// but, it might not come before the end of the move.
	step_return = intervals - intervals_remaining;
	steps_to_calc = MAX_TIME_TO_RECALCULATE_BLOCK / feedrate_inverted;
	
	
	
	// Lock the feedrate for the amount of time we estimate the recalc to take
	feedrate_changerate = 0;
	feedrate_steps_remaining = steps_to_calc;
	
	// Just in case we go over, lock it in for even more time...
	// We *really* don't want it to hit this, but it's better than crashing.
	// We give ourselves 3 chances to catch it...
	current_feedrate_index = 0;
	feedrate_elements[0].rate = 0;
	feedrate_elements[0].steps = 10;
	feedrate_elements[1].rate = 0;
	feedrate_elements[1].steps = 10;
	feedrate_elements[2].rate = 0;
	feedrate_elements[2].steps = 10;
	feedrate_elements[3].rate = 0;
	feedrate_elements[3].steps = INT32_MAX;
	
	acceleration_tick_counter = 0; // insure that, once the counter is up, the feedrate is recalculated immediately
	return feedrate;
}

// load up the next movment
// WARNING: called from inside the ISR, so get out fast
bool getNextMove() {
	// stepperTimingDebugPin.setValue(true);
	is_running = false; // this ensures that the interrupt does not .. interrupt us
	if (current_block != NULL) {
		// if ((current_block->flags & planner::Block::PlannedToStop) && planner::bufferCount() > 1) {
		// 	// keep running -- we have something in the planner
		// 	is_running = true;
		// 	return false;
		// }
		
		current_block->flags &= ~planner::Block::Busy;
		planner::doneWithNextBlock();
		current_block = NULL;
	}
	
	if (planner::isBufferEmpty()) {
		// stepperTimingDebugPin.setValue(true);
		// stepperTimingDebugPin.setValue(false);
		return false;
	}
	
	current_block = planner::getNextBlock();
	// Mark block as busy (being executed by the stepper interrupt)
	current_block->flags |= planner::Block::Busy;
	
	Point &target = current_block->target;
	
	int32_t max_delta = current_block->step_event_count;
// Unroll the folling into the part after the loop
#if 0
	for (int i = 0; i < STEPPER_COUNT; i++) {
		axes[i].setTarget(target[i], false);
		const int32_t delta = axes[i].delta;

		// Only shut z axis on inactivity
		if (i == 2 && !holdZ) axes[i].enableStepper(delta != 0);
		else if (delta != 0) axes[i].enableStepper(true);
		
		// if (delta > max_delta) {
		// 	max_delta = delta;
		// }
	}
#else
	//X
	axes[0].setTarget(target[0], false);
	if (axes[0].delta != 0) axes[0].enableStepper(true);
	//Y
	axes[1].setTarget(target[1], false);
	if (axes[1].delta != 0) axes[1].enableStepper(true);
	//Z
	axes[2].setTarget(target[2], false);
	// Disable z axis on inactivity, unless holdZ is true
	if (axes[2].delta != 0)
		axes[2].enableStepper(true);
	else if (!holdZ)
		axes[2].enableStepper(false);

#if STEPPER_COUNT > 3
	axes[3].setTarget(target[3], false);
	if (axes[3].delta != 0) axes[3].enableStepper(true);
#endif
#if STEPPER_COUNT > 4
	axes[4].setTarget(target[4], false);
	if (axes[4].delta != 0) axes[4].enableStepper(true);
#endif

#endif
		
	current_feedrate_index = 0;
	int feedrate_being_setup = 0;
	// setup acceleration
	feedrate = 0;
	if (current_block->accelerate_until > 0) {
		feedrate = current_block->initial_rate;

		feedrate_elements[feedrate_being_setup].steps     = current_block->accelerate_until;
		feedrate_elements[feedrate_being_setup].rate      = current_block->acceleration_rate;
		feedrate_elements[feedrate_being_setup].target    = current_block->nominal_rate;
		feedrate_being_setup++;
	}

	// setup plateau
	if (current_block->decelerate_after > current_block->accelerate_until) {
		if (feedrate_being_setup == 0)
			feedrate = current_block->nominal_rate;
		
		feedrate_elements[feedrate_being_setup].steps     = current_block->decelerate_after - current_block->accelerate_until;
		feedrate_elements[feedrate_being_setup].rate      = 0;
		feedrate_elements[feedrate_being_setup].target    = current_block->nominal_rate;
		feedrate_being_setup++;
	}
	

	// setup deceleration
	if (current_block->decelerate_after < current_block->step_event_count) {
		if (feedrate_being_setup == 0)
			feedrate = current_block->nominal_rate;

		// To prevent "falling off the end" we will say we have a "bazillion" steps left...
		feedrate_elements[feedrate_being_setup].steps     = INT32_MAX; //current_block->step_event_count - current_block->decelerate_after;
		feedrate_elements[feedrate_being_setup].rate      = -current_block->acceleration_rate;
		feedrate_elements[feedrate_being_setup].target    = current_block->final_rate;
	} else {
		// and in case there wasn't a deceleration phase, we'll do the same for whichever phase was last...
		feedrate_elements[feedrate_being_setup-1].steps     = INT32_MAX;
		// We don't setup anything else because we limit to the target speed anyway.
	}
	
	if (feedrate == 0) {
		is_running = false;
		return false;
	}
	
	prepareFeedrateIntervals();
	recalcFeedrate();
	acceleration_tick_counter = TICKS_PER_ACCELERATION;
	
	timer_counter = 0;

	intervals = max_delta;
	intervals_remaining = intervals;
	const int32_t negative_half_interval = -(intervals>>1);
	axes[0].counter = negative_half_interval;
	axes[1].counter = negative_half_interval;
	axes[2].counter = negative_half_interval;
#if STEPPER_COUNT > 3
	axes[3].counter = negative_half_interval;
#endif
#if STEPPER_COUNT > 4
	axes[4].counter = negative_half_interval;
#endif
	is_running = true;
	
	// stepperTimingDebugPin.setValue(false);
	return true;
}

// This needs to be called with interuupts off
bool currentBlockChanged(const planner::Block *block_check) {
	// If we are here, then we are moving AND the interrupts are frozen, so get out *fast*
	if (block_check != current_block) {
		return false;
	}
	// stepperTimingDebugPin.setValue(true);
	
	// clear PlannedToStop so we know we got the new plan in
	current_block->flags &= ~planner::Block::PlannedToStop;
	
	current_feedrate_index = 0;
	// We use one here so that later, current_feedrate_index++ means this one.
	// We also reserve a fourth feedrate_elements[] for this occasion as well.
	int feedrate_being_setup = 1;
	// setup acceleration
	feedrate = 0;
	if (current_block->accelerate_until > 0) {
		feedrate = current_block->initial_rate;

		feedrate_elements[feedrate_being_setup].steps     = current_block->accelerate_until;
		feedrate_elements[feedrate_being_setup].rate      = current_block->acceleration_rate;
		feedrate_elements[feedrate_being_setup].target    = current_block->nominal_rate;
		feedrate_being_setup++;
	}

	// setup plateau
	if (current_block->decelerate_after > current_block->accelerate_until) {
		if (feedrate_being_setup == 0)
			feedrate = current_block->nominal_rate;
		
		feedrate_elements[feedrate_being_setup].steps     = current_block->decelerate_after - current_block->accelerate_until;
		feedrate_elements[feedrate_being_setup].rate      = 0;
		feedrate_elements[feedrate_being_setup].target    = current_block->nominal_rate;
		feedrate_being_setup++;
	}
	

	// setup deceleration
	if (current_block->decelerate_after < current_block->step_event_count) {
		if (feedrate_being_setup == 0)
			feedrate = current_block->nominal_rate;

		// To prevent "falling off the end" we will say we have a "bazillion" steps left...
		feedrate_elements[feedrate_being_setup].steps     = INT32_MAX; //current_block->step_event_count - current_block->decelerate_after;
		feedrate_elements[feedrate_being_setup].rate      = -current_block->acceleration_rate;
		feedrate_elements[feedrate_being_setup].target    = current_block->final_rate;
	} else {
		// and in case there wasn't a deceleration phase, we'll do the same for whichever phase was last...
		feedrate_elements[feedrate_being_setup-1].steps     = INT32_MAX;
		// We don't setup anything else because we limit to the target speed anyway.
	}

	// We set a feedrate_steps_remaining earlier that we need to time out first...
	// prepareFeedrateIntervals();
	// recalcFeedrate();
	// 
	// timer_counter = 0;
	
	// the steppers themselves haven't changed...
	
	// stepperTimingDebugPin.setValue(false);
	return true;
}

/// Start homing
void startHoming(const bool maximums, const uint8_t axes_enabled, const uint32_t us_per_step) {
	intervals_remaining = INT32_MAX;
	intervals = 1;
	feedrate_inverted = us_per_step;
	// ToDo: Return to using the interval if the us_per_step > INTERVAL_IN_MICROSECONDS
	const int32_t negative_half_interval = -1;
	for (int i = 0; i < STEPPER_COUNT; i++) {
		axes[i].counter = negative_half_interval;
		if ((axes_enabled & (1<<i)) != 0) {
			axes[i].setHoming(maximums);
		} else {
			axes[i].delta = 0;
		}
	}
	timer_counter = feedrate_inverted;
	is_homing = true;
}

/// Enable/disable the given axis.
void enableAxis(uint8_t index, bool enable) {
	if (index < STEPPER_COUNT) {
		axes[index].enableStepper(enable);
	}
}

void startRunning() {
	if (is_running)
		return;
	// is_running = true;
	getNextMove();
}

bool doInterrupt() {
	if (is_running) {
                stepperTimingDebugPin.setValue(true);
		timer_counter -= INTERVAL_IN_MICROSECONDS;

		if (timer_counter < 0) {
			// if we are supposed to step too fast, we simulate double-size microsteps
			int8_t feedrate_multiplier = 1;
			timer_counter += feedrate_inverted;
			while (timer_counter < -feedrate_inverted && feedrate_multiplier < intervals_remaining) {
				feedrate_multiplier++;
				timer_counter += feedrate_inverted;
			}
			
			axes[0].doInterrupt(intervals, feedrate_multiplier);
			axes[1].doInterrupt(intervals, feedrate_multiplier);
			axes[2].doInterrupt(intervals, feedrate_multiplier);
#if STEPPER_COUNT > 3
			axes[3].doInterrupt(intervals, feedrate_multiplier);
#endif
#if STEPPER_COUNT > 4
			axes[4].doInterrupt(intervals, feedrate_multiplier);
#endif
			
			intervals_remaining -= feedrate_multiplier;
			
			stepperTimingDebugPin.setValue(false);
			stepperTimingDebugPin.setValue(true);

			if (intervals_remaining <= 0) { // should never need the < part, but just in case...
				stepperTimingDebugPin.setValue(false);
				stepperTimingDebugPin.setValue(true);
				// sei(); // allow interrupts again
				bool got_a_move = getNextMove();
				if (!got_a_move) {
	                                stepperTimingDebugPin.setValue(false);
					return is_running;
				}
			}
			
			if ((feedrate_steps_remaining-=feedrate_multiplier) <= 0) {
				current_feedrate_index++;
				// stepperTimingDebugPin.setValue(true);
				// stepperTimingDebugPin.setValue(false);
				prepareFeedrateIntervals();
			}
			
			if (feedrate_dirty) {
				recalcFeedrate();
			}
		}
		
		if (feedrate_changerate != 0 && acceleration_tick_counter-- <= 0) {
			acceleration_tick_counter = TICKS_PER_ACCELERATION;
			// Change our feedrate. Here it's important to note that we can over/undershoot

			feedrate += feedrate_changerate;
			feedrate_dirty = 1;
		
			if ((feedrate_changerate > 0 && feedrate > feedrate_target)
			    || (feedrate_changerate < 0 && feedrate < feedrate_target)) {
				feedrate_changerate = 0;
				feedrate = feedrate_target;
			} 

		}
		
                stepperTimingDebugPin.setValue(false);
		return is_running;
	} else if (is_homing) {
		timer_counter -= INTERVAL_IN_MICROSECONDS;
		if (timer_counter <= 0) {
			is_homing = false;
			// if we are supposed to step too fast, we simulate double-size microsteps
			int8_t feedrate_multiplier = 1;
			while (timer_counter <= -feedrate_inverted) {
				feedrate_multiplier++;
				timer_counter += feedrate_inverted;
			}

			stepperTimingDebugPin.setValue(false);
			stepperTimingDebugPin.setValue(true);
			
			// Warning: do N || is_homing
			// is_homing || N will not execute N if is_homing
			is_homing = axes[0].doHoming(1, feedrate_multiplier) || is_homing;
			is_homing = axes[1].doHoming(1, feedrate_multiplier) || is_homing;
			is_homing = axes[2].doHoming(1, feedrate_multiplier) || is_homing;
	#if STEPPER_COUNT > 3
			is_homing = axes[3].doHoming(1, feedrate_multiplier) || is_homing;
	#endif
	#if STEPPER_COUNT > 4
			is_homing = axes[4].doHoming(1, feedrate_multiplier) || is_homing;
	#endif
			// if we're done, force a sync with the planner
			if (!is_homing)
				planner::abort();
			
			timer_counter += feedrate_inverted;
		}
		return is_homing;
	}
	return false;
}

} // namespace steppers
