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
	int32_t rate; // interval value of the feedrate axis
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
	if (current_feedrate_index > 2)
		return;
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

#if 0
uint32_t getCurrentStep() {
	return intervals - intervals_remaining;
}

// step_return contains the next block's nominal_rate
uint32_t getCurrentFeedrateAndStep(uint32_t &step_return, uint32_t &steps_to_calc) {
	uint32_t next_start_rate = step_return;
	
	// When this is called we have a new move block to plan
	// We know only the nominal_rate of that next move
	// and that the next block will, by default, be planned from
	// this running block's nominal_rate.
	
	// We have a few scenarios to handle:
	// A- The next move's nominal_rate is as fast or faster
	//  1- We are still accelerating or in plateau
	//    Solution- We simply extend the plateau phase to the end of the move
	//  2- We have decelerated, but are still less than half-way through deceleration
	//    Solution- We accelerate at the acceleration rate back to nominal_rate
	//  3- We are more than half-way through deceleration
	//    Solution- ???
	// B- The next move's nominal_rate is slower
	//  1- We are still moving faster then the next nominal rate
	//    Solution- Set the target on the deceleration to the next nominal_rate
	//  2- We are already below the next nominal rate
	//    Solution- ???
		
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
#endif

// load up the next movment
// WARNING: called from inside the ISR, so get out fast
bool getNextMove() {
	// stepperTimingDebugPin.setValue(true);
	is_running = false; // this ensures that the interrupt does not .. interrupt us

	if (current_block != NULL) {
		current_block->flags &= ~planner::Block::Busy;
		planner::doneWithNextBlock();
		current_block = NULL;
	}
	
	if (!planner::isReady()) {
		is_running = !planner::isBufferEmpty();
		// stepperTimingDebugPin.setValue(true);
		// stepperTimingDebugPin.setValue(false);
		return false;
	}
	
	current_block = planner::getNextBlock();

	// Mark block as busy (being executed by the stepper interrupt)
	// Also mark it a locked
	current_block->flags |= planner::Block::Busy | planner::Block::Locked;
	
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
	
	int32_t local_acceleration_rate = current_block->acceleration_rate;
	uint32_t local_accelerate_until = current_block->accelerate_until;
	uint32_t local_decelerate_after = current_block->decelerate_after;
	uint32_t local_nominal_rate = current_block->nominal_rate;
	uint32_t local_final_rate = current_block->final_rate;
	
	current_feedrate_index = 0;
	int feedrate_being_setup = 0;
	// setup acceleration
	feedrate = 0;
	if (local_accelerate_until > 0) {
		feedrate = current_block->initial_rate;

		feedrate_elements[feedrate_being_setup].steps     = local_accelerate_until;
		feedrate_elements[feedrate_being_setup].rate      = local_acceleration_rate;
		feedrate_elements[feedrate_being_setup].target    = local_nominal_rate;
		feedrate_being_setup++;
	}

	// setup plateau
	if (local_decelerate_after > local_accelerate_until) {
		if (feedrate_being_setup == 0)
			feedrate = local_nominal_rate;
		
		feedrate_elements[feedrate_being_setup].steps     = local_decelerate_after - local_accelerate_until;
		feedrate_elements[feedrate_being_setup].rate      = 0;
		feedrate_elements[feedrate_being_setup].target    = local_nominal_rate;
		feedrate_being_setup++;
	}
	

	// setup deceleration
	if (local_decelerate_after < current_block->step_event_count) {
		if (feedrate_being_setup == 0)
			feedrate = local_nominal_rate;

		// To prevent "falling off the end" we will say we have a "bazillion" steps left...
		feedrate_elements[feedrate_being_setup].steps     = INT32_MAX; //current_block->step_event_count - local_decelerate_after;
		feedrate_elements[feedrate_being_setup].rate      = -local_acceleration_rate;
		feedrate_elements[feedrate_being_setup].target    = local_final_rate;
	} else {
		// and in case there wasn't a deceleration phase, we'll do the same for whichever phase was last...
		feedrate_elements[feedrate_being_setup-1].steps     = INT32_MAX;
		// We don't setup anything else because we limit to the target speed anyway.
	}
	
	// unlock the block
	current_block->flags &= ~planner::Block::Locked;
	
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

// This needs to be called with interrupts off
bool currentBlockChanged(const planner::Block *block_check) {
	// If we are here, then we are moving AND the interrupts are frozen, so get out *fast*
	
	// If the block passed in is not this block, then the planner was too slow, bail
	if (block_check != current_block) {
		return false;
	}
	// stepperTimingDebugPin.setValue(true);
	
	// We have a few scenarios to handle:
	// A- We are still accelerating, and are below current_block->accelerate_until steps
	//  Then plan as usual, and drop us in the right spot
	// B- We are still in plateau, and are below current_block->decelerate_after steps
	//  Then plan as usual, and drop us in the right spot
	// C- We are decelerating, and are still above current_block->final_rate
	//  Then set the destination speed of the deceleration phase, and call it good
	//  Kinda bad, because we slowed down too soon, but otherwise it's ok
		
	uint32_t steps_in = intervals - intervals_remaining;
	
	// clear PlannedToStop so we know we got the new plan in
	current_block->flags &= ~planner::Block::PlannedToStop;

	int32_t temp_changerate = feedrate_elements[current_feedrate_index].rate;
	int32_t local_acceleration_rate = current_block->acceleration_rate;
	uint32_t local_accelerate_until = current_block->accelerate_until;
	uint32_t local_decelerate_after = current_block->decelerate_after;
	uint32_t local_nominal_rate = current_block->nominal_rate;
	uint32_t local_final_rate = current_block->final_rate;
	

	int feedrate_being_setup = 0;
	// A- We are still accelerating. (The phase can only get longer, so we'lll assume the rest.)
	if (temp_changerate > 0) {
		// one beep
		stepperTimingDebugPin.setValue(true);
		stepperTimingDebugPin.setValue(false);
                
		// If we're accelerating, then we will only possibly extend the acceleration phase,
		// which means we have one for sure, and it has to be the first one, index 0.
		feedrate_elements[0].steps     = local_accelerate_until;
		feedrate_elements[0].rate      = local_acceleration_rate;
		feedrate_elements[0].target    = local_nominal_rate;
		
		feedrate_steps_remaining = local_accelerate_until - steps_in;
		feedrate_target = local_nominal_rate;
		feedrate_changerate = local_acceleration_rate;
		
		// leave it ready to setup plateau and deceleration
		feedrate_being_setup = 1;

		// We do the rest after the last else below
	}
	// B- We are still in plateau. (The plateau speed won't change, and won't get shorter.)
	else if (temp_changerate == 0) {
		// two beeps
		stepperTimingDebugPin.setValue(true);
		stepperTimingDebugPin.setValue(false);
		stepperTimingDebugPin.setValue(true);
		stepperTimingDebugPin.setValue(false);
		
		feedrate_steps_remaining = local_decelerate_after - steps_in;
		feedrate_target = local_nominal_rate;
		feedrate_changerate = 0;
		
		// We do the rest after the last else below
	}
	// C- We are decelerating, and are still above local_final_rate
	else if (feedrate > local_final_rate) {
		// three beeps
		stepperTimingDebugPin.setValue(true);
		stepperTimingDebugPin.setValue(false);
		stepperTimingDebugPin.setValue(true);
		stepperTimingDebugPin.setValue(false);
		stepperTimingDebugPin.setValue(true);
		stepperTimingDebugPin.setValue(false);

		feedrate_elements[0].steps     = INT32_MAX;
		feedrate_elements[0].rate      = -local_acceleration_rate;
		feedrate_elements[0].target    = local_final_rate;

		// 'Till the end of *time*, er, this move...
		feedrate_steps_remaining = INT32_MAX;
		feedrate_changerate = -local_acceleration_rate;
		feedrate_target = local_final_rate;

		return true;
	}
	// In all other cases, we got here too late. Return that we failed.
	else {
		return false;
	}
	
	current_feedrate_index = 0;
	
	// setup plateau
	if (local_decelerate_after > local_accelerate_until) {
		feedrate_elements[feedrate_being_setup].steps     = local_decelerate_after - local_accelerate_until;
		feedrate_elements[feedrate_being_setup].rate      = 0;
		feedrate_elements[feedrate_being_setup].target    = current_block->nominal_rate;
		feedrate_being_setup++;
	}
	
	// setup deceleration
	if (local_decelerate_after < current_block->step_event_count) {
		// To prevent "falling off the end" we will say we have a "bazillion" steps left...
		feedrate_elements[feedrate_being_setup].steps     = INT32_MAX; //current_block->step_event_count - local_decelerate_after;
		feedrate_elements[feedrate_being_setup].rate      = -local_acceleration_rate;
		feedrate_elements[feedrate_being_setup].target    = local_final_rate;
	} else {
		// and in case there wasn't a deceleration phase, we'll do the same for whichever phase was last...
		feedrate_elements[feedrate_being_setup-1].steps     = INT32_MAX;
		// We don't setup anything else because we limit to the target speed anyway.
	}

	// We should be setup now so that the stepper interrupt will just flow into the new plan.
	// The steppers themselves haven't changed.
	
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
	is_running = true;
	// getNextMove();
}

bool doInterrupt() {
	if (is_running) {
		if (current_block == NULL) {
			bool got_a_move = getNextMove();
			if (!got_a_move) {
				return is_running;
			}
		}
		
                // stepperTimingDebugPin.setValue(true);
		timer_counter -= INTERVAL_IN_MICROSECONDS;

		if (timer_counter < 0) {
			// if we are supposed to step too fast, we simulate double-size microsteps
			int8_t feedrate_multiplier = 1;
			timer_counter += feedrate_inverted;
			while (timer_counter < 0 && feedrate_multiplier < intervals_remaining) {
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
			
			// stepperTimingDebugPin.setValue(false);
			// stepperTimingDebugPin.setValue(true);

			if (intervals_remaining <= 0) { // should never need the < part, but just in case...
				stepperTimingDebugPin.setValue(true);
				stepperTimingDebugPin.setValue(false);
				stepperTimingDebugPin.setValue(true);
				stepperTimingDebugPin.setValue(false);
				bool got_a_move = getNextMove();
				if (!got_a_move) {
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
				// four beeps
				stepperTimingDebugPin.setValue(true);
		                stepperTimingDebugPin.setValue(false);
		                stepperTimingDebugPin.setValue(true);
		                stepperTimingDebugPin.setValue(false);
		                stepperTimingDebugPin.setValue(true);
		                stepperTimingDebugPin.setValue(false);
		                stepperTimingDebugPin.setValue(true);
		                stepperTimingDebugPin.setValue(false);
		                
				feedrate_changerate = 0;
				feedrate = feedrate_target;
			} 

		}
		
                // stepperTimingDebugPin.setValue(false);
		return is_running;
	}
	else if (is_homing) {
		timer_counter -= INTERVAL_IN_MICROSECONDS;
		if (timer_counter <= 0) {
			is_homing = false;
			// if we are supposed to step too fast, we simulate double-size microsteps
			int8_t feedrate_multiplier = 1;
			while (timer_counter <= -feedrate_inverted) {
				feedrate_multiplier++;
				timer_counter += feedrate_inverted;
			}
			
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
