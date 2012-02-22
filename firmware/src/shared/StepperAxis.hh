#ifndef STEPPERAXIS_HH
#define STEPPERAXIS_HH

#include "StepperInterface.hh"
#include "Configuration.hh"

/// The stepper axis module implements a driver for a single stepper axis. It is designed
/// to be accessed via the Steppers namespace, and uses a StepperInterface to talk to the
/// actual hardware.
/// \ingroup SoftwareLibraries
class StepperAxis
{
public:
        StepperInterface* interface;         ///< Interface this axis is connected to
        volatile int32_t position;           ///< Current position of this axis, in steps
        int32_t minimum;                     ///< Minimum position, in steps
        int32_t maximum;                     ///< Maximum position, in steps
        volatile int32_t target;             ///< Target position, in steps
        volatile int32_t counter;            ///< Step counter; represents the proportion of
                                             ///< a step so far passed.  When the counter hits
                                             ///< zero, a step is taken.
        volatile int32_t delta;              ///< Amount to increment counter per tick
        volatile bool direction;             ///< True for positive, false for negative
        volatile int8_t  step_multiplier;    ///< Used to simulate dynamic microstep switching, must be > 0 and 2^N
        volatile int8_t  step_change;        ///< Uses internally. step_change = direction ? step_multiplier : -step_multiplier;
#if defined(SINGLE_SWITCH_ENDSTOPS) && (SINGLE_SWITCH_ENDSTOPS == 1)
        volatile bool prev_direction;        ///< Record the previous direction for endstop detection
        volatile int32_t endstop_play;       ///< Amount to move while endstop triggered, to see which way to move
                                          
        enum endstop_status_t {              ///< State of the endstop
            ESS_UNKNOWN,
            ESS_TRAVELING,
            ESS_AT_MAXIMUM,
            ESS_AT_MINIMUM
        };
        
        volatile endstop_status_t endstop_status;

        // If we started with an endstop triggered, then we don't know where 
        // we are. We can go this many steps either way until we find out.
        const static uint16_t ENDSTOP_DEFAULT_PLAY =10000;
        const static uint16_t ENDSTOP_DEBOUNCE =20;

#endif //SINGLE_SWITCH_ENDSTOPS
        // Return true if the endstop for the current direction is triggered.
        inline bool checkEndstop(const bool isHoming);

public:
        /// Construct a stepper axis with a null interface
        StepperAxis();

        /// Construct a stepper axis, using the given stepper
        /// interface
        /// \param[in] Stepper interface to use
        StepperAxis(StepperInterface& stepper_interface);

        /// Set the target position for the axis to travel to.
        /// \param[in] target_in Postion to move to, in steps
        /// \param[in] relative If true, consider the target position
        ///                     to be relative to the current position.
        void setTarget(const int32_t target_in, bool relative);

        /// Set the step multiplier
        /// \param[in] new_multiplier
        void setStepMultiplier(const int8_t new_multiplier);

        /// Start a homing procedure
        /// \param[in] direction_in If true, home in the positive direction.
        void setHoming(const bool direction_in);

        /// Reset the axis position to the given position.
        /// \param[in] position_in New axis position
        void definePosition(const int32_t position_in);

        /// Set whether the stepper motor driver on the given axis should be enabled
        /// \param[in] enable If true, enable the axis; otherwise, disable it.
        void enableStepper(bool enable);

        /// Reset to initial state
        void reset();

        /// Handle interrupt for the given axis.
        /// \param[in] intervals Intervals that have passed since the previous interrupt
        inline void doInterrupt(const int32_t intervals);

        /// Run the next step of the homing procedure.
        /// \param[in] intervals Intervals that have passed since the previous interrupt
        /// \return True if the axis is still homing.
        bool doHoming(const int32_t intervals);
};


bool StepperAxis::checkEndstop(const bool isHoming) {
#if defined(SINGLE_SWITCH_ENDSTOPS) && (SINGLE_SWITCH_ENDSTOPS == 1)
        bool hit_endstop = interface->isAtMinimum();
  // We must move at least ENDSTOP_DEBOUNCE from where we hit the endstop before we declare traveling
        if (hit_endstop || (endstop_status == (direction?ESS_AT_MAXIMUM:ESS_AT_MINIMUM) && (endstop_play < ENDSTOP_DEFAULT_PLAY - ENDSTOP_DEBOUNCE))) {
                hit_endstop = true;
    // Did we *just* hit the endstop?
                if (endstop_status == ESS_TRAVELING || (isHoming && endstop_status == ESS_UNKNOWN)) {
                        endstop_play   = ENDSTOP_DEFAULT_PLAY;
                        if (isHoming?direction:prev_direction)
                                endstop_status = ESS_AT_MAXIMUM;
                        else
                                endstop_status = ESS_AT_MINIMUM;

      // OR, are we traveling away from the endstop we just hit and still have play...
                } else if ((direction && endstop_status != ESS_AT_MAXIMUM) || (!direction && endstop_status != ESS_AT_MINIMUM)) {
                        if (endstop_play > 0) {
                                --endstop_play;
                                hit_endstop = false; // pretend this never happened...
                        } else {
        // we ran out of play, so we must be ramming into the side, switch directions
        // endstop_status = !direction ? ESS_AT_MAXIMUM : ESS_AT_MINIMUM;
        // endstop_play   = ENDSTOP_DEFAULT_PLAY;
                        }
                }
    // otherwise we hit the endstop

    // but if we didn't hit an endstop, clear the status
        } else {
                endstop_status = ESS_TRAVELING;
                if (!isHoming) {
                        endstop_play   = ENDSTOP_DEFAULT_PLAY;
                }
        }
        prev_direction = direction;
        return hit_endstop;
#else
        return direction ? interface->isAtMaximum() : interface->isAtMinimum();
#endif
}

void StepperAxis::doInterrupt(const int32_t intervals) {
	bool hit_endstop = false;
	bool checked_endstop = false;
	for (int8_t steps = step_multiplier; steps > 0; steps--) {
		counter += delta;

		if (counter >= 0) {
			counter -= intervals;
			if (!checked_endstop) {
				checked_endstop = true;
				hit_endstop = checkEndstop(false);
			}
			if (!hit_endstop) {
				interface->step(true);
				interface->step(false);
			} else {
				break; // stop wasting time, we hit an endstop
			}
		}
	}
	// note that if the step_multiplier is > 1, and we hit endstops, we'll be a little wrong
	position += step_change;
}


#endif // STEPPERAXIS_HH