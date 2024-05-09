/* SPDX-License-Identifier: GPL-2.0 */
/*******************************************************************************
 *** Copyright (C), 2020-2021, Awinic.All rights reserved. ************
 *******************************************************************************
 * Author        : awinic
 * Date          : 2021-09-10
 * Description   : .C file function description
 * Version       : 1.0
 * Function List :
 ******************************************************************************/
#ifndef _AW_TIMER_H_
#define _AW_TIMER_H_

#include "aw_types.h"

/* Should cause the timer to be disabled during core_get_next_timeout calls. */
#define TIMER_DISABLE_COUNT 4

/* Struct object to contain the timer related members */
struct TimerObj {
	AW_U64 starttime_;           /* Time-stamp when timer started */
	AW_U32 period_;              /* Timer period */
	AW_U8  disablecount_;        /* Auto-disable timers if they keep getting*/
					/* checked and aren't explicitly disabled.*/

};

/**
 * @brief Start the timer
 *
 * Records the current system timestamp as well as the time value
 *
 * @param Timer object
 * @param time non-zero, in units of (milliseconds / (platform scale value)).
 * @return None
 */
void TimerStart(struct TimerObj *obj, AW_U32 time);

/**
 * @brief Restart the timer using the last used delay value.
 *
 * @param Timer object
 * @return None
 */
void TimerRestart(struct TimerObj *obj);

/**
 * @brief Set time and period to zero to indicate no current period.
 *
 * @param Timer object
 * @return AW_TRUE if the timer is currently disabled
 */
void TimerDisable(struct TimerObj *obj);
AW_BOOL TimerDisabled(struct TimerObj *obj);

/**
 * @brief Determine if timer has expired
 *
 * Check the current system time stamp against (start_time + time_period)
 *
 * @param Timer object
 * @return AW_TRUE if time period has finished.
 */
AW_BOOL TimerExpired(struct TimerObj *obj);

/**
 * @brief Returns the time remaining in microseconds, or zero if disabled/done.
 *
 * @param Timer object
 */
AW_U32 TimerRemaining(struct TimerObj *obj);

#endif /* _AW_TIMER_H_ */

