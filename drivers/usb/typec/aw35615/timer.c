// SPDX-License-Identifier: GPL-2.0
/*******************************************************************************
 *** Copyright (C), 2020-2021, Awinic.All rights reserved. ************
 *******************************************************************************
 * Author        : awinic
 * Date          : 2021-09-10
 * Description   : .C file function description
 * Version       : 1.0
 * Function List :
 ******************************************************************************/
#include "platform_helpers.h"
#include "timer.h"

void TimerStart(struct TimerObj *obj, AW_U32 time)
{
	/* Grab the current time stamp and store the wait period. */
	/* Time must be > 0 */
	obj->starttime_ = get_system_time_ms();
	obj->period_ = time;

	obj->disablecount_ = TIMER_DISABLE_COUNT;

	if (obj->period_ == 0)
		obj->period_ = 1;
}

void TimerRestart(struct TimerObj *obj)
{
	/* Grab the current time stamp for the next period. */
	obj->starttime_ = get_system_time_ms();
}

void TimerDisable(struct TimerObj *obj)
{
	/* Zero means disabled */
	obj->starttime_ = obj->period_ = 0;
	obj->disablecount_ = 0;
}

AW_BOOL TimerDisabled(struct TimerObj *obj)
{
	/* Zero means disabled */
	return (obj->period_ == 0) ? AW_TRUE : AW_FALSE;
}

AW_BOOL TimerExpired(struct TimerObj *obj)
{
	AW_BOOL result = AW_FALSE;

	if (TimerDisabled(obj)) {
		/* Disabled */
		/* TODO - possible cases where this return value might case issue? */
		result = AW_FALSE;
	} else {
		/* Elapsed time >= period? */
		result = ((AW_U32)(get_system_time_ms() - obj->starttime_) >=
			obj->period_) ? AW_TRUE : AW_FALSE;
	}

	/* Check for auto-disable if expired and not explicitly disabled */
	if (result)
		if (obj->disablecount_-- == 0)
			TimerDisable(obj);

	return result;
}

AW_U32 TimerRemaining(struct TimerObj *obj)
{
	AW_U64 currenttime = get_system_time_ms();

	if (TimerDisabled(obj))
		return 0;

	/* If expired before it could be handled, return a minimum delay. */
	if (TimerExpired(obj))
		return 1;

	/* Timer hasn't expired, so this should return a valid time left. */
	return (AW_U32)(obj->starttime_ + obj->period_ - currenttime);
}

