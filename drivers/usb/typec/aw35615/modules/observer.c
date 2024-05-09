// SPDX-License-Identifier: GPL-2.0
/*******************************************************************************
 **** Copyright (C), 2020-2021,Awinic.All rights reserved. ************
 *******************************************************************************
 * File Name: observer.c
 * Author: awinic
 * Date: 2021-09-10
 * Description   : .C file function description
 * Version : 1.0
 * Function List :
 ******************************************************************************/
#include "observer.h"

/**
 * Private structure for manipulation list of observers
 */
typedef struct {
	AW_U32 event;
	EventHandler event_handler;
	void *context;
} Observer_t;

/**
 * Private list of registered observers. upto MAX_OBSERVERS
 */
typedef struct {
	AW_U8 obs_count;
	Observer_t list[MAX_OBSERVERS];
} ObserversList_t;

static ObserversList_t observers = {0};

AW_BOOL register_observer(AW_U32 event, EventHandler handler, void *context)
{
	AW_BOOL status = AW_FALSE;

	if (observers.obs_count < MAX_OBSERVERS) {
		observers.list[observers.obs_count].event = event;
		observers.list[observers.obs_count].event_handler = handler;
		observers.list[observers.obs_count].context = context;
		observers.obs_count++;
		status = AW_TRUE;
	}
	return status;
}

/**
 * It is slightly expensive to remove since all pointers have to be checked
 * and moved in array when an observer is deleted.
 */
void remove_observer(EventHandler handler)
{
	AW_U32 i, j = 0;
	AW_BOOL status = AW_FALSE;

	/* Move all the observer pointers in arary and over-write the */
	/* one being deleted */
	for (i = 0; i < observers.obs_count; i++) {
		if (observers.list[i].event_handler == handler) {
			/* Object found */
			status = AW_TRUE;
			continue;
		}
		observers.list[j] = observers.list[i];
		j++;
	}

	/* If observer was found and removed decrement the count */
	if (status == AW_TRUE)
		observers.obs_count--;
}

void notify_observers(AW_U32 event, AW_U8 portId, void *app_ctx)
{
	AW_U32 i;

	for (i = 0; i < observers.obs_count; i++)
		if (observers.list[i].event & event)
			observers.list[i].event_handler(event, portId,
							observers.list[i].context, app_ctx);
}
