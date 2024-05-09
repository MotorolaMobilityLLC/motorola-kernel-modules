/* SPDX-License-Identifier: GPL-2.0 */
/*******************************************************************************
 **** Copyright (C), 2020-2021, All rights reserved. ************
 *******************************************************************************
 * File Name     : observer.h
 * Author        : awinic
 * Date          : 2021-09-10
 * Description   : .C file function description
 * Version       : 1.0
 * Function List :
 *******************************************************************************/
#ifndef MODULES_OBSERVER_H_
#define MODULES_OBSERVER_H_
#include "../aw_types.h"

#ifndef MAX_OBSERVERS
#define MAX_OBSERVERS       10
#endif ///< MAX_OBSERVERS

typedef void (*EventHandler)(AW_U32 event, AW_U8 portId, void *usr_ctx, void *app_ctx);

/**
 * @brief register an observer.
 * @param[in] event to subscribe
 * @param[in] handler to be called
 * @param[in] context data sent to the handler
 */
AW_BOOL register_observer(AW_U32 event, EventHandler handler, void *context);

/**
 * @brief removes the observer. Observer stops getting notified
 * @param[in] handler to remove
 */
void remove_observer(EventHandler handler);

/**
 * @brief notifies all observer that are listening to the event.
 * @param[in] event that occured
 */
void notify_observers(AW_U32 event, AW_U8 portId, void *app_ctx);


#endif /* MODULES_OBSERVER_H_ */
