/* Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *
 *
 */

/* for task_even.c*/
#ifndef __TASK_EVENT_H
#define __TASK_EVENT_H
#include "NuMicro.h"
#include "common.h"

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t task_set_event(task_id_t tskid, uint32_t event_mask);
uint32_t task_clear_event(task_id_t tskid, uint32_t event_mask);
uint32_t task_wait_event(int port);
task_id_t task_get_current(void);
void task_wake(uint8_t tskid);

#endif  /* __TASK_EVENT_H */

