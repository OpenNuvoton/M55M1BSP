/*
 * This file is part of the OpenMV project.
 *
 * Copyright (c) 2013-2021 Ibrahim Abdelkader <iabdalkader@openmv.io>
 * Copyright (c) 2013-2021 Kwabena W. Agyeman <kwagyeman@openmv.io>
 *
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * Mutex implementation.
 */
#ifndef __MUTEX_INTERNAL_H__
#define __MUTEX_INTERNAL_H__
#include "mutex.h"

void mutex_init0(omv_mutex_t *mutex);
int mutex_try_lock_alternate(omv_mutex_t *mutex, uint32_t tid);
void mutex_unlock(omv_mutex_t *mutex, uint32_t tid);
#endif /* __MUTEX_INTERNAL_H__ */
