// Copyright 2013, Fredrik Hultin.
// Copyright 2013, Jakob Bornecrantz.
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

/* Internal Interface for Platform Specific Functions */


#ifndef PLATFORM_H
#define PLATFORM_H

#include "openhmd.h"

double ohmd_get_tick();
void ohmd_toggle_ovr_service(int state);

typedef struct ohmd_thread ohmd_thread;
typedef struct ohmd_mutex ohmd_mutex;
typedef struct ohmd_cond ohmd_cond;

ohmd_mutex* ohmd_create_mutex(ohmd_context* ctx);
void ohmd_destroy_mutex(ohmd_mutex* mutex);

void ohmd_lock_mutex(ohmd_mutex* mutex);
void ohmd_unlock_mutex(ohmd_mutex* mutex);

ohmd_cond* ohmd_create_cond(ohmd_context* ctx);
void ohmd_destroy_cond(ohmd_cond* cond);

void ohmd_cond_wait(ohmd_cond* cond, ohmd_mutex* mutex);
void ohmd_cond_signal(ohmd_cond* cond);
void ohmd_cond_broadcast(ohmd_cond* cond);

ohmd_thread* ohmd_create_thread(ohmd_context* ctx, unsigned int (*routine)(void* arg), void* arg);
void ohmd_destroy_thread(ohmd_thread* thread);

int ohmd_ensure_path(const char* pathname);
int ohmd_read_file(const char* filename, char **out_buf, unsigned long *out_len);
int ohmd_write_file(const char* filename, char *buf, unsigned long buf_len);

/* String functions */

int findEndPoint(char* path, int endpoint);

#endif
