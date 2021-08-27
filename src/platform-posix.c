// Copyright 2013, Fredrik Hultin.
// Copyright 2013, Jakob Bornecrantz.
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

/* Platform Specific Functions, Unix/Posix Implementation */


#if defined(__unix__) || defined(__unix) || defined(__APPLE__) || defined(__MACH__)

#ifdef __CYGWIN__
#define CLOCK_MONOTONIC (clockid_t)4
#endif

#define _POSIX_C_SOURCE 199309L

#include <time.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <string.h>
#include <errno.h>
#include <assert.h>

#include "platform.h"
#include "openhmdi.h"

// Use clock_gettime if the system implements posix realtime timers
#ifndef CLOCK_MONOTONIC
double ohmd_get_tick()
{
	struct timeval now;
	gettimeofday(&now, NULL);
	return (double)now.tv_sec * 1.0 + (double)now.tv_usec / 1000000.0;
}
#else
double ohmd_get_tick()
{
	struct timespec now;
	clock_gettime(CLOCK_MONOTONIC, &now);
	return (double)now.tv_sec * 1.0 + (double)now.tv_nsec / 1000000000.0;
}
#endif

#ifndef CLOCK_MONOTONIC

static const uint64_t NUM_1_000_000 = 1000000;

void ohmd_monotonic_init(ohmd_context* ctx)
{
	ctx->monotonic_ticks_per_sec = NUM_1_000_000;
}

uint64_t ohmd_monotonic_get(ohmd_context* ctx)
{
	struct timeval now;
	gettimeofday(&now, NULL);
	return now.tv_sec * NUM_1_000_000 + now.tv_usec;
}

#else

static const uint64_t NUM_1_000_000_000 = 1000000000;

void ohmd_monotonic_init(ohmd_context* ctx)
{
		struct timespec ts;
		if (clock_getres(CLOCK_MONOTONIC, &ts) !=  0) {
			ctx->monotonic_ticks_per_sec = NUM_1_000_000_000;
			return;
		}

		ctx->monotonic_ticks_per_sec =
			ts.tv_nsec >= 1000 ?
			NUM_1_000_000_000 :
			NUM_1_000_000_000 / ts.tv_nsec;
}

uint64_t ohmd_monotonic_get(ohmd_context* ctx)
{
	struct timespec now;
	clock_gettime(CLOCK_MONOTONIC, &now);

	return ohmd_monotonic_conv(
		now.tv_sec * NUM_1_000_000_000 + now.tv_nsec,
		NUM_1_000_000_000,
		ctx->monotonic_ticks_per_sec);
}

#endif

OHMD_APIENTRYDLL void OHMD_APIENTRY ohmd_sleep(double seconds)
{
	struct timespec sleepfor;

	sleepfor.tv_sec = (time_t)seconds;
	sleepfor.tv_nsec = (long)((seconds - sleepfor.tv_sec) * 1000000000.0);

	nanosleep(&sleepfor, NULL);
}

// threads
struct ohmd_thread
{
	pthread_t thread;
	unsigned int (*routine)(void* arg);
	void* arg;
};

static void* pthread_wrapper(void* arg)
{
	ohmd_thread* my_thread = (ohmd_thread*)arg;
	my_thread->routine(my_thread->arg);
	return NULL;
}

ohmd_thread* ohmd_create_thread(ohmd_context* ctx, unsigned int (*routine)(void* arg), void* arg)
{
	ohmd_thread* thread = ohmd_alloc(ctx, sizeof(ohmd_thread));
	if(thread == NULL)
		return NULL;

	thread->arg = arg;
	thread->routine = routine;

	int ret = pthread_create(&thread->thread, NULL, pthread_wrapper, thread);

	if(ret != 0){
		free(thread);
		thread = NULL;
	}

	return thread;
}

ohmd_mutex* ohmd_create_mutex(ohmd_context* ctx)
{
	pthread_mutex_t* mutex = ohmd_alloc(ctx, sizeof(pthread_mutex_t));
	if(mutex == NULL)
		return NULL;

	int ret = pthread_mutex_init(mutex, NULL);

	if(ret != 0){
		free(mutex);
		mutex = NULL;
	}

	return (ohmd_mutex*)mutex;
}

void ohmd_destroy_thread(ohmd_thread* thread)
{
	pthread_join(thread->thread, NULL);
	free(thread);
}

void ohmd_destroy_mutex(ohmd_mutex* mutex)
{
	pthread_mutex_destroy((pthread_mutex_t*)mutex);
	free(mutex);
}

void ohmd_lock_mutex(ohmd_mutex* mutex)
{
	if(mutex)
		pthread_mutex_lock((pthread_mutex_t*)mutex);
}

void ohmd_unlock_mutex(ohmd_mutex* mutex)
{
	if(mutex)
		pthread_mutex_unlock((pthread_mutex_t*)mutex);
}

ohmd_cond* ohmd_create_cond(ohmd_context* ctx)
{
	pthread_cond_t* cond = ohmd_alloc(ctx, sizeof(pthread_cond_t));
	if(cond == NULL)
		return NULL;

	int ret = pthread_cond_init(cond, NULL);

	if(ret != 0){
		free(cond);
		cond = NULL;
	}

	return (ohmd_cond*)cond;
}

void ohmd_destroy_cond(ohmd_cond* cond)
{
	pthread_cond_destroy((pthread_cond_t*)cond);
	free(cond);
}

void ohmd_cond_wait(ohmd_cond* cond, ohmd_mutex* mutex)
{
	if (cond && mutex)
		pthread_cond_wait((pthread_cond_t*)cond, (pthread_mutex_t*)mutex);
}

void ohmd_cond_signal(ohmd_cond* cond)
{
	if (cond)
		pthread_cond_signal((pthread_cond_t*)cond);
}

void ohmd_cond_broadcast(ohmd_cond* cond)
{
	if (cond)
		pthread_cond_broadcast((pthread_cond_t*)cond);
}


/// Handling ovr service
void ohmd_toggle_ovr_service(int state) //State is 0 for Disable, 1 for Enable
{
	//Empty implementation
}

int findEndPoint(char* path, int endpoint)
{
	char comp[6];
	sprintf(comp,":0%d",endpoint);
	if (strstr(path, comp) != NULL) {
		return 1;
	}
	return 0;
}

int ohmd_read_file(const char* filename, char **out_buf, unsigned long *out_len)
{
	FILE* f = fopen(filename, "rb");
  long file_len, total_read = 0;
  char *buffer;
  int ret;

  if (f == NULL)
    return errno;

	ret = fseek(f, 0, SEEK_END);
  if (ret != 0)
    goto fail;

	file_len = ftell(f);
  if (file_len < 0) {
    ret = errno;
    goto fail;
  }

	ret = fseek(f, 0, SEEK_SET);

	buffer = malloc(file_len + 1);
  buffer[file_len] = 0;

  while (total_read < file_len) {
	  size_t n_read = fread(buffer, file_len, 1, f);
    if (n_read > 0) {
      total_read += file_len;
    } else {
      if (feof(f))
        break;
      ret = errno;
      if (ret != EINTR)
        goto fail;
    }
  }

  *out_buf = buffer;
  *out_len = (unsigned long) file_len;
fail:
	fclose(f);
	return ret;
}

int ohmd_ensure_path(const char* pathname)
{
	int ret;
	struct stat sb;
	char pathbuf[OHMD_STR_SIZE+1];
	char *separator, *cur;

	/* Path must be at least 2 chars */
	assert (strlen(pathname) > 1);

	/* Check if the path exists */
	ret = stat(pathname, &sb);
	if (ret < 0 && errno != ENOENT)
		return errno;
	if (ret == 0) {
		if (!S_ISDIR(sb.st_mode)) {
			return errno;
		}
		return 0; /* Success. It already exists and is a dir */
	}

	/* Recursively create all components of a path name */
	strncpy(pathbuf, pathname, OHMD_STR_SIZE);
	cur = pathbuf + 1;
	while ((separator = strstr(cur, "/")) != NULL) {
		*separator = '\0'; /* Shorten the string to just the path so far */

		/* Check if the path exists */
		ret = stat(pathbuf, &sb);
		if (ret != 0) {
			if (errno != ENOENT)
				return errno;
			/* This path doesn't exist, create it */
			if ((ret = mkdir(pathbuf, 0777)) != 0)
				return ret;
		} else if (!S_ISDIR(sb.st_mode)) {
			return ENOTDIR;
		}

		/* Replace the separator and advance to the next */
		*separator = '/';
		cur = separator + 1;
	}

	/* Create the final path component */
	return mkdir(pathname, 0777);
}

int ohmd_write_file(const char* filename, char *buf, unsigned long buf_len)
{
	FILE* f = fopen(filename, "w");

	if (f == NULL)
		return errno;

	if (fwrite(buf, 1, buf_len, f) != 1) {
		if (ferror(f))
			return errno;
	}

	return fclose(f);
}

const char *ohmd_get_config_dir(ohmd_context *ctx)
{
  char *env;

  if (ctx->config_dir[0] != 0)
    return ctx->config_dir;

  env = getenv("XDG_CONFIG_HOME");
  if (env) {
    snprintf (ctx->config_dir, OHMD_STR_SIZE, "%s/openhmd", env);
    return ctx->config_dir;
  }
  env = getenv("HOME");
  if (env) {
    snprintf (ctx->config_dir, OHMD_STR_SIZE, "%s/.config/openhmd", env);
    return ctx->config_dir;
  }

  snprintf (ctx->config_dir, OHMD_STR_SIZE, "/tmp/openhmd");
  return ctx->config_dir;
}

#endif
