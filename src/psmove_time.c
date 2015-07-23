
/**
* PS Move API - An interface for the PS Move Motion Controller
* Copyright (c) 2011, 2012 Thomas Perl <m@thp.io>
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*    1. Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*
*    2. Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
**/

// -- Headers ---
#include "psmove.h"
#include "psmove_private.h"

#ifdef WIN32
#include <windows.h>
#endif

// -- Constants ---
#if defined(MSVC_BUILD) || defined(__APPLE__)
#define CLOCK_MONOTONIC 0
#endif // defined(MSVC_BUILD) || defined(__APPLE__)

#if defined(MSVC_BUILD)
/* FILETIME of Jan 1 1970 00:00:00. */
static const unsigned __int64 epoch = ((unsigned __int64)116444736000000000ULL);
#endif // MSVC_BUILD

// -- Globals ---
#ifdef WIN32
LARGE_INTEGER g_startup_time = { .QuadPart = 0 };
LARGE_INTEGER g_frequency = { .QuadPart = 0 };
#else
long g_startup_time = 0;
#endif

// -- Functions ---
#if defined(MSVC_BUILD) || defined(__APPLE__)
static int gettimeofday(struct timeval * tp, struct timezone * tzp)
{
	FILETIME    file_time;
	SYSTEMTIME  system_time;
	ULARGE_INTEGER ularge;

	GetSystemTime(&system_time);
	SystemTimeToFileTime(&system_time, &file_time);

	ularge.LowPart = file_time.dwLowDateTime;
	ularge.HighPart = file_time.dwHighDateTime;

	tp->tv_sec = (long)((ularge.QuadPart - epoch) / 10000000L);
	tp->tv_usec = (long)(system_time.wMilliseconds * 1000);

	return 0;
}
#endif // MSVC_BUILD

#ifdef MSVC_BUILD
static int clock_gettime(int unused, struct timespec *ts)
{
	struct timeval tv;
	gettimeofday(&tv, NULL);

	ts->tv_sec = tv.tv_sec;
	ts->tv_nsec = tv.tv_usec * 1000;

	return 0;
}
#endif // MSVC_BUILD

void 
psmove_sleep(unsigned long milliseconds)
{
#ifdef MSVC_BUILD
	Sleep(milliseconds);
#else
	sleep(milliseconds);
#endif
}

void 
psmove_usleep(__int64 usec)
{
#ifdef MSVC_BUILD
	HANDLE timer;
	LARGE_INTEGER ft;

	ft.QuadPart = -(10 * usec); // Convert to 100 nanosecond interval, negative value indicates relative time

	timer = CreateWaitableTimer(NULL, TRUE, NULL);
	SetWaitableTimer(timer, &ft, 0, NULL, NULL, 0);
	WaitForSingleObject(timer, INFINITE);
	CloseHandle(timer);
#else
	usleep(usec);
#endif // MSVC_BUILD

}

enum PSMove_Bool 
psmove_time_init()
{
#ifdef WIN32
	psmove_return_val_if_fail(QueryPerformanceFrequency(&g_frequency), PSMove_False);
	psmove_return_val_if_fail(QueryPerformanceCounter(&g_startup_time), PSMove_False);

	return PSMove_True;
#else
	struct timeval tv;

	psmove_return_val_if_fail(gettimeofday(&tv, NULL) == 0, PSMove_False);
	g_startup_time = (tv.tv_sec * 1000 + tv.tv_usec / 1000);

	return PSMove_True;
#endif // WIN32
}

PSMove_timestamp 
psmove_timestamp()
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts;
}

PSMove_timestamp 
psmove_timestamp_diff(PSMove_timestamp a, PSMove_timestamp b)
{
	struct timespec ts;
	if (a.tv_nsec >= b.tv_nsec) {
		ts.tv_sec = a.tv_sec - b.tv_sec;
		ts.tv_nsec = a.tv_nsec - b.tv_nsec;
	}
	else {
		ts.tv_sec = a.tv_sec - b.tv_sec - 1;
		ts.tv_nsec = 1000000000 + a.tv_nsec - b.tv_nsec;
	}
	return ts;
}

double 
psmove_timestamp_value(PSMove_timestamp ts)
{
	return ts.tv_sec + ts.tv_nsec * 0.000000001;
}

long psmove_util_get_ticks()
{
#ifdef WIN32
	LARGE_INTEGER now;

	psmove_return_val_if_fail(QueryPerformanceCounter(&now), 0);

	return (long)((now.QuadPart - g_startup_time.QuadPart) * 1000 /
		g_frequency.QuadPart);
#else
	long now;
	struct timeval tv;

	psmove_return_val_if_fail(gettimeofday(&tv, NULL) == 0, 0);
	now = (tv.tv_sec * 1000 + tv.tv_usec / 1000);

	return (now - startup_time);
#endif // WIN32
}