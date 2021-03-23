
/*
 * Copyright (c) 2021 Actility. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *    This product includes software developed by ACTILITY.
 * 4. Neither the name of ACTILITY  nor the names of its contributors may be
 *    used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ACTILITY  "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ACTILITY  BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifdef __cplusplus
extern "C" {
#endif

//------- timespec macros	Taken from Mac OSX /usr/include/mach/clock_types.h

#define NSEC_PER_USEC	1000		/* nanoseconds per microsecond */
#define USEC_PER_SEC	1000000		/* microseconds per second */
#define NSEC_PER_SEC	1000000000	/* nanoseconds per second */
#define NSEC_PER_MSEC	1000000ull	/* nanoseconds per millisecond */

#define BAD_TIMESPEC(t)\
	((t).tv_nsec < 0 || (t).tv_nsec >= NSEC_PER_SEC)

#define NULL_TIMESPEC(t)\
	((t).tv_sec == 0 && (t).tv_nsec == 0)

/* t1 <=> t2, also (t1 - t2) in nsec with max of +- 1 sec */
#define CMP_TIMESPEC(t1, t2)\
	((t1).tv_sec > (t2).tv_sec ? +NSEC_PER_SEC :\
	((t1).tv_sec < (t2).tv_sec ? -NSEC_PER_SEC :\
			(t1).tv_nsec - (t2).tv_nsec))

/* t1  += t2 */
#define ADD_TIMESPEC(t1, t2)\
  do {\
	if (((t1).tv_nsec += (t2).tv_nsec) >= NSEC_PER_SEC) {\
		(t1).tv_nsec -= NSEC_PER_SEC;\
		(t1).tv_sec  += 1;\
	}\
	(t1).tv_sec += (t2).tv_sec;\
  } while (0)

/* dest = t1+t2 */
#define	ADD_TIMESPEC3(dest, t1, t2)\
  do {\
	dest = t1;\
	ADD_TIMESPEC(dest,t2);\
  } while (0)

/* t1  -= t2 */
#define SUB_TIMESPEC(t1, t2)\
  do {\
	if (((t1).tv_nsec -= (t2).tv_nsec) < 0) {\
		(t1).tv_nsec += NSEC_PER_SEC;\
		(t1).tv_sec  -= 1;\
	}\
	(t1).tv_sec -= (t2).tv_sec;\
  } while (0)

/* dest = t1-t2 */
#define	SUB_TIMESPEC3(dest, t1, t2)\
  do {\
	dest = t1;\
	SUB_TIMESPEC(dest,t2);\
  } while (0)

/* t1  += t2 */
#define ADD_TIMEVAL(t1, t2)\
  do {\
	if (((t1).tv_usec += (t2).tv_usec) >= USEC_PER_SEC) {\
		(t1).tv_usec -= USEC_PER_SEC;\
		(t1).tv_sec  += 1;\
	}\
	(t1).tv_sec += (t2).tv_sec;\
  } while (0)

/* t1  -= t2 */
#define SUB_TIMEVAL(t1, t2)\
  do {\
	if (((t1).tv_usec -= (t2).tv_usec) < 0) {\
		(t1).tv_usec += USEC_PER_SEC;\
		(t1).tv_sec  -= 1;\
	}\
	(t1).tv_sec -= (t2).tv_sec;\
  } while (0)


#define TIMEVAL2TIMESPEC(tv, ts)\
  do {\
	(ts).tv_nsec = (tv).tv_usec*1000;\
	(ts).tv_sec  = (tv).tv_sec;\
  } while (0)

#ifdef __cplusplus
};
#endif
