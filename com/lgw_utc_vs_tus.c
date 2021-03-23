
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

/*! @file lgw.c
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <math.h>
#include <pthread.h>
#include <errno.h>
#include <limits.h>
#include <signal.h>
#include <poll.h>
#include <ctype.h>
#ifndef MACOSX
#include <malloc.h>
#endif

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

/* no more use of PCAP */
#undef	PCAP_OK

#ifdef  PCAP_OK
#include <pcap/pcap.h>
#endif

#include "rtlbase.h"
#include "rtlimsg.h"
#include "rtllist.h"
#include "rtlhtbl.h"

#include "timeoper.h"

#include "semtech.h"
#include "headerloramac.h"

#include "xlap.h"
#include "struct.h"
#include "define.h"
#include "infrastruct.h"
#include "cproto.h"
#include "extern.h"


int	UtcVsTus	= 0;


//#define CLOCKID CLOCK_MONOTONIC
#define CLOCKID CLOCK_REALTIME

static	void timespec_diff(struct timespec *start, struct timespec *stop,
			struct timespec *result)
{
	if ((stop->tv_nsec - start->tv_nsec) < 0) 
	{
		result->tv_sec = stop->tv_sec - start->tv_sec - 1;
		result->tv_nsec = stop->tv_nsec - start->tv_nsec + 1000000000UL;
	}
	else
	{
		result->tv_sec = stop->tv_sec - start->tv_sec;
		result->tv_nsec = stop->tv_nsec - start->tv_nsec;
	}

    return;
}

#define	NB_SAMPLES	100

static	void	LoopGetClock()
{
	u_int	tus[NB_SAMPLES];
	int	dtus;

	struct	timespec	utc[NB_SAMPLES];
	struct	timespec	dutc;
	int64_t			dns;

	int	i;
	int	nb = NB_SAMPLES;
	double	ratio;

	for	(i = 0 ; i < nb ; i++)
	{
		clock_gettime(CLOCKID,&utc[i]);
		LgwGetTrigNow(0,&tus[i]);
		usleep(1000000);	// 1 sec
	}

	memset	(&dutc,0,sizeof(dutc));
	for	(i = 1 ; i < nb ; i++)
	{
		timespec_diff(&utc[i-1],&utc[i],&dutc);
		dns	= (dutc.tv_sec * 1E9) + dutc.tv_nsec;
		dtus	= tus[i] - tus[i-1];
		if	(dtus == 0)	continue;
		ratio	= (double)dns / (double)dtus;
		ratio	= ratio / 1000.0;	// ns vs us
	printf("[%03d] utc=%ld.%09ld ~utc=%010ld tus=%010u ~tus=%d ratio=%f\n",
			i,utc[i].tv_sec,utc[i].tv_nsec,
			dns,tus[i],dtus,ratio);
	}
}

void	LgwUtcVsTus()
{
	struct timespec resol;

	if (clock_getres(CLOCKID,&resol) == -1)
	{
               return;
	}
	printf("resol %ld.%09ld\n",resol.tv_sec,resol.tv_nsec);

	LoopGetClock();
}

