
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

/*! @file main.c
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
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

#include "define.h"
#include "struct.h"



void	AvDvUiClear(t_avdv_ui *ad)
{
	memset	(ad,0,sizeof(t_avdv_ui));
}

void	AvDvUiAdd(t_avdv_ui *ad,uint32_t val,time_t when)
{
	if	(!ad || !when)
		return;
	ad->ad_time[ad->ad_slot % AVDV_NBELEM]	= when;
	ad->ad_hist[ad->ad_slot % AVDV_NBELEM]	= val;
	ad->ad_slot++;

	if	(val > ad->ad_vmax)
	{
		ad->ad_vmax	= val;
		ad->ad_tmax	= when;
	}
}

int	AvDvUiCompute(t_avdv_ui *ad,time_t tmax,time_t when)
{
	int		i;
	int		nb;
	uint32_t	histo[AVDV_NBELEM];
	uint32_t	total;
	int32_t		diff;

	ad->ad_vmax	= 0;
	ad->ad_tmax	= 0;

	nb		= 0;
	total		= 0;
	for	(i = 0 ; i < AVDV_NBELEM && ad->ad_time[i] ; i++)
	{
		if	(tmax && when && ABS(when - ad->ad_time[i]) > tmax)
			continue;
		histo[nb]	= ad->ad_hist[i];
		total		= total + histo[nb];
		if	(histo[nb] > ad->ad_vmax)
		{
			ad->ad_vmax	= ad->ad_hist[i];
			ad->ad_tmax	= ad->ad_time[i];
		}
		nb++;
	}
	if	(nb <= 0)
	{
		ad->ad_aver	= 0;
		ad->ad_sdev	= 0;
		return	nb;
	}

	ad->ad_aver	= total	/ nb;

	if	(nb <= 1)
	{
		ad->ad_sdev	= 0;
		return	nb;
	}

	total	= 0;
	for	(i = 0 ; i < nb ; i++)
	{
		diff	= histo[i] - ad->ad_aver;
		total	= total + (diff * diff);
	}

	total	= total / nb;
	ad->ad_sdev	= (uint32_t)sqrt((double)total);

	return	nb;
}


