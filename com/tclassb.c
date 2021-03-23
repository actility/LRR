
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

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <ctype.h>
#include <sys/types.h>

#include "define.h"

#define	TRACE

typedef	struct	s_lrr_pkt
{
	u_int	lp_gss;		// requested downlink time gss,gns utc
	u_int	lp_gns;


	u_short	lp_firstslot;	// input from LRC
	u_short	lp_firstslot2;	// input from LRC
	u_short	lp_nbslot;	// input from LRC
	u_short	lp_currslot;	// first slot (lp_firstslot or lp_firstslot2)
	u_int	lp_gss0;	// input from LRC
	u_int	lp_gns0;	// input from LRC
	u_short	lp_idxtry;
	u_short	lp_nbtry;
	u_short	lp_maxtry;		
}	t_lrr_pkt;

int	LgwPacketDelayMsFromUtc(t_lrr_pkt *downpkt,struct timespec *utc)
{
	double	fdelay;

	fdelay	= (double)(downpkt->lp_gss) - (double)(utc->tv_sec);

	fdelay	+= 1E-9 * ((double)(downpkt->lp_gns) - (double)(utc->tv_nsec));

	fdelay	*= 1000;	// in ms

	return	(int)fdelay;
}


int	LgwNextPingSlot(t_lrr_pkt *pkt,struct timespec *eutc,int maxdelay,int *retdelay)
{
	double	slotLen	= 0.03;
	double	pingPeriod;		// en nombre de slots
	double	sDur;
	int	sIdx;
	int	delay;

	if	(!pkt)
		return	-1;

retry:
	pkt->lp_nbtry++;
	pkt->lp_idxtry++;
	if	(pkt->lp_nbtry > pkt->lp_maxtry)
		return	-1;

	pingPeriod	= 4096.0 / pkt->lp_nbslot;

	if	(pkt->lp_nbtry == pkt->lp_nbslot + 1)
	{	// change beacon period P1 => P2
		pkt->lp_idxtry	= 1;
		pkt->lp_currslot= pkt->lp_firstslot2;
#ifdef	TRACE
printf("beacon period change\n");
#endif
	}

	sIdx	= (pkt->lp_idxtry - 1) * pingPeriod;
	sIdx	= pkt->lp_currslot + sIdx;
	sDur	= (double)(sIdx) * slotLen;

#ifdef	TRACE
printf	("sIdx=%04d sDur=%f ",sIdx,sDur);
#endif

	pkt->lp_gss	= pkt->lp_gss0 + (int)sDur; 
	if	(pkt->lp_nbtry >= pkt->lp_nbslot + 1)
	{	// beacon P2
		pkt->lp_gss	+= 128;
	}
	sDur	= (sDur - (int)sDur) * 1E9;
	pkt->lp_gns	= pkt->lp_gns0 + (int)sDur; 
	if	(pkt->lp_gns > 1E9)
	{
		pkt->lp_gss++;
		pkt->lp_gns	= pkt->lp_gns - 1E9;
	}

	delay	= LgwPacketDelayMsFromUtc(pkt,eutc);

#ifdef	TRACE
printf	("ss=%09u ns=%09u delay(ms)=%d\n",pkt->lp_gss,pkt->lp_gns,delay);
#endif

	if	(delay < maxdelay)
		goto	retry;

	*retdelay	= delay;

	if	(pkt->lp_nbtry + 1 > pkt->lp_maxtry)
		return	0;	// this was the last try
	return	1;
}


void	LgwResetPingSlot(t_lrr_pkt *pkt)
{
	pkt->lp_maxtry	= 2 * pkt->lp_nbslot;
	pkt->lp_currslot= pkt->lp_firstslot;
	pkt->lp_nbtry	= 0;
	pkt->lp_idxtry	= 0;
}

void	Usage()
{
printf	("-S xxxxxxxxx : fix utc time in sec (default host time)\n");
printf	("-N xxxxxxxxx : fix utc time in nsec (default 0)\n");
printf	("-n : number of slots per beacon period (default 16)\n");
printf	("-s slot1:slot2 : #slot for beacon periods 1 & 2 (default 0:0)\n");
}


int	main(int argc,char *argv[])
{
	int	opt;
	char	*pt;
	int	ret;
	int	delay;
	int	timecmp	= 0;

	t_lrr_pkt	pkt;
	t_lrr_pkt	*downpkt;

	struct	timespec	eutc;

#if	0
	if	(argc == 1)
	{
		Usage();
		exit(0);
	}
#endif

	downpkt		= &pkt;
	memset	(downpkt,0,sizeof(t_lrr_pkt));
	downpkt->lp_gss0	= time(NULL)+1;
	downpkt->lp_gns0	= 0;
	downpkt->lp_nbslot	= 16;

	eutc.tv_sec	= downpkt->lp_gss0;
	eutc.tv_nsec	= 500000000;


	while	((opt=getopt(argc,argv,"E:U:S:N:n:s:")) != -1)
	{
		switch	(opt)
		{
		case	'E'	: eutc.tv_sec		= atoi(optarg);	break;
		case	'U'	: eutc.tv_nsec		= atoi(optarg);	break;
		case	'S'	: downpkt->lp_gss0	= atoi(optarg);	break;
		case	'N'	: downpkt->lp_gns0	= atoi(optarg);	break;
		case	'n'	: downpkt->lp_nbslot	= atoi(optarg);	break;
		case	's'	:
			pt	= strchr(optarg,':');
			if	(pt)
				*pt	= '\0';
			downpkt->lp_firstslot	= atoi(optarg);
			if	(pt)
				downpkt->lp_firstslot2	= atoi(pt+1);
		break;
		default :
			Usage();
			exit(1);
		break;
		}
	}


	LgwResetPingSlot(downpkt);

	printf	("eutc=%09u eutc=%09u\n",(uint)eutc.tv_sec,(uint)eutc.tv_nsec);
	printf	("gss0=%09u gns0=%09u\n",downpkt->lp_gss0,downpkt->lp_gns0);
	printf	("nbslot=%d slot1=%d slot2=%d\n",downpkt->lp_nbslot,
				downpkt->lp_firstslot,downpkt->lp_firstslot2);

	while	((ret=LgwNextPingSlot(downpkt,&eutc,300,&delay) >= 0))
	{
		if	(timecmp)
		printf	("\twe have %dms to send packet at this slot\n",delay);
	}

	exit(0);
}
