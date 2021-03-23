
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



// RDTP-5475
int		SynchroUtcLrc	= 1;	// use utc time given by LRC
int		SynchroMaxLrr	= LP_MAX_LRR_TRACKING;
int 		SynchroPeriod	= 0;	// period for UL sync radio, 0 disalbled
int 		SynchroLC	= 1;	// LC to use
int 		SynchroSF	= 9;	// SF to use
u_short		SynchroCnt	= 0;	// next frame number
u_int		SynchroTus;		// last synchro tus sent
time_t		SynchroSentAt;		// last synchro sent at 
time_t		SynchroRecvAt;		// last synchro sent at 
int		SynchroTimeout	= 10;	// repeat if no response after 10s
t_channel	*SynchroChannel = NULL;
double		SynchroDrift;
double		SynchroDriftMin	= 0.999;
double		SynchroDriftMax	= 1.001;
time_t		SynchroDriftAt;		// last drift computed at
int		SynchroForced	= 0;
int		SynchroReset	= 0;

static	t_lrr_sync_dn	Hist1;
static	t_lrr_sync_dn	Hist2;

static	t_lrr_sync_inf	Sync1;
static	t_lrr_sync_inf	Sync2;

// RDTP-5475
int	LgwSynchroUp()
{
	if	(SynchroDrift >= SynchroDriftMin && SynchroDrift <= SynchroDriftMax)
		return	1;
	return	0;
}

// RDTP-5475
static	void	SendRadioGwSynchro()
{
	unsigned char	data[50];
	u_int		tusstx = 0;
	u_int		tusetx = 0;
	u_int		mic = 0xFFFFFFFF;
	char		buff[LP_MACLORA_SIZE_MAX*3];
	t_lrr_pkt	downpkt;
	int		ret;


	if	(SynchroPeriod == 0)
	{
		return;
	}
	if	(SynchroChannel == NULL)
	{
		SynchroChannel = &TbChannel[SynchroLC];
	}

	memset	(&downpkt,0,sizeof(t_lrr_pkt));

	ret	= LgwStarted();
	if	(LgwThreadStopped || !ret)
	{
		RTL_TRDBG(1,"Radio thread KO start=%d cmdstop=%d\n",
			ret,LgwThreadStopped);
		return;
	}

#if defined(KONA) && defined(WITH_GPS) 	// do not send anything before 3 min !
	if	(DownRadioStop || !LgwDlReady())
#else
	if	(DownRadioStop)
#endif
	{
		if	(!DownRadioStop)
		{
			RTL_TRDBG(1,"Tektelic: downlink disabled for 3 minutes after radio startup\n");
		}
		else
		{
			RTL_TRDBG(1,"Radio stopped in downlink direction\n");
		}
		return;
	}

	downpkt.lp_synchro	= 1;
	downpkt.lp_type		= LP_TYPE_LRR_PKT_RADIO;
	downpkt.lp_flag		= LP_RADIO_PKT_DOWN;
	downpkt.lp_tms		= rtl_tmmsmono();
	downpkt.lp_trip		= 0;
	downpkt.lp_delay	= 300;
	downpkt.lp_lgwdelay	= 1;
	downpkt.lp_lk		= NULL;

	downpkt.lp_chain	= 0;
	downpkt.lp_bandwidth	= 0;	// 125K
	downpkt.lp_channel	= SynchroChannel->channel;
	downpkt.lp_subband	= SynchroChannel->subband;
	downpkt.lp_spfact	= SynchroSF;
	downpkt.lp_correct	= CodeCorrectingCode(CR_LORA_4_5);
	downpkt.lp_size		= 17;	// see below, but tmoa must be set befor
	TmoaLrrPacket(&downpkt);
#if 0
	/* defined in lgw_x1.c: 1st parameter is the board # */
	LgwGetTrigCnt(0, &tusstx);
#endif
	downpkt.lp_tus	= tusstx;
	tusstx	= downpkt.lp_tus + (downpkt.lp_delay * 1000);
	tusetx	= tusstx + downpkt.lp_tmoa;


/* Actility Gateway Protocol
   Same structure as LoRaWAN pkt :
	MType = 111 (LORA_Proprietary)	byte:0
	DevAddr contains LRRID		byte:1..4
	FCtrl : all zeros		byte:5
	FCnt				byte:6,7
	FPort contains Cmd		byte:8
	<Cmd> here the tus etx		byte:9..12
	MIC				byte:13..16
*/

	data[0]	= 0xe0;
	memcpy	(data+1,(void *)&LrrID,sizeof(LrrID));
	data[5]	= 0x00;
	memcpy	(data+6,(void *)&SynchroCnt,sizeof(SynchroCnt));
	data[8]	= 0x00;	// actility command
	memcpy	(data+9,(void *)&tusetx,sizeof(tusetx));
	memcpy	(data+13,(void *)&mic,sizeof(mic));


	RTL_TRDBG(1,"PKT synchro lrrid=%08x tusstx=%09u tusetx=%09u fcnt=%u diff=%u\n",
		LrrID,tusstx,tusetx,SynchroCnt,ABS(tusetx-tusstx));

	RTL_TRDBG(1,"PKT synchro sz=%d tmoa=%fus chan=%u freqhz=%u sf=%u\n"
		,downpkt.lp_size,downpkt.lp_tmoa,downpkt.lp_channel,
		SynchroChannel->freq_hz,downpkt.lp_spfact);

	SynchroCnt++;
	SynchroTus	= tusetx;
	rtl_timemono(&SynchroSentAt);


	downpkt.lp_payload	= (u_char *)malloc(downpkt.lp_size);
	if	(!downpkt.lp_payload)
	{
		RTL_TRDBG(0,"ERROR alloc payload %d\n",downpkt.lp_size);
		return;
	}
	memcpy	(downpkt.lp_payload,data,downpkt.lp_size);
	rtl_binToStr((unsigned char *)downpkt.lp_payload,downpkt.lp_size,
						buff,sizeof(buff)-10);
	RTL_TRDBG(1,"PKT synchro data='%s'\n",buff);
	LgwSendPacket(&downpkt,0,0,0);
}

// RDTP-5475
static	void	LgwSynchroReset()
{
	RTL_TRDBG(1,"synchro reset\n");
	memset	(&Hist1,0,sizeof(Hist1));
	memset	(&Hist2,0,sizeof(Hist2));
	memset	(&Sync1,0,sizeof(Sync1));
	memset	(&Sync2,0,sizeof(Sync2));
	SynchroDrift	= 0.0;
	SynchroDriftAt	= 0;
}

static	t_lrr_sync_inf	*FindLrrInSynchro(u_int lrrid,t_lrr_sync_dn *sync)
{
	int	i;

	if	(!sync)
		return	NULL;

	for	(i = 0 ; i < sync->sy_macroCnt && i < SynchroMaxLrr ; i++)
	{
		if	(sync->sy_macroInf[i].sy_macroLrr == lrrid)
			return	&sync->sy_macroInf[i];
	}
	return	NULL;
}

static	t_lrr_sync_inf	*SynchroLrrShared(t_lrr_sync_dn *sy1,t_lrr_sync_dn *sy2)
{
	int		i;
	t_lrr_sync_inf	*ret;

	if	(!sy1 || !sy2)
		return	NULL;

	for	(i = 0 ; i < sy1->sy_macroCnt && i < SynchroMaxLrr ; i++)
	{
		ret	= FindLrrInSynchro(sy1->sy_macroInf[i].sy_macroLrr,sy2);
		if	(ret)
			return	ret;
	}
	return	NULL;
}

// RDTP-5475
void	LgwSynchroTimeUpdated(t_lrr_sync_dn *sync)
{
	int		i;
	t_lrr_sync_inf	*lrrshared;
	t_lrr_sync_inf	*inf1;
	t_lrr_sync_inf	*inf2;
	t_lrr_sync_inf	keepinf[LP_MAX_LRR_TRACKING];
	u_int		lrrid	= 0;
	u_int		keepnb	= 0;

	if	(SynchroPeriod == 0)
		return;

	if	(SynchroMaxLrr <= 0)
		return;

	if	(sync->sy_macroCnt <= 0)
		return;

	memset	(keepinf,0,sizeof(keepinf));
	for	(i = 0 ; i < sync->sy_macroCnt && i < LP_MAX_LRR_TRACKING ; i++)
	{
		int	ignore	= 0;

		if	(i >= SynchroMaxLrr)	ignore	= 1;
		// only use macroTyp == 1 => no linux time, no fine timestamp
		if	(sync->sy_macroInf[i].sy_macroTyp != 1)	ignore	= 1;
		RTL_TRDBG(1,
			"synchro[%d] from lrrid=%08x gss=%09u gns=%09u typ=%u "
			"tus=%u fcnt=%u ignore=%d\n",i,
			sync->sy_macroInf[i].sy_macroLrr,
			sync->sy_macroInf[i].sy_macroGss,
			sync->sy_macroInf[i].sy_macroGns,
			sync->sy_macroInf[i].sy_macroTyp,
			sync->sy_microTus,sync->sy_microFcntdn,ignore);
		if	(ignore == 0)
		{
			memcpy	(&keepinf[keepnb],&sync->sy_macroInf[i],
					sizeof(t_lrr_sync_inf));
			keepnb++;
		}
	}

	if	(keepnb <= 0)
	{
		RTL_TRDBG(1,"synchro all lrr source are ignored\n");
		return;
	}
	sync->sy_macroCnt	= keepnb;
	memcpy	(sync->sy_macroInf,keepinf,sizeof(keepinf));

	if	((u_short)(sync->sy_microFcntdn + 1) != SynchroCnt)
	{
		RTL_TRDBG(1,"synchro recv fcnt=%u != expected fcnt=%u\n",
			sync->sy_microFcntdn,SynchroCnt);
		return;
	}
	if	(sync->sy_microTus != SynchroTus)
	{
		RTL_TRDBG(1,"synchro recv tus=%u != expected tus=%u\n",
			sync->sy_microTus,SynchroTus);
		return;
	}

	SynchroSentAt	= 0;
	rtl_timemono(&SynchroRecvAt);

	if	(Hist1.sy_macroInf[0].sy_macroLrr == 0)
	{ // no histo(1) (ie begin or reset)
		memcpy	(&Hist1,sync,sizeof(t_lrr_sync_dn));
		memset	(&Hist2,0,sizeof(t_lrr_sync_dn));
		SynchroDrift	= 0.0;
		RTL_TRDBG(1,"synchro (1) saved tus=%u fcnt=%u\n",
			Hist1.sy_microTus,Hist1.sy_microFcntdn);
		return;
	}

	if	(SynchroMaxLrr > 1 && Hist2.sy_macroInf[0].sy_macroLrr == 0)
	{ // no histo(2)
		lrrshared	= SynchroLrrShared(&Hist1,sync);
		if	(lrrshared == NULL)
		{ // none of the lrr are present in histo(1)
			RTL_TRDBG(1,"synchro no lrr shared => (1) replaced\n");
			memcpy	(&Hist1,sync,sizeof(t_lrr_sync_dn));
			memset	(&Hist2,0,sizeof(t_lrr_sync_dn));
			SynchroDrift	= 0.0;
			RTL_TRDBG(1,"synchro (1) saved tus=%u fcnt=%u\n",
					Hist1.sy_microTus,Hist1.sy_microFcntdn);
			return;
		}
	}

	if	(Hist2.sy_macroInf[0].sy_macroLrr == 0)
	{	// no histo(2) just save current in histo(2)
		memcpy	(&Hist2,sync,sizeof(t_lrr_sync_dn));
	}
	else
	{	// push history
		memcpy	(&Hist1,&Hist2,sizeof(t_lrr_sync_dn));
		memcpy	(&Hist2,sync,sizeof(t_lrr_sync_dn));
	}
	RTL_TRDBG(1,"synchro (1) saved tus=%u fcnt=%u\n",
		Hist1.sy_microTus,Hist1.sy_microFcntdn);
	RTL_TRDBG(1,"synchro (2) saved tus=%u fcnt=%u\n",
		Hist2.sy_microTus,Hist2.sy_microFcntdn);

	if	(SynchroMaxLrr <= 1)
	{
		inf1	= &Hist1.sy_macroInf[0];
		inf2	= &Hist2.sy_macroInf[0];
		goto	dosynchro;
	}

	if	(Sync1.sy_macroLrr && Sync1.sy_macroLrr == Sync2.sy_macroLrr)
	{	// already sync on this LRR try to continue with it
		lrrid	= Sync1.sy_macroLrr;
		RTL_TRDBG(1,"synchro previous was done with lrrid=%08x\n",
									lrrid);
	}
	else
	{
		lrrid	= Hist1.sy_macroInf[0].sy_macroLrr;
		RTL_TRDBG(1,"synchro not done => try with lrrid=%08x\n",lrrid);
	}
	inf1	= FindLrrInSynchro(lrrid,&Hist1);
	if	(!inf1)
	{	// "impossible"
		RTL_TRDBG(1,"synchro internal error lrrid=%08x => reset\n",
									lrrid);
		LgwSynchroReset();
		SynchroDrift	= 0.0;
		return;
	}
	inf2	= FindLrrInSynchro(lrrid,&Hist2);
	if	(inf1 && !inf2)
	{
		for	(i = 0 ; i < Hist2.sy_macroCnt && i < SynchroMaxLrr ; i++)
		{
RTL_TRDBG(1,"synchro lrrid=%08x no more in synchro (2) => try lrrid=%08x\n",
					lrrid,Hist2.sy_macroInf[i].sy_macroLrr);

			lrrid	= Hist2.sy_macroInf[i].sy_macroLrr;
			inf1	= FindLrrInSynchro(lrrid,&Hist1);
			inf2	= FindLrrInSynchro(lrrid,&Hist2);
			if	(inf1 && inf2)
				break;
		}
	}
	if	(!inf1 || !inf2)
	{
RTL_TRDBG(1,"synchro can not be done lrrid=%08x not found 1=%p 2=%p => reset\n",
			lrrid,inf1,inf2);
		LgwSynchroReset();
		SynchroDrift	= 0.0;
		return;
	}
	if	(inf1->sy_macroLrr != inf2->sy_macroLrr)
	{	// "impossible"
RTL_TRDBG(1,"synchro internal error lrrid=%08x 1=%08x 2=%08x => reset\n",
			lrrid,inf1->sy_macroLrr,inf2->sy_macroLrr);
		LgwSynchroReset();
		SynchroDrift	= 0.0;
		return;
	}

dosynchro:

	// save points of synchro
	memcpy	(&Sync1,inf1,sizeof(Sync1));
	memcpy	(&Sync2,inf2,sizeof(Sync2));

	// compute drift
	double	utc1,utc2;
	utc1	= (double)Sync1.sy_macroGss + (double)Sync1.sy_macroGns * 1E-9;
	utc2	= (double)Sync2.sy_macroGss + (double)Sync2.sy_macroGns * 1E-9;
	SynchroDrift	= ABS(Hist2.sy_microTus - Hist1.sy_microTus);
	SynchroDrift	= SynchroDrift / (utc2 - utc1) / 1E6;
RTL_TRDBG(1,"synchro done with lrrid=%08x drift=%f\n",Sync1.sy_macroLrr,SynchroDrift);
	rtl_timemono(&SynchroDriftAt);
}

int	LgwSynchroUtc2Cnt(u_int gss,u_int gns,u_int *trig_tstamp)
{
	double	utc;
	double	utc2;

	if	(!trig_tstamp)
		return	-1;
	if	(SynchroDrift == 0)
		return	-2;
	if	(Hist2.sy_macroCnt == 0)
		return	-3;
	if	(Sync2.sy_macroLrr == 0)
		return	-4;

	utc	= (double)gss + (double)gns * 1E-9;
	utc2	= (double)Sync2.sy_macroGss + (double)Sync2.sy_macroGns * 1E-9;
	utc	= 1E6 * (utc - utc2) * SynchroDrift;

	*trig_tstamp	= Hist2.sy_microTus + (u_int)utc;
RTL_TRDBG(1,"try synchro lrrid=%08x gss=%09u gns=%09u => tusstx=%u+%u=%u\n",
	Sync2.sy_macroLrr,gss,gns,Hist2.sy_microTus,(u_int)utc,*trig_tstamp);

	return	0;
}

// RDTP-5475
void	LgwSynchroPseudoGps()
{
	if	(LgwSynchroUp())
	{
		DoGpsRefresh(0,LP_TYPE_LRR_INF_GPS_UP,'U',1.0);
RTL_TRDBG(1,"synchro drift=%f send pseudo gps up\n",SynchroDrift);
	}
	else
	{
		DoGpsRefresh(0,LP_TYPE_LRR_INF_GPS_DOWN,'D',0.0);
RTL_TRDBG(1,"synchro drift=%f send pseudo gps down\n",SynchroDrift);
	}
}

// RDTP-5475
void	LgwSynchroDoClockSc(time_t now)
{
	static	unsigned	int	nbclock	= 0;

	if	(SynchroPeriod == 0)
		return;

	if	(SynchroForced)
	{
		SynchroForced	= 0;
		RTL_TRDBG(1,"synchro tx forced\n");
		SendRadioGwSynchro();
		return;
	}

	if	(SynchroReset)
	{
		SynchroReset	= 0;
		RTL_TRDBG(1,"synchro reset forced => reset\n");
		LgwSynchroReset();
		LgwSynchroPseudoGps();
		return;
	}

	if	(LrrIDGetFromTwa >= 2 && LrrIDFromTwaGot == 0)
	{
		return;
	}

	// TODO : check CurrTmoaRequested ...
	
	if	((nbclock % SynchroPeriod) == 0)
	{
		SendRadioGwSynchro();
	}
	else
	{
		if	(SynchroSentAt && ABS(now - SynchroSentAt) > SynchroTimeout)
		{
			RTL_TRDBG(1,"synchro sent and no response => new ul\n");
			SendRadioGwSynchro();
		}
	}
	if	(SynchroDriftAt && ABS(now - SynchroDriftAt) >= 2 * SynchroPeriod)
	{
		RTL_TRDBG(1,"synchro too old => reset\n");
		LgwSynchroReset();
		LgwSynchroPseudoGps();
	}

	nbclock++;
}
