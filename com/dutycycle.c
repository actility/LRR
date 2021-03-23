
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

/*! @file dutycycle.c
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>
#include <poll.h>
#include <ctype.h>
#ifndef MACOSX
#include <malloc.h>
#endif

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>

#include "rtlbase.h"

#include "define.h"
#include "infrastruct.h"

#ifdef LP_TP31
#define	DC_LISTDURATION	3600	// in seconds

#define	DC_COEFF	10000	// tmoa us (10^6) in us and %% (100)

#define	DC_HISTOFN	"usr/etc/lrr/dchisto"	// dclt historic filename
#define	DC_HISTOLOGFN	"var/log/lrr/DTCLT.log"	// file generated by using CLI command 'dtclt'

#define DC_ADDELEM	1
#define DC_DELELEM	2

extern	int	TraceLevel;
extern	char	*RootAct;

// store information about a downlink msg
typedef struct s_dc_elem
{
	float	tmoa;		// time on air
	u_char	antenna;	// antenna that received the pkt
	u_char	channel;	// logical channel
	u_char	subband;	// subband
	time_t	expires;	// expires time
	struct s_dc_elem	*next;
} t_dc_elem;

// tmoa for one channel
typedef struct s_dc_countchan
{
	u_char	channel;
	float	tmoaup;
	float	tmoadown;
	struct s_dc_countsub *sub;
	struct s_dc_countchan *next;
} t_dc_countchan;

// tmoa for one subband
typedef struct s_dc_countsub
{
	u_char	subband;
	float	tmoaup;
	float	tmoadown;
	struct s_dc_countsub *next;
} t_dc_countsub;

// store tmoa informations relative to one antenna
typedef struct s_dc_countant
{
	u_char	antenna;
	struct s_dc_countchan *chans;
	struct s_dc_countsub *subs;
	struct s_dc_countant *next;
} t_dc_countant;

typedef struct 
{
	time_t	ts;	// timestamp
	float	tmoa;	// global tmoa for last hour
} t_dc_histo;

int DcLtPeriod;			// period for calculation of long time dl dutycycle, in hours
float DcLt;			// long term downlink dutycycle

static t_dc_elem *DcListUp;		// Uplink elem list
static t_dc_elem *DcListUpLast;		// last elem of the list
static t_dc_elem *DcListDown;		// Downlink elem list
static t_dc_elem *DcListDownLast;	// last elem of the list
static void *DcCounters;		// tmoa list
static int DcMemUsed;			// memory used with malloc done here
static int DcIsInitialized=0;		// Does DcInit has been called ?
static t_dc_histo *DcLtHisto;		// Historic for long time dl dutycycle
static int DcLtHistoIdx;		// Index for DcLtHisto
static int DcLtHistoIdxMax;		// Index max for DcLtHisto

// Protos public
void DcCheckLists();
int DcSearchCounters(u_char ant, u_char chan, u_char sub, float *tcu, float *tsu, float *tcd, float *tsd);
int DcTreatUplink(t_lrr_pkt *pkt);
int DcTreatDownlink(t_lrr_pkt *pkt);

// Protos static
static void DcCheckList(time_t now, t_dc_elem **list, t_dc_elem **last);
static void DcDumpCounters();
static int DcInit();
static int DcLoadHisto();
static t_dc_countchan *DcUpdateCounters(t_dc_elem **list, t_dc_elem *elem, int add);
static t_dc_countant *DcNewCountAnt(u_char antenna);
static t_dc_countchan *DcNewCountChan(u_char channel);
static t_dc_countsub *DcNewCountSub(u_char subband);
static t_dc_elem *DcNewElem(t_lrr_pkt *pkt);
static int DcSaveHisto();
static t_dc_countchan *DcSearchCounter(u_char antenna, u_char channel, u_char subband, int create);

// Calculate long time dl dutycycle on 7 days
void DcCalcLt(time_t now, int afterload)
{
	t_dc_countant	*ca;
	t_dc_countsub	*cs;
	time_t		oldest, limit;
	double		tmoatot=0;	// need a double to sum a small and a big value
	int		i;
	
	// if after DcLoadHisto, just calculate DcLt
	if (!afterload)
	{
		// sum of tmoa for last hour, use values of subband to limit treatments (shortest list)
		ca = DcCounters;
		while (ca)
		{
			RTL_TRDBG(3,"DcCalcLt: treat antenna=%d\n", ca->antenna);
			cs = ca->subs;
			while (cs)
			{
				RTL_TRDBG(3,"DcCalcLt: subband %d tmoatot+=%f\n", cs->subband, cs->tmoadown);
				tmoatot += cs->tmoadown;
				cs = cs->next;
			}
			ca = ca->next;
		}

		// save tmoa for last hour
		RTL_TRDBG(3,"DcCalcLt: DcLtHistoIdx=%d/%d\n", DcLtHistoIdx, DcLtHistoIdxMax);
		DcLtHisto[DcLtHistoIdx].tmoa = tmoatot;
		DcLtHisto[DcLtHistoIdx].ts = now;
		DcLtHistoIdx = (DcLtHistoIdx + 1) % DcLtHistoIdxMax;
	}

	oldest = now;
	// oldest value accepted
	limit = now - (DcLtPeriod*3600);

	tmoatot = 0;
	// sum of the tmoa for the period
	for (i=0; i<DcLtHistoIdxMax && DcLtHisto[i].ts>0; i++)
	{
		RTL_TRDBG(3,"DcLtHisto[%d]: tmoa = %f, ts = %lu\n", i, DcLtHisto[i].tmoa, DcLtHisto[i].ts);
		if (DcLtHisto[i].ts < limit)
		{
			RTL_TRDBG(3,"DcLtHisto[%d]: too old, %lu < %lu\n", i, DcLtHisto[i].ts, limit);
			continue;
		}

		if (DcLtHisto[i].ts < oldest)
		{
			oldest = DcLtHisto[i].ts;
			RTL_TRDBG(3,"DcLtHisto[%d]: new oldest = %lu\n", i, oldest);
		}

		tmoatot += DcLtHisto[i].tmoa;
	}

	// calculate dclt
	DcLt = tmoatot / ((double) (now-(oldest-DC_LISTDURATION))*DC_COEFF);
	RTL_TRDBG(1,"DcLt = %f\n", DcLt);

	// save historic
	DcSaveHisto();
}

// Check a list to clean old msg if required
static void DcCheckList(time_t now, t_dc_elem **list, t_dc_elem **last)
{
	t_dc_elem	*elem;

	elem = *list;
	while (elem && elem->expires < now)
	{
		// set list start to following elem
		*list = elem->next;
		if (*last == elem)
			*last = NULL;
		RTL_TRDBG(3,"DcCheckList: remove elem=0x%x\n", elem);
		DcUpdateCounters(list, elem, DC_DELELEM);
		free(elem);
		DcMemUsed -= sizeof(t_dc_elem);
		elem = *list;
	}

}

// Check uplink and downlink lists to clean old msg if required
void DcCheckLists()
{
	static time_t	lastcalc=0;
	time_t		now;

	if (!DcIsInitialized)
		DcInit();

	now = rtl_timemono(NULL);
	DcCheckList(now, &DcListDown, &DcListDownLast);
	DcCheckList(now, &DcListUp, &DcListUpLast);

	// use time() instead of rtl_timemon() because timestamp is saved in a file
	// and must be significant after lrr restart
	now = time(NULL);
	// first call, no value yet, wait a period for the first calculation
	if (lastcalc == 0)
		lastcalc = now;

	if (now > lastcalc + DC_LISTDURATION)
	{
		DcCalcLt(now, 0);
		lastcalc = now;
	}
}

// Dump all counters
static void DcDumpCounters()
{
	t_dc_countant	*ca;
	t_dc_countchan	*cc;
	t_dc_countsub	*cs;

	if	(TraceLevel < 3)
		return;
	
	RTL_TRDBG(3,"DcDump: memory used=%d\n", DcMemUsed);

	ca = DcCounters;
	while (ca)
	{
		RTL_TRDBG(3,"  antenna %d\n", ca->antenna);

		cc = ca->chans;
		while (cc)
		{
			RTL_TRDBG(3,"    channel %d\n", cc->channel);
			RTL_TRDBG(3,"      tmoaup %f\n", cc->tmoaup);
			RTL_TRDBG(3,"      tmoadown %f\n", cc->tmoadown);
			RTL_TRDBG(3,"      sub %d (0x%lx)\n", cc->sub->subband, cc->sub);
			cc = cc->next;
		}
		cs = ca->subs;
		while (cs)
		{
			RTL_TRDBG(3,"    subband %d (0x%lx)\n", cs->subband, cs);
			RTL_TRDBG(3,"      tmoaup %f\n", cs->tmoaup);
			RTL_TRDBG(3,"      tmoadown %f\n", cs->tmoadown);
			cs = cs->next;
		}
		ca = ca->next;
	}
}

// Dump histo in a file, in order to be used by cli command dtclt
void DcDumpHisto()
{
	FILE	*f;
	int 	i;
	char	fn[256];

	sprintf(fn, "%s/%s", RootAct, DC_HISTOLOGFN);
	f = fopen(fn, "w");
	if (!f)
	{
		RTL_TRDBG(0,"Failed to create '%s' file !\n", fn);
		return;
	}

	fprintf(f, "dtclt=%f period=%d\n", DcLt, DcLtHistoIdxMax);

	for (i=0; i<DcLtHistoIdxMax; i++)
	{
		if (DcLtHisto[i].ts <= 0)
			continue;

		fprintf(f, "%d: dtclt=%f tmoa=%.0f ts=%lu\n",
			i, DcLtHisto[i].tmoa/(DC_LISTDURATION*DC_COEFF),
			DcLtHisto[i].tmoa, DcLtHisto[i].ts);
	}
	fclose(f);
}

// Save all counters
void DcSaveCounters(FILE *f)
{
	t_dc_countant	*ca;
	t_dc_countchan	*cc;
	t_dc_countsub	*cs;
	
	ca = DcCounters;
	while (ca)
	{
		cc = ca->chans;
		while (cc)
		{
			fprintf(f,"ant=%d chan=%03d up=%f dn=%f\n",
				ca->antenna,cc->channel,
				cc->tmoaup/(DC_LISTDURATION*DC_COEFF),
				cc->tmoadown/(DC_LISTDURATION*DC_COEFF));
			cc = cc->next;
		}
		fflush(f);
		ca = ca->next;
	}

	ca = DcCounters;
	while (ca)
	{
		cs = ca->subs;
		while (cs)
		{
			fprintf(f,"ant=%d subb=%03d up=%f dn=%f\n",
				ca->antenna,cs->subband,
				cs->tmoaup/(DC_LISTDURATION*DC_COEFF),
				cs->tmoadown/(DC_LISTDURATION*DC_COEFF));
			cs = cs->next;
		}
		fflush(f);
		ca = ca->next;
	}
	fflush(f);
}

// Walk all counters channels first then subbands
void DcWalkCounters(void *f,void (*fct)(void *pf,int type,int ant,int idx,
                        float up,float dn))
{
	t_dc_countant	*ca;
	t_dc_countchan	*cc;
	t_dc_countsub	*cs;
	
	ca = DcCounters;
	while (ca)
	{
		cc = ca->chans;
		while (cc)
		{
//			fprintf(f,"ant=%d chan=%03d up=%f dn=%f\n",
			(*fct)(f,'C',
				ca->antenna,cc->channel,
				cc->tmoaup/(DC_LISTDURATION*DC_COEFF),
				cc->tmoadown/(DC_LISTDURATION*DC_COEFF));
			cc = cc->next;
		}
		ca = ca->next;
	}

	ca = DcCounters;
	while (ca)
	{
		cs = ca->subs;
		while (cs)
		{
//			fprintf(f,"ant=%d subb=%03d up=%f dn=%f\n",
			(*fct)(f,'S',
				ca->antenna,cs->subband,
				cs->tmoaup/(DC_LISTDURATION*DC_COEFF),
				cs->tmoadown/(DC_LISTDURATION*DC_COEFF));
			cs = cs->next;
		}
		ca = ca->next;
	}
}

// Walk all counters channels
void DcWalkChanCounters(void *f,void (*fct)(void *pf,int ant,int c,int s,
                        float up,float dn,float upsub,float dnsub))
{
	t_dc_countant	*ca;
	t_dc_countchan	*cc;
	
	ca = DcCounters;
	while (ca)
	{
		cc = ca->chans;
		while (cc)
		{
			if (cc->sub)
			{
				(*fct)(f,
				ca->antenna,cc->channel,cc->sub->subband,
				cc->tmoaup/(DC_LISTDURATION*DC_COEFF),
				cc->tmoadown/(DC_LISTDURATION*DC_COEFF),
				cc->sub->tmoaup/(DC_LISTDURATION*DC_COEFF),
				cc->sub->tmoadown/(DC_LISTDURATION*DC_COEFF));
			}
			cc = cc->next;
		}
		ca = ca->next;
	}
}

// Initialization
static int DcInit()
{
	int	i;

	DcCounters = NULL;
	DcListDown = NULL;
	DcListDownLast = NULL;
	DcListUp = NULL;
	DcListUpLast = NULL;
	DcIsInitialized = 1;
	DcLtHistoIdx = 0;
	DcLtHistoIdxMax = DcLtPeriod*3600/DC_LISTDURATION;	// nb of entries in LDcLtHisto
	DcLtHisto = malloc(sizeof(t_dc_histo)*DcLtHistoIdxMax);
	if (!DcLtHisto)
	{
		RTL_TRDBG(0,"DcInit: cannot alloc DcLtHisto !\n");
		return -1;
	}
	else
	{
		RTL_TRDBG(0,"DcInit: DcLtHisto allocated for %d entries\n", DcLtHistoIdxMax);
	}
	for (i=0; i<DcLtHistoIdxMax; i++)
		DcLtHisto[i].ts = 0;
	DcLoadHisto();
	return 0;
}

// load tmao historic for long term downlink dutycycle calculation
int DcLoadHisto()
{
	FILE	*f;
	char	fn[256], buf[256];
	int	i=0;
	time_t	now;

	sprintf(fn, "%s/%s", RootAct, DC_HISTOFN);
	f = fopen(fn, "r");
	if (!f)
		return -1;

	now = time(NULL);
	while (fgets(buf, sizeof(buf)-1, f))
	{
		if (buf[0] == ';')
			continue;
		sscanf(buf, "%f %lu", &DcLtHisto[i].tmoa, &DcLtHisto[i].ts);

		// ignore values too old
		if (DcLtHisto[i].ts < now - DcLtPeriod*3600)
			continue;

		i++;
	}
	fclose(f);
	DcLtHistoIdx = i%DcLtHistoIdxMax;
	DcCalcLt(time(NULL), 1);
	return 0;
}

// Create new antenna counter
static t_dc_countant *DcNewCountAnt(u_char antenna)
{
	t_dc_countant	*ca;

	ca = (t_dc_countant *) malloc(sizeof(t_dc_countant));
	if (!ca)
	{
		RTL_TRDBG(0,"DcNewCountAnt: cannot alloc counter !\n");
		return NULL;
	}
	DcMemUsed += sizeof(t_dc_countant);
	ca->antenna = antenna;
	ca->chans = NULL;
	ca->subs = NULL;
	ca->next = NULL;
	return ca;
}

// Create new chan counter
static t_dc_countchan *DcNewCountChan(u_char channel)
{
	t_dc_countchan	*cc;

	cc = (t_dc_countchan *) malloc(sizeof(t_dc_countchan));
	if (!cc)
	{
		RTL_TRDBG(0,"DcNewCountChan: cannot alloc counter !\n");
		return NULL;
	}
	DcMemUsed += sizeof(t_dc_countchan);
	cc->channel = channel;
	cc->tmoaup = 0;
	cc->tmoadown = 0;
	cc->sub = NULL;
	cc->next = NULL;
	return cc;
}

// Create new subband counter
static t_dc_countsub *DcNewCountSub(u_char subband)
{
	t_dc_countsub	*cs;

	cs = (t_dc_countsub *) malloc(sizeof(t_dc_countsub));
	if (!cs)
	{
		RTL_TRDBG(0,"DcNewCountSub: cannot alloc counter !\n");
		return NULL;
	}
	DcMemUsed += sizeof(t_dc_countsub);
	cs->subband = subband;
	cs->tmoaup = 0;
	cs->tmoadown = 0;
	cs->next = NULL;
	return cs;
}

static t_dc_elem *DcNewElem(t_lrr_pkt *pkt)
{
	t_dc_elem	*elem;

	// create elem
	elem = (t_dc_elem *) malloc(sizeof(t_dc_elem));
	if (!elem)
	{
		RTL_TRDBG(0,"DcNewElem: cannot allocate elem !\n");
		return NULL;
	}

	DcMemUsed += sizeof(t_dc_elem);

	// set elem
	elem->expires = rtl_timemono(NULL)+DC_LISTDURATION;
	elem->tmoa = pkt->lp_tmoa;
	elem->antenna = pkt->lp_chain>>4;
	elem->channel = pkt->lp_channel;
	elem->subband = pkt->lp_subband;
	elem->next = NULL;
	RTL_TRDBG(3,"DcNewElem: new elem = 0x%lx, ant=%d, chan=%d, sub=%d, tmoa=%f\n",
		elem, elem->antenna, elem->channel, elem->subband, elem->tmoa);

	return elem;
}

// save tmao historic for long term downlink dutycycle calculation
// saved sorted, oldest first
int DcSaveHisto()
{
	FILE	*f;
	char	fn[256];
	int	i, idx;

	sprintf(fn, "%s/%s", RootAct, DC_HISTOFN);
	f = fopen(fn, "w");
	if (!f)
	{
		RTL_TRDBG(0,"DcSaveHisto error, can't save histo file '%s': %s\n", fn, strerror(errno));
		return -1;
	}

	fprintf(f, ";historic of tmoa for long term downlink dutycycle calculation\n");
	fprintf(f, ";one line = one hour\n");
	fprintf(f, ";format for each line: <tmoa> <timestamp>\n");
	for (i=0; i<DcLtHistoIdxMax; i++)
	{
		// save oldest first
		idx = (DcLtHistoIdx+i)%DcLtHistoIdxMax;
		if (DcLtHisto[idx].ts > 0)
			fprintf(f, "%f\t%lu\n", DcLtHisto[idx].tmoa, DcLtHisto[idx].ts);
	}
	fclose(f);
	return 0;
}

// Search for a channel counter, create it if it doesn't exist
static t_dc_countchan *DcSearchCounter(u_char antenna, u_char channel, u_char subband, int create)
{
	t_dc_countant	*ca, *lastca;
	t_dc_countchan	*cc, *lastcc;
	t_dc_countsub	*cs, *lastcs;
	
	// search antenna
	ca = DcCounters;
	lastca = NULL;
	while (ca)
	{
		if (ca->antenna == antenna)
			break;
		lastca = ca;
		ca = ca->next;
	}
	
	// not found and must not be created
	if (!ca && !create)
		return NULL;

	// if no antenna found create one
	if (!ca)
	{
		ca = DcNewCountAnt(antenna);
		if (!ca)
			return NULL;

		// add it in the list
		if (lastca)
			lastca->next = ca;
		if (!DcCounters)
			DcCounters = ca;
	}

	// search channel
	cc = ca->chans;
	lastcc = NULL;
	while (cc)
	{
		if (cc->channel == channel)
			break;
		lastcc = cc;
		cc = cc->next;
	}
	
	// not found and must not be created
	if (!cc && !create)
		return NULL;

	// if no chan found create one
	if (!cc)
	{
		cc = DcNewCountChan(channel);
		if (!cc)
			return NULL;

		// add it in the list
		if (lastcc)
			lastcc->next = cc;
		if (!ca->chans)
			ca->chans = cc;

		// search subband
		cs = ca->subs;
		lastcs = NULL;
		while (cs)
		{
			if (cs->subband == subband)
				break;
			lastcs = cs;
			cs = cs->next;
		}

		// if no subband found create one
		if (!cs)
		{
			cs = DcNewCountSub(subband);
			if (!cs)
				return NULL;

			// add it in the list
			if (lastcs)
				lastcs->next = cs;
			if (!ca->subs)
				ca->subs = cs;

		}

		// set subband to channel
		cc->sub = cs;
	}

	return cc;
}

// Search for a channel counter, return tmoas
// tcu: tmoa for channel in uplink
// tsu: tmoa for subband in uplink
// tcd: tmoa for channel in downlink
// tsd: tmoa for subband in downlink
int DcSearchCounters(u_char ant, u_char chan, u_char sub, float *tcu, float *tsu, float *tcd, float *tsd)
{
	t_dc_countchan	*cc;

	cc = DcSearchCounter(ant, chan, sub, 0);
	if (!cc)
		return -1;

	if (tcu)
		*tcu = cc->tmoaup/(DC_LISTDURATION*DC_COEFF);
	if (tsu)
		*tsu = cc->sub->tmoaup/(DC_LISTDURATION*DC_COEFF);
	if (tcd)
		*tcd = cc->tmoadown/(DC_LISTDURATION*DC_COEFF);
	if (tsd)
		*tsd = cc->sub->tmoadown/(DC_LISTDURATION*DC_COEFF);

	return 0;
}

// Treat downlink packets
int DcTreatDownlink(t_lrr_pkt *pkt)
{
	t_dc_elem	*elem;
	t_dc_countchan	*cc;

	if (!pkt)
		return -1;

	if (!DcIsInitialized)
		DcInit();

	elem = DcNewElem(pkt);
	if (!elem)
		return -1;

	// Add elem in List

	// if the List is empty
	if (!DcListDown)
	{
		DcListDown = elem;
		DcListDownLast = elem;
	}
	else
	{
		DcListDownLast->next = elem;
		DcListDownLast = elem;
	}

	// update counters
	cc = DcUpdateCounters(&DcListDown, elem, DC_ADDELEM);

	if (cc)
	{
		RTL_TRDBG(3,"DcTreatDownlink: ant%d chan%d sub%d tmoachanup=%f tmoachandown=%f tmoasubup=%f tmoasubdown=%f\n",
			elem->antenna, elem->channel, elem->subband, cc->tmoaup,
			cc->tmoadown, cc->sub->tmoaup, cc->sub->tmoadown);
#ifdef LP_TP31
		pkt->lp_flag	= pkt->lp_flag | LP_RADIO_PKT_DTC;
		pkt->lp_u.lp_sent_indic.lr_dtc_ud.lr_dtcchannelup = 
				cc->tmoaup/(DC_LISTDURATION*DC_COEFF);
		pkt->lp_u.lp_sent_indic.lr_dtc_ud.lr_dtcsubbandup = 
				cc->sub->tmoaup/(DC_LISTDURATION*DC_COEFF);
		pkt->lp_u.lp_sent_indic.lr_dtc_ud.lr_dtcchanneldn = 
				cc->tmoadown/(DC_LISTDURATION*DC_COEFF);
		pkt->lp_u.lp_sent_indic.lr_dtc_ud.lr_dtcsubbanddn = 
				cc->sub->tmoadown/(DC_LISTDURATION*DC_COEFF);
#endif
	}

	DcDumpCounters();

	return 0;
}

// Set uplink packet with dutycycle info
int DcTreatUplink(t_lrr_pkt *pkt)
{
	t_dc_countchan	*cc;
	t_dc_elem	*elem;

	if (!pkt)
		return -1;

	if (!DcIsInitialized)
		DcInit();

	elem = DcNewElem(pkt);
	if (!elem)
		return -1;

	// Add elem in List

	// if the List is empty
	if (!DcListUp)
	{
		DcListUp = elem;
		DcListUpLast = elem;
	}
	else
	{
		DcListUpLast->next = elem;
		DcListUpLast = elem;
	}

	// update counters
	cc = DcUpdateCounters(&DcListUp, elem, DC_ADDELEM);

	if (cc)
	{
		RTL_TRDBG(3,"DcTreatUplink: ant%d chan%d sub%d tmoachanup=%f tmoachandown=%f tmoasubup=%f tmoasubdown=%f\n",
			elem->antenna, elem->channel, elem->subband, cc->tmoaup,
			cc->tmoadown, cc->sub->tmoaup, cc->sub->tmoadown);
#ifdef LP_TP31
		pkt->lp_flag	= pkt->lp_flag | LP_RADIO_PKT_DTC;
		pkt->lp_u.lp_radio.lr_u2.lr_dtc_ud.lr_dtcchannelup = 
				cc->tmoaup/(DC_LISTDURATION*DC_COEFF);
		pkt->lp_u.lp_radio.lr_u2.lr_dtc_ud.lr_dtcsubbandup = 
				cc->sub->tmoaup/(DC_LISTDURATION*DC_COEFF);
		pkt->lp_u.lp_radio.lr_u2.lr_dtc_ud.lr_dtcchanneldn = 
				cc->tmoadown/(DC_LISTDURATION*DC_COEFF);
		pkt->lp_u.lp_radio.lr_u2.lr_dtc_ud.lr_dtcsubbanddn = 
				cc->sub->tmoadown/(DC_LISTDURATION*DC_COEFF);
#endif
	}
	else
	{
		RTL_TRDBG(3,"DcTreatUplink: no counter, should be impossible !!!\n");
	}
	return 0;
}

// Update counters when adding or removing a elem in the list
static t_dc_countchan *DcUpdateCounters(t_dc_elem **list, t_dc_elem *elem, int add)
{
	t_dc_countchan	*cc;

	if (!elem)
		return NULL;

	// search counter. Create it if it doesn't exist
	cc = DcSearchCounter(elem->antenna, elem->channel, elem->subband, (add == DC_ADDELEM));
	if (!cc)
		return NULL;

	// update tmoa
	if (list == &DcListUp)
	{
		if (add == DC_ADDELEM)
		{
			cc->tmoaup += elem->tmoa;
			cc->sub->tmoaup += elem->tmoa;
		}
		else
		{
			cc->tmoaup -= elem->tmoa;
			cc->sub->tmoaup -= elem->tmoa;
		}
		RTL_TRDBG(3,"DcUpdateCounters: new tmoas ant%d chan%d sub%d tmoachanup=%f (%.03f%%) tmoasubup=%f (%.03f%%)\n", 
			elem->antenna, cc->channel, cc->sub->subband, cc->tmoaup,
			cc->tmoaup/(DC_LISTDURATION*DC_COEFF), cc->sub->tmoaup,
			cc->sub->tmoaup/(DC_LISTDURATION*DC_COEFF));
	}
	else
	{
		if (add == DC_ADDELEM)
		{
			cc->tmoadown += elem->tmoa;
			cc->sub->tmoadown += elem->tmoa;
		}
		else
		{
			cc->tmoadown -= elem->tmoa;
			cc->sub->tmoadown -= elem->tmoa;
		}
		RTL_TRDBG(3,"DcUpdateCounters: new tmoas ant%d chan%d sub%d tmoachandown=%f (%.03f%%) tmoasubdown=%f (%.03f%%)\n", 
			elem->antenna, cc->channel, cc->sub->subband, cc->tmoadown,
			cc->tmoadown/(DC_LISTDURATION*DC_COEFF), cc->sub->tmoadown,
			cc->sub->tmoadown/(DC_LISTDURATION*DC_COEFF));
	}

	return cc;
}

#endif