
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

/* board used to perform lgw_utc2cnt conversion: 0 or "board" (variable) (0 in lgw_x8.c) */
#define BOARD_CNT	board

/*! @file lgw.c
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

#include <dlfcn.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#include "rtlbase.h"
#include "rtlimsg.h"
#include "rtllist.h"
#include "rtlhtbl.h"

#include "timeoper.h"

#if defined(WITH_USB) || defined(NATRBPI_USB)
#define LGW_RF_TX_ENABLE        { true, false }  /* radio B TX not connected*/
#define LGW_RF_CLKOUT           { true, true }   /* both radios have clkout enabled */
#endif

#undef WITH_SX1301_X8
#ifndef WITH_SX1301_X1
#define WITH_SX1301_X1
#endif

#include "semtech.h"

#include "headerloramac.h"

#ifndef	TX_FREE
#define	TX_FREE	TX_EMPTY
#endif

#include "xlap.h"
#include "define.h"
#include "infrastruct.h"
#include "struct.h"
#include "cproto.h"
#include "extern.h"

#if defined(WIRMANA)
#include "gpio.h"
#define LGW_GPIO_RADIO_RESET 102
#endif

/* time reference used for UTC <-> timestamp conversion */
struct tref Gps_time_ref[MAX_BOARD];
/* is GPS reference acceptable (ie. not too old) */
int Gps_ref_valid = false;
int	DriftComp = -1;		// xtal drift compensation activation
double	XtalCorr = 1.0;		// correction for xtal drift compensation
bool	XtalCorrOk = false;	// xtal correction is up to date
#define XERR_INIT_AVG       128         /* nb of measurements the XTAL correction is averaged on as initial value */
#define XERR_FILT_COEF      256         /* coefficient for low-pass XTAL error tracking */

struct timespec	LgwCurrUtcTime;		// last UTC + nsec computation
struct timespec	LgwBeaconUtcTime;	// last beacon
struct timespec	LgwClassCUtcTime;	// last ClassC
time_t		LgwTmmsUtcTime;		// tmms mono of the last UTC/GPS

static	int	_LgwStarted;
static	bool	LgwTxEnable[MAX_BOARD][LGW_RF_CHAIN_NB];	// rfchain txenabled
#ifdef	TEST_ONLY
static	int	LgwWaitSending	= 1;
#else
static	int	LgwWaitSending	= 0;
#endif

static	int	RType0	= 1257;	// radio type of rfconf0 (sx1255,sx1257,...)

static	t_define	TbDefine[] =
{
	{ "MOD_LORA"		,	MOD_LORA },
	{ "MOD_FSK"		,	MOD_FSK },

	{ "BW_500KHZ"		,	BW_500KHZ },
	{ "BW_250KHZ"		,	BW_250KHZ },
	{ "BW_125KHZ"		,	BW_125KHZ },
	{ "BW_62K5HZ"		,	BW_62K5HZ },
	{ "BW_31K2HZ"		,	BW_31K2HZ },
	{ "BW_15K6HZ"		,	BW_15K6HZ },
	{ "BW_7K8HZ"		,	BW_7K8HZ },


	{ "DR_LORA_SF7"		,	DR_LORA_SF7 },
	{ "DR_LORA_SF8"		,	DR_LORA_SF8 },
	{ "DR_LORA_SF9"		,	DR_LORA_SF9 },
	{ "DR_LORA_SF10"	,	DR_LORA_SF10 },
	{ "DR_LORA_SF11"	,	DR_LORA_SF11 },
	{ "DR_LORA_SF12"	,	DR_LORA_SF12 },
	{ "DR_LORA_MULTI"	,	DR_LORA_MULTI },

	{ "CR_LORA_4_5"		,	CR_LORA_4_5 },
	{ "CR_LORA_4_6"		,	CR_LORA_4_6 },
	{ "CR_LORA_4_7"		,	CR_LORA_4_7 },
	{ "CR_LORA_4_8"		,	CR_LORA_4_8 },


	{ NULL			,	0 },
	{ NULL			,	0 },
	{ NULL			,	0 },

};

/* requires for shared libray loading */
#include "semtech_fctdef.h"
#define LIBLORABOARD	"lrr/com/exe_spi_x1/libloraboard%d.so"
/* size unknown at this stage so initialize to 0 because in DATA region*/
static int	  libLoraLoaded[MAX_BOARD] ;
static void   *libLora[MAX_BOARD];


static void	loadLoraLibrary(int board) {

	if (libLoraLoaded[board]) {
		return;
	}

#if defined(WITH_MULTI_BOARD)

    char    partialname[PATH_MAX];
	char	libname[PATH_MAX];

    sprintf(partialname, LIBLORABOARD, board);
	sprintf(libname, "%s/%s", RootAct, partialname);
    RTL_TRDBG(0, "loading board-specific Lora library %s\n", libname);
    libLora[board] = dlopen(libname, RTLD_LAZY | RTLD_LOCAL);
    if (libLora[board] == NULL) {
        RTL_TRDBG(0, "Cannot load %s: %s\n", libname, dlerror());
        exit(1);
    }
    RTL_TRDBG(0, "lib loaded: %08x\n", libLora[board]);

#else

    RTL_TRDBG(0, "Using static Lora library\n");

    /* dlsym will look for symbol into the program space */
    libLora[board] = NULL;

#endif 
}

#if defined(WITH_MULTI_BOARD)
static void *getFctAddr(void * lib, char *name) {

    void    *fct;
    fct = dlsym(lib, name);
    if (fct == NULL) {
        RTL_TRDBG(0, "Cannot dlsym %s: %s\n", name, dlerror());
        exit(1);
    }
    RTL_TRDBG(0, "lib: %08x   %s: %08x\n", lib, name, fct);
    return fct;
}
#endif

static void loadLoraSymbol(int i) {

#if defined(WITH_MULTI_BOARD)
#define GETFCTADDR(lib,name)	getFctAddr(lib, #name)
#else
#define GETFCTADDR(lib, name) 	&name
#endif

    /*
        note: each function, once processed, is redefined as error in order to
        cause a compilation error if used directly in a multi-board building.
    */

    /* loragw_hal */
    tlgw_board_setconf[i]		= GETFCTADDR(libLora[i], lgw_board_setconf);
#define lgw_board_setconf error
    tlgw_lbt_setconf[i]			= GETFCTADDR(libLora[i], lgw_lbt_setconf);
#define lgw_lbt_setconf error
    tlgw_rxrf_setconf[i]		= GETFCTADDR(libLora[i], lgw_rxrf_setconf);
#define lgw_rxrf_setconf error
    tlgw_rxif_setconf[i]		= GETFCTADDR(libLora[i], lgw_rxif_setconf);
#define lgw_rxif_setconf error
    tlgw_txgain_setconf[i]		= GETFCTADDR(libLora[i], lgw_txgain_setconf);
#define lgw_txgain_setconf error
    tlgw_start[i]				= GETFCTADDR(libLora[i], lgw_start);
#define lgw_start error
    tlgw_stop[i]				= GETFCTADDR(libLora[i], lgw_stop);
#define lgw_stop error
    tlgw_receive[i]				= GETFCTADDR(libLora[i], lgw_receive);
#define lgw_receive error
    tlgw_send[i]				= GETFCTADDR(libLora[i], lgw_send);
#define lgw_send error
    tlgw_status[i]				= GETFCTADDR(libLora[i], lgw_status);
#define lgw_status error
    tlgw_abort_tx[i]			= GETFCTADDR(libLora[i], lgw_abort_tx);
#define lgw_abort_tx error
    tlgw_get_trigcnt[i]			= GETFCTADDR(libLora[i], lgw_get_trigcnt);
#define lgw_get_trigcnt error
    tlgw_version_info[i]		= GETFCTADDR(libLora[i], lgw_version_info);
#define lgw_version_info error
    tlgw_time_on_air[i]			= GETFCTADDR(libLora[i], lgw_time_on_air);
#define lgw_time_on_air error

    /* loragw_reg */
    tlgw_connect[i]				= GETFCTADDR(libLora[i], lgw_connect);
#define lgw_connect error
    tlgw_disconnect[i]          = GETFCTADDR(libLora[i], lgw_disconnect);
#define lgw_disconnect error
    tlgw_soft_reset[i]			= GETFCTADDR(libLora[i], lgw_soft_reset);
#define lgw_soft_reset error
    tlgw_reg_check[i]           = GETFCTADDR(libLora[i], lgw_reg_check);
#define lgw_reg_check error
    tlgw_reg_w[i]               = GETFCTADDR(libLora[i], lgw_reg_w);
#define lgw_reg_w error
    tlgw_reg_r[i]				= GETFCTADDR(libLora[i], lgw_reg_r);
#define lgw_reg_r error
    tlgw_reg_wb[i]              = GETFCTADDR(libLora[i], lgw_reg_wb);
#define lgw_reg_wb error
    tlgw_reg_rb[i]				= GETFCTADDR(libLora[i], lgw_reg_rb);
#define lgw_reg_rb error


#ifdef WITH_GPS
	/* loragw_gps */
	tlgw_gps_sync[i]			= GETFCTADDR(libLora[i], lgw_gps_sync);
#define lgw_gps_sync error
	tlgw_cnt2utc[i]				= GETFCTADDR(libLora[i], lgw_cnt2utc);
#define lgw_cnt2utc error
	tlgw_utc2cnt[i]				= GETFCTADDR(libLora[i], lgw_utc2cnt);
#define lgw_utc2cnt error
#endif


}

static void initLoraLibrary() {
	int board;

	for (board = 0; board < LgwBoard; board++)
    {
		/* at least board #0 can be already loaded */
		if (! libLoraLoaded[board]) {
        	loadLoraLibrary(board);
        	loadLoraSymbol(board);
			libLoraLoaded[board] = 1;
		}
    }
}

// Check if tx enabled on requested rfchain
int	LgwCheckTxEnable(int board, int rfchain)
{
	// if tx disabled on rfchain requested but tx enabled on other rfchain
	// return the second rfchain
	if (!LgwTxEnable[board][rfchain] && LgwTxEnable[board][(rfchain+1)%2])
		return (rfchain+1)%2;
	else
		return rfchain;
}


void	LgwForceDefine(void *htbl)
{
	int	i;
	char	value[64];

	for	(i = 0 ; TbDefine[i].name ; i++)
	{
		sprintf	(value,"0x%02x",TbDefine[i].value);
		rtl_htblRemove(htbl,TbDefine[i].name);
		rtl_htblInsert(htbl,TbDefine[i].name,(void *)strdup(value));
	}

}

char	*LgwVersionInfo()
{
	static	char	vers[128];

	/* load symbols at least for board #0 */
	initLoraLibrary();

	sprintf	(vers,"@(#) Semtech hal %s",(char *)tlgw_version_info[0]());
	return	vers;
}

int	LgwTxFree(uint8_t board,uint8_t *txstatus)
{
	tlgw_status[board](TX_STATUS,txstatus);
	if	(*txstatus != TX_FREE)
	{
		return 0;
	}
	return	1;
}

int	LgwGetTrigNow(uint8_t board,uint32_t *tus)
{
	int		ret;

	tlgw_reg_w[board](LGW_GPS_EN,0);
	ret	= tlgw_get_trigcnt[board](tus);
	tlgw_reg_w[board](LGW_GPS_EN,1);
	return	ret;
}

#ifdef	WITH_GPS
int	LgwEstimTrigCnt(int board,uint32_t *trig_tstamp)
{
	int	ret;
	struct	timespec	utc;

	*trig_tstamp	= 0;
	LgwEstimUtc(&utc);
	ret	= tlgw_utc2cnt[BOARD_CNT](Gps_time_ref[BOARD_CNT],utc,trig_tstamp);
	return	ret;
}
#endif

/* used by lgw_synchro.c */
int LgwGetTrigCnt(uint8_t board, uint32_t *ptrig_tstamp)
{
	int ret;

	tlgw_reg_w[board](LGW_GPS_EN,0);
	ret	= tlgw_get_trigcnt[board](ptrig_tstamp); 
	tlgw_reg_w[board](LGW_GPS_EN,1);
	return ret;
}

#ifdef	WITH_GPS
int	LgwEstimUtcBoard(uint8_t board, struct timespec *utc)
{
	int		ret;
	uint32_t	trig_tstamp;

	tlgw_reg_w[board](LGW_GPS_EN,0);
	ret	= tlgw_get_trigcnt[board](&trig_tstamp); 
	tlgw_reg_w[board](LGW_GPS_EN,1);
	if	(ret != LGW_HAL_SUCCESS)
		return	LGW_HAL_ERROR;

	ret	= tlgw_cnt2utc[board](Gps_time_ref[board],trig_tstamp,utc);
	if	(ret != LGW_HAL_SUCCESS)
		return	LGW_HAL_ERROR;

    RTL_TRDBG(1,"LgwEstimUtcBoard board=%d tus=%u utc=(%u,%03ums)\n", board, trig_tstamp, utc->tv_sec, utc->tv_nsec/1000000);

	return	LGW_HAL_SUCCESS;
}
#endif

char	*PktStatusTxt(uint8_t status)
{
	static	char	buf[64];

	switch	(status)
	{
	case	STAT_NO_CRC :
		return	"CRCNO";
	case	STAT_CRC_BAD :
		return	"CRCERR";
	case	STAT_CRC_OK :
		return	"CRCOK";
	}
	sprintf	(buf,"CRC?(0x%x)",status);
	return	buf;
}

char	*BandWidthTxt(int bandwidth)
{
	static	char	buf[64];

	switch	(bandwidth)
	{
	case	BW_500KHZ :
		return	"BW500";
	case	BW_250KHZ :
		return	"BW250";
	case	BW_125KHZ :
		return	"BW125";
	case	BW_62K5HZ :
		return	"BW62.5";
	case	BW_31K2HZ :
		return	"BW31.2";
	case	BW_15K6HZ :
		return	"BW15.6";
	case	BW_7K8HZ :
		return	"BW7.8";
	}
	sprintf	(buf,"BW?(0x%x)",bandwidth);
	return	buf;
}

char	*SpreadingFactorTxt(int sf)
{
	static	char	buf[64];

	switch	(sf)
	{
	case	DR_LORA_SF7 :
		return	"SF7";
	case	DR_LORA_SF8 :
		return	"SF8";
	case	DR_LORA_SF9 :
		return	"SF9";
	case	DR_LORA_SF10 :
		return	"SF10";
	case	DR_LORA_SF11 :
		return	"SF11";
	case	DR_LORA_SF12 :
		return	"SF12";
	}

	sprintf	(buf,"SF?(0x%x)",sf);
	return	buf;
}


uint8_t	CodeSpreadingFactor(int sf)
{
	switch	(sf)
	{
	case	DR_LORA_SF7 :
		return	7;
	case	DR_LORA_SF8 :
		return	8;
	case	DR_LORA_SF9 :
		return	9;
	case	DR_LORA_SF10 :
		return	10;
	case	DR_LORA_SF11 :
		return	11;
	case	DR_LORA_SF12 :
		return	12;
	default :
		return	0;
	}
}

uint8_t	DecodeSpreadingFactor(int spf)
{
	switch	(spf)
	{
	case	7 :
		return	DR_LORA_SF7;
	case	8 :
		return	DR_LORA_SF8;
	case	9 :
		return	DR_LORA_SF9;
	case	10 :
		return	DR_LORA_SF10;
	case	11 :
		return	DR_LORA_SF11;
	case	12 :
		return	DR_LORA_SF12;
	default :
		return	DR_LORA_SF10;
	}
}

char	*CorrectingCodeTxt(int coderate)
{
	static	char	buf[64];

	switch	(coderate)
	{
	case	CR_LORA_4_5 :
		return	"CC4/5";
	case	CR_LORA_4_6 :
		return	"CC4/6";
	case	CR_LORA_4_7 :
		return	"CC4/7";
	case	CR_LORA_4_8 :
		return	"CC4/8";
	}
	sprintf	(buf,"CC?(0x%x)",coderate);
	return	buf;
}

uint8_t	CodeCorrectingCode(int coderate)
{
	switch	(coderate)
	{
	case	CR_LORA_4_5 :
		return	5;
	case	CR_LORA_4_6 :
		return	6;
	case	CR_LORA_4_7 :
		return	7;
	case	CR_LORA_4_8 :
		return	8;
	default :
		return	0;
	}
}

uint8_t	DecodeCorrectingCode(int cr)
{
	switch	(cr)
	{
	case	5 :
		return	CR_LORA_4_5;
	case	6 :
		return	CR_LORA_4_6;
	case	7 :
		return	CR_LORA_4_7;
	case	8 :
		return	CR_LORA_4_8;
	default :
		return	CR_LORA_4_5;
	}
}

uint8_t	CodeBandWidth(int bandwidth)
{
	switch	(bandwidth)
	{
	case	BW_500KHZ :
		return	2;
	case	BW_250KHZ :
		return	1;
	case	BW_125KHZ :
		return	0;
	case	BW_62K5HZ :
		return	100;
	case	BW_31K2HZ :
		return	99;
	case	BW_15K6HZ :
		return	98;
	case	BW_7K8HZ :
		return	97;
	}
	return	0;
}

uint8_t	DeCodeBandWidth(int bandwidth)
{
	switch	(bandwidth)
	{
	case	2 :
		return	BW_500KHZ;
	case	1 :
		return	BW_250KHZ;
	case	0 :
		return	BW_125KHZ;
	case	100 :
		return	BW_62K5HZ;
	case	99 :
		return	BW_31K2HZ;
	case	98 :
		return	BW_15K6HZ;
	case	97 :
		return	BW_7K8HZ;
	}
	return	BW_125KHZ;
}

float	FreqBandWidth(int bandwidth)
{
	switch	(bandwidth)
	{
	case	2 :
		return	500e3;
	case	1 :
		return	250e3;
	case	0 :
		return	125e3;
	case	100 :
		return	62500.0;
	case	99 :
		return	31200.0;
	case	98 :
		return	15600.0;
	case	97 :
		return	7800.0;
	}
	return	125e3;
}

#if	0
static	unsigned int	LgwTimeStamp()
{
	uint8_t	buf[4];
	unsigned int value;
	int	nbbits=32;
	int	ret;

	ret	= lgw_reg_rb(LGW_TIMESTAMP,buf,nbbits);
	if	(ret != LGW_REG_SUCCESS)
	{
		RTL_TRDBG(0,"cannot read LGW timestamp REG ret=%d\n",ret);
		return	0;	
	}
//	value	= (buf[0] << 24) + (buf[1] << 16) + (buf[2] << 8) + buf[3];
	value	= *(unsigned int *)buf;
	return	value;
}
#endif

int	LgwLINKUP()
{
	return	1;
#if	0
	static	unsigned int prev;
	static	unsigned int err;
	uint8_t	buf[4];
	unsigned int value;
	unsigned int diff;

	value	= LgwTimeStamp();
	RTL_TRDBG(9,"LGW timestamp REG current=%u previous=%u\n",
							value,prev);
	if	(!prev || !value)
	{
		prev	= value;
		return	1;
	}
	if	(prev == value)
	{
		LgwNbSyncError++;
		err++;
		RTL_TRDBG(0,"LGW timestamp REG no diff=%u err=%d\n",prev,err);
		prev	= value;
		if	(err >= 3)
		{
			RTL_TRDBG(0,"LGW LINK DOWN\n");
			err	= 0;
			prev	= 0;
			return	0;
		}
	}
	prev	= value;
	err	= 0;
	return	1;
#endif
}


t_channel	*FindChannelForPacket(T_lgw_pkt_rx_t *up)
{
	int		i;
	t_channel	*c;
	t_channel_entry	*e;
	int		sortindex;
	if	(!up)
		return	NULL;

	// find first index in sorted TbChannelEntry for this frequency
	sortindex	= BinSearchFirstChannel(up->freq_hz);
	if	(sortindex < 0)
		goto	not_found;

	for	(i = sortindex ; i < NbChannelEntry && i < NB_CHANNEL ; i++)
	{
		e	= &TbChannelEntry[i];
		if	(e->freq_hz != up->freq_hz)
			break;
		c	= &TbChannel[e->index];
		if	(c->name[0] == '\0' || c->freq_hz == 0)
			continue;
		if	(c->modulation != up->modulation)
			continue;
		if	(c->freq_hz != up->freq_hz)
			continue;
		if	(c->bandwidth != up->bandwidth)
			continue;
		if	(c->modulation == MOD_FSK)
		{	// do not compare datarate
			return	c;
		}
		if	(c->modulation == MOD_LORA)
		{
			if	((c->datarate & up->datarate) == up->datarate)
				return	c;
		}
	}

not_found :
	RTL_TRDBG(0,"no chan(%d) found for frz=%d mod=0x%02x bdw=0x%02x dtr=0x%02x\n",
		sortindex,up->freq_hz,up->modulation,up->bandwidth,up->datarate);

	return	NULL;
}

static	void LgwDumpRfConf(FILE *f,int rfi,T_lgw_conf_rxrf_t rfconf)
{
	if	(rfconf.enable == 0)
		return;

#ifdef USELIBLGW3
	RTL_TRDBG(1,"rf%d enab=%d frhz=%d tx_enab=%d rssi_off=%f sx=%d\n",
		rfi,rfconf.enable,rfconf.freq_hz,
		rfconf.tx_enable, rfconf.rssi_offset,rfconf.type);
#else
	RTL_TRDBG(1,"rf%d enab=%d frhz=%d\n",rfi,rfconf.enable,
		rfconf.freq_hz);
#endif

	if	(f == NULL)
		return;
#ifdef USELIBLGW3
	fprintf(f,"rf%d enab=%d frhz=%d tx_enab=%d rssi_off=%f sx=%d\n",
	rfi,rfconf.enable,rfconf.freq_hz,
	rfconf.tx_enable, rfconf.rssi_offset,rfconf.type);
#else
	fprintf(f,"rf%d enab=%d frhz=%d\n",rfi,rfconf.enable,
		rfconf.freq_hz);
#endif
	fflush(f);
}

static	void LgwDumpIfConf(FILE *f,int ifi,T_lgw_conf_rxif_t ifconf,int fbase)
{
	if	(ifconf.enable == 0)
		return;

	RTL_TRDBG(1,"if%d enab=%d frhz=%d rfchain=%d bandw=0x%x datar=0x%x\n",
		ifi,ifconf.enable,
		(ifconf.freq_hz+fbase),ifconf.rf_chain,
		ifconf.bandwidth,ifconf.datarate);

	if	(f == NULL)
		return;
	fprintf(f,"if%d enab=%d frhz=%d rfchain=%d bandw=0x%x datar=0x%x\n",
		ifi,ifconf.enable,
		(ifconf.freq_hz+fbase),ifconf.rf_chain,
		ifconf.bandwidth,ifconf.datarate);
	fflush(f);
}

#ifdef USELIBLGW3
#ifdef	WITH_LBT
static	void LgwDumpLbtConf(FILE *f, T_lgw_conf_lbt_t lbtconf)
{
	int i;
	RTL_TRDBG(1, "lbt nb_channel=%d\n", lbtconf.nb_channel);
	for	(i = 0 ; i < lbtconf.nb_channel ; i++) 
	{

		RTL_TRDBG(1,"lbt channels[%d] .freq_hz=%d .scan_time_us=%d\n", 
			i,lbtconf.channels[i].freq_hz,lbtconf.channels[i].scan_time_us);
	}

	if	(f == NULL)
		return;
	fprintf(f, "lbt nb_channel=%d\n", lbtconf.nb_channel);
	for	(i = 0 ; i < lbtconf.nb_channel ; i++) 
	{
		fprintf(f,"lbt channels[%d] .freq_hz=%d .scan_time_us=%d\n", 
		i,lbtconf.channels[i].freq_hz,lbtconf.channels[i].scan_time_us);
	}
	fflush(f);
}
#endif
#endif

#ifdef USELIBLGW3
static	void LgwDumpLutConf(FILE *f,int l,struct lgw_tx_gain_s *lut)
{
	RTL_TRDBG(2,"lut%d power=%d pa=%d mix=%d dig=%d dac=%d\n",l,
		lut->rf_power,lut->pa_gain,lut->mix_gain,lut->dig_gain,lut->dac_gain);

	if	(f == NULL)
		return;
	fprintf(f,"lut%d power=%d pa=%d mix=%d dig=%d dac=%d\n",l,
	lut->rf_power,lut->pa_gain,lut->mix_gain,lut->dig_gain,lut->dac_gain);
	fflush(f);
}
#endif

#ifdef USELIBLGW3
static	int LgwConfigureLut(FILE *f,struct lgw_tx_gain_lut_s *txlut, int board, int config)
{
	char	lutsection[64];
	int	lut;
	char	*pt;
	int	ret;

	sprintf	(lutsection,"lut/%d/%d/%d",IsmFreq,LgwPowerMax,RType0);
	pt	= CfgStr(HtVarLgw,lutsection,-1,"0","");
	if	(!pt || !*pt)
	{
		RTL_TRDBG(0,"LUT '%s' not found\n",lutsection);
		strcpy	(lutsection,"lut");
	}

	memset(txlut,0,sizeof(struct lgw_tx_gain_lut_s));
	for (lut = 0; lut < TX_GAIN_LUT_SIZE_MAX; lut++)
	{
		char	var[64];
		int	dig,pa,dac,mix,pow;

		sprintf	(var,"%d",lut);
		pt	= CfgStr(HtVarLgw,lutsection,-1,var,"");
		if	(!pt || !*pt)	continue;
		txlut->size++;
		dig	= 0;
		dac	= 3;
		sscanf	(pt,"%d%d%d%d%d",&pow,&pa,&mix,&dig,&dac);
		txlut->lut[lut].rf_power	= (int8_t)pow;	// signed !!!
		txlut->lut[lut].pa_gain		= (uint8_t)pa;
		txlut->lut[lut].mix_gain	= (uint8_t)mix;
		txlut->lut[lut].dig_gain	= (uint8_t)dig;
		txlut->lut[lut].dac_gain	= (uint8_t)dac;
		LgwDumpLutConf(f,lut,&txlut->lut[lut]);
	}
	if	(config && txlut->size > 0)
	{
		ret	= tlgw_txgain_setconf[board](txlut);
		if	(ret == LGW_HAL_ERROR)
		{
			RTL_TRDBG(0,"LUT%d cannot be configured ret=%d\n",
							lut,ret);
			if	(f)
				fprintf(f,"LUT%d cannot be configured ret=%d\n", lut,ret);
			return	-1;
		}
	}
	if	(config)
	{
		RTL_TRDBG(1,"%s %s configured nblut=%d\n",
			lutsection,txlut->size==0?"not":"",txlut->size);
	}

	if	(f)
	{
		fprintf(f,"%s %s configured nblut=%d\n",
			lutsection,txlut->size==0?"not":"",txlut->size);
	}
	return	txlut->size;
}
#endif

#ifdef USELIBLGW3
int8_t GetTxCalibratedEIRP(int8_t tx_requested, float antenna_gain, float cable_loss, uint8_t board, uint8_t rfc)
{
	int8_t	tx_found;
	float	tx_tmp;
	int	i, nb_lut_max;
	struct lgw_tx_gain_lut_s	txlut;

	tx_tmp = tx_requested - antenna_gain + cable_loss;
	LgwConfigureLut(NULL, &txlut, board, 0);

	nb_lut_max	= TX_GAIN_LUT_SIZE_MAX;
	tx_found = txlut.lut[0].rf_power; /* If requested value is less than the lowest LUT power value */
	for (i = (nb_lut_max-1); i >= 0; i--)
	{
		if ( tx_tmp >= (float)txlut.lut[i].rf_power)
		{
			tx_found = txlut.lut[i].rf_power;
			break;
		}
	}
	RTL_TRDBG(1,"LUT(%d) => %d\n",tx_tmp,tx_found);
	return ((int)roundf((float)tx_found + antenna_gain - cable_loss));
}
#else
int8_t GetTxCalibratedEIRP(int8_t tx_requested, int8_t antenna_gain, int8_t cable_loss, uint8_t board, uint8_t rfc)
{
	return	tx_requested;
}
#endif

#ifdef	WITH_LBT
int	CmpChannelLbt(const void *m1, const void *m2)
{
	struct lgw_conf_lbt_chan_s *e1	= (struct lgw_conf_lbt_chan_s *)m1;
	struct lgw_conf_lbt_chan_s *e2	= (struct lgw_conf_lbt_chan_s *)m2;

	return	e1->freq_hz - e2->freq_hz;
}
#endif

int	LgwConfigure(int hot,int config)
{
	char	file[PATH_MAX];
	FILE	*f	= NULL;
	T_lgw_conf_rxrf_t rfconf;
	T_lgw_conf_rxif_t ifconf;
	int board;
	int	rfi;
	int	ifi;
	int	ret;
    int multi_board_format = 0;
	int defval = 0;
	char sectionrf[64];
	char sectionif[64];
	
	initLoraLibrary();

	if	(config)
	{
		sprintf	(file,"%s/var/log/lrr/radioparams.txt",RootAct);
		f	= fopen(file,"w");
		RTL_TRDBG(1,"RADIO configuring ...\n");
	}
	else
	{
		RTL_TRDBG(1,"RADIO dump configuration ...\n");
	}

	LgwGenConfigure(hot,config);

	LgwDumpGenConf(f);

    if (LgwBoard > 1) {
        multi_board_format = 1;
#ifdef ONE_BOARD_ONLY
        /* for testing, we want to activate only 1 board even with a multi-board lgw.ini */
        LgwBoard = 1;
#endif
    }

	for (board = 0; board < LgwBoard; board++)
	{
#if defined(FCMLB) || defined(FCPICO) || defined(FCLAMP)
		if (!strcmp(SpiDevice, "/dev/spidev1.0"))
		{
			lgw_config_spi(SpiDevice);
			lgw_config_sx1301_reset_gpio(87);
		}
		else if (!strcmp(SpiDevice, "/dev/spidev1.1"))
		{
			lgw_config_spi(SpiDevice);
			lgw_config_sx1301_reset_gpio(86);
		}
		else if (!strcmp(SpiDevice, "/dev/spidev2.0"))
		{
			lgw_config_spi(SpiDevice);
			lgw_config_sx1301_reset_gpio(88);
		}
#endif

#define ENABLE_GPIO
#if defined(WIRMANA) && defined(ENABLE_GPIO)
		void    *pt_gpio;

		pt_gpio = gpio_open(LGW_GPIO_RADIO_RESET);

		RTL_TRDBG(0,"Set GPIO %d down and up to reset radio\n", LGW_GPIO_RADIO_RESET);

		if (pt_gpio == NULL)
		{
			RTL_TRDBG(0,"Cannot initialize GPIO (gpio_open)\n");
			return -1;
		}
		RTL_TRDBG(1,"set GPIO %d mode as OUTPUT\n", LGW_GPIO_RADIO_RESET)
		if (gpio_set_mode(pt_gpio, GPIO_OUTPUT) < 0)
		{
			RTL_TRDBG(0,"Cannot set GPIO %d mode to OUTPUT (gpio_set_mode)\n", LGW_GPIO_RADIO_RESET);
			return -1;
		}
		RTL_TRDBG(1,"set GPIO %d down\n", LGW_GPIO_RADIO_RESET)
		if (gpio_set_value(pt_gpio, GPIO_LOW) < 0)
		{
			RTL_TRDBG(0,"Cannot set GPIO %d to LOW (gpio_set_value)\n", LGW_GPIO_RADIO_RESET);
			return -1;
		}

		sleep(1);
		RTL_TRDBG(1,"set GPIO %d up\n", LGW_GPIO_RADIO_RESET)
		if (gpio_set_value(pt_gpio, GPIO_HIGH) < 0)
		{
			RTL_TRDBG(0,"Cannot set GPIO %d to HIGH (gpio_set_value)\n", LGW_GPIO_RADIO_RESET);
			return -1;
		}

		sleep(1);
		RTL_TRDBG(1,"close GPIO %d\n", LGW_GPIO_RADIO_RESET)
		gpio_close(pt_gpio);
#endif


#ifdef	WITH_TTY
		{
			uint8_t uid[8];  //unique id
			if	(tlgw_connect[board](TtyDevice) != LGW_REG_SUCCESS)
			{
				RTL_TRDBG(0,"Can not connect to board via tty link %s\n"
					,TtyDevice);
				return	-1;
			}
					lgw_mcu_get_unique_id(&uid[0]);
			RTL_TRDBG(1,"stpico gw id: '%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x'\n",
				uid[0], uid[1], uid[2], uid[3], uid[4], uid[5], uid[6], uid[7]);
		}
#endif

#ifdef USELIBLGW3
		struct lgw_conf_board_s boardconf;
		memset(&boardconf, 0, sizeof(boardconf));
		boardconf.clksrc =	CfgInt(HtVarLgw,"gen",-1,"clksrc",1);
		boardconf.lorawan_public = true;
		if	(LgwSyncWord == 0x12)
			boardconf.lorawan_public = false;
		ret = tlgw_board_setconf[board](boardconf);
		if	(ret == LGW_HAL_ERROR)
		{
			RTL_TRDBG(0,"Board cannot be configured ret=%d\n", ret);
			return	-1;
		}
#endif

#ifdef	WITH_LBT
		if	(LgwLbtEnable)
		{
			T_lgw_conf_lbt_t  lbtconf;
			memset(&lbtconf, 0, sizeof(lbtconf));
			lbtconf.enable		= 1;
			lbtconf.rssi_target	= LgwLbtRssi;
			lbtconf.rssi_offset	= LgwLbtRssiOffset;
			lbtconf.nb_channel	= LgwLbtNbChannel;
			
			int	i, j, idx;
			int	lfi	= 0;
			int	alreadyConfigured	= 0;

			// search freq for LBT in "lowlvlgw.ini" first
			for	(i = 0 ; i < lbtconf.nb_channel ; i++)
			{
				int	freq_hz;
				int	scantime;

				freq_hz	= CfgInt(HtVarLgw,"lbt/chan_cfg",lfi,"freq",0);
				scantime= CfgInt(HtVarLgw,"lbt/chan_cfg",lfi,"scantime",
							LgwLbtScantime);
				if	(freq_hz == 0)	continue;
				if	(scantime == 0)	continue;
				lbtconf.channels[lfi].freq_hz		= freq_hz;
				lbtconf.channels[lfi].scan_time_us	= scantime;

				RTL_TRDBG(1, "LBT add %d freq = %d scan = %d \n", lfi, freq_hz, scantime);
				lfi++;
				if	(lfi >= LBT_CHANNEL_FREQ_NB)	break;
			}
			// no freq found search in "channels.ini"
			if	(lfi == 0)
			{
				// RDTP-14026: to avoid double, load form DN freq and complete with UL freq
				for	(i = 0 ; i < NbChannelEntryDn && i < NB_CHANNEL ; i++)
				{
					t_channel	*p;

					idx	= TbChannelEntryDn[i].index;
					p	= &TbChannel[idx];
					if	(p->name[0] == '\0')	continue;
	    			if	(p->freq_hz == 0)	continue;
					if	(p->lbtscantime == 0)	continue;

					lbtconf.channels[lfi].freq_hz		= p->freq_hz;
					lbtconf.channels[lfi].scan_time_us	= p->lbtscantime;
					lfi++;
					if	(lfi >= LBT_CHANNEL_FREQ_NB)	break;
				}
				lbtconf.nb_channel = lfi;
				for	(i = 0 ; i < NbChannelEntry && i < NB_CHANNEL ; i++)
				{
					t_channel	*p;

					idx	= TbChannelEntry[i].index;
					p	= &TbChannel[idx];
					if	(p->name[0] == '\0')	continue;
					if	(p->freq_hz == 0)	continue;
					if	(p->lbtscantime == 0)	continue;
					alreadyConfigured = 0;
					for (j=0; j<lbtconf.nb_channel && j < NB_CHANNEL; j++)
					{
					    if (lbtconf.channels[j].freq_hz == p->freq_hz)
						{
							alreadyConfigured = 1;
							break;
						}
					}
					if (alreadyConfigured)
						continue;

					lbtconf.channels[lfi].freq_hz		= p->freq_hz;
					lbtconf.channels[lfi].scan_time_us	= p->lbtscantime;
					lfi++;
					if	(lfi >= LBT_CHANNEL_FREQ_NB)	break;
				}
			}
			lbtconf.nb_channel = lfi;
			if	(lbtconf.nb_channel > 0)
			{ // sort lbtconf.channels by freq_hz
				int szelem	= sizeof(struct lgw_conf_lbt_chan_s);
				qsort(&lbtconf.channels[0],lfi,szelem,CmpChannelLbt);
				LgwDumpLbtConf(f, lbtconf);
				ret = tlgw_lbt_setconf[board](lbtconf);
				if	(ret == LGW_HAL_ERROR)
				{
					RTL_TRDBG(0,"LBT cannot be configured ret=%d\n", ret);
					return	-1;
				}
				RTL_TRDBG(1, "LBT configured with %d channels\n", lfi);
			}
			else
			{
				RTL_TRDBG(1, "LBT enabled but no channel configured\n");
			}
		}
		else
		{	// RDTP-7315: lgw_stop() does not reset LBT force to disabled
			T_lgw_conf_lbt_t  lbtconf;
			memset(&lbtconf, 0, sizeof(lbtconf));
			lbtconf.enable		= 0;
			lbtconf.nb_channel	= 0;
			tlgw_lbt_setconf[board](lbtconf);
			RTL_TRDBG(1, "LBT disabled\n");
		}
#endif


		for	(rfi = 0 ; rfi < LGW_RF_CHAIN_NB ; rfi++)
		{
			if (multi_board_format == 1) {
				sprintf(sectionrf, "rfconf:%d", board);
			} else {
				sprintf(sectionrf, "rfconf");
			}
			memset(&rfconf,0,sizeof(rfconf));
			rfconf.enable	= CfgInt(HtVarLgw,sectionrf,rfi,"enable",0);
			rfconf.freq_hz	= CfgInt(HtVarLgw,sectionrf,rfi,"freqhz",0);
#ifndef USELIBGW3
			defval = 1;
			if (rfi != 0) {
				defval = 0;
			}
#else
			defval = 1;
#endif
			rfconf.tx_enable	= CfgInt(HtVarLgw,sectionrf,rfi,"txenable",defval);
			LgwTxEnable[board][rfi]	= rfconf.tx_enable;

#ifdef USELIBLGW3
			int	rtype;
			rtype	= CfgInt(HtVarLgw,sectionrf,rfi,"radiotype",1257);
			switch	(rtype)
			{
			case	1255:	// 400-510 MHz
				rfconf.type	= LGW_RADIO_TYPE_SX1255;
			break;
			case	1257 :	// 860-1000 MHz
			default :
				rfconf.type	= LGW_RADIO_TYPE_SX1257;
			break;
			}
			if	(rfi == 0)
			{
				RType0	= rtype;
			}
			rfconf.rssi_offset	= atof(CfgStr(HtVarLgw,sectionrf,rfi,
								"rssioffset","-166.0"));
#endif
			LgwDumpRfConf(f,rfi,rfconf);
			if	(config)
			{
				ret	= tlgw_rxrf_setconf[board](rfi,rfconf);
				if	(ret == LGW_HAL_ERROR)
				{
					RTL_TRDBG(0,"RF%d cannot be configured ret=%d\n",
									rfi,ret);
					if	(f)	fclose(f);
					return	-1;
				}
			}
		}

		for	(ifi = 0 ; ifi < LGW_IF_CHAIN_NB ; ifi++)
		{
			int	fbase;

			if (multi_board_format == 1) {
				sprintf(sectionif, "ifconf:%d", board);
			} else {
				sprintf(sectionif, "ifconf");
			}
			memset(&ifconf,0,sizeof(ifconf));
			ifconf.enable	= CfgInt(HtVarLgw,sectionif,ifi,"enable",0);
			ifconf.rf_chain	= CfgInt(HtVarLgw,sectionif,ifi,"rfchain",0);
			ifconf.freq_hz	= CfgInt(HtVarLgw,sectionif,ifi,"freqhz",0);
			ifconf.bandwidth= CfgInt(HtVarLgw,sectionif,ifi,"bandwidth",0);
			ifconf.datarate	= CfgInt(HtVarLgw,sectionif,ifi,"datarate",0);
#ifdef USELIBLGW3
			ifconf.sync_word	= CfgInt(HtVarLgw,sectionif,ifi,"syncword",0);
			ifconf.sync_word_size	= CfgInt(HtVarLgw,sectionif,ifi,"syncwordsize",0);
#endif

			rfi		= ifconf.rf_chain;
			fbase		= CfgInt(HtVarLgw,sectionrf,rfi,"freqhz",0);
			LgwDumpIfConf(f,ifi,ifconf,fbase);
			if	(config)
			{
				ret	= tlgw_rxif_setconf[board](ifi,ifconf);
				if	(ret == LGW_HAL_ERROR)
				{
					RTL_TRDBG(0,"IF%d cannot be configured ret=%d\n",
									ifi,ret);
					if	(f)	fclose(f);
					return	-1;
				}
			}
		}
#ifdef USELIBLGW3
		struct lgw_tx_gain_lut_s txlut;

		ret	= LgwConfigureLut(f,&txlut, board, config);
#endif
	}

	if	(config)
	{
		RTL_TRDBG(1,"RADIO configured\n");
	}

	if	(f)
	{
		fprintf(f,"RADIO configured\n");
		fclose(f);
	}

	// RDTP-15343: freq compensation for xtal drift, for beacons and multiC
	if	(DriftComp == -1)
	{
		DriftComp = CfgInt(HtVarLrr,"lrr",-1,"driftcompensation",0);
		RTL_TRDBG(1,"lrr.driftcompensation=%d\n", DriftComp);
	}

	return	0;
}

void	LgwRegister(uint8_t board)
{
	unsigned char	syncword;

	unsigned char	sw1;
	unsigned char	sw2;

#ifdef	USELIBLGW3	 // sync word set by lgw_rxrf_setconf() hal 3.2.0
	RTL_TRDBG(1,"SET SYNC WORD %s (0x%02x)\n",LgwSyncWordStr,LgwSyncWord);
	return;		
#endif

	syncword	= (unsigned char)LgwSyncWord;


	sw1	= syncword >> 4;
	sw2	= syncword & 0x0F;

	RTL_TRDBG(1,"SET SYNC WORD sw=0x%02x registers sw1=%x sw2=%x\n",
			syncword,sw1,sw2);

	tlgw_reg_w[board](LGW_FRAME_SYNCH_PEAK1_POS,sw1); /* default 1 */
	tlgw_reg_w[board](LGW_FRAME_SYNCH_PEAK2_POS,sw2); /* default 2 */

	tlgw_reg_w[board](LGW_MBWSSF_FRAME_SYNCH_PEAK1_POS,sw1); /* default 1 */
	tlgw_reg_w[board](LGW_MBWSSF_FRAME_SYNCH_PEAK2_POS,sw2); /* default 2 */

	tlgw_reg_w[board](LGW_TX_FRAME_SYNCH_PEAK1_POS,sw1); /* default 1 */
	tlgw_reg_w[board](LGW_TX_FRAME_SYNCH_PEAK2_POS,sw2); /* default 2 */
}

void	LgwReverseRX(uint8_t board)
{
#ifndef	WIRMAV2
	return;
#endif

#if	0
	int	ret;
	ret	= CfgInt(HtVarLgw,"gen",-1,"reverserx",0);
	if	(ret >= 1)
	{
		extern	int LGW_REG_MODEM_INVERT_IQ;
		extern	int LGW_REG_ONLY_CRC_EN;

		LGW_REG_MODEM_INVERT_IQ	= 0;
		LGW_REG_ONLY_CRC_EN	= 0;
		RTL_TRDBG(1,"RADIO reverse RX ...\n");
	}
#endif
}

int	LgwStart()
{
	int	ret;
	int board;
	FILE	*chk;
	FILE	*reg;
	char	file[PATH_MAX];

	sprintf	(file,"%s/var/log/lrr/radioparams.txt",RootAct);
	chk	= fopen(file,"a");

	for (board = 0; board < LgwBoard; board++)
	{

#ifdef	USELIBLGW3	 // LUT calibration set by lgw_rxrf_setconf() hal 3.2.0
#else
		setBoardCalibration(board);
#endif
		LgwReverseRX(board);

		RTL_TRDBG(1,"RADIO starting (board %d)...\n", board);
#if defined(TRACKNET) || defined(HAL_HAVE_LGW_START_WITH_DEVICE) // TRACKNET: to be removed
#if defined(WITH_MULTI_BOARD)
		RTL_TRDBG(1, "Using SPI device: %s\n", SpiDevice[board]);
		ret	= tlgw_start[board](SpiDevice[board]);
#else
		RTL_TRDBG(1, "Using SPI device: %s\n", SpiDevice);
		ret	= tlgw_start[board](SpiDevice);
#endif
#else
		ret	= tlgw_start[board]();
#endif
		if	(ret == LGW_HAL_ERROR)
		{
			RTL_TRDBG(0,"BOARD%d RADIO cannot be started ret=%d\n", board, ret);
			if	(chk)
			{
				fprintf(chk,"BOARD%d RADIO cannot be started ret=%d\n", board,ret);
				fclose(chk);
			}
			return	-1;
		}

		LgwRegister(board);

		RTL_TRDBG(1,"RADIO board %d started ret=%d\n", board, ret);
		sprintf	(file,"%s/var/log/lrr/checkreg.out",RootAct);
		reg	= fopen(file,"w");
		if	(reg)
		{
			tlgw_reg_check[board](reg);
			fclose(reg);
		}

		if	(chk)
		{
			fprintf(chk,"RADIO board %d started ret=%d\n", board, ret);
		}
	}

	if (chk) {
		fclose(chk);
	}
	_LgwStarted	= 1;
	return	0;
}

int	LgwStarted()
{
	return	_LgwStarted;
}

void	LgwStop()
{
	int board;
	_LgwStarted	= 0;
	RTL_TRDBG(1,"RADIO stopping ...\n");
	for (board = 0; board < LgwBoard; board++) {
		tlgw_stop[board]();

#ifdef WITH_TTY
		tlgw_disconnect[board]();
#endif
	}
	RTL_TRDBG(1,"RADIO stopped\n");
}

#ifdef	WITH_GPS
void	LgwGpsTimeUpdated(struct timespec *utc_from_gps, struct timespec * ubxtime_from_gps)
{
	static unsigned	init_cpt = 0;
	static double	init_acc = 0.0;
	static double	xtal_err_cpy;
	int		ret;
	uint32_t	trig_tstamp;
	time_t		now;
	double		x;
	int board;
	now	= rtl_tmmsmono();

    /* Only 1 board can managed GPS, at least of Gemtek */

	for (board = 0; board < 1 /* LgwBoard */ ; board++)
	{
		if (Gps_ref_valid == false)
		{
			Gps_time_ref[board].systime      = time(NULL);
		}

	    long	gps_ref_age = 0;
	    gps_ref_age = (long)difftime(time(NULL), Gps_time_ref[board].systime);

#if	1
		if ((gps_ref_age >= 0) && (gps_ref_age <= GPS_REF_MAX_AGE))
		{	/* time ref is ok, validate and  */
			Gps_ref_valid = true;
		}
		else
		{	/* time ref is too old, invalidate */
			Gps_ref_valid = false;
			RTL_TRDBG(0,"LGW GPS SYNC board %d (sec=%u,ns=%09u) INVALID\n",
				board, utc_from_gps->tv_sec,utc_from_gps->tv_nsec);
				continue;
		}
#else
		Gps_ref_valid	= true;
#endif

		// get trig counter corresponding to the last GPS TIC
		ret	= tlgw_get_trigcnt[board](&trig_tstamp);
		if	(ret != LGW_HAL_SUCCESS)
		{
			RTL_TRDBG(0,"board %d cannot read trig for GPS SYNC\n", board);
			return;
		}

#ifdef HAL_VERSION_5
		ret	= tlgw_gps_sync[board](&Gps_time_ref[board],trig_tstamp,*utc_from_gps,*ubxtime_from_gps);
#else
		ret	= tlgw_gps_sync[board](&Gps_time_ref[board],trig_tstamp,*utc_from_gps);
#endif
		if	(ret != LGW_HAL_SUCCESS)
		{
			RTL_TRDBG(0,"board %d cannot sync trig for GPS SYNC slope=%f\n",
				board, Gps_time_ref[board].xtal_err);
			return;
		}

		RTL_TRDBG(3,
			"LGW GPS SYNC board=%d tus=%u difs=%d difms=%d (sec=%u,ns=%09u) (sec=%u,ns=%09u) \n",
			board, trig_tstamp,gps_ref_age,ABS(now - LgwTmmsUtcTime),
			utc_from_gps->tv_sec,utc_from_gps->tv_nsec,
			LgwCurrUtcTime.tv_sec,LgwCurrUtcTime.tv_nsec);
	}

	LgwCurrUtcTime.tv_sec	= utc_from_gps->tv_sec;
	LgwCurrUtcTime.tv_nsec	= 0;
	LgwTmmsUtcTime		= now;

	//RDTP-15343:  if drift compensation activated, compute xtal correction
	if (DriftComp == 1)
	{
		if (Gps_ref_valid == false)
		{
			XtalCorr = 1.0;
			XtalCorrOk = false;
			init_cpt = 0;
			init_acc = 0.0;
		}
		else
		{
			if (init_cpt < XERR_INIT_AVG)
			{
				if (xtal_err_cpy != 0.0)
				{
					/* initial accumulation */
					init_acc += xtal_err_cpy;
					++init_cpt;
				}
			}
			else if (init_cpt == XERR_INIT_AVG)
			{
				/* initial average calculation */
				XtalCorr = (double)(XERR_INIT_AVG) / init_acc;
				//printf("XERR_INIT_AVG=%d, init_acc=%.15lf\n", XERR_INIT_AVG, init_acc);
				XtalCorrOk = true;
				++init_cpt;
				// fprintf(log_file,"%.18lf,\"average\"\n", xtal_correct); // DEBUG
			}
			else
			{
				/* tracking with low-pass filter */
				x = 1 / xtal_err_cpy;
				XtalCorr = XtalCorr - XtalCorr/XERR_FILT_COEF + x/XERR_FILT_COEF;
			}
		}
		RTL_TRDBG(5,"drift compensation, correctionok=%d correction=%.15lf init_cpt=%d/%d init_acc=%.15lf xtal_err=%.15lf\n",
			XtalCorrOk, XtalCorr, init_cpt, XERR_INIT_AVG, init_acc, xtal_err_cpy);
        }
}
#endif


static	int	ProceedRecvPacket(T_lgw_pkt_rx_t    *p,time_t tms, int board)
{
	t_imsg		*msg;
	t_lrr_pkt	uppkt;
	int		sz;
	u_char		*data;
	t_channel	*chan;
	struct		timespec	tv;

	u_int		numchan	= -1;
	char		*namchan= "?";
	u_int		numband	= -1;

	uint8_t		txstatus = 0;
	uint8_t		timebygps = 0;
	u_char		*pt;
	u_char		mtype;

	clock_gettime(CLOCK_REALTIME,&tv);

	chan	= FindChannelForPacket(p);
	if	(chan)
	{
		numchan	= chan->channel;
		namchan	= (char *)chan->name;
		numband	= chan->subband;
	}

	tlgw_status[board](TX_STATUS,&txstatus);
	RTL_TRDBG(1,"PKT RECV board%d tms=%09u tus=%09u if=%d status=%s sz=%d freq=%d mod=0x%02x bdw=%s spf=%s ecc=%s rssi=%f snr=%f channel=%d nam='%s' G%d txstatus=%u\n",
		board,tms,p->count_us,p->if_chain,PktStatusTxt(p->status),p->size,
		p->freq_hz,p->modulation,BandWidthTxt(p->bandwidth),
		SpreadingFactorTxt(p->datarate),CorrectingCodeTxt(p->coderate),
		p->rssi,p->snr,numchan,namchan,numband,txstatus);

	DoPcap((char *)p->payload,p->size);

//	if	(p->status != STAT_NO_CRC && p->status != STAT_CRC_OK)
	if	(p->status == STAT_CRC_BAD)
	{
		LgwNbCrcError++;
		return	0;
	}

	if	(TraceLevel >= 1)
	{
	char	buff[LP_MACLORA_SIZE_MAX*3];
	char	src[64];
	LoRaMAC_t	mf;
	u_char		*pt;

	memset	(&mf,0,sizeof(mf));
	LoRaMAC_decodeHeader(p->payload,p->size,&mf);	// we assumed this is a loramac packet
	pt	= (u_char *)&mf.DevAddr;
	sprintf	(src,"%02x%02x%02x%02x",*(pt+3),*(pt+2),*(pt+1),*pt);

	rtl_binToStr((unsigned char *)p->payload,p->size,buff,sizeof(buff)-10);
	RTL_TRDBG(1,"PKT RECV data='%s' seq=%d devaddr=%s \n",buff,mf.FCnt,src);
	if	(OkDevAddr(src) <= 0)
		return	0;
	}

	// RDTP-5475
	if	(SynchroPeriod && p->size >= 5)
	{
		u_char	*pt;
		pt	= p->payload;
		mtype	= *pt >> 5;
		if	(mtype == 7)
		{
			if	(memcmp(pt+1,(void *)&LrrID,4) == 0)
			{
				RTL_TRDBG(1,"PKT RECV synchro echo tusetx=%u => drop\n",p->count_us);
			}
			else
			{
				RTL_TRDBG(1,"PKT RECV synchro from lrrid=%08u => drop\n",*(u_int *)(pt+1));
			}
			return	0;
		}
	}


	if	(p->size > LP_MACLORA_SIZE_MAX)
	{
		LgwNbSizeError++;
		return	0;
	}

	if	(!chan)
	{
		LgwNbChanUpError++;
		return	0;
	}

	data	= (u_char *)malloc(p->size);
	if	(!data)
		return	-1;
	memcpy	(data,p->payload,p->size);

	msg	= rtl_imsgAlloc(IM_DEF,IM_LGW_RECV_DATA,NULL,0);
	if	(!msg)
		return	-2;

	memset	(&uppkt,0,sizeof(t_lrr_pkt));
	uppkt.lp_flag	= LP_RADIO_PKT_UP;
	if	(p->status == STAT_NO_CRC)
	{
		uppkt.lp_flag	= uppkt.lp_flag | LP_RADIO_PKT_NOCRC;
	}
	uppkt.lp_lrrid	= LrrID;
	uppkt.lp_tms	= tms;
	if	(!uppkt.lp_tms)	uppkt.lp_tms	= 1;
	// by default we set linux time
	uppkt.lp_gss	= (time_t)tv.tv_sec;
	uppkt.lp_gns	= (u_int)tv.tv_nsec;
#ifdef	WITH_GPS
	if	(UseGpsTime && Gps_ref_valid && GpsStatus == 'U')
	{
		// for classb we have to use a precise UTC time otherwise
		// the LRC will not be able to give a good reply to commands
		// like TiminigBeaconRequest
		int	destim;
		int	ret;
		struct timespec butc;
		struct timespec lutc;

		LgwEstimUtc(&lutc);
		ret	= tlgw_cnt2utc[board](Gps_time_ref[board],p->count_us,&butc);
		if	(ret == LGW_HAL_SUCCESS)
		{
			destim	= LgwDiffMsUtc(&butc,&lutc);
			// time comes from the radio board not from linux
			uppkt.lp_flagm	= uppkt.lp_flagm | LP_RADIO_PKT_GPSTIME;
			uppkt.lp_gss	= (time_t)butc.tv_sec;
			uppkt.lp_gns	= (u_int)butc.tv_nsec;

			RTL_TRDBG(1,"LGW PPS DATE eutc=(%09u,%03ums) butc=(%09u,%09u) diff=%d tus=%u\n",
				lutc.tv_sec,lutc.tv_nsec/1000000,
				butc.tv_sec,butc.tv_nsec,
				destim,p->count_us);
			timebygps	= 1;
		}
	}
#endif
	// RDTP-5475
	if	(p->size >= 5)
	{
		pt	= p->payload;
		mtype	= *pt >> 5;
		if	(mtype == 7)
		{
			// macro GW do not report synchro if not configured
			if	(SynchroForward == 0)
			{
				RTL_TRDBG(1,"PKT RECV synchro forwarding not activated => drop\n");
				free	(data);
				return	0;
			}
			// macro GW do not report synchro not timestamped by GPS
			if	(timebygps == 0)
			{
				RTL_TRDBG(1,"PKT RECV synchro but no gps time => drop\n");
				free	(data);
				return	0;
			}
			RTL_TRDBG(1,"PKT RECV synchro lrrid=%08x cnt=%u tus=%u butc=(%09u,%09u)\n",
				*(u_int *)(pt+1),*(u_short *)(pt+6),*(u_int *)(pt+9),
				uppkt.lp_gss,uppkt.lp_gns);
		}
	}



	uppkt.lp_tus		= p->count_us;

	/* from lgw_x8 */
	/* formerly: uppkt.lp_chain		= p->rf_chain; */
	//lp_chain =  antennaid(4b) + board(3b) + rfchain(1b)
	// with antenna: 0, rfchain (chipid): 0
	uppkt.lp_chain		= board << 4; /* 1 antenna per board ?*/
	uppkt.lp_chain		|= (board&0x0f) << 1 ;
	uppkt.lp_chain		|= 0&0x01;

	uppkt.lp_rssi		= p->rssi;
	uppkt.lp_snr		= p->snr;
	uppkt.lp_channel	= chan->channel;
	uppkt.lp_subband	= chan->subband;
	uppkt.lp_spfact		= CodeSpreadingFactor(p->datarate);
	uppkt.lp_correct	= CodeCorrectingCode(p->coderate);
	uppkt.lp_bandwidth	= CodeBandWidth(p->bandwidth);
	uppkt.lp_freq		= p->freq_hz;
	uppkt.lp_freqoffset	= 0;	// not available 1.0/1.5
	uppkt.lp_size		= p->size;
	uppkt.lp_payload	= data;

#ifdef LP_TP31
	uppkt.lp_tmoa	= TmoaLrrPacketUp(p);
	RTL_TRDBG(3,"TmoaLrrPacketUp()=%f\n", uppkt.lp_tmoa);
	DcTreatUplink(&uppkt);
#endif

	sz	= sizeof(t_lrr_pkt);
	if	( rtl_imsgDupData(msg,&uppkt,sz) != msg)
//	if	( rtl_imsgCpyData(msg,&uppkt,sz) != msg)
	{
		rtl_imsgFree(msg);
		return	-3;
	}

	rtl_imsgAdd(MainQ,msg);
	return	1;
}

int	LgwDoRecvPacket(time_t tms)
{
	T_lgw_pkt_rx_t	rxpkt[LGW_RECV_PKT];
	T_lgw_pkt_rx_t	*p;
	int			nbpkt;
	int			i;
	int			ret;
	int			nb	= 0;
	int			board;
	static int  lastboard = -1;

	board = (lastboard + 1) % LgwBoard;
	lastboard = board;

#if	0
	time_t			t0,t1;
	rtl_timemono(&t0);
	rtl_timemono(&t1);
	RTL_TRDBG(1,"lgw_receive() => %dms\n",ABS(t1-t0));
#endif

	nbpkt	= tlgw_receive[board](LGW_RECV_PKT,rxpkt);
	if	(nbpkt <= 0)
	{
		RTL_TRDBG(9,"Board %d: PKT nothing to Recv=%d\n",board, nbpkt);
		return	0;
	}

#if defined(KEROS) && defined(WITH_LED_MGT)
	LedShotRxLora();
#endif

#if	0
{
	uint32_t	mycnt;
	lgw_get_trigcnt(&mycnt);
	RTL_TRDBG(1,"after lgw_receive lgw_get_trigcnt=%u\n", mycnt);
}
#endif

	LgwNbPacketRecv	+= nbpkt;
	nb	= 0;
	for	(i = 0 ; i < nbpkt ; i++)
	{
		p	= &rxpkt[i];
		if	((ret=ProceedRecvPacket(p,tms, board)) < 0)
		{
			RTL_TRDBG(1,"PKT RECV board%d not treated ret=%d\n",board,ret);
		}
		else
		{
			nb++;
		}
	}

	return	nb;
}

static	u_int	WaitSendPacket(int blo,time_t tms,uint8_t *txstatus, int board)
{
	time_t	diff;
	int	j	= 0;

	if	(!blo)
	{
		if	(LgwWaitSending <= 0)
			return	0;
	}
	RTL_TRDBG(1,"PKT SEND enter blocking mode\n");
	do
	{
		j++;
		tlgw_status[board](TX_STATUS,txstatus);
		usleep(1000);	// * 6000 => 6s max
	} while ((*txstatus != TX_FREE) && (j < 6000));
	diff	= rtl_tmmsmono();
	diff	= ABS(diff - tms);

	RTL_TRDBG(3,"PKT sent j=%d status=%d\n",j,*txstatus);
	if	(*txstatus != TX_FREE)
	{
		RTL_TRDBG(0,"PKT SEND status(=%d) != TX_FREE\n",*txstatus);
	}
	return	diff;
}

static	int	SendPacketNow(int blo,t_lrr_pkt *downpkt,T_lgw_pkt_tx_t txpkt, int board)
{
	uint8_t txstatus = 0;
	int	left;
	int	ret;
	int	diff	= 0;
	int	duration = 0;
	u_int	tms;
	int	tempgain;
	int	diff2	= 0;

	left	= LgwNbPacketWait;

	tlgw_status[board](TX_STATUS,&txstatus);
	if	(downpkt->lp_lgwdelay == 0 && txstatus != TX_FREE)
	{
		RTL_TRDBG(1,"PKT board%d busy status=%d left=%d\n",board,txstatus,left-1);
		LgwNbBusySend++;
		return	-1;
	}
	if	(0 && downpkt->lp_lgwdelay && txstatus != TX_FREE) // TODO
	{
		RTL_TRDBG(1,"PKT board%d busy status=%d left=%d\n",board,txstatus,left-1);
		LgwNbBusySend++;
		return	-1;
	}


	ChangeChannelFreq(downpkt,&txpkt); //	NFR703
	downpkt->lp_freq= txpkt.freq_hz;

	tempgain	= LgwTempPowerGain();
	txpkt.rf_power	= txpkt.rf_power + tempgain;
	/* PT-1350: gateways with SAW filter may require compensation */
	txpkt.rf_power  = txpkt.rf_power + LgwTxPowerComp(IsmBand, txpkt.freq_hz);
	if	(txpkt.rf_power > LgwPowerMax)
		txpkt.rf_power	= LgwPowerMax;

	tms	= rtl_tmmsmono();
	diff	= ABS(tms - downpkt->lp_tms);
	downpkt->lp_stopbylbt	= 0;

	if	(txpkt.tx_mode != IMMEDIATE)
	{
		ret	= LgwTrigControl(board,txpkt.count_us, NULL);
		if	(ret <= 0)
		{
			if	(downpkt->lp_beacon)
				return	-4;
			if	(downpkt->lp_classb || downpkt->lp_classcmc)
			{
				SetIndic(downpkt,0,-1,-1,LP_CB_NA);
				return	-4;
			}
			if	(Rx2Channel && Rx2Channel->freq_hz == txpkt.freq_hz)
				SetIndic(downpkt,0,-1,LP_C2_NA,-1);
			else
				SetIndic(downpkt,0,LP_C1_NA,-1,-1);

			return	-4;
		}
	}

	// RDTP-15343: freq compensation for xtal drift, for beacons and multiC
	if (DriftComp == 1 && XtalCorrOk && txpkt.tx_mode == ON_GPS)
	{
		uint32_t	newfreq;

		newfreq = (uint32_t)(XtalCorr * (double)txpkt.freq_hz);
		RTL_TRDBG(1,"drift compensation, correction = %.15f, frequency %u Hz %+d Hz => %u Hz\n",
			XtalCorr, txpkt.freq_hz, newfreq - txpkt.freq_hz, newfreq);
		txpkt.freq_hz = newfreq;
	}

	ret	= tlgw_send[board](txpkt);
	diff2	= ABS(rtl_tmmsmono() - tms);
	if	(ret != LGW_HAL_SUCCESS)
	{
#ifdef	WITH_LBT
		if	(ret == LGW_LBT_ISSUE)
		{
			downpkt->lp_stopbylbt	= 1;
			if	(downpkt->lp_beacon)
			{
				LgwBeaconLastDeliveryCause	= LP_CB_LBT;
				goto	stop_by_blt;
			}
			if	(downpkt->lp_classb)
			{
				SetIndic(downpkt,0,-1,-1,LP_CB_LBT);
				goto	stop_by_blt;
			}
			if	(downpkt->lp_classcmc)
			{
				SetIndic(downpkt,0,-1,-1,LP_CB_LBT);
				goto	stop_by_blt;
			}
			if	(Rx2Channel && Rx2Channel->freq_hz == txpkt.freq_hz)
				SetIndic(downpkt,0,-1,LP_C2_LBT,-1);
			else
				SetIndic(downpkt,0,LP_C1_LBT,-1,-1);
stop_by_blt:
			RTL_TRDBG(1,"PKT send stop by lbt=%d freq=%d\n",ret,
				txpkt.freq_hz);
			return	-3;
		}
#endif
		RTL_TRDBG(0,"PKT board%d send error=%d\n",board,ret);
		return	-2;
	}
#if defined(KEROS) && defined(WITH_LED_MGT)
	LedShotTxLora();
#endif
	if	(downpkt->lp_beacon)
	{
		LgwBeaconSentCnt++;
		LgwBeaconLastDeliveryCause	= 0;
	}
	if	(downpkt->lp_classcmc)
	{
		LgwClassCSentCnt++;
		LgwClassCLastDeliveryCause	= 0;
	}
	LgwNbPacketSend++;
	tlgw_status[board](TX_STATUS,&txstatus);
	if	(!downpkt->lp_bypasslbt && downpkt->lp_lgwdelay == 0
						&& txstatus != TX_EMITTING)
	{
		RTL_TRDBG(0,"PKT SEND board%d status(=%d) != TX_EMITTING\n",board,txstatus);
	}
	if	(0 && downpkt->lp_lgwdelay && txstatus != TX_SCHEDULED)	// TODO
	{
		RTL_TRDBG(0,"PKT SEND board%d status(=%d) != TX_SCHEDULED\n",board,txstatus);
	}

#ifdef LP_TP31
	// must be done only if packet was really sent
	DcTreatDownlink(downpkt);
#endif

	RTL_TRDBG(1,"PKT SEND board%d tms=%09u/%d rfchain=%d status=%d sz=%d left=%d freq=%d mod=0x%02x bdw=%s spf=%s ecc=%s pr=%d nocrc=%d ivp=%d pw=%d temppw=%d tx=%d\n",
		board,tms,diff,txpkt.rf_chain,txstatus,txpkt.size,left,txpkt.freq_hz,
		txpkt.modulation,BandWidthTxt(txpkt.bandwidth),
		SpreadingFactorTxt(txpkt.datarate),CorrectingCodeTxt(txpkt.coderate),
		txpkt.preamble,txpkt.no_crc,txpkt.invert_pol,txpkt.rf_power,tempgain,
		diff2);


	if	(downpkt->lp_lgwdelay == 0)
	{
		duration	= WaitSendPacket(blo,tms,&txstatus, board);
	}

	// the packet is now passed to the sx13, compute time added by all
	// treatements LRC/LRR, only if lgwdelay or non blocking mode

	diff	= 0;
	if	(downpkt->lp_lgwdelay || (!blo && !LgwWaitSending))
	{
		u_int	scheduled;	// time before sx13 schedule request

		diff	= ABS(rtl_tmmsmono() - downpkt->lp_tms);

		scheduled		= downpkt->lp_lgwdelay;
		LastTmoaRequested[board]	= ceil(downpkt->lp_tmoa/1000);
		LastTmoaRequested[board]	= LastTmoaRequested [board]+
					(LastTmoaRequested[board] * 10)/100;
		RTL_TRDBG(1,"LGW DELAY tmao request=%dms + sched=%dms\n",
				LastTmoaRequested,scheduled);
		LastTmoaRequested[board]	= LastTmoaRequested[board] + scheduled + 70;
		CurrTmoaRequested[board]	= LastTmoaRequested[board];
	}
	else
	{
	// TODO meme si no delay il faut compter la requete
		LastTmoaRequested[board]	= 0;
		CurrTmoaRequested[board]	= 0;
	}

	if	(TraceLevel >= 1)
	{
		char	buff[LP_MACLORA_SIZE_MAX*3];
		char	src[64];
		LoRaMAC_t	mf;
		u_char		*pt;
		char		*pktclass = "";

		memset	(src,0,sizeof(src));
		memset	(&mf,0,sizeof(mf));
		if	(downpkt->lp_beacon == 0)
		{
			// we assumed this is a loramac packet
			LoRaMAC_decodeHeader(txpkt.payload,txpkt.size,&mf);	
			pt	= (u_char *)&mf.DevAddr;
			sprintf	(src,"%02x%02x%02x%02x",
						*(pt+3),*(pt+2),*(pt+1),*pt);
			if	(downpkt->lp_classb)
			{
				diff		= 0;
				pktclass	= "classb";
				RTL_TRDBG(1,"PKT SEND classb period=%d sidx=%d sdur=%f window(%09u,%09u) %d/%d/%d\n",
					downpkt->lp_period,downpkt->lp_sidx,downpkt->lp_sdur,
					((downpkt->lp_period-1)*128)+downpkt->lp_gss0,
					downpkt->lp_gns0,
					downpkt->lp_idxtry,downpkt->lp_nbtry,downpkt->lp_maxtry);
			} else {
				diff		= 0;
				pktclass	= "classa";
			}
			if	(downpkt->lp_classc)
			{
				diff		= 0;
				pktclass	= "classc";
			}
			if	(downpkt->lp_classcmc)
			{
				diff		= 0;
				pktclass	= "classcmc";
			}
			if	(downpkt->lp_synchro)
			{
				diff		= 0;
				pktclass	= "synchro";
			}
		}
		else
		{
			pktclass	= "beacon";
		}


		rtl_binToStr(txpkt.payload,txpkt.size,buff,sizeof(buff)-10);
		if	(downpkt->lp_lgwdelay == 0)
		{
			if	(blo || LgwWaitSending)
			{
				RTL_TRDBG(1,"PKT SEND blocking board%d %s dur=%ums data='%s' seq=%d devaddr=%s\n",
					board,pktclass,duration,buff,mf.FCnt,src);
			}
			else
			{
				RTL_TRDBG(1,"PKT SEND noblock board%d %s dur=%fms diff=%ums data='%s' seq=%d devaddr=%s\n",
					board,pktclass,downpkt->lp_tmoa/1000,diff,buff,mf.FCnt,src);
			}
		}
		else
		{
			RTL_TRDBG(1,"PKT SEND async board%d %s dur=%fms diff=%ums data='%s' seq=%d devaddr=%s\n",
				board,pktclass,downpkt->lp_tmoa/1000,diff,buff,mf.FCnt,src);
		}
	}

	DoPcap((char *)txpkt.payload,txpkt.size);

	return	0;
}

static	void	SetTrigTargetCommun(t_lrr_pkt *downpkt,T_lgw_pkt_tx_t *txpkt)
{
	int board;

	board = (downpkt->lp_chain&0x0F) >> 1;
	
	if	(downpkt->lp_lgwdelay == 0)
	{
		if	(LgwLbtEnable == 0)
		{
			txpkt->tx_mode 	= IMMEDIATE;
			RTL_TRDBG(1,"LRR DELAY -> tx mode immediate tms=%u tus=%u\n",
				downpkt->lp_tms,downpkt->lp_tus);
			return;
		}

		// RDTP-857 LBT is enable HAL refuses IMMEDIATE mode
		// => we force TIMESTAMPED mode in 5ms
		uint32_t	trig_tstamp;


		tlgw_reg_w[board](LGW_GPS_EN,0);
		tlgw_get_trigcnt[board](&trig_tstamp);
		tlgw_reg_w[board](LGW_GPS_EN,1);

		downpkt->lp_tus		= trig_tstamp + (LgwLbtClasscDelay*1000);
		downpkt->lp_delay	= 0;
		downpkt->lp_bypasslbt	= 1;

		RTL_TRDBG(1,"LRR DELAY -> tx mode immediate with LBT tms=%u tus=%u\n",
			downpkt->lp_tms,downpkt->lp_tus);
		// and no return to force TIMESTAMPED mode
	}

	txpkt->tx_mode 	= TIMESTAMPED;
	txpkt->count_us	= downpkt->lp_tus;
	txpkt->count_us	= txpkt->count_us + (downpkt->lp_delay * 1000);
	txpkt->count_us	= txpkt->count_us - Sx13xxStartDelay;

	RTL_TRDBG(3,"LGW DELAY trigtarget=%u trigorig=%u\n",
		txpkt->count_us,downpkt->lp_tus);

	return;
}

#ifdef	WITH_GPS
static	void	SetTrigTargetGps(t_lrr_pkt *downpkt,T_lgw_pkt_tx_t *txpkt, int board)
{
	static	int	classbdelay	= -1;
	int		ret;
	uint32_t	trig_tstamp;
	uint32_t	trig_estim;
	struct timespec utc_time;

	if	(classbdelay == -1)
	{
		classbdelay	= CfgInt(HtVarLrr,"classb",-1,"adjustdelay",0);
		RTL_TRDBG(1,"classb.adjustdelay=%d\n",classbdelay);
	}
	if	(downpkt->lp_beacon)
	{
		utc_time.tv_sec	= downpkt->lp_gss;
		utc_time.tv_nsec= downpkt->lp_gns;
		ret	= tlgw_utc2cnt[BOARD_CNT](Gps_time_ref[BOARD_CNT],utc_time,&trig_tstamp);
		if	(ret != LGW_GPS_SUCCESS)
		{
			RTL_TRDBG(0,"PKT SEND beacon error lgw_utc2cnt() from (%u,%09u)\n",
				downpkt->lp_gss,downpkt->lp_gns);
			LgwBeaconLastDeliveryCause	= LP_C1_DELAY;
			return;
		}
		LgwEstimTrigCnt(board, &trig_estim);
		txpkt->tx_mode 	= ON_GPS;
		txpkt->count_us	= trig_tstamp - Sx13xxStartDelay;
		downpkt->lp_lgwdelay	=
				ABS(txpkt->count_us - trig_estim ) / 1000;

		RTL_TRDBG(1,"PKT SEND beacon trigtarget=%u trigonpps=%u trigestim=%u diffestim=%d pkt=(%u,%09u) utc=(%u,%09u)\n",
			txpkt->count_us,Gps_time_ref[BOARD_CNT].count_us,trig_estim,
			downpkt->lp_lgwdelay,downpkt->lp_gss,downpkt->lp_gns,
			Gps_time_ref[BOARD_CNT].utc.tv_sec,Gps_time_ref[BOARD_CNT].utc.tv_nsec);

		return;
	}
	if	(downpkt->lp_classcmc)
	{
		utc_time.tv_sec	= downpkt->lp_gss;
		utc_time.tv_nsec= downpkt->lp_gns;
		ret	= tlgw_utc2cnt[BOARD_CNT](Gps_time_ref[BOARD_CNT],utc_time,&trig_tstamp);
		if	(ret != LGW_GPS_SUCCESS)
		{
			RTL_TRDBG(0,"PKT SEND classcmc error lgw_utc2cnt() from (%u,%09u)\n",
				downpkt->lp_gss,downpkt->lp_gns);
			LgwClassCLastDeliveryCause	= LP_C1_DELAY;
			return;
		}
		LgwEstimTrigCnt(board, &trig_estim);
		txpkt->tx_mode 	= ON_GPS;
		txpkt->count_us	= trig_tstamp - Sx13xxStartDelay;
		downpkt->lp_lgwdelay	=
				ABS(txpkt->count_us - trig_estim ) / 1000;

		RTL_TRDBG(1,"PKT SEND classcmc trigtarget=%u trigonpps=%u trigestim=%u diffestim=%d pkt=(%u,%09u) utc=(%u,%09u)\n",
			txpkt->count_us,Gps_time_ref[BOARD_CNT].count_us,trig_estim,
			downpkt->lp_lgwdelay,downpkt->lp_gss,downpkt->lp_gns,
			Gps_time_ref[BOARD_CNT].utc.tv_sec,Gps_time_ref[BOARD_CNT].utc.tv_nsec);

		return;
	}
	if	(downpkt->lp_classb)
	{
		utc_time.tv_sec	= downpkt->lp_gss;
		utc_time.tv_nsec= downpkt->lp_gns;
		ret	= tlgw_utc2cnt[BOARD_CNT](Gps_time_ref[BOARD_CNT],utc_time,&trig_tstamp);
		if	(ret != LGW_GPS_SUCCESS)
		{
			RTL_TRDBG(0,"PKT SEND classb error lgw_utc2cnt() from (%u,%09u)\n",
				downpkt->lp_gss,downpkt->lp_gns);
			return;
		}
		LgwEstimTrigCnt(board, &trig_estim);
		txpkt->tx_mode 	= TIMESTAMPED;
		txpkt->count_us	= trig_tstamp - Sx13xxStartDelay + classbdelay;
		downpkt->lp_lgwdelay	= 
				ABS(txpkt->count_us - trig_estim ) / 1000;
		RTL_TRDBG(1,"PKT SEND classb trigtarget=%u trigonpps=%u trigestim=%u diffestim=%d adj=%d pkt=(%u,%09u) utc=(%u,%09u)\n",
			txpkt->count_us,Gps_time_ref[BOARD_CNT].count_us,trig_estim,
			downpkt->lp_lgwdelay,classbdelay,downpkt->lp_gss,downpkt->lp_gns,
			Gps_time_ref[BOARD_CNT].utc.tv_sec,Gps_time_ref[BOARD_CNT].utc.tv_nsec);

		return;
	}
	SetTrigTargetCommun(downpkt,txpkt);
	return;
}
#endif

// RDTP-5475
static	void	SetTrigTargetSynchro(t_lrr_pkt *downpkt,T_lgw_pkt_tx_t *txpkt)
{
	static	int	classbdelay	= -1;
	int		ret;
	int		board;
	uint32_t	trig_tstamp;
	uint32_t	trig_estim;

	board = (downpkt->lp_chain&0x0F) >> 1;

	if	(classbdelay == -1)
	{
		classbdelay	= CfgInt(HtVarLrr,"classb",-1,"adjustdelay",0);
		RTL_TRDBG(1,"classb.adjustdelay=%d\n",classbdelay);
	}
	if	(SynchroPeriod && downpkt->lp_beacon)
	{
		ret	= LgwSynchroUtc2Cnt(downpkt->lp_gss,downpkt->lp_gns,&trig_tstamp);
		if	(ret != 0)
		{
			RTL_TRDBG(0,"PKT SEND beacon error LgwSynchroUtc2Cnt() from (%u,%09u)\n",
				downpkt->lp_gss,downpkt->lp_gns);
			LgwBeaconLastDeliveryCause	= LP_C1_DELAY;
			return;
		}
		tlgw_reg_w[board](LGW_GPS_EN,0);
		ret	= tlgw_get_trigcnt[board](&trig_estim); 
		tlgw_reg_w[board](LGW_GPS_EN,1);

		if	(trig_tstamp < trig_estim)
		{
			RTL_TRDBG(0,"PKT SEND beacon error past target=%u curr=%u diff=%d drift=%f from (%u,%09u)\n",
				trig_tstamp,trig_estim,
				trig_tstamp-trig_estim,
				SynchroDrift,
				downpkt->lp_gss,downpkt->lp_gns);
			LgwBeaconLastDeliveryCause	= LP_C1_DELAY;
			return;
		}
		txpkt->tx_mode 	= TIMESTAMPED;	// instead on ON_GPS
		txpkt->count_us	= trig_tstamp + 1500;
		downpkt->lp_lgwdelay	=
				ABS(txpkt->count_us - trig_estim ) / 1000;

		RTL_TRDBG(1,"PKT SEND beacon trigtarget=%u trigestim=%u diffestim=%d drift=%f pkt=(%09u,%09u)\n",
			txpkt->count_us,trig_estim,downpkt->lp_lgwdelay,SynchroDrift,
			downpkt->lp_gss,downpkt->lp_gns);

		return;
	}
	if	(SynchroPeriod && downpkt->lp_classcmc)
	{
		ret	= LgwSynchroUtc2Cnt(downpkt->lp_gss,downpkt->lp_gns,&trig_tstamp);
		if	(ret != 0)
		{
			RTL_TRDBG(0,"PKT SEND classmc error LgwSynchroUtc2Cnt() from (%u,%09u)\n",
				downpkt->lp_gss,downpkt->lp_gns);
			LgwClassCLastDeliveryCause	= LP_C1_DELAY;
			return;
		}
		tlgw_reg_w[board](LGW_GPS_EN,0);
		ret	= tlgw_get_trigcnt[board](&trig_estim); 
		tlgw_reg_w[board](LGW_GPS_EN,1);

		if	(trig_tstamp < trig_estim)
		{
			RTL_TRDBG(0,"PKT SEND classmc error past target=%u curr=%u diff=%d drift=%f from (%u,%09u)\n",
				trig_tstamp,trig_estim,
				trig_tstamp-trig_estim,
				SynchroDrift,
				downpkt->lp_gss,downpkt->lp_gns);
			LgwClassCLastDeliveryCause	= LP_C1_DELAY;
			return;
		}
		txpkt->tx_mode 	= TIMESTAMPED;	// instead on ON_GPS
		txpkt->count_us	= trig_tstamp + 1500;
		downpkt->lp_lgwdelay	=
				ABS(txpkt->count_us - trig_estim ) / 1000;

		RTL_TRDBG(1,"PKT SEND classmc trigtarget=%u trigestim=%u diffestim=%d drift=%f pkt=(%09u,%09u)\n",
			txpkt->count_us,trig_estim,downpkt->lp_lgwdelay,SynchroDrift,
			downpkt->lp_gss,downpkt->lp_gns);

		return;
	}
	if	(SynchroPeriod && downpkt->lp_classb)
	{
		ret	= LgwSynchroUtc2Cnt(downpkt->lp_gss,downpkt->lp_gns,&trig_tstamp);
		if	(ret != 0)
		{
			RTL_TRDBG(0,"PKT SEND classb error LgwSynchroUtc2Cnt() from (%u,%09u)\n",
				downpkt->lp_gss,downpkt->lp_gns);
			return;
		}
		tlgw_reg_w[board](LGW_GPS_EN,0);
		ret	= tlgw_get_trigcnt[board](&trig_estim); 
		tlgw_reg_w[board](LGW_GPS_EN,1);

		if	(trig_tstamp < trig_estim)
		{
			RTL_TRDBG(0,"PKT SEND classb error past target=%u curr=%u diff=%d drift=%f from (%u,%09u)\n",
				trig_tstamp,trig_estim,
				trig_tstamp-trig_estim,
				SynchroDrift,
				downpkt->lp_gss,downpkt->lp_gns);
			return;
		}
		txpkt->tx_mode 	= TIMESTAMPED;
		txpkt->count_us	= trig_tstamp + classbdelay;
		downpkt->lp_lgwdelay	=
				ABS(txpkt->count_us - trig_estim ) / 1000;

		RTL_TRDBG(1,"PKT SEND classb trigtarget=%u trigestim=%u diffestim=%d drift=%f pkt=(%09u,%09u)\n",
			txpkt->count_us,trig_estim,downpkt->lp_lgwdelay,SynchroDrift,
			downpkt->lp_gss,downpkt->lp_gns);

		return;
	}

	SetTrigTargetCommun(downpkt,txpkt);
	return;
}

static	int	SendPacket(t_imsg  *msg)
{
	t_lrr_pkt	*downpkt;
	T_lgw_pkt_tx_t 	txpkt;
	t_channel	*chan;
	int	ret;
	int board;
	int rfchain;
	int	blo	= 0;

	downpkt	= msg->im_dataptr;

	if	(downpkt->lp_channel >= MaxChannel ||
		TbChannel[downpkt->lp_channel].name[0] == '\0')
	{
		LgwNbChanDownError++;	
		RTL_TRDBG(1,"SendPacket: ChanDownError += 1 chan=%d maxchan=%d name=%s\n",
			downpkt->lp_channel, MaxChannel, TbChannel[downpkt->lp_channel].name[0]);
		return	0;
	}
	chan	= &TbChannel[downpkt->lp_channel];

	memset	(&txpkt,0,sizeof(txpkt));

//	txpkt.freq_hz = 868100000;
//	txpkt.modulation = MOD_LORA;
//	txpkt.bandwidth = BW_125KHZ;
//	txpkt.datarate = DR_LORA_SF10;
//	txpkt.coderate = CR_LORA_4_5;


	board = (downpkt->lp_chain & 0x0F) >> 1;

	rfchain = downpkt->lp_chain&0x01;
	if	(rfchain >= LGW_RF_CHAIN_NB)
	{
		rfchain	= 0;
	}
	// RDTP-5475
	if	(SynchroPeriod)
		SetTrigTargetSynchro(downpkt,&txpkt);
	else
#ifdef	WITH_GPS
		SetTrigTargetGps(downpkt,&txpkt,board);
#else
		SetTrigTargetCommun(downpkt,&txpkt);
#endif

	txpkt.rf_chain 		= LgwCheckTxEnable(board, rfchain);
	txpkt.freq_hz 		= chan->freq_hz;
	txpkt.modulation 	= chan->modulation;
	txpkt.bandwidth 	= chan->bandwidth;
	txpkt.datarate 		= DecodeSpreadingFactor(downpkt->lp_spfact);
	txpkt.coderate		= DecodeCorrectingCode(downpkt->lp_correct);
	txpkt.invert_pol	= LgwInvertPol;
	txpkt.no_crc 		= LgwNoCrc;
	txpkt.no_header 	= LgwNoHeader;
	txpkt.preamble 		= LgwPreamble;
	txpkt.rf_power 		=
		chan->power - AntennaGain[0] + CableLoss[0];	// TODO index
	if	(downpkt->lp_beacon)
	{
		txpkt.invert_pol	= LgwInvertPolBeacon;
		txpkt.no_crc		= 1;
		txpkt.no_header		= 1;
		txpkt.preamble 		= 10;
	}

	// RDTP-5475
	if	(downpkt->lp_synchro)
	{
		RTL_TRDBG(0,"Tests Synchro: force invert_pol=0 and no_crc=0\n");
		txpkt.invert_pol	= 0;
		txpkt.no_crc 		= 0;
		txpkt.tx_mode		= TIMESTAMPED;
	}

	if ((downpkt->lp_flag&LP_RADIO_PKT_ACKMAC) == LP_RADIO_PKT_ACKMAC)
		txpkt.preamble = LgwPreambleAck;

	if ((downpkt->lp_flag&LP_RADIO_PKT_ACKDATA) == LP_RADIO_PKT_ACKDATA
	&& (downpkt->lp_flag&LP_RADIO_PKT_802154) == LP_RADIO_PKT_802154)
	{
		u_char	ackseq	= downpkt->lp_opaque[2];
		u_char	ackpend	= (downpkt->lp_opaque[0] >> 4) & 1;
		u_char	ackack	= (downpkt->lp_opaque[0] >> 5) & 1;

		u_char	dataseq	= downpkt->lp_payload[2];
		u_char	datapend= (downpkt->lp_payload[0] >> 4) & 1;
		u_char	dataack	= (downpkt->lp_payload[0] >> 5) & 1;

		RTL_TRDBG(1,"PKT SEND ACK(seq=%u,p=%u,a=%u)+wait=%u+DATA(seq=%u,p=%u,a=%u)\n",
			ackseq,ackpend,ackack,
			LgwAckData802Wait,
			dataseq,datapend,dataack);
		txpkt.size = 5;		// size for an ACK frame 802
		memcpy	(txpkt.payload,downpkt->lp_opaque,txpkt.size);
		ret	= SendPacketNow(blo=1,downpkt,txpkt, board);
		if	(ret < 0)
		{
			RTL_TRDBG(0,"SendPacketNow() error => %d\n",ret);
		}
		usleep(LgwAckData802Wait*1000);	// 10 ms
	}

	txpkt.size = downpkt->lp_size;
	memcpy	(txpkt.payload,downpkt->lp_payload,downpkt->lp_size);
#if	0	// no more free when trying to send so we can retry ...
	free	(downpkt->lp_payload);
	downpkt->lp_payload	= NULL;
#endif

	ret	= SendPacketNow(blo,downpkt,txpkt, board);
	if	(ret < 0)
	{
		RTL_TRDBG(0,"SendPacketNow() error => %d\n",ret);
		return	0;
	}

	return	1;
}

static	void	FreeMsgAndPacket(t_imsg *msg,t_lrr_pkt *downpkt)
{
	if	(downpkt && downpkt->lp_payload)
	{
		free(downpkt->lp_payload);
		downpkt->lp_payload	= NULL;
	}
	rtl_imsgFree(msg);
}

static	int	Rx1StopByLbtTryRx2(t_imsg *msg,t_lrr_pkt *downpkt)
{
	if	(!msg || !downpkt)
		return	0;
	if	(Rx2Channel == NULL)
		return	0;

	if	(downpkt->lp_stopbylbt == 0)
	{	// not stopped
		return	0;
	}
	if	(downpkt->lp_delay == 0 || downpkt->lp_beacon 
		|| downpkt->lp_classb || downpkt->lp_classcmc 
		|| downpkt->lp_synchro)
	{	// classC ou beacon ou classB
		return	0;
	}
	if	((downpkt->lp_flag&LP_RADIO_PKT_RX2)==LP_RADIO_PKT_RX2)
	{	// already RX2 (by LRC)
		return	0;
	}
	if	(downpkt->lp_rx2lrr)
	{	// already RX2 (by LRR)
		return	0;
	}

	RTL_TRDBG(1,"PKT SEND RX1 stopped by LBT try RX2\n");

	downpkt->lp_rx2lrr = 1;
	downpkt->lp_delay += 1000;
	AutoRx2Settings(downpkt);
	return	1;
}

// called by radio thread lgw_gen.c:LgwMainLoop()
int	LgwDoSendPacket(time_t now)
{
	t_imsg	*msg;
	int	nbs	= 0;
	int	left;
	int	ret;
	int	board;
	static	int	classbuseall	= -1;
	static  int classbmargin    = -1;

	if	(classbuseall == -1)
	{
		classbuseall	= CfgInt(HtVarLrr,"classb",-1,"useallslot",0);
		RTL_TRDBG(1,"classb.useallslot=%d\n",classbuseall);
	}
	if (classbmargin == -1) {
		classbmargin = CfgInt(HtVarLrr, System, -1, "classbdnmargincpu", 200);
		RTL_TRDBG(1, "classb dnmargincpu=%d\n", classbmargin);
	}

	left = 0;
	for (board=0; board<LgwBoard; board++)
		left	+= rtl_imsgCount(LgwSendQ[board]);

	if	(left > 0 && left > LgwNbPacketWait)
		LgwNbPacketWait	= left;


	for (board=0 ; board < LgwBoard; board++)
	{

		if	((msg= rtl_imsgGet(LgwSendQ[board],IMSG_BOTH)) != NULL)
		{
			t_lrr_pkt	*downpkt;

			downpkt	= msg->im_dataptr;
			// RDTP-5475
			if	(SynchroPeriod && downpkt->lp_beacon && downpkt->lp_delay == 0)
			{
				int		odelay;
				int		delay;	// postpone delay
				struct timespec lutc;

				if	(!LgwSynchroUp())
				{
				RTL_TRDBG(1,
					"PKT SEND beacon dropped syncdrift=%f !in [%f..%f] pkt=(%u,%09u)\n",
					SynchroDrift,SynchroDriftMin,SynchroDriftMax,
					downpkt->lp_gss,downpkt->lp_gns);
					LgwBeaconLastDeliveryCause	= LP_C1_DELAY;
					return	0;
				}

				rtl_timespec(&lutc);
				delay	= LgwPacketDelayMsFromUtc(downpkt,&lutc);
				odelay	= delay;
				delay	= odelay - classbmargin - 100;

				RTL_TRDBG(1,
					"PKT SEND beacon postpone=%d/%d pkt=(%u,%09u) eutc=(%u,%03ums)\n",
					delay,odelay,
					downpkt->lp_gss,downpkt->lp_gns,
					lutc.tv_sec,lutc.tv_nsec/1000000);

				if	(delay <= 0)
				{
					RTL_TRDBG(0,"too old beacon %dms\n",delay);
					LgwBeaconRequestedLateCnt++;
					LgwBeaconLastDeliveryCause	= LP_C1_DELAY;
					return	0;
				}

				downpkt->lp_trip	= 0;
				downpkt->lp_delay	= delay;
				downpkt->lp_lgwdelay	= 1;
				rtl_imsgAddDelayed(LgwSendQ[board],msg,delay);
				return	0;
				}
			// RDTP-5475
			if	(SynchroPeriod && downpkt->lp_beacon && downpkt->lp_delay != 0)
			{
				RTL_TRDBG(1,"PKT SEND beacon retrieved\n");
				goto	send_pkt;
			}
			// RDTP-5475
			if	(SynchroPeriod && downpkt->lp_classcmc && downpkt->lp_delay == 0)
			{
				int		odelay;
				int		delay;	// postpone delay
				struct timespec lutc;

				if	(!LgwSynchroUp())
				{
					RTL_TRDBG(1,
						"PKT SEND classcmc dropped syncdrift=%f !in [%f..%f] pkt=(%u,%09u)\n",
						SynchroDrift,SynchroDriftMin,SynchroDriftMax,
						downpkt->lp_gss,downpkt->lp_gns);
					LgwClassCLastDeliveryCause	= LP_C1_DELAY;
					return	0;
				}

				rtl_timespec(&lutc);

				delay	= LgwPacketDelayMsFromUtc(downpkt,&lutc);
				odelay	= delay;
				delay	= odelay - classbmargin - 100;

				RTL_TRDBG(1,
					"PKT SEND classcmc postpone=%d/%d pkt=(%u,%09u) eutc=(%u,%03ums)\n",
					delay,odelay,
					downpkt->lp_gss,downpkt->lp_gns,
					lutc.tv_sec,lutc.tv_nsec/1000000);

				if	(delay <= 0)
				{
					RTL_TRDBG(0,"too old classcmc %dms\n",delay);
					LgwClassCRequestedLateCnt++;
					LgwClassCLastDeliveryCause	= LP_C1_DELAY;
					return	0;
				}

				downpkt->lp_trip	= 0;
				downpkt->lp_delay	= delay;
				downpkt->lp_lgwdelay	= 1;
				rtl_imsgAddDelayed(LgwSendQ[board],msg,delay);
				return	0;
			}
			// RDTP-5475
			if	(SynchroPeriod && downpkt->lp_classcmc && downpkt->lp_delay != 0)
			{
				RTL_TRDBG(1,"PKT SEND classcmc retrieved\n");
				goto	send_pkt;
			}
			// RDTP-5475
			if	(SynchroPeriod && downpkt->lp_classb && downpkt->lp_delay == 0)
			{
				int		odelay;
				int		delay;	// postpone delay
				struct timespec lutc;

				if	(!LgwSynchroUp())
				{
					RTL_TRDBG(1,
						"PKT SEND classb dropped syncdrift=%f !in [%f..%f] pkt=(%u,%09u)\n",
						SynchroDrift,SynchroDriftMin,SynchroDriftMax,
						downpkt->lp_gss,downpkt->lp_gns);
					SetIndic(downpkt,0,-1,-1,LP_CB_DELAY);
					SendIndicToLrc(downpkt);
					FreeMsgAndPacket(msg,downpkt);
					return	0;
				}
				rtl_timespec(&lutc);
				ret	= LgwNextPingSlot(downpkt,&lutc, classbmargin+100,&delay);
				if	(ret < 0)
				{
					// SetIndic should have be done previously and RDTP-14405 requires
					// the last reason of failure must be returned
					if (!IsIndicSet(downpkt))
					{
						RTL_TRDBG(0,"use cause CB_DELAY because no other cause set previously\n");
						SetIndic(downpkt,0,-1,-1,LP_CB_DELAY);
					}
					SendIndicToLrc(downpkt);
					FreeMsgAndPacket(msg,downpkt);
					RTL_TRDBG(0,"PKT SEND classb too late\n");
					return	0;
				}
				odelay	= delay;
				delay	= odelay - classbmargin;
				if	(0 && CurrTmoaRequested [board]>= odelay)
				{
					downpkt->lp_delay	= 0;
					rtl_imsgAddDelayed(LgwSendQ[board],msg,CurrTmoaRequested[board]+3);
					return	0;
				}

				RTL_TRDBG(1,
					"PKT SEND classb postpone=%d/%d period=%d sidx=%d sdur=%f pkt=(%u,%09u) eutc=(%u,%03ums)\n",
					delay,odelay,
					downpkt->lp_period,downpkt->lp_sidx,downpkt->lp_sdur,
					downpkt->lp_gss,downpkt->lp_gns,
					lutc.tv_sec,lutc.tv_nsec/1000000);

				downpkt->lp_trip	= 0;
				downpkt->lp_delay	= delay;
				downpkt->lp_lgwdelay	= 1;
				rtl_imsgAddDelayed(LgwSendQ[board],msg,delay);


				return	0;
			}
			// RDTP-5475
			if	(SynchroPeriod && downpkt->lp_classb && downpkt->lp_delay != 0)
			{
				RTL_TRDBG(1,"PKT SEND classb retrieved\n");
				goto	send_pkt;
			}
#ifdef	WITH_GPS
			if	(downpkt->lp_beacon && downpkt->lp_delay == 0)
			{
				int		destim;
				int		odelay;
				int		delay;	// postpone delay
				struct timespec *utc;

				// delayed thread/board and use UTC given by GPS+estim
				struct timespec lutc;
				struct timespec butc;
				utc	= &lutc;
				LgwEstimUtc(utc);
				destim	= LgwEstimUtcBoard(board, &butc);
				if	(destim != LGW_HAL_SUCCESS)
				{
					RTL_TRDBG(0,"can not estim UTC board for beacon\n");
					LgwBeaconLastDeliveryCause	= LP_C1_DELAY;
					return	0;
				}
				destim	= LgwDiffMsUtc(&butc,&lutc);
				delay	= LgwPacketDelayMsFromUtc(downpkt,&butc);
				odelay	= delay;
				delay	= odelay - classbmargin - 100;
				

				RTL_TRDBG(1,
					"PKT SEND beacon postpone=%d/%d pkt=(%u,%09u) eutc=(%u,%03ums) butc=(%u,%09u) diff=%d\n",
					delay,odelay,
					downpkt->lp_gss,downpkt->lp_gns,
					utc->tv_sec,utc->tv_nsec/1000000,
					butc.tv_sec,butc.tv_nsec,destim);

				if	(delay <= 0)
				{
					RTL_TRDBG(0,"too old beacon %dms\n",delay);
					LgwBeaconRequestedLateCnt++;
					LgwBeaconLastDeliveryCause	= LP_C1_DELAY;
					return	0;
				}

				downpkt->lp_trip	= 0;
				downpkt->lp_delay	= delay;
				downpkt->lp_lgwdelay	= 1;
				rtl_imsgAddDelayed(LgwSendQ[board],msg,delay);
				return	0;
			}
			if	(downpkt->lp_beacon && downpkt->lp_delay != 0)
			{
				RTL_TRDBG(1,"PKT SEND beacon retrieved\n");
				goto	send_pkt;
			}
			if	(downpkt->lp_classcmc && downpkt->lp_delay == 0)
			{
				int		destim;
				int		odelay;
				int		delay;	// postpone delay
				struct timespec *utc;

				// delayed thread/board and use UTC given by GPS+estim
				struct timespec lutc;
				struct timespec butc;
				utc	= &lutc;
				LgwEstimUtc(utc);
				destim	= LgwEstimUtcBoard(board, &butc);
				if	(destim != LGW_HAL_SUCCESS)
				{
					RTL_TRDBG(0,"can not estim UTC board for classcmc\n");
					LgwClassCLastDeliveryCause	= LP_C1_DELAY;
					return	0;
				}
				destim	= LgwDiffMsUtc(&butc,&lutc);
				delay	= LgwPacketDelayMsFromUtc(downpkt,&butc);
				odelay	= delay;
				delay	= odelay - classbmargin - 100;
				

				RTL_TRDBG(1,
					"PKT SEND classcmc postpone=%d/%d pkt=(%u,%09u) eutc=(%u,%03ums) butc=(%u,%09u) diff=%d\n",
					delay,odelay,
					downpkt->lp_gss,downpkt->lp_gns,
					utc->tv_sec,utc->tv_nsec/1000000,
					butc.tv_sec,butc.tv_nsec,destim);

				if	(delay <= 0)
				{
					RTL_TRDBG(0,"too old classcmc %dms\n",delay);
					LgwClassCRequestedLateCnt++;
					LgwClassCLastDeliveryCause	= LP_C1_DELAY;
					return	0;
				}

				downpkt->lp_trip	= 0;
				downpkt->lp_delay	= delay;
				downpkt->lp_lgwdelay	= 1;
				rtl_imsgAddDelayed(LgwSendQ[board],msg,delay);
				return	0;
			}
#endif
			if	(downpkt->lp_classcmc && downpkt->lp_delay != 0)
			{
				RTL_TRDBG(1,"PKT SEND classcmc retrieved\n");
				goto	send_pkt;
			}
#ifdef	WITH_GPS
			if	(downpkt->lp_classb && downpkt->lp_delay == 0)
			{
				int	odelay;
				int	delay;
				int	ret;
				struct timespec eutc;
				struct timespec *utc;

				utc	= &eutc;
				LgwEstimUtc(utc);
				ret	= LgwNextPingSlot(downpkt,utc,300,&delay);
				if	(ret < 0)
				{
					// SetIndic should have be done previously and RDTP-14405 requires
					// the last reason of failure must be returned
					if (!IsIndicSet(downpkt))
					{
						RTL_TRDBG(0,"use cause CB_DELAY because no other cause set previously\n");
						SetIndic(downpkt,0,-1,-1,LP_CB_DELAY);
					}
					SendIndicToLrc(downpkt);
					FreeMsgAndPacket(msg,downpkt);
					RTL_TRDBG(0,"PKT SEND classb too late\n");
					return	0;
				}
				odelay	= delay;
				delay	= odelay - classbmargin;
				if	(0 && CurrTmoaRequested[board] >= odelay)
				{
					downpkt->lp_delay	= 0;
					rtl_imsgAddDelayed(LgwSendQ[board],msg,CurrTmoaRequested[board]+3);
					return	0;
				}

				RTL_TRDBG(1,
					"PKT SEND classb postpone=%d/%d period=%d sidx=%d sdur=%f pkt=(%u,%09u) eutc=(%u,%03ums)\n",
					delay,odelay,
					downpkt->lp_period,downpkt->lp_sidx,downpkt->lp_sdur,
					downpkt->lp_gss,downpkt->lp_gns,
					utc->tv_sec,utc->tv_nsec/1000000);

				downpkt->lp_trip	= 0;
				downpkt->lp_delay	= delay;
				downpkt->lp_lgwdelay	= 1;
				rtl_imsgAddDelayed(LgwSendQ[board],msg,delay);
				return	0;
			}
#endif
			if	(downpkt->lp_classb && downpkt->lp_delay != 0)
			{
				RTL_TRDBG(1,"PKT SEND classb retrieved\n");
				if	(CurrTmoaRequested[board])
				{
					RTL_TRDBG(1,"PKT SEND classb avoid collision repostpone=%d\n",
							CurrTmoaRequested[board]);
					SetIndic(downpkt,0,-1,-1,LP_CB_BUSY);	// cause will be used if no more pingslot available
					downpkt->lp_delay	= 0;
					rtl_imsgAddDelayed(LgwSendQ[board],msg,CurrTmoaRequested[board]+3);
					return	0;
				}
				goto	send_pkt;
			}

			if	(downpkt->lp_delay == 0
				&& ABS(now - downpkt->lp_tms) > MaxReportDnImmediat)
			{	// mode immediate
				RTL_TRDBG(1,"PKT SEND NODELAY not sent after 60s => dropped\n");
					SetIndic(downpkt,0,LP_C1_MAXTRY,LP_C2_MAXTRY,-1);
					SendIndicToLrc(downpkt);
					FreeMsgAndPacket(msg,downpkt);
					return	0;
			}
			if	(downpkt->lp_delay == 0 && CurrTmoaRequested[board])
			{	// mode immediate
				RTL_TRDBG(1,"PKT SEND NODELAY avoid collision repostpone=%d\n",
							CurrTmoaRequested[board]);
				rtl_imsgAddDelayed(LgwSendQ[board],msg,CurrTmoaRequested[board]+3);
				return	0;
			}
	send_pkt:
			if	(classbuseall && downpkt->lp_classb 
							&& downpkt->lp_nbslot <= 16)
			{
				t_lrr_pkt	pkt;
				t_imsg		*repeat;

				memcpy(&pkt,downpkt,sizeof(t_lrr_pkt));
				pkt.lp_payload = (u_char *)malloc(pkt.lp_size);
				memcpy(pkt.lp_payload,downpkt->lp_payload,pkt.lp_size);
				pkt.lp_delay	= 0;
				repeat	= rtl_imsgAlloc(IM_DEF,IM_LGW_SEND_DATA,NULL,0);
				rtl_imsgDupData(repeat,&pkt,sizeof(t_lrr_pkt));
				rtl_imsgAddDelayed(LgwSendQ[board],repeat,1000);
			}
			ret	= SendPacket(msg);
			if	(ret > 0)
			{
				SetIndic(downpkt,1,-1,-1,-1);
				nbs	+= ret;
			}
			else
			{
				RTL_TRDBG(1,"SendPacket() beacon=%d classb=%d classc=%d classcmc=%d rx2=%d lbt=%d error => %d\n",
					downpkt->lp_beacon,downpkt->lp_classb,
					downpkt->lp_delay == 0 ? 1 : 0,
					downpkt->lp_classcmc,
					downpkt->lp_rx2lrr,
					downpkt->lp_stopbylbt,ret);

				if (downpkt->lp_beacon || downpkt->lp_classcmc)
					goto	set_failure;

				if (LgwLbtEnable && downpkt->lp_delay == 0 
							&& downpkt->lp_stopbylbt)
				{	// retry packet immediate if stopped by LBT
					downpkt->lp_stopbylbt	= 0;
					rtl_imsgAddDelayed(LgwSendQ[board],msg,3000);
					return	0;
				}
				if (LgwLbtEnable && downpkt->lp_stopbylbt 
						&& Rx1StopByLbtTryRx2(msg,downpkt))
				{	// retry classA/RX1 on RX2 if stopped by LBT
					downpkt->lp_stopbylbt	= 0;
					rtl_imsgAddDelayed(LgwSendQ[board],msg,950);
					return	0;
				}
	set_failure:
				SetIndic(downpkt,0,-1,-1,-1);
			}
			SendIndicToLrc(downpkt);

			FreeMsgAndPacket(msg,downpkt);
		}
	}
	return	nbs;
}
