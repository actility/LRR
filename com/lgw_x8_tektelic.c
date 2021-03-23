
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

#include "rtlbase.h"
#include "rtlimsg.h"
#include "rtllist.h"
#include "rtlhtbl.h"

#include "timeoper.h"

#undef	WITH_SX1301_X1		// !!!
#ifndef WITH_SX1301_X8
#define	WITH_SX1301_X8		// !!!
#endif
#include "semtech.h"
#include "sx1301ar_lbs.h"
#include "headerloramac.h"

#include "xlap.h"
#include "define.h"
#include "infrastruct.h"
#include "struct.h"
#include "cproto.h"
#include "extern.h"

// PT-1197: difference between
// . 1-Jan-1970 (UTC starting date on Unix)
// . 6-Jan-1980 (GPS reference)
// GPS_TO_UTC: value to add to GPS seconds to convert them into UTC seconds
#define GPS_TO_UTC_SEC	315964800


#ifdef WITH_GPS
sx1301ar_tref_t Gps_time_ref[SX1301AR_MAX_BOARD_NB];
int Gps_ref_valid; 
int		DriftComp = -1;		// xtal drift compensation activation
time_t		LgwTmmsUtcTime;		// tmms mono of the last UTC/GPS
#endif /* WITH_GPS */
struct timespec	LgwCurrUtcTime;		// last UTC + nsec computation
struct timespec	LgwBeaconUtcTime;	// last beacon
struct timespec	LgwClassCUtcTime;	// last ClassC

static	int	_LgwStarted;
static	int	LgwAntennas[SX1301AR_MAX_BOARD_NB][SX1301AR_BOARD_CHIPS_NB];	// Antenna id connected to each chip
#ifdef	TEST_ONLY
static	int	LgwWaitSending	= 1;
#else
static	int	LgwWaitSending	= 0;
#endif
time_t	RadioStartTime	= 0;	// radio started timestamp

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
	return	"SX1301_AR";
}

// First version: if at least one chip is not free the status will
// be 'not free'
int IsBoardStatusTxFree(int board,sx1301ar_tstat_t *txstatus)
{
	int	chip;

	for (chip=0; chip<num_sx1301_chips; chip++)
	{
		sx1301ar_tx_status(board,chip,txstatus);
		if (*txstatus != TX_FREE)
		{
			RTL_TRDBG(3,"IsBoardStatusTxFree: chip%d status not TX_FREE = %d\n",
				chip, *txstatus);
			return 0;
		}
	}
	RTL_TRDBG(4,"IsBoardStatusTxFree: status = TX_FREE\n");
	return 1;
}

int	LgwTxFree(uint8_t board,uint8_t *txstatus)
{
	sx1301ar_tstat_t	status;

	*txstatus	= 0;

	IsBoardStatusTxFree(board, &status);
	*txstatus	= status;
	if	(*txstatus != TX_FREE)
		return	0;
	return	1;
}

int	LgwEstimUtcBoard(uint8_t board, struct timespec *utc)
{
	int		ret;
	uint32_t	cnt;

	ret = sx1301ar_get_instcnt(0, 0, &cnt);
	if (ret <= LGW_HAL_ERROR) {
		RTL_TRDBG(0, "sx1301ar_get_instcnt error '%s'\n", sx1301ar_err_message(sx1301ar_errno));
		return	LGW_HAL_ERROR;
	}
	RTL_TRDBG(3, "  return instcnt()=%d get_instcnt(brd=0, chip=0)=%09u\n", ret, cnt);
	memset(utc, 0, sizeof(struct timespec));
#ifdef WITH_GPS
	ret = sx1301ar_cnt2gps(cnt, utc);
	if (ret <= LGW_HAL_ERROR) {
		RTL_TRDBG(0, "sx1301ar_cnt2gps error '%s'\n", sx1301ar_err_message(sx1301ar_errno));
		return	LGW_HAL_ERROR;
	}
	RTL_TRDBG(3, "  return cnt2gps()=%d count_us=%09u => gps_time(%u,%09u)\n",
		ret, cnt, utc->tv_sec, utc->tv_nsec);

	// GPS time -> UTC time
	// PT-1197: fix GPS-UTC reference issue in addition leap seconds
	utc->tv_sec -= GpsUtcLeap;
	utc->tv_sec += GPS_TO_UTC_SEC;
#endif

	return	LGW_HAL_SUCCESS;
}

char	*PktStatusTxt(sx1301ar_pstat_t status)
{
	static	char	buf[64];

	switch	(status)
	{
	case	STAT_UNDEFINED :
		return	"UNDEFINED";
	case	STAT_NO_CRC :
		return	"CRCNO";
	case	STAT_CRC_ERR :
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

#if 0
static	unsigned int	LgwTimeStamp()
{
	return	0;
}
#endif

int	LgwLINKUP()
{
	return	1;
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
			if	((c->datarate & up->modrate) == up->modrate)
				return	c;
		}
	}

not_found :
RTL_TRDBG(0,"no chan(%d) found for frz=%d mod=0x%02x bdw=0x%02x dtr=0x%02x\n",
	sortindex,up->freq_hz,up->modulation,up->bandwidth,up->modrate);

	return	NULL;
}


static	void LgwDumpRfConf(FILE *f, int board, int rfi, sx1301ar_chip_cfg_t *rfconf)
{
	if	(rfconf->enable == 0)
		return;
	RTL_TRDBG(1,"chip%d enab=%d frhz=%d rfchain=%d antennaid=%d\n",rfi,rfconf->enable,
		rfconf->freq_hz, rfconf->rf_chain, LgwAntennas[board][rfi]);

	if	(f == NULL)
		return;
	fprintf(stderr,"chip%d enab=%d frhz=%d rfchain=%d antennaid=%d\n",rfi,rfconf->enable,
		rfconf->freq_hz, rfconf->rf_chain, LgwAntennas[board][rfi]);
}

static	void LgwDumpIfConf(FILE *f, int ifi, sx1301ar_chan_cfg_t *ifconf, int chan)
{
	if	(ifconf->enable == 0)
		return;

	RTL_TRDBG(1,"phychan%d enab=%d frhz=%d halchan=%d(%d:%d) bandw=0x%x datar=0x%x\n",
		ifi,ifconf->enable,
		ifconf->freq_hz,
		chan,chan>>4,chan&0x0F,
		ifconf->bandwidth,ifconf->modrate);

	if	(f == NULL)
		return;
	fprintf(f,"phychan%d enab=%d frhz=%d halchan=%d(%d:%d) bandw=0x%x datar=0x%x\n",
		ifi,ifconf->enable,
		ifconf->freq_hz,
		chan,chan>>4,chan&0x0F,
		ifconf->bandwidth,ifconf->modrate);
	fflush(f);
}

static	void LgwDumpBdConf(FILE *f, int board, sx1301ar_board_cfg_t *bdconf)
{
	int	rfc, i;
	char	key[80], tmp[20];

	key[0] = '\0';
	for (i=0; i<SX1301AR_BOARD_AES_KEY_SIZE/8; i++)
	{
		sprintf(tmp, "%02X", bdconf->aes_key[i]);
		strcat(key, tmp);
	}

#if defined(TEK_MICRO8)
	RTL_TRDBG(1,"board%d rxfrhz=%d public=%d roomtemp=%d aeskey='%s'\n",
		board, bdconf->rx_freq_hz, bdconf->loramac_public, bdconf->room_temp_ref,
		key);
#else
	RTL_TRDBG(1,"board%d rxfrhz=%d public=%d roomtemp=%d ad9361temp=%d aeskey='%s'\n",
		board, bdconf->rx_freq_hz, bdconf->loramac_public, bdconf->room_temp_ref,
		bdconf->ad9361_temp_ref, key);
#endif

	if	(f)
	{
#if defined(TEK_MICRO8)
		fprintf(f,"board%d rxfrhz=%d public=%d roomtemp=%d aeskey='%s'\n",
			board, bdconf->rx_freq_hz, bdconf->loramac_public, bdconf->room_temp_ref,
			key);
#else
		fprintf(f,"board%d rxfrhz=%d public=%d roomtemp=%d ad9361temp=%d aeskey='%s'\n",
			board, bdconf->rx_freq_hz, bdconf->loramac_public, bdconf->room_temp_ref,
			bdconf->ad9361_temp_ref, key);
#endif
	}
#if !defined(TEK_MICRO8)
	for	(rfc = 0 ; rfc < SX1301AR_BOARD_RFCHAIN_NB ; rfc++)
	{
		RTL_TRDBG(1,"  rfchain[%d]: rxenable=%d txenable=%d rssi=%f rssioffsetcoeffa=%d rssioffsetcoeffb=%d\n",
			rfc, bdconf->rf_chain[rfc].rx_enable, bdconf->rf_chain[rfc].tx_enable,
			bdconf->rf_chain[rfc].rssi_offset, bdconf->rf_chain[rfc].rssi_offset_coeff_a,
			bdconf->rf_chain[rfc].rssi_offset_coeff_b);
		if (f)
		{
			fprintf(f,"  rfchain[%d]: rxenable=%d txenable=%d rssi=%f rssioffsetcoeffa=%d rssioffsetcoeffb=%d\n",
				rfc, bdconf->rf_chain[rfc].rx_enable, bdconf->rf_chain[rfc].tx_enable,
				bdconf->rf_chain[rfc].rssi_offset, bdconf->rf_chain[rfc].rssi_offset_coeff_a,
				bdconf->rf_chain[rfc].rssi_offset_coeff_b);
		}
	}
#endif
}

#define LINUXDEV_PATH_DEFAULT   "/dev/spidev0.0"

sx1301ar_bband_t	LgwGetFreqBand()
{
	if (!IsmBand || !*IsmBand)
	{
		RTL_TRDBG(1,"IsmBand empty, use freq band UNKNOWN\n");
		return BRD_FREQ_BAND_UNKNOWN;
	}

	if (strstr(IsmBand, "868"))
	{
		RTL_TRDBG(1,"freq band used for '%s' is EU868\n", IsmBand);
		return BRD_FREQ_BAND_EU868;
	}
		
	RTL_TRDBG(1,"freq band used for '%s' is US915\n", IsmBand);
	return BRD_FREQ_BAND_US915;
}

int	LgwConfigure(int hot,int config)
{
	unsigned long long ull = 0, ull2 = 0;
	sx1301ar_board_cfg_t bdconf;
	sx1301ar_chip_cfg_t rfconf;
	sx1301ar_chan_cfg_t ifconf;
	int	board;
	int	rfi;
	int	ifi;
	int	ret;
	int	i;
	char	def[40];
	char	section[64];
	char	*key;
	char	file[PATH_MAX];
	FILE	*f	= NULL;

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

	for	(board = 0 ; board < LgwBoard && board < SX1301AR_MAX_BOARD_NB ; board++)
	{
		ull = 0;
		ull2 = 0;
		sprintf	(section,"board:%d",board);
		bdconf = sx1301ar_init_board_cfg();
		bdconf.freq_band	= LgwGetFreqBand();
		bdconf.rx_freq_hz	= CfgInt(HtVarLgw,section,-1,"freqhz",0);
		bdconf.loramac_public	= CfgInt(HtVarLgw,section,-1,"public",1);
#ifdef	WITH_LBT
		if (LgwLbtEnable)
			bdconf.lbt.lbt_enabled = 1;
		else
			bdconf.lbt.lbt_enabled = 0;
		bdconf.lbt.size = 16; /*!> Number of LUT indexes */
		RTL_TRDBG(0,"LBT Configuration : enabled=%d, noise threshold=%d, listen_time= %d, number of lut indexes=%d'\n",bdconf.lbt.lbt_enabled,
		LgwLbtRssiOffset, LgwLbtScantime, bdconf.lbt.size);
		int lbt_time_interval_lut;
		for (lbt_time_interval_lut = 0 ; lbt_time_interval_lut < LBT_TIME_INTERVAL_LUT_SIZE_MAX ; ++lbt_time_interval_lut)
		{
			bdconf.lbt.lut[lbt_time_interval_lut].lbt_threshold = LgwLbtRssiOffset; /* in dB, default -85 - found in lgw.ini [lbt].rssioffset */
			if ( LgwLbtScantime == 5000)
			{
				bdconf.lbt.lut[lbt_time_interval_lut].talk_time_ms = 4000;
				bdconf.lbt.lut[lbt_time_interval_lut].listen_time_us = 5000;
			}
			else if ( LgwLbtScantime == 128)
			{
				bdconf.lbt.lut[lbt_time_interval_lut].talk_time_ms = 400;
				bdconf.lbt.lut[lbt_time_interval_lut].listen_time_us = 128;
			}
			else
			{
				RTL_TRDBG(0,"LBT Configuration : No Carrier Sensing Time configured'\n");
				bdconf.lbt.lbt_enabled = 0;  /* LBT enable flag, overriden if LBT is REQUIRED */
			}
		}
#endif /* WITH_LBT */
		// use HtVarLrr instead of HtVarLgw because aeskey should be
		// set in custom.ini file
		key = CfgStr(HtVarLrr,section,-1,"aeskey",NULL);
		if (key && *key)
		{
			i = sscanf(key, "%16llx%16llx", &ull, &ull2);
			if ( i != 2 )
			{
				RTL_TRDBG(0, "ERROR: failed to parse hex string for aeskey\n");
			}
			else
			{
				RTL_TRDBG(3, "aeskey is configured to %016llX%016llX\n", ull, ull2 );
			}
		}
#if !defined(TEK_MICRO8)
		bdconf.room_temp_ref	= CfgInt(HtVarLgw,section,-1,"roomtemp",SX1301AR_DEFAULT_ROOM_TEMP_REF);
		bdconf.ad9361_temp_ref	= CfgInt(HtVarLgw,section,-1,"ad9361temp",SX1301AR_DEFAULT_AD9361_TEMP_REF);
#else
		bdconf.room_temp_ref	= CfgInt(HtVarLgw,section,-1,"roomtemp",22);
#endif
            	if (ull)
		{
			for (i=0; i<8; i++)
			{
				bdconf.aes_key[i]   = (uint8_t)((ull  >> (56 - i*8)) & 0xFF);
				bdconf.aes_key[i+8] = (uint8_t)((ull2 >> (56 - i*8)) & 0xFF);
			}
		}
		sprintf(def, "%f", SX1301AR_DEFAULT_RSSI_OFFSET);
#if !defined(TEK_MICRO8)
        int rfc; //rfchain
	    for	(rfc = 0 ; rfc < SX1301AR_BOARD_RFCHAIN_NB ; rfc++)

		{
			sprintf	(section,"rfconf:%d:%d",board,rfc);
			bdconf.rf_chain[rfc].rssi_offset	= atof(CfgStr(HtVarLgw,section,-1,"rssioffset",def));
			bdconf.rf_chain[rfc].rssi_offset_coeff_a = CfgInt(HtVarLgw,section,-1,"rssioffsetcoeffa",0);
			bdconf.rf_chain[rfc].rssi_offset_coeff_b = CfgInt(HtVarLgw,section,-1,"rssioffsetcoeffb",0);
			bdconf.rf_chain[rfc].rx_enable	= CfgInt(HtVarLgw,section,-1,"rxenable",1);
			bdconf.rf_chain[rfc].tx_enable	= CfgInt(HtVarLgw,section,-1,"txenable",1);
		}
#endif
		LgwDumpBdConf(f,board,&bdconf);
		if	(config)
		{
			ret	= sx1301ar_conf_board(board,&bdconf);
			if	(ret <= LGW_HAL_ERROR)
			{
				RTL_TRDBG(0,"BD%d cannot be configured ret=%d '%s'\n",
					board, ret, sx1301ar_err_message(sx1301ar_errno));
				if	(f)
				{
					fprintf(f,"BD%d cannot be configured ret=%d '%s'\n",
						board, ret, sx1301ar_err_message(sx1301ar_errno));
					fclose(f);
				}
				return	-1;
			}
		}

		for	(rfi = 0 ; rfi < LgwChipsPerBoard; rfi++)
		{
			sprintf	(section,"chip:%d:%d",board,rfi);
			rfconf = sx1301ar_init_chip_cfg();
			rfconf.enable	= CfgInt(HtVarLgw,section,-1,"enable",0);
			rfconf.freq_hz	= CfgInt(HtVarLgw,section,-1,"freqhz",0);
			rfconf.rf_chain = CfgInt(HtVarLgw,section,-1,"rfchain",0);
			LgwAntennas[board][rfi] = CfgInt(HtVarLgw,section,-1,"antid",0);
			if	(rfconf.enable == 0 || rfconf.freq_hz == 0)	
				continue;
			LgwDumpRfConf(f,board,rfi,&rfconf);
			if	(config)
			{
				ret	= sx1301ar_conf_chip(board,rfi,&rfconf);
				if	(ret <= LGW_HAL_ERROR)
				{
					RTL_TRDBG(0,"RF%d cannot be configured ret=%d '%s'\n",
									rfi,ret, sx1301ar_err_message(sx1301ar_errno));
					if	(f)
					{
						fprintf(f,"RF%d cannot be configured ret=%d '%s'\n",
									rfi,ret, sx1301ar_err_message(sx1301ar_errno));
						fclose(f);
					}
					return	-1;
				}
			}
		}

		for	(rfi = 0 ; rfi < LgwChipsPerBoard; rfi++)
		{
			sprintf	(section,"chip:%d:%d",board,rfi);
			rfconf.enable	= CfgInt(HtVarLgw,section,-1,"enable",0);
			rfconf.freq_hz	= CfgInt(HtVarLgw,section,-1,"freqhz",0);
			if	(rfconf.enable == 0 || rfconf.freq_hz == 0)	
				break;

			// SX1301AR_CHIP_MULTI_NB+1 in order to configure the "stand-alone" channel
			for	(ifi = 0 ; ifi < SX1301AR_CHIP_MULTI_NB+2 ; ifi++)
			{
				uint8_t	chan	= 0;

				sprintf	(section,"ifconf:%d:%d:%d",board,rfi,ifi);
				memset(&ifconf,0,sizeof(ifconf));
				ifconf = sx1301ar_init_chan_cfg();

				// for compatibility with existing configurations, if "stand-alone" channel
				// is not configured just ignore it
				if (rfi >= SX1301AR_CHIP_MULTI_NB && CfgStr(HtVarLgw,section,-1,"enable",NULL) == NULL)
					continue;

				if (rfi != 0 && CfgStr(HtVarLgw,section,-1,"enable",NULL) == NULL)
				{
					// Print this trace only once
					if (ifi == 0)
						RTL_TRDBG(1,"No ifconf for chip %d, use the same as board 0 chip 0\n", rfi);
					sprintf	(section,"ifconf:0:0:%d",ifi);
				}

				ifconf.enable	= CfgInt(HtVarLgw,section,-1,"enable",0);
				ifconf.freq_hz	= CfgInt(HtVarLgw,section,-1,"freqhz",0);
				if	(ifi != SX1301AR_CHIP_MULTI_NB+1 && (ifconf.enable == 0 || ifconf.freq_hz == 0))
					continue;
				ifconf.bandwidth= CfgInt(HtVarLgw,section,-1,"bandwidth",0);
				if	(ifconf.bandwidth == 0)
					ifconf.bandwidth	= BW_125KHZ;
				ifconf.modrate	= CfgInt(HtVarLgw,section,-1,"datarate",0);
				if	(ifconf.modrate == 0)
					ifconf.modrate		= DR_LORA_MULTI;
				else
					ifconf.modrate		= DecodeSpreadingFactor(ifconf.modrate);

				// Tektelic requires that all channels must be configured,
				// even FSK channel
				if (ifi == SX1301AR_CHIP_MULTI_NB+1)
				{
					// use same freq as for stand-alone chan
					sprintf	(section,"ifconf:%d:%d:%d",board,rfi,ifi-1);
					ifconf.freq_hz	= CfgInt(HtVarLgw,section,-1,"freqhz",0);
					ifconf.bandwidth	= BW_125KHZ;
					ifconf.modrate		= MR_10000;
					ifconf.enable		= CfgInt(HtVarLgw,section,-1,"enable",0);
				}

				chan	= ((uint8_t)rfi<<4)+(uint8_t)ifi;

				LgwDumpIfConf(f,ifi,&ifconf,chan);
				if	(config)
				{
					ret	= sx1301ar_conf_chan(board,chan,&ifconf);
					if	(ret <= LGW_HAL_ERROR)
					{
						RTL_TRDBG(0,"IF%d cannot be configured ret=%d '%s'\n",
										ifi,ret, sx1301ar_err_message(sx1301ar_errno));
						if	(f)
						{
							fprintf(f,"IF%d cannot be configured ret=%d '%s'\n",
										ifi,ret, sx1301ar_err_message(sx1301ar_errno));
							fclose(f);
						}
						return	-1;
					}
				}
			}
		}
	}
#if !defined(TEK_MICRO8)
	RTL_TRDBG(1,"Enable geolocation timestamp\n");
	sx1301ar_loc_config();
	sx1301ar_loc_enable(1);
#endif

	if	(config)
	{
		RTL_TRDBG(1,"RADIO configured\n");
	}

	if	(f)
	{
		fprintf(f,"RADIO configured\n");
		fclose(f);
	}

#ifdef WITH_GPS
	// RDTP-15343: freq compensation for xtal drift, for beacons and multiC
	if	(DriftComp == -1)
	{
		DriftComp = CfgInt(HtVarLrr,"lrr",-1,"driftcompensation",0);
		RTL_TRDBG(1,"lrr.driftcompensation=%d\n", DriftComp);
	}
#endif
	return	0;
}

int	LgwStart()
{
	int	ret, b;
	FILE	*chk=NULL;
	char	file[PATH_MAX];

	sprintf	(file,"%s/var/log/lrr/radioparams.txt",RootAct);
	chk	= fopen(file,"a");
	for (b=0; b<LgwBoard; b++)
	{
		RTL_TRDBG(1,"RADIO starting (board %d) ...\n", b);
		ret	= sx1301ar_start(b);
		if	(ret <= LGW_HAL_ERROR)
		{
			RTL_TRDBG(0,"BOARD%d RADIO cannot be started ret=%d '%s'\n",
				b, ret, sx1301ar_err_message(sx1301ar_errno));
			if	(chk)
			{
				fprintf(chk,"BOARD%d RADIO cannot be started ret=%d '%s'\n",
					b, ret, sx1301ar_err_message(sx1301ar_errno));
				fclose(chk);
			}
			return	-1;
		}

		RadioStartTime = rtl_timemono(NULL);
		RTL_TRDBG(1,"RADIO board %d started ret=%d\n", b, ret);
		if	(chk)
			fprintf(chk,"RADIO board %d started ret=%d\n",b,ret);
	}
	if (chk)
		fclose(chk);
	_LgwStarted	= 1;
	return	0;
}

int	LgwStarted()
{
	return	_LgwStarted;
}

void	LgwStop()
{
	int	b;
	int	ret = -1;
	int	err = 0;

	RTL_TRDBG(1,"RADIO stopping ...\n");
	for (b=0; b<LgwBoard; b++)
	{
		ret = sx1301ar_stop(b);
		if (ret != 0)
		{
			RTL_TRDBG(0, "ERROR: Board %d error stopping radio ret=%d '%s'\n",
				b, ret, sx1301ar_err_message(sx1301ar_errno));
			err = 1;
		}
		// TODO: close all spi devices
	}
	if (!err)
	{
		RTL_TRDBG(1,"RADIO stopped\n");
		_LgwStarted	= 0;
		RadioStartTime = 0;
	}
}

int LgwDlReady()
{
	time_t diff;

	if	(RadioStartTime)
	{
		diff = rtl_timemono(NULL) - RadioStartTime;
		if (diff > 180)
			return 1;
		RTL_TRDBG(1,"RADIO startup uptime = %d s\n", diff);
	}
	else
		RTL_TRDBG(1,"RADIO startup uptime = 0 s\n");
	return 0;
}

#ifdef WITH_GPS
void	LgwGpsTimeUpdated(struct timespec *utc_from_gps, struct timespec * ubxtime_from_gps)
{
	time_t	now;
	now = rtl_tmmsmono();

	/* Time sync is handled internally by Tektelic HAL */

	LgwTmmsUtcTime = now;
}
#endif /* WITH_GPS */


int FindChipFromFreq(uint32_t freq, int rfchain, int board)
{
	int		chip, chan, r;
	uint32_t	f;
	char		section[64];

	for (chip=0; chip<SX1301AR_BOARD_CHIPS_NB; chip++)
	{
		for (chan=0; chan<SX1301AR_CHIP_MULTI_NB+1; chan++)
		{
			sprintf	(section,"ifconf:%d:%d:%d",board,chip,chan);
			f = CfgInt(HtVarLgw,section,-1,"freqhz",0);
			if (f == freq)
			{
				// check the rfchain, in diversity mode the same freq
				// could be configured on 2 different chips
				sprintf	(section,"chip:%d:%d",board,chip);
				r = CfgInt(HtVarLgw,section,-1,"rfchain",0);
				if (r == rfchain)
					return chip;
			}
		}
	}
	return -1;
}

static	int	ProceedRecvPacket(T_lgw_pkt_rx_t    *p,time_t tms, int board)
{
	t_imsg		*msg;
	t_lrr_pkt	uppkt;
	int		sz, i;
	u_char		*data;
	t_channel	*chan;
	struct		timespec	tv;

	u_int		numchan	= -1;
	char		*namchan= "?";
	u_int		numband	= -1;
	int		ret;
	uint32_t	curtus;
	int		delayUplinkRecv=0;
	int		chipid;

#if WITH_GPS
	struct timespec fetch_time;
	struct tm * fetch_tm;
#endif

	char gpstm[SX1301AR_BOARD_CHIPS_NB][50];

	clock_gettime(CLOCK_REALTIME,&tv);

	chan	= FindChannelForPacket(p);
	if	(chan)
	{
		numchan	= chan->channel;
		namchan	= (char *)chan->name;
		numband	= chan->subband;
	}

RTL_TRDBG(1,"PKT RECV board%d tms=%09u tus=%09u status=%s sz=%d freq=%d mod=0x%02x bdw=%s spf=%s ecc=%s channel=%d nam='%s' G%d modul=%s\n",
		board, tms,p->count_us,PktStatusTxt(p->status),p->size,
		p->freq_hz, p->modulation,BandWidthTxt(p->bandwidth),
		SpreadingFactorTxt(p->modrate),CorrectingCodeTxt(p->coderate),
		numchan,namchan,numband, p->modulation == MOD_LORA ? "lora" : "not lora");

	delayUplinkRecv = 0;
#if !defined(TEK_MICRO8)
	for (i=0; i<SX1301AR_BOARD_RFCHAIN_NB; i++)
#else
	for (i=0; i<num_sx1301_chips; i++)
#endif
	{
		gpstm[i][0] = '\0';
#ifdef WITH_GPS
		if (UseGpsTime && GpsStatus == 'U')
		{
			sync_status_t sync_stat = SS_OUT_OF_SYNC;
			if ( 0 == sx1301ar_get_sync_status(&sync_stat) && sync_stat == SS_SYNCED )
			{
				if (sx1301ar_cnt2utc(Gps_time_ref[board], p->count_us, &fetch_time) == 0)
				{
					fetch_tm = gmtime( &(fetch_time.tv_sec) );
					sprintf(gpstm[i], "gpstime=%02i:%02i:%02i.%03li ", fetch_tm->tm_hour, fetch_tm->tm_min, fetch_tm->tm_sec, (fetch_time.tv_nsec)/1000000 );
				}
			}
			else
			{
				RTL_TRDBG(1, "GPS is out-of-sync or sync status can not be retrieved.\n");
			}
		}
#endif /* WITH_GPS */

		chipid = 0;
		if (p->rsig[i].is_valid)
		{
			// chipid = FindChipFromFreq(p->freq_hz, i, board);
			// cnt is board specific, not chip specific. sx1301ar_get_instcnt always returns -1 if chip != 0 => Force 0
                        chipid = 0;
			ret = sx1301ar_get_instcnt(board, chipid, &curtus);
			if (ret == 0 && p->status == STAT_CRC_OK)
			{
				// calculate delay between the time the packet was really received
				// and now, the time we are informed of this packet
				delayUplinkRecv = (ABS(curtus-p->count_us))/1000;
				if (delayUplinkRecv > 500)
					delayUplinkRecv = 500;
				RTL_TRDBG(3,"PKT RECV delayuplinkrecv=%d (curtus=%09u diff=%d)\n",
					delayUplinkRecv, curtus, curtus-p->count_us);
			}
		}
RTL_TRDBG(1,"    rf%d: valid=%d finetime=%d finetimestamp=%09u %schip=%d halchan=%d snr=%f rssi_chan=%f rssi_sig=%f rssi_sig_std=%d antennaid=%d delay=%d\n",
		i, p->rsig[i].is_valid, p->rsig[i].fine_received, p->rsig[i].fine_tmst, gpstm[i],
		chipid, p->rsig[i].chan, p->rsig[i].snr, p->rsig[i].rssi_chan,
		p->rsig[i].rssi_sig, p->rsig[i].rssi_sig_std, LgwAntennas[board][chipid], delayUplinkRecv);
	}

	DoPcap((char *)p->payload,p->size);

//	if	(p->status != STAT_NO_CRC && p->status != STAT_CRC_OK)
	if	(p->status == STAT_CRC_BAD)
	{
		LgwNbCrcError++;
		return	0;
	}

	if	(TraceLevel >= 1)
	{
	char	buff[512];
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

	memset	(&uppkt,0,sizeof(t_lrr_pkt));
	uppkt.lp_flag	= LP_RADIO_PKT_UP;
	if	(p->status == STAT_NO_CRC)
	{
		uppkt.lp_flag	= uppkt.lp_flag | LP_RADIO_PKT_NOCRC;
	}

	uppkt.lp_lrrid	= LrrID;
	uppkt.lp_tms	= tms - delayUplinkRecv;

	if	(!uppkt.lp_tms)	uppkt.lp_tms	= 1;

	uppkt.lp_gss	= (time_t)tv.tv_sec;
	uppkt.lp_gns	= (u_int)tv.tv_nsec;

	uppkt.lp_tus		= p->count_us;
	uppkt.lp_channel	= chan->channel;
	uppkt.lp_subband	= chan->subband;
	uppkt.lp_spfact		= CodeSpreadingFactor(p->modrate);
	uppkt.lp_correct	= CodeCorrectingCode(p->coderate);
	uppkt.lp_bandwidth	= CodeBandWidth(p->bandwidth);
	uppkt.lp_size		= p->size;

	int nbpkt = 0;
#if !defined(TEK_MICRO8)
	for (i=0; i<SX1301AR_BOARD_RFCHAIN_NB; i++)
#else
	for (i=0; i<num_sx1301_chips; i++)
#endif
	{
		if (!p->rsig[i].is_valid)
			continue;

#ifdef WITH_GPS
		if (UseGpsTime && gpstm[i][0] && GpsStatus == 'U')	// check if we got a valid cnt2utc value
		{
			uppkt.lp_gss = fetch_time.tv_sec;
			if (p->rsig[i].fine_received)
			{
				uppkt.lp_flag	= uppkt.lp_flag | LP_RADIO_PKT_FINETIME;
				/* ppkt.lp_gns = p->rsig[i].fine_tmst; */

				/*
					PT-1308: adjust finetimestamp at the end of the packet
					add the Time-On-Air of the payload to get the actual finetimestamp
				*/
				if (AdjustFineTimeStamp && (p->rsig[i].fine_tmst != 0))
				{
					double t_preamble = 0.0;
					double t_payload = 0.0;
					double t_syncword = 0.0;

					/* compute TOA (Time-On-Air) of the syncword and header+payload */
					/* note: syncword and payload in ms */
					time_on_air_rx(p, false, &t_preamble, &t_syncword, &t_payload);

					/* keep the result as a double to avoid accuracy lost during operation */
					/* note: toa in ms */
					double toa_ms = t_syncword + t_payload;

					/* note: rsig.fine_tmst in ns */
					/* 114ms subtracted to micmic gwloc_match_timestamps (HAL): unknown reason */
#define HAL_OFFSET 114
					uppkt.lp_gns = (p->rsig[i].fine_tmst + (((uint32_t)((toa_ms * 1000.0) - HAL_OFFSET) * 1000))) % 1000000000;

					/* detect if standard timestamp and finetimestamp could be in a different second */
					int32_t diff = (int32_t)(uppkt.lp_gns - fetch_time.tv_nsec);
#define TIME_SHIFT_NS 500000000 
					if (abs(diff) > TIME_SHIFT_NS) {
						if (diff < 0) {
							/* it means
									"fine  <-    >0.5s   -> fetch"
								likely
									"fetch |new sec| fine" with a shift < 0.5s
							*/
							uppkt.lp_gss++;
						} else {
							uppkt.lp_gss--;
						}
						RTL_TRDBG(4, "  finetime shift > 0.5s (%d): gns:%09u tv_nsec:%09u\n", diff, uppkt.lp_gns, fetch_time.tv_nsec);
					}

					RTL_TRDBG(3, "    correction: fetch: %d.%0.09d ftmst: %uns toa:%fms  -> %d.%0.09d\n",
						fetch_time.tv_sec, fetch_time.tv_nsec,
						p->rsig[i].fine_tmst, toa_ms,
						uppkt.lp_gss, uppkt.lp_gns);
				} else {
				    uppkt.lp_gns = p->rsig[i].fine_tmst;
				}
			}
			else
			{
				uppkt.lp_flag	= uppkt.lp_flag & ~LP_RADIO_PKT_FINETIME;
				uppkt.lp_gns = fetch_time.tv_nsec;
			}
		}
#endif
		data	= (u_char *)malloc(p->size);
		if	(!data)
			return	-1;
		memcpy	(data,p->payload,p->size);

		uppkt.lp_payload	= data;

		//lp_chain =  antennaid(4b) + board(3b) + rfchain(1b)
		uppkt.lp_chain		= LgwAntennas[board][chipid] << 4 ;
		uppkt.lp_chain		|= (board&0x0f) << 1 ;
//		uppkt.lp_chain		|= (p->rsig[i].chan&0x10) >> 4;	// chip num is on 4 MSB of chan
		// set rf chain instead of chip number because that's what is
		// required when sending downlink response
		uppkt.lp_chain		|= i&0x01;	// set rf chain
		// WARNING: we consider 'i' is chip number in LgwAntennas[board][i] and after
		// that we consider 'i' is rfchain number in 'uppkt.lp_chain |= i&0x01'
		// it could be a problem !

		uppkt.lp_rssi		= p->rsig[i].rssi_chan;
		uppkt.lp_snr		= p->rsig[i].snr;


#ifdef LP_TP31
		uppkt.lp_tmoa	= TmoaLrrPacketUp(p);
		RTL_TRDBG(3,"TmoaLrrPacketUp()=%f\n", uppkt.lp_tmoa);
		DcTreatUplink(&uppkt);
#endif

		msg	= rtl_imsgAlloc(IM_DEF,IM_LGW_RECV_DATA,NULL,0);
		if	(!msg)
			return	-2;

		sz	= sizeof(t_lrr_pkt);
		if	( rtl_imsgDupData(msg,&uppkt,sz) != msg)
		{
			rtl_imsgFree(msg);
			return	-3;
		}

		RTL_TRDBG(3,"PKT RECV add pkt received on chip %d\n", i);
		rtl_imsgAdd(MainQ,msg);
		nbpkt += 1;
	}
	return	nbpkt;
}

int	LgwDoRecvPacket(time_t tms)
{
	T_lgw_pkt_rx_t	rxpkt[LGW_RECV_PKT];
	T_lgw_pkt_rx_t	*p;
	uint8_t		nbpkt;
	int		i, b, ret;
	int		nb	= 0;
	static	int	lastboard = -1;
	time_t		exacttms;	// updated tms

//	for (b=0; b<LgwBoard; b++)
	b = (lastboard + 1) % LgwBoard;
	lastboard = b;
	{
		ret	= sx1301ar_fetch(b,rxpkt,LGW_RECV_PKT,&nbpkt);
		if	(ret != LGW_HAL_SUCCESS || nbpkt <= 0)
		{
			RTL_TRDBG(9,"PKT board%d nothing to Recv=%d ret=%d\n",b,nbpkt,ret);
			return	0;
		}

		// need to get tms here because sx1301ar_fetch takes at least 65 ms
		exacttms	= rtl_tmmsmono();
		LgwNbPacketRecv	+= nbpkt;
		for	(i = 0 ; i < nbpkt ; i++)
		{
			p	= &rxpkt[i];
			if	((ret=ProceedRecvPacket(p,exacttms,b)) < 0)
			{
				RTL_TRDBG(1,"PKT RECV board%d not treated ret=%d\n",b,ret);
			}
			else
			{
				nb+=ret;
			}
		}
	}

	return	nb;
}

/*!
* \fn int8_t GetTxCalibratedEIRP(int8_t tx_requested, float antenna_gain, float cable_loss, uint8_t board, uint8_t rfc)
* \brief Get the LUT-calibrated EIRP value
* \param tx_requested: Requested EIRP value set by the user (dBm)
* \param antenna_gain: Gain of the antenna (dBm)
* \param cable_loss: Loss of power due to the cable (dBm)
* \param board: Board index used for searching in LUT table
* \param rfc: RF chain used for searching in LUT table
* \return The theorical LUT-calibrated EIRP value
*
* EIRP stands for Equivalent Isotropically Radiated Power. This is the amount of power that an antenna would emit.
* EIRP takes into account the losses in transmission line and includes the gain of the antenna.
* This function takes the user-requested Tx EIRP and returns the nearest inferior workable value, based on the LUT calibration table.
*
*/
int8_t GetTxCalibratedEIRP(int8_t tx_requested, float antenna_gain, float cable_loss, uint8_t board, uint8_t rfc)
{
    return 0;
}

static	u_int	WaitSendPacket(int blo,time_t tms,sx1301ar_tstat_t *txstatus, int board)
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
		IsBoardStatusTxFree(board, txstatus);
#if	0
		u_int	t;
		t	= LgwTimeStamp();
RTL_TRDBG(3,"PKT sending j=%d status=%d t=%u\n",j,*txstatus,t);
#endif
		usleep(1000);	// * 3000 => 3s max
	} while ((*txstatus != TX_FREE) && (j < 3000));
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
	sx1301ar_tstat_t txstatus = 0;
	int32_t	newfreq;
	int	left;
	int	ret;
	int	diff	= 0;
	int	duration = 0;
	u_int	tms;

	left	= LgwNbPacketWait;

	IsBoardStatusTxFree(board, &txstatus);

	if (txstatus != TX_FREE)
	{
		if (downpkt->lp_beacon)
			LgwBeaconLastDeliveryCause = LP_CB_BUSY;
		else if (downpkt->lp_classb)
			SetIndic(downpkt, 0, -1, -1, LP_CB_BUSY);
		else if (downpkt->lp_classcmc)
			SetIndic(downpkt,0,-1,-1,LP_CB_BUSY);
		else if (Rx2Channel && Rx2Channel->freq_hz == txpkt.freq_hz)
			SetIndic(downpkt,0,-1,LP_C2_BUSY,-1);
		else
			SetIndic(downpkt,0,LP_C1_BUSY,-1,-1);
	}

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

	if	(txpkt.rf_power > LgwPowerMax)
		txpkt.rf_power	= LgwPowerMax;
#ifdef WITH_GPS
	RTL_TRDBG(1,"txpkt: tx_mode=%d use_gps_time=%d gps_time.tv_sec=%u tv_usec=%u count_us=%u size=%d\n",
		txpkt.tx_mode, txpkt.use_gps_time, txpkt.gps_time.tv_sec, txpkt.gps_time.tv_usec,
		txpkt.count_us, txpkt.size);
#else
	RTL_TRDBG(1,"txpkt: tx_mode=%d  count_us=%u size=%d\n",
		txpkt.tx_mode, txpkt.count_us, txpkt.size);
#endif
	tms	= rtl_tmmsmono();
	diff	= (time_t)tms - (time_t)downpkt->lp_tms;

#ifdef WITH_GPS
	// RDTP-15343: freq compensation for xtal drift, for beacons and multiC
	if (DriftComp == 1 && txpkt.tx_mode == TX_ON_GPS)
	{
		/* Compensate breacon frequency with xtal error */
		newfreq = (int32_t)((1.0 - Gps_time_ref[0].xtal_err) * (double)txpkt.freq_hz);
		RTL_TRDBG(1,"drift compensation, xtal_err = %.15lf, frequency %u Hz %+d Hz => %u Hz\n",
			Gps_time_ref[0].xtal_err, txpkt.freq_hz, newfreq, txpkt.freq_hz+newfreq);
		txpkt.freq_hz += newfreq;
	}
#endif

	if	((ret=sx1301ar_send(board,&txpkt)) != LGW_HAL_SUCCESS)
	{
#ifdef WITH_LBT
		if	(ret == ERR_SEND_LBT_CHAN_BUSY)
		{
			downpkt->lp_stopbylbt = 1;
			if (downpkt->lp_beacon) {
				LgwBeaconLastDeliveryCause = LP_CB_LBT;
				goto stop_by_lbt;
			}
			if (downpkt->lp_classb) {
				SetIndic(downpkt, 0, -1, -1, LP_CB_LBT);
				goto stop_by_lbt;
			}
			if	(downpkt->lp_classcmc)
			{
				SetIndic(downpkt,0,-1,-1,LP_CB_LBT);
				goto	stop_by_lbt;
			}
			if	(Rx2Channel && Rx2Channel->freq_hz == txpkt.freq_hz)
				SetIndic(downpkt,0,-1,LP_C2_LBT,-1);
			else
				SetIndic(downpkt,0,LP_C1_LBT,-1,-1);
stop_by_lbt:
			RTL_TRDBG(1,"PKT send stop by lbt=%d freq=%d\n",ret,
				txpkt.freq_hz);
			return	-3;
		}
#endif
		RTL_TRDBG(0,"PKT board%d send error=%d (%s)\n",board,ret,sx1301ar_err_message(sx1301ar_errno));
		return  -2;
	}
	if (downpkt->lp_beacon) {
		LgwBeaconSentCnt++;
		LgwBeaconLastDeliveryCause = 0;
	}
	if	(downpkt->lp_classcmc)
	{
		LgwClassCSentCnt++;
		LgwClassCLastDeliveryCause	= 0;
	}
	LgwNbPacketSend++;
//	sx1301ar_tx_status(board,0,&txstatus);
	IsBoardStatusTxFree(board, &txstatus);
	if	(downpkt->lp_lgwdelay == 0 && txstatus != TX_EMITTING)
	{
		RTL_TRDBG(0,"PKT SEND board%d  status(=%d) != TX_EMITTING\n",board,txstatus);
	}
	if	(0 && downpkt->lp_lgwdelay && txstatus != TX_SCHEDULED)	// TODO
	{
		RTL_TRDBG(0,"PKT SEND board%d status(=%d) != TX_SCHEDULED\n",board,txstatus);
	}

#ifdef LP_TP31
	// must be done only if packet was really sent
	DcTreatDownlink(downpkt);
#endif

RTL_TRDBG(1,"PKT SEND board%d tms=%09u/%d status=%d sz=%d left=%d freq=%d mod=0x%02x bdw=%s spf=%s ecc=%s pr=%d nocrc=%d ivp=%d pw=%.2f rfchain=%d\n",
	board,tms,diff,txstatus,txpkt.size,left,txpkt.freq_hz,
	txpkt.modulation,BandWidthTxt(txpkt.bandwidth),
	SpreadingFactorTxt(txpkt.modrate),CorrectingCodeTxt(txpkt.coderate),
	txpkt.preamble,txpkt.no_crc,txpkt.invert_pol,txpkt.rf_power,txpkt.rf_chain);

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
		LastTmoaRequested[board]	= LastTmoaRequested[board] +
					(LastTmoaRequested[board] * 10)/100;
		RTL_TRDBG(1,"LGW DELAY tmoa request=%dms + sched=%dms\n",
				LastTmoaRequested[board],scheduled);
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
		char	buff[1024];
		char	src[64] = "";
		char	pktclass[16] = "";
		LoRaMAC_t	mf;
		u_char		*pt;

		memset(&mf,0,sizeof(mf));
		if (downpkt->lp_beacon == 0) {
			// we assumed this is a loramac packet
			LoRaMAC_decodeHeader(txpkt.payload,txpkt.size,&mf);	
			pt	= (u_char *)&mf.DevAddr;
			sprintf	(src,"%02x%02x%02x%02x",*(pt+3),*(pt+2),*(pt+1),*pt);
			if (downpkt->lp_classb) {
				diff = 0;
				strcpy(pktclass, "classb");
				RTL_TRDBG(1,"PKT SEND classb period=%d sidx=%d sdur=%f window(%09u,%09u) %d/%d/%d\n",
					downpkt->lp_period,downpkt->lp_sidx,downpkt->lp_sdur,
				((downpkt->lp_period-1)*128)+downpkt->lp_gss0,
				downpkt->lp_gns0,
				downpkt->lp_idxtry,downpkt->lp_nbtry,downpkt->lp_maxtry);
			} else {
				strcpy(pktclass, "classa");
			}
			if	(downpkt->lp_classc)
			{
				strcpy(pktclass, "classcc");
			}
			if	(downpkt->lp_classcmc)
			{
				strcpy(pktclass, "classcmc");
			}
		}
		else
		{
			strcpy(pktclass, "beacon");
			strcpy(src, "<broadcast>");
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
	board,pktclass, downpkt->lp_tmoa/1000,diff,buff,mf.FCnt,src);
		}
	}

	DoPcap((char *)txpkt.payload,txpkt.size);

	return	0;
}

static	void	SetTrigTarget(t_lrr_pkt *downpkt,T_lgw_pkt_tx_t *txpkt)
{
	uint32_t	diffus = 0;
	int		ret = -1;
	static int      beacondelay = -1;
	static int      classbdelay = -1;

	if (classbdelay == -1)
	{
		classbdelay = CfgInt(HtVarLrr, "classb", -1, "adjustdelay", 0);
		RTL_TRDBG(1, "classb.adjustdelay=%d\n", classbdelay);
	}

	if (beacondelay == -1)
	{
		beacondelay = CfgInt(HtVarLrr, "classb", -1, "beacondelay", 1500);
		RTL_TRDBG(1, "classb.beacondelay=%d\n", beacondelay);
	}
	
	if (downpkt->lp_beacon)
	{
		int res;
		struct timespec butc;
		struct timespec * utc = &butc;
#if WITH_GPS
//		txpkt->tx_mode  = TX_TIMESTAMPED;
		txpkt->tx_mode  = TX_ON_GPS;
		// TEKTELIC expect a Gps time instead of UTC time
		// PT-1197: convert from UTC Linux to GPS origin
		txpkt->gps_time.tv_sec	= downpkt->lp_gss + GpsUtcLeap - GPS_TO_UTC_SEC;
		txpkt->gps_time.tv_usec	= downpkt->lp_gns/1000 + beacondelay;
		txpkt->use_gps_time	= 1;
		// butc: Linux origin
		res = LgwEstimUtcBoard(0, utc);
		if (res == LGW_HAL_SUCCESS)
			downpkt->lp_lgwdelay  = LgwPacketDelayMsFromUtc(downpkt,utc);
		else
			downpkt->lp_lgwdelay = 0;

		RTL_TRDBG(1, "PKT SEND beacon gps time=(%u,%06u us) leap seconds=%d, lgwdelay=%u\n",
			txpkt->gps_time.tv_sec, txpkt->gps_time.tv_usec, GpsUtcLeap, downpkt->lp_lgwdelay);
#else
		txpkt->tx_mode  = TX_IMMEDIATE;
		res = LgwEstimUtcBoard(0, utc);
		if (res == LGW_HAL_SUCCESS)
			downpkt->lp_lgwdelay  = LgwPacketDelayMsFromUtc(downpkt,utc);
		else
			downpkt->lp_lgwdelay = 0;
		RTL_TRDBG(1, "PKT SEND beacon lgwdelay=%u\n", downpkt->lp_lgwdelay);
#endif
		return;
	}

	if (downpkt->lp_classcmc) {
		int res;
		struct timespec butc;
		struct timespec * utc = &butc;
#if WITH_GPS
//		txpkt->tx_mode  = TX_TIMESTAMPED;
		txpkt->tx_mode  = TX_ON_GPS;
		// TEKTELIC expect a Gps time instead of UTC time
		// PT-1197: convert from UTC Linux to GPS origin
		txpkt->gps_time.tv_sec	= downpkt->lp_gss + GpsUtcLeap - GPS_TO_UTC_SEC;
		txpkt->gps_time.tv_usec	= downpkt->lp_gns/1000 + beacondelay;
		txpkt->use_gps_time	= 1;
		// butc: Linux origin
		res = LgwEstimUtcBoard(0, utc);
		if (res == LGW_HAL_SUCCESS)
			downpkt->lp_lgwdelay  = LgwPacketDelayMsFromUtc(downpkt,utc);
		else
			downpkt->lp_lgwdelay = 0;

		RTL_TRDBG(1, "PKT SEND classcmc gps time=(%u,%06u us) leap seconds=%d, lgwdelay=%u\n",
			txpkt->gps_time.tv_sec, txpkt->gps_time.tv_usec, GpsUtcLeap, downpkt->lp_lgwdelay);
#else
		txpkt->tx_mode  = TX_IMMEDIATE;
		res = LgwEstimUtcBoard(0, utc);
		if (res == LGW_HAL_SUCCESS)
			downpkt->lp_lgwdelay  = LgwPacketDelayMsFromUtc(downpkt,utc);
		else
			downpkt->lp_lgwdelay = 0;
		RTL_TRDBG(1, "PKT SEND classcmc lgwdelay=%u\n", downpkt->lp_lgwdelay);
#endif
		return;
	}

	if (downpkt->lp_classb) {
		int res;
		uint32_t trig_tstamp;
		uint32_t trig_estim;
		struct timespec butc;
		struct timespec * utc = &butc;

		res = LgwEstimUtcBoard(0, utc);
		RTL_TRDBG(1, "EstimUtcBoard=(%us,%09uns) res=%d\n",
			utc->tv_sec, utc->tv_nsec, res);

		// Convert UTC time -> GPS time
		// PT-1197: UTC with Linux origin to GPS witg GPS origin
		utc->tv_sec = 	downpkt->lp_gss + GpsUtcLeap - GPS_TO_UTC_SEC;
		utc->tv_nsec = 	downpkt->lp_gns;
		res = sx1301ar_gps2cnt(*utc, &trig_tstamp);
		if (res != LGW_HAL_SUCCESS) {
			RTL_TRDBG(0, "PKT SEND classb error sx1301ar_gps2cnt() from (%u,%09u) '%s'\n",
				downpkt->lp_gss + GpsUtcLeap, downpkt->lp_gns, sx1301ar_err_message(sx1301ar_errno));
			return;
		}

		res = sx1301ar_get_instcnt(0, 0, &trig_estim);
		if (res <= LGW_HAL_ERROR) {
			RTL_TRDBG(0, "sx1301ar_get_instcnt error '%s'\n", sx1301ar_err_message(sx1301ar_errno));
			return;
		}
		txpkt->tx_mode  = TX_TIMESTAMPED;
		txpkt->count_us = trig_tstamp - Sx13xxStartDelay + classbdelay;
		txpkt->use_gps_time	= 0;
		downpkt->lp_lgwdelay = ABS(txpkt->count_us - trig_estim ) / 1000;
		RTL_TRDBG(1,"PKT SEND classb trigtarget=%u trigestim=%u diffestim=%d adj=%d pkt=(%u,%09u) utc=(%u,%09u)\n",
			txpkt->count_us, trig_estim,
			downpkt->lp_lgwdelay, classbdelay, downpkt->lp_gss, downpkt->lp_gns,
			utc->tv_sec, utc->tv_nsec);

		return;
	}

	if (downpkt->lp_lgwdelay == 0)
	{
		if (LgwLbtEnable == 0) {
			txpkt->tx_mode = TX_IMMEDIATE;
			RTL_TRDBG(1, "LRR DELAY -> tx mode immediate tms=%u tus=%u\n",
					downpkt->lp_tms, downpkt->lp_tus);
					return;
		}
		/* RDTP-857 LBT is enable HAL refuses IMMEDIATE mode
		   => We force TX_TIMESTAMPED mode in 5ms */
		uint32_t tstamp;
		ret = sx1301ar_get_instcnt(0, 0, &tstamp);
		if (ret <= LGW_HAL_ERROR) {
			RTL_TRDBG(0, "sx1301ar_get_instcnt error '%s'\n", sx1301ar_err_message(sx1301ar_errno));
		}
		downpkt->lp_tus = tstamp + (LgwLbtClasscDelay*1000);
		downpkt->lp_delay = 0;
		downpkt->lp_bypasslbt = 1;
		RTL_TRDBG(1, "LRR DELAY -> tx mode immediate with LBT tms=%u tus=%u\n",
					downpkt->lp_tms, downpkt->lp_tus);
		
	}

	txpkt->tx_mode 	= TX_TIMESTAMPED;

	txpkt->count_us	= downpkt->lp_tus;
	txpkt->count_us	= txpkt->count_us + (downpkt->lp_delay * 1000);
	txpkt->count_us	= txpkt->count_us - Sx13xxStartDelay;

	diffus	= ABS(txpkt->count_us - downpkt->lp_tus);
RTL_TRDBG(3,"LGW DELAY trigtarget=%u trigorig=%u delayus=%u\n",
	txpkt->count_us,downpkt->lp_tus,diffus);

	return;
}

static	int	SendPacket(t_imsg  *msg)
{
	t_lrr_pkt	*downpkt;
	T_lgw_pkt_tx_t 	txpkt;
	t_channel	*chan;
	int	ret;
	int	blo	= 0;
	int	board	= 0;

	downpkt	= msg->im_dataptr;

	if	(downpkt->lp_channel >= MaxChannel ||
		TbChannel[downpkt->lp_channel].name[0] == '\0')
	{
		LgwNbChanDownError++;	
		RTL_TRDBG(1,"SendPacket: ChanDownError += 1 chan=%d maxchan=%d name=%s\n",
			downpkt->lp_channel, MaxChannel, (downpkt->lp_channel >= MaxChannel?"":TbChannel[downpkt->lp_channel].name[0]));
		return	0;
	}
	chan	= &TbChannel[downpkt->lp_channel];

	memset	(&txpkt,0,sizeof(txpkt));

//	txpkt.freq_hz = 868100000;
//	txpkt.modulation = MOD_LORA;
//	txpkt.bandwidth = BW_125KHZ;
//	txpkt.modrate = DR_LORA_SF10;
//	txpkt.coderate = CR_LORA_4_5;

	SetTrigTarget(downpkt,&txpkt);
	board			= (downpkt->lp_chain&0x0F) >> 1;
//	antennaid		= downpkt->lp_chain >> 4;
	// RDTP-5911
	if (DlShiftLc && (downpkt->lp_flag&LP_RADIO_PKT_SHIFTLC) == LP_RADIO_PKT_SHIFTLC)
	{
		RTL_TRDBG(1,"SendPacket: ShiftChannel flag set => force rf chain to 1\n");
		txpkt.rf_chain 		= 1;
	}
	else
		txpkt.rf_chain 		= downpkt->lp_chain&0x01;
	txpkt.freq_hz 		= chan->freq_hz;
	txpkt.modulation 	= chan->modulation;
	txpkt.bandwidth 	= chan->bandwidth;
	txpkt.modrate 		= DecodeSpreadingFactor(downpkt->lp_spfact);
	txpkt.coderate		= DecodeCorrectingCode(downpkt->lp_correct);
	txpkt.invert_pol	= LgwInvertPol;
	txpkt.no_crc 		= LgwNoCrc;
	txpkt.no_header 	= LgwNoHeader;
	txpkt.preamble 		= LgwPreamble;
	txpkt.rf_power 		=
		chan->power - AntennaGain[0] + CableLoss[0];	// TODO index

	if (downpkt->lp_beacon) {
		txpkt.invert_pol	= LgwInvertPolBeacon;
		txpkt.no_crc		= 1;
		txpkt.no_header		= 1;
		txpkt.preamble 		= 10;
/*	Done in SetTrigTarget
		txpkt.use_utc 		= 1;
		txpkt.utc_time.tv_sec	= downpkt->lp_gss;
		txpkt.utc_time.tv_usec	= downpkt->lp_gns/1000;
*/
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
		ret	= SendPacketNow(blo=1,downpkt,txpkt,board);
		if	(ret < 0)
		{
			RTL_TRDBG(0,"SendPacketNow() error => %d\n",ret);
		}
		usleep(LgwAckData802Wait*1000);	// 10 ms
	}

	txpkt.size = downpkt->lp_size;
	memcpy	(txpkt.payload,downpkt->lp_payload,downpkt->lp_size);
#if 0		// no more free when trying to send se we can retry
	free	(downpkt->lp_payload);
	downpkt->lp_payload	= NULL;
#endif

	ret	= SendPacketNow(blo=0,downpkt,txpkt,board);
	if	(ret < 0)
	{
		RTL_TRDBG(0,"SendPacketNow() error => %d\n",ret);
		return	0;
	}

	return	1;
}

static void FreeMsgAndPacket(t_imsg * msg, t_lrr_pkt * downpkt)
{
	if (downpkt && downpkt->lp_payload) {
		free(downpkt->lp_payload);
		downpkt->lp_payload	= NULL;
	}
	rtl_imsgFree(msg);
}

static int Rx1MissedTryRx2(t_imsg * msg, t_lrr_pkt * downpkt) {
	if (!msg || !downpkt)
		return	0;
	if (Rx2Channel == NULL)
		return	0;

	if (downpkt->lp_delay == 0 || downpkt->lp_beacon || downpkt->lp_classb
		|| downpkt->lp_classcmc)
	{	// classC ou beacon ou classB
		return	0;
	}
	if ((downpkt->lp_flag&LP_RADIO_PKT_RX2)==LP_RADIO_PKT_RX2)
	{	// already RX2 (by LRC)
		return	0;
	}
	if (downpkt->lp_rx2lrr)
	{	// already RX2 (by LRR)
		return	0;
	}

RTL_TRDBG(1,"PKT SEND RX1 missed (lbt=%d) try RX2\n", downpkt->lp_stopbylbt);

	downpkt->lp_rx2lrr = 1;
	downpkt->lp_delay += 1000;
	AutoRx2Settings(downpkt);
	return	1;
}

// calculate delay from now + shift (ms) to next pingslot
int	DelayToPingSlot(t_lrr_pkt *downpkt, int board, time_t shift, int margin, int *delay)
{
	struct timespec eutc;
	struct timespec *utc;

	if (!downpkt || !delay)
		return -2;

	utc = &eutc;
	if (LgwEstimUtcBoard(board, utc) != LGW_HAL_SUCCESS) {
		RTL_TRDBG(0, "can not estim UTC board %d for class B dl\n", board);
		return -2;
	}
	RTL_TRDBG(1, "DelayToPingSlot utc=(%u,%03ums)\n", utc->tv_sec, utc->tv_nsec/1000000);
	// add shift (ms) to current time
	eutc.tv_nsec += (shift%1000)*1000000;
	eutc.tv_sec += shift/1000 + eutc.tv_nsec/1000000000;
	eutc.tv_nsec = eutc.tv_nsec % 1000000000;

	if (LgwNextPingSlot(downpkt, utc, margin, delay) < 0) {
		RTL_TRDBG(0, "PKT SEND classb too late\n");
		return -1;
	}
	RTL_TRDBG(1, "DelayToPingSlot utc + %d ms=(%u,%03ums) => delay=%d ms\n", shift, utc->tv_sec, utc->tv_nsec/1000000, *delay);
	return 0;
}

// called by radio thread lgw_gen.c:LgwMainLoop()
int	LgwDoSendPacket(time_t now)
{
	t_imsg	*msg;
	int	nbs	= 0;
	int	left;
	int	ret;
	time_t	ref_age;
	int	board;
	static int	classbuseall = -1;
	static int	classbmargin = -1;
	static int	retryOnMissWin = -1;

	if (classbuseall == -1) {
		classbuseall = CfgInt(HtVarLrr, "classb", -1, "useallslot", 0);
		RTL_TRDBG(1, "classb useallslot=%d\n", classbuseall);
	}

	if (classbmargin == -1) {
		classbmargin = CfgInt(HtVarLrr, System, -1, "classbdnmargincpu", 200);
		RTL_TRDBG(1, "classb dnmargincpu=%d\n", classbmargin);
	}
	if (retryOnMissWin == -1) {
		retryOnMissWin = CfgInt(HtVarLrr,"lrr",-1,"retryonmisswin",0);
		RTL_TRDBG(1, "retryonmisswin=%d\n", retryOnMissWin);
	}

#ifdef WITH_GPS
	ref_age = abs(time( NULL ) - Gps_time_ref[0].systime);
	Gps_ref_valid = (ref_age >= 0) && (ref_age < 60);
#endif /* WITH_GPS */

	left = 0;

	for (board=0; board<LgwBoard; board++)
		left	+= rtl_imsgCount(LgwSendQ[board]);

	if	(left > 0 && left > LgwNbPacketWait)
		LgwNbPacketWait	= left;

	for (board=0; board<LgwBoard; board++)
	{
		if	((msg = rtl_imsgGet(LgwSendQ[board], IMSG_BOTH)) != NULL)
		{
			t_lrr_pkt	*downpkt;

			downpkt	= msg->im_dataptr;

			if (downpkt->lp_beacon && downpkt->lp_delay == 0) {
				int odelay;
				int delay;	// postpone delay
				struct timespec butc;

				RTL_TRDBG(1, "PKT SEND beacon utc pkt=(%us,%09uns)\n",
					downpkt->lp_gss, downpkt->lp_gns);

				ret = LgwEstimUtcBoard(board, &butc);
				if (ret != LGW_HAL_SUCCESS) {
					RTL_TRDBG(0, "can not estim UTC board %d for beacon\n", board);
					LgwBeaconLastDeliveryCause = LP_C1_DELAY;
					return 0;
				}
				RTL_TRDBG(1, "EstimUtcBoard=(%us,%09uns) ret=%d\n",
					butc.tv_sec, butc.tv_nsec, ret);

				delay  = LgwPacketDelayMsFromUtc(downpkt,&butc);
				odelay = delay;
				delay  = odelay - classbmargin - 100;

				RTL_TRDBG(1, "PKT SEND beacon postpone=%dms/%dms utc pkt=(%us,%09uns) utc board=(%us,%09uns)\n",
					delay, odelay,
					downpkt->lp_gss, downpkt->lp_gns,
					butc.tv_sec, butc.tv_nsec);

				if (delay <= 0) {
					RTL_TRDBG(0, "too old beacon %dms\n", delay);
					LgwBeaconRequestedLateCnt++;
					LgwBeaconLastDeliveryCause	= LP_C1_DELAY;
					return 0;
				}

				downpkt->lp_trip     = 0;
				downpkt->lp_delay    = delay;
				downpkt->lp_lgwdelay = 1;
				rtl_imsgAddDelayed(LgwSendQ[board], msg, delay);
				return	0;
			}

			if (downpkt->lp_beacon && downpkt->lp_delay != 0) {
				RTL_TRDBG(1,"PKT SEND beacon retrieved board %d\n", board);
				goto send_pkt;
			}

			if	(downpkt->lp_classcmc && downpkt->lp_delay == 0)
			{
				int		odelay;
				int		delay;	// postpone delay
				struct timespec butc;

				RTL_TRDBG(1, "PKT SEND classcmc utc pkt=(%us,%09uns)\n",
					downpkt->lp_gss, downpkt->lp_gns);

				ret = LgwEstimUtcBoard(board, &butc);
				if (ret != LGW_HAL_SUCCESS) {
					RTL_TRDBG(0, "can not estim UTC board %d for classcmc\n", board);
					LgwBeaconLastDeliveryCause = LP_C1_DELAY;
					return 0;
				}
				RTL_TRDBG(1, "EstimUtcBoard=(%us,%09uns) ret=%d\n",
					butc.tv_sec, butc.tv_nsec, ret);

				delay	= LgwPacketDelayMsFromUtc(downpkt,&butc);
				odelay	= delay;
				delay	= odelay - 300;
				

	RTL_TRDBG(1,
	"PKT SEND classcmc postpone=%d/%d pkt=(%us,%09uns) utc board=(%us,%09uns)\n",
					delay, odelay,
					downpkt->lp_gss, downpkt->lp_gns,
					butc.tv_sec, butc.tv_nsec);

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

			if	(downpkt->lp_classcmc && downpkt->lp_delay != 0)
			{
				RTL_TRDBG(1,"PKT SEND classcmc retrieved\n");
				goto	send_pkt;
			}

			if (downpkt->lp_classb && downpkt->lp_delay == 0) {

				int odelay;
				int delay;
				int ret;
				/*
				struct timespec eutc;
				struct timespec *utc;

				utc = &eutc;
				if (LgwEstimUtcBoard(board, utc) != LGW_HAL_SUCCESS) {
					RTL_TRDBG(0, "can not estim UTC board %d for class B dl\n", board);
					return 0;
				}
				ret = LgwNextPingSlot(downpkt, utc, classbmargin+100, &delay);
				if (ret < 0) {
					SetIndic(downpkt, 0, -1, -1, -1);
					SendIndicToLrc(downpkt);
					FreeMsgAndPacket(msg, downpkt);
					RTL_TRDBG(0, "PKT SEND classb too late\n");
					return 0;
				}
				*/
				ret = DelayToPingSlot(downpkt, board, 0, classbmargin+100, &delay);
				if (ret == -1)	// must send indic
				{
					// SetIndic should have be done previously and RDTP-14405 requires
					// the last reason of failure must be returned
					if (!IsIndicSet(downpkt))
					{
						SetIndic(downpkt, 0, -1, -1, LP_CB_DELAY);
						RTL_TRDBG(0,"use cause CB_DELAY because no other cause set previously\n");
					}
					SendIndicToLrc(downpkt);
					FreeMsgAndPacket(msg, downpkt);
					return 0;
				}
				else if (ret == -2)	// just return
					return 0;

				odelay = delay;
				delay  = odelay - classbmargin;
				if (0 && CurrTmoaRequested[board] >= odelay) {
					downpkt->lp_delay = 0;
					rtl_imsgAddDelayed(LgwSendQ[board], msg, CurrTmoaRequested[board]+3);
					return 0;
				}

				RTL_TRDBG(1, "PKT SEND classb postpone=%d/%d period=%d sidx=%d sdur=%f pkt=(%u,%09u)\n",
					delay, odelay,
					downpkt->lp_period, downpkt->lp_sidx, downpkt->lp_sdur,
					downpkt->lp_gss, downpkt->lp_gns);

				downpkt->lp_trip         = 0;
				downpkt->lp_delay        = delay;
				downpkt->lp_lgwdelay     = 1;
				rtl_imsgAddDelayed(LgwSendQ[board], msg, delay);
				return 0;
			}

			if (downpkt->lp_classb && downpkt->lp_delay != 0)
			{
				RTL_TRDBG(1, "PKT SEND classb retrieved\n");
				if (CurrTmoaRequested[board])
				{
					int delay;
					int odelay;
					// set indic now as required by RDTP-14405, when no more pingslot is
					// available the last cause of failure must be returned
					SetIndic(downpkt, 0, -1, -1, LP_CB_BUSY);
					if (CfgInt(HtVarLrr, "classb", -1, "adjustretryoncollision", 0) == 1)
					{
						// request delay from end of radio use (=now+CurrTmoaRequested[board]+3) to
						// next pingslot
						ret = DelayToPingSlot(downpkt, board, CurrTmoaRequested[board]+3,
								classbmargin+100, &odelay);
						if (ret == -1)	// must send indic
						{
							SendIndicToLrc(downpkt);
							FreeMsgAndPacket(msg, downpkt);
							return 0;
						}
						else if (ret == -2)	// just return
							return 0;
						// add radioo used time
						odelay += CurrTmoaRequested[board]+3;
						delay = odelay - classbmargin;
						RTL_TRDBG(1,"PKT SEND classb avoid collision repostpone to next pingslot=%d/%d\n",
							delay, odelay);
						downpkt->lp_delay = delay;
						rtl_imsgAddDelayed(LgwSendQ[board], msg, delay);
					}
					else
					{
						RTL_TRDBG(1,"PKT SEND classb avoid collision repostpone=%d\n",
							CurrTmoaRequested[board]);
						downpkt->lp_delay = 0;
						rtl_imsgAddDelayed(LgwSendQ[board], msg, CurrTmoaRequested[board]+3);
					}
					return 0;
				}
				goto send_pkt;
			}

			if (downpkt->lp_delay == 0 && ABS(now - downpkt->lp_tms) > MaxReportDnImmediat)
			{	// mode immediate
				RTL_TRDBG(1,"PKT SEND NODELAY board%d not sent after 60s => dropped\n", board);
				SetIndic(downpkt,0,LP_C1_MAXTRY,LP_C2_MAXTRY,-1);
				SendIndicToLrc(downpkt);
				FreeMsgAndPacket(msg, downpkt);
				return	0;
			}
			
			if (downpkt->lp_delay == 0 && CurrTmoaRequested[board])
			{	// mode immediate
				RTL_TRDBG(1,"PKT SEND NODELAY board%d avoid collision repostpone=%d\n",
							board, CurrTmoaRequested[board]);
				rtl_imsgAddDelayed(LgwSendQ[board],msg,CurrTmoaRequested[board]+3);
				return	0;
			}

send_pkt:
			if (classbuseall && downpkt->lp_classb && downpkt->lp_nbslot <= 16)
			{
				t_lrr_pkt    pkt;
				t_imsg     * repeat;

				memcpy(&pkt, downpkt, sizeof(t_lrr_pkt));
				pkt.lp_payload = (u_char *)malloc(pkt.lp_size);
				memcpy(pkt.lp_payload, downpkt->lp_payload, pkt.lp_size);
				pkt.lp_delay = 0;
				repeat = rtl_imsgAlloc(IM_DEF, IM_LGW_SEND_DATA, NULL, 0);
				rtl_imsgDupData(repeat, &pkt, sizeof(t_lrr_pkt));
				rtl_imsgAddDelayed(LgwSendQ[board], repeat, 1000);
			}

			ret	= SendPacket(msg);
			if	(ret > 0)
			{
				SetIndic(downpkt,1,-1,-1,-1);
				nbs	+= ret;
			}
			else
			{
				RTL_TRDBG(1, "SendPacket() beacon=%d classb=%d classc=%d classcmc=%d rx2=%d lbt=%d error => %d\n",
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
					&& Rx1MissedTryRx2(msg,downpkt))
				{	// retry classA/RX1 on RX2 if stopped by LBT
					downpkt->lp_stopbylbt	= 0;
					rtl_imsgAddDelayed(LgwSendQ[board],msg,950);
					return	0;
				}

				// DL class A on rx1 failed for any reason, try rx2
				if (retryOnMissWin && downpkt->lp_classb == 0 && downpkt->lp_rx2lrr == 0 &&
					Rx1MissedTryRx2(msg,downpkt))
				{	
					int	delay=0;
					// retry classA/RX1 on RX2 if stopped by LBT
					if (downpkt->lp_misswin < 0)
					{
						delay = downpkt->lp_misswin;
						RTL_TRDBG(1, "SendPacket() delay=%d\n", delay);
					}

					// next window=1000, error done for rx1=delay, time between each loop=80
					delay = 1000 + delay - 85;
					RTL_TRDBG(1, "SendPacket() dl classa missed rx1, try rx2, postponed %d ms, misswin=%d\n",
						delay, downpkt->lp_misswin);
					rtl_imsgAddDelayed(LgwSendQ[board],msg,delay);
					return	0;
				}
				// DL class B failed, sx1301ar_send returned an error
				if (downpkt->lp_classb == 1 && ret == 0)
				{	
					int odelay;
					int delay;
					int ret;

					RTL_TRDBG(1,"PKT SEND classb error while sending\n");
					if (!IsIndicSet(downpkt))
						SetIndic(downpkt, 0, -1, -1, LP_CB_BOARDERR);
					ret = DelayToPingSlot(downpkt, board, 0, classbmargin+100, &odelay);
					if (ret == -1)	// must send indic
					{
//						SetIndic(downpkt, 0, -1, -1, LP_CB_DELAY);
						SendIndicToLrc(downpkt);
						FreeMsgAndPacket(msg, downpkt);
						return 0;
					}
					else if (ret == 0)	// found a next pingslot
					{
						delay  = odelay - classbmargin;

						RTL_TRDBG(1,"PKT SEND classb error while sending, repostpone to next pingslot=%d/%d\n",
							delay, odelay);

						downpkt->lp_trip         = 0;
						downpkt->lp_delay        = delay;
						downpkt->lp_lgwdelay     = 1;
						rtl_imsgAddDelayed(LgwSendQ[board], msg, delay);
						return 0;
					}
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
