
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

#undef	WITH_SX1301_X8		// !!!
#define	WITH_SX1301_X1		// !!!
#include "semtech.h"
#include "headerloramac.h"

#ifndef	TX_FREE
#define	TX_FREE	TX_EMPTY
#endif

#include "define.h"
#include "infrastruct.h"
#include "struct.h"
#include "cproto.h"
#include "extern.h"

/* time reference used for UTC <-> timestamp conversion */
struct tref Time_reference_gps; 
/* is GPS reference acceptable (ie. not too old) */
int Gps_ref_valid; 

static	int	_LgwStarted;
#ifdef	TEST_ONLY
static	int	LgwWaitSending	= 1;
#else
static	int	LgwWaitSending	= 0;
#endif

#ifdef USELIBLGW3
bool	LgwTxEnable[LGW_RF_CHAIN_NB];
#endif

static	unsigned	int	LgwMxDiffTrigRaw;
static	unsigned	int	LgwMxDiffTrig;

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
	return	lgw_version_info();
}

int	LgwTxFree(uint8_t board,uint8_t *txstatus)
{
	LGW_STATUS(board, TX_STATUS,txstatus);
	if	(*txstatus != TX_FREE)
		return	0;
	return	1;
}

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

char	*BandWidthTxt(bandwidth)
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

static	unsigned int	LgwTimeStamp()
{
	uint8_t	buf[4];
	unsigned int value;
	int	nbbits=32;
	int	ret;

	ret	= LGW_REG_RB(0, LGW_TIMESTAMP,buf,nbbits);	// TODO: treat board
	if	(ret != LGW_REG_SUCCESS)
	{
		RTL_TRDBG(0,"cannot read LGW timestamp REG ret=%d\n",ret);
		return	0;	
	}
//	value	= (buf[0] << 24) + (buf[1] << 16) + (buf[2] << 8) + buf[3];
	value	= *(unsigned int *)buf;
	return	value;
}

int	LgwLINKUP()
{
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

static	void LgwDumpRfConf(FILE *f,int board,int rfi,T_lgw_conf_rxrf_t rfconf)
{
	if	(rfconf.enable == 0)
		return;

#ifdef USELIBLGW3
	RTL_TRDBG(1,"board%d rf%d enab=%d frhz=%d tx_enab=%d rssi_off=%f sx=%d\n",
	board,rfi,rfconf.enable,rfconf.freq_hz,
	rfconf.tx_enable, rfconf.rssi_offset,rfconf.type);
#else
	RTL_TRDBG(1,"rf%d enab=%d frhz=%d\n",rfi,rfconf.enable,
		rfconf.freq_hz);
#endif

	if	(f == NULL)
		return;
#ifdef USELIBLGW3
	fprintf(f,"board%d rf%d enab=%d frhz=%d tx_enab=%d rssi_off=%f sx=%d\n",
	board,rfi,rfconf.enable,rfconf.freq_hz,
	rfconf.tx_enable, rfconf.rssi_offset,rfconf.type);
#else
	fprintf(f,"rf%d enab=%d frhz=%d\n",rfi,rfconf.enable,
		rfconf.freq_hz);
#endif
	fflush(f);
}

static	void LgwDumpIfConf(FILE *f,int board,int ifi,T_lgw_conf_rxif_t ifconf,int fbase)
{
	if	(ifconf.enable == 0)
		return;

	RTL_TRDBG(1,"board%d if%d enab=%d frhz=%d rfchain=%d bandw=0x%x datar=0x%x\n",
		board,ifi,ifconf.enable,
		(ifconf.freq_hz+fbase),ifconf.rf_chain,
		ifconf.bandwidth,ifconf.datarate);

	if	(f == NULL)
		return;
	fprintf(f,"board%d if%d enab=%d frhz=%d rfchain=%d bandw=0x%x datar=0x%x\n",
		board,ifi,ifconf.enable,
		(ifconf.freq_hz+fbase),ifconf.rf_chain,
		ifconf.bandwidth,ifconf.datarate);
	fflush(f);
}

#ifdef USELIBLGW3
static	void LgwDumpLutConf(FILE *f,int l,struct lgw_tx_gain_s *lut)
{
	RTL_TRDBG(1,"lut%d power=%d pa=%d mix=%d dig=%d dac=%d\n",l,
	lut->rf_power,lut->pa_gain,lut->mix_gain,lut->dig_gain,lut->dac_gain);

	if	(f == NULL)
		return;
	fprintf(f,"lut%d power=%d pa=%d mix=%d dig=%d dac=%d\n",l,
	lut->rf_power,lut->pa_gain,lut->mix_gain,lut->dig_gain,lut->dac_gain);
	fflush(f);
}
#endif

int	LgwConfigure(int hot,int config)
{
	struct lgw_conf_board_s boardconf;
	char	file[PATH_MAX], tmp[40];
	FILE	*f	= NULL;
	T_lgw_conf_rxrf_t rfconf;
	T_lgw_conf_rxif_t ifconf;
	int	rfi;
	int	ifi;
	int	ret;
	int	rtype0	= 1257;	// radio type of rfconf0 (sx1255,sx1257,...)
	int	b;
	int	dupcfg	= 0;	// duplicate config


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

#ifdef USELIBLGW3
	for (b=0; b<LgwBoard; b++)
	{
		memset(&boardconf, 0, sizeof(boardconf));
		boardconf.clksrc =	CfgInt(HtVarLgw,"gen",0,"clksrc",1);
		boardconf.lorawan_public = true;
		if	(LgwSyncWord == 0x12)
			boardconf.lorawan_public = false;
		ret = LGW_BOARD_SETCONF(b, boardconf);
		if	(ret == LGW_HAL_ERROR)
		{
			RTL_TRDBG(0,"Board %d cannot be configured ret=%d\n", b, ret);
			return	-1;
		}
	}
#endif

#ifdef WIRMAMS
	if	(CfgStr(HtVarLgw,"rfconf",0,"freqhz",NULL) != NULL)
	{
		RTL_TRDBG(0,"mono slot cfg detected on multi slots platform => dupl cfg x %d boards\n",LgwBoard);
		dupcfg	= 1;
		if	(f)
		{
		fprintf(f,"mono slot cfg detected on multi slots platform => dupl cfg x %d boards\n",LgwBoard);
		}
	}
#endif

	for (b=0; b<LgwBoard; b++)
	{
		for	(rfi = 0 ; rfi < LGW_RF_CHAIN_NB ; rfi++)
		{
			memset(&rfconf,0,sizeof(rfconf));
#ifdef WIRMAMS
			if	(dupcfg == 0)
				sprintf(tmp, "rfconf:%d", b);
			else
				strcpy(tmp, "rfconf");
#else
			strcpy(tmp, "rfconf");
#endif
			rfconf.enable	= CfgInt(HtVarLgw,tmp,rfi,"enable",0);
			rfconf.freq_hz	= CfgInt(HtVarLgw,tmp,rfi,"freqhz",0);
#ifdef USELIBLGW3
			int	rtype;
			rtype	= CfgInt(HtVarLgw,tmp,rfi,"radiotype",1257);
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
				rtype0	= rtype;
#ifdef WIRMAMS
			sprintf(tmp, "rfconf:%d", b);
#else
			strcpy(tmp, "rfconf");
#endif
			rfconf.rssi_offset	= atof(CfgStr(HtVarLgw,tmp,rfi,
								"rssioffset","-166.0"));
			rfconf.tx_enable	= CfgInt(HtVarLgw,tmp,rfi,
									"txenable",1);
//			Force tx_enable=1 for rfconf 0, tx_enable=0 for all others
//			rfconf.tx_enable	= (rfi==0)?1:0;
			LgwTxEnable[rfi]	= rfconf.tx_enable;
#endif
			LgwDumpRfConf(f,b,rfi,rfconf);
			if	(config)
			{
				ret	= LGW_RXRF_SETCONF(b,rfi,rfconf);
				if	(ret == LGW_HAL_ERROR)
				{
				RTL_TRDBG(0,"BOARD%d RF%d cannot be configured ret=%d\n",
									b,rfi,ret);
					if	(f)	fclose(f);
					return	-1;
				}
			}
		}

		for	(ifi = 0 ; ifi < LGW_IF_CHAIN_NB ; ifi++)
		{
			int	fbase;

			memset(&ifconf,0,sizeof(ifconf));
#ifdef WIRMAMS
			if	(dupcfg == 0)
				sprintf(tmp, "ifconf:%d", b);
			else
				strcpy(tmp, "ifconf");
#else
			strcpy(tmp, "ifconf");
#endif
			ifconf.enable	= CfgInt(HtVarLgw,tmp,ifi,"enable",0);
			ifconf.rf_chain	= CfgInt(HtVarLgw,tmp,ifi,"rfchain",0);
			ifconf.freq_hz	= CfgInt(HtVarLgw,tmp,ifi,"freqhz",0);
			ifconf.bandwidth= CfgInt(HtVarLgw,tmp,ifi,"bandwidth",0);
			ifconf.datarate	= CfgInt(HtVarLgw,tmp,ifi,"datarate",0);
#ifdef USELIBLGW3
			ifconf.sync_word	= CfgInt(HtVarLgw,tmp,ifi,"syncword",0);
			ifconf.sync_word_size	= CfgInt(HtVarLgw,tmp,ifi,"syncwordsize",0);
#endif

			rfi		= ifconf.rf_chain;
#ifdef WIRMAMS
			if	(dupcfg == 0)
				sprintf(tmp, "rfconf:%d", b);
			else
				strcpy(tmp, "rfconf");
#else
			strcpy(tmp, "rfconf");
#endif
			fbase		= CfgInt(HtVarLgw,tmp,rfi,"freqhz",0);
			LgwDumpIfConf(f,b,ifi,ifconf,fbase);
			if	(config)
			{
				ret	= LGW_RXIF_SETCONF(b,ifi,ifconf);
				if	(ret == LGW_HAL_ERROR)
				{
					RTL_TRDBG(0,"BOARD%d IF%d cannot be configured ret=%d\n",
									b,ifi,ret);
					if	(f)	fclose(f);
					return	-1;
				}
			}
		}
	}
#ifdef USELIBLGW3
	struct lgw_tx_gain_lut_s txlut;
	char	lutsection[64];
	int	lut;
	char	*pt;

	sprintf	(lutsection,"lut/%d/%d/%d",IsmFreq,LgwPowerMax,rtype0);
	pt	= CfgStr(HtVarLgw,lutsection,-1,"0","");
	if	(!pt || !*pt)
	{
		RTL_TRDBG(0,"LUT '%s' not found\n",lutsection);
		strcpy	(lutsection,"lut");
	}

	memset(&txlut,0,sizeof txlut);
	for (lut = 0; lut < TX_GAIN_LUT_SIZE_MAX; lut++)
	{
		char	var[64];
		int	dig,pa,dac,mix,pow;

		sprintf	(var,"%d",lut);
		pt	= CfgStr(HtVarLgw,lutsection,-1,var,"");
		if	(!pt || !*pt)	continue;
		txlut.size++;
		dig	= 0;
		dac	= 3;
		sscanf	(pt,"%d%d%d%d%d",&pow,&pa,&mix,&dig,&dac);
		txlut.lut[lut].rf_power	= (int8_t)pow;	// signed !!!
		txlut.lut[lut].pa_gain	= (uint8_t)pa;
		txlut.lut[lut].mix_gain	= (uint8_t)mix;
		txlut.lut[lut].dig_gain	= (uint8_t)dig;
		txlut.lut[lut].dac_gain	= (uint8_t)dac;
		LgwDumpLutConf(f,lut,&txlut.lut[lut]);
	}
	if	(config && txlut.size > 0)
	{
		// same calibration for all boards
		for (b=0; b<LgwBoard; b++)
		{
			ret	= LGW_TXGAIN_SETCONF(b, &txlut);
			if	(ret == LGW_HAL_ERROR)
			{
				RTL_TRDBG(0,"BOARD%d LUT%d cannot be configured ret=%d\n",
								b,lut,ret);
				if	(f)	fclose(f);
				return	-1;
			}
		}
	}
	if	(config)
	{
		RTL_TRDBG(1,"%s %s configured nblut=%d\n",
			lutsection,txlut.size==0?"not":"",txlut.size);
	}

	if	(f)
	{
		fprintf(f,"%s %s configured nblut=%d\n",
			lutsection,txlut.size==0?"not":"",txlut.size);
	}

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

	return	0;
}

// not called with WIRMAMS
void	LgwRegister()
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

	LGW_REG_W(0,LGW_FRAME_SYNCH_PEAK1_POS,sw1); /* default 1 */
	LGW_REG_W(0,LGW_FRAME_SYNCH_PEAK2_POS,sw2); /* default 2 */

	LGW_REG_W(0,LGW_MBWSSF_FRAME_SYNCH_PEAK1_POS,sw1); /* default 1 */
	LGW_REG_W(0,LGW_MBWSSF_FRAME_SYNCH_PEAK2_POS,sw2); /* default 2 */

	LGW_REG_W(0,LGW_TX_FRAME_SYNCH_PEAK1_POS,sw1); /* default 1 */
	LGW_REG_W(0,LGW_TX_FRAME_SYNCH_PEAK2_POS,sw2); /* default 2 */
}

int	LgwStart()
{
	int	ret, b;
	FILE	*chk;
	char	file[PATH_MAX];

#ifdef	USELIBLGW3	 // LUT calibration set by lgw_rxrf_setconf() hal 3.2.0
#else
	setBoardCalibration();
#endif

	RTL_TRDBG(1,"RADIO starting ...\n");
	for (b=0; b<LgwBoard; b++)
	{
		ret	= LGW_START(b);
		if	(ret == LGW_HAL_ERROR)
		{
			RTL_TRDBG(0,"BOARD%d RADIO cannot be started ret=%d\n",b,ret);
			sprintf	(file,"%s/var/log/lrr/radioparams.txt",RootAct);
			chk	= fopen(file,"a");
			if	(chk)
			{
				fprintf(chk,"BOARD%d RADIO cannot be started ret=%d\n",b,ret);
				fclose(chk);
			}
			return	-1;
		}
	}

#ifndef USELIBLGW3
	LgwRegister();
#endif
	_LgwStarted	= 1;
	RTL_TRDBG(1,"RADIO started ret=%d\n",ret);
	sprintf	(file,"%s/var/log/lrr/checkreg.out",RootAct);
	chk	= fopen(file,"w");
	if	(chk)
	{
		for (b=0; b<LgwBoard; b++)
		{
			fprintf(chk, "Board %d :\n", b);
			LGW_REG_CHECK(b, chk);
		}
		fclose(chk);
	}
	sprintf	(file,"%s/var/log/lrr/radioparams.txt",RootAct);
	chk	= fopen(file,"a");
	if	(chk)
	{
		for (b=0; b<LgwBoard; b++)
			fprintf(chk,"RADIO board %d started ret=%d\n",b,ret);
		fclose(chk);
	}
	return	0;
}

int	LgwStarted()
{
	return	_LgwStarted;
}

void	LgwStop()
{
	int	b;
	_LgwStarted	= 0;
	RTL_TRDBG(1,"RADIO stopping ...\n");
	for (b=0; b<LgwBoard; b++)
		LGW_STOP(b);
	RTL_TRDBG(1,"RADIO stopped\n");
}

#ifdef	WITH_GPS
void	LgwGpsTimeUpdated(struct timespec *utc_from_gps, struct timespec * ubxtime_from_gps)
{
	int	ret;
	uint32_t trig_tstamp;
	struct timespec utc_time;


#ifdef	TODO
	long	gps_ref_age = 0;
	gps_ref_age = (long)difftime(time(NULL), Time_reference_gps.systime);
	if ((gps_ref_age >= 0) && (gps_ref_age <= GPS_REF_MAX_AGE)) 
	{	/* time ref is ok, validate and  */
		Gps_ref_valid = true;
	} 
	else 
	{	/* time ref is too old, invalidate */
		Gps_ref_valid = false;
		RTL_TRDBG(1,"LGW GPS SYNC (sec=%u,ns=%u) INVALID\n",
		utc_from_gps->tv_sec,utc_from_gps->tv_nsec);
		return;
	}
#else
	Gps_ref_valid	= true;
#endif

	// get trig counter corresponding to the last GPS TIC
	ret	= LGW_GET_TRIGCNT(0, &trig_tstamp);	// TODO: treat board
	if	(ret != LGW_HAL_SUCCESS)
	{
		RTL_TRDBG(0,"cannot read trig for GPS SYNC\n");
		return;
	}

	ret	= lgw_gps_sync(&Time_reference_gps,trig_tstamp,*utc_from_gps);
	if	(ret != LGW_HAL_SUCCESS)
	{
		RTL_TRDBG(0,"cannot sync trig for GPS SYNC\n");
		return;
	}
#if	0
// TODO test
	usleep(1000);
	ret	= LGW_GET_TRIGCNT(&trig_tstamp);
	trig_tstamp	+= 1000000;
#endif

	lgw_cnt2utc(Time_reference_gps,trig_tstamp,&utc_time);


	RTL_TRDBG(3,"LGW GPS SYNC (sec=%u,ns=%u) (sec=%u,ns=%u) \n",
	utc_from_gps->tv_sec,utc_from_gps->tv_nsec,
	utc_time.tv_sec,utc_time.tv_nsec);

}
#endif


static	int	ProceedRecvPacket(T_lgw_pkt_rx_t    *p,time_t tms)
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

	clock_gettime(CLOCK_REALTIME,&tv);

	chan	= FindChannelForPacket(p);
	if	(chan)
	{
		numchan	= chan->channel;
		namchan	= (char *)chan->name;
		numband	= chan->subband;
	}

RTL_TRDBG(1,"PKT RECV board%d tms=%09u tus=%09u if=%d status=%s sz=%d freq=%d mod=0x%02x bdw=%s spf=%s ecc=%s rssi=%f snr=%f channel=%d nam='%s' G%d\n",
		p->rf_chain>>1,tms,p->count_us,p->if_chain,PktStatusTxt(p->status),p->size,
		p->freq_hz,p->modulation,BandWidthTxt(p->bandwidth),
		SpreadingFactorTxt(p->datarate),CorrectingCodeTxt(p->coderate),
		p->rssi,p->snr,numchan,namchan,numband);

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
#ifdef	WITH_GPS
	if	(UseGpsTime && Gps_ref_valid)
	{
		int	ret;
		struct timespec utc_time;
		struct timespec tmp;

		ret	= lgw_cnt2utc(Time_reference_gps,p->count_us,&utc_time);
		if	(ret != LGW_HAL_SUCCESS)
		{	// keep linux time
			uppkt.lp_gss	= (time_t)tv.tv_sec;
			uppkt.lp_gns	= (u_int)tv.tv_nsec;
		}
		else
		{

			uppkt.lp_flag	= uppkt.lp_flag | LP_RADIO_PKT_LGWTIME;
			uppkt.lp_gss	= (time_t)utc_time.tv_sec;
			uppkt.lp_gns	= (u_int)utc_time.tv_nsec;

			lgw_utc2cnt(Time_reference_gps,utc_time,&(p->count_us));
#if	0
			long	d;
			memcpy	(&tmp,&utc_time,sizeof(tmp));
			d	= CMP_TIMESPEC(tv,tmp);
			if	(d > 0)
			{
				SUB_TIMESPEC(tv,tmp);
			}
			else
			{
				SUB_TIMESPEC(tmp,tv);
				memcpy	(&tv,&tmp,sizeof(tv));
			}


RTL_TRDBG(1,"LGW GPS DATE+TUS (sec=%u,ns=%u) (sec=%u,ns=%u) tus=%u\n",
				utc_time.tv_sec,utc_time.tv_nsec,
				tv.tv_sec,tv.tv_nsec,p->count_us);
#endif
		}
		
	}
	else
#endif
	{
		uppkt.lp_gss	= (time_t)tv.tv_sec;
		uppkt.lp_gns	= (u_int)tv.tv_nsec;
	}

	uppkt.lp_tus		= p->count_us;
	uppkt.lp_chain		= p->rf_chain;
#ifdef	WIRMAMS
	uppkt.lp_chain		= (p->rf_chain>>1)<<4 | (p->rf_chain&0x1);
#endif
	uppkt.lp_rssi		= p->rssi;
	uppkt.lp_snr		= p->snr;
	uppkt.lp_channel	= chan->channel;
	uppkt.lp_subband	= chan->subband;
	uppkt.lp_spfact		= CodeSpreadingFactor(p->datarate);
	uppkt.lp_correct	= CodeCorrectingCode(p->coderate);
#if	0
	uppkt.lp_frequency	= p->freq_hz;
	uppkt.lp_board		= 0;
#endif
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

#ifdef WIRMAMS
int	LgwDoRecvPacket(time_t tms)
{
	T_lgw_pkt_rx_t	rxpkt[LGW_RECV_PKT];
	T_lgw_pkt_rx_t	*p;
	int			nbpkt;
	int			i;
	int			b;
	int			ret;
	int			nb	= 0;

	for (b=0; b<LgwBoard; b++)
	{
		nbpkt	= LGW_RECEIVE(b, LGW_RECV_PKT, rxpkt);
		if	(nbpkt <= 0)
		{
			RTL_TRDBG(9,"PKT nothing to Recv on board %d, nbpkt=%d\n", b, nbpkt);
			continue;
		}

		LgwNbPacketRecv	+= nbpkt;
		nb	= 0;
		for	(i = 0 ; i < nbpkt ; i++)
		{
			p	= &rxpkt[i];
			p->rf_chain = p->rf_chain | (b << 1);	// add board number in rf_chain
			if	((ret=ProceedRecvPacket(p,tms)) < 0)
			{
				RTL_TRDBG(1,"PKT RECV not treated ret=%d\n",ret);
			}
			else
			{
				nb++;
			}
		}
	}

	return	nb;
}
#else
int	LgwDoRecvPacket(time_t tms)
{
	T_lgw_pkt_rx_t	rxpkt[LGW_RECV_PKT];
	T_lgw_pkt_rx_t	*p;
	int			nbpkt;
	int			i;
	int			ret;
	int			nb	= 0;

	nbpkt	= LGW_RECEIVE(LGW_RECV_PKT,rxpkt);
	if	(nbpkt <= 0)
	{
		RTL_TRDBG(9,"PKT nothing to Recv=%d\n",nbpkt);
		return	0;
	}

	LgwNbPacketRecv	+= nbpkt;
	nb	= 0;
	for	(i = 0 ; i < nbpkt ; i++)
	{
		p	= &rxpkt[i];
		if	((ret=ProceedRecvPacket(p,tms)) < 0)
		{
			RTL_TRDBG(1,"PKT RECV not treated ret=%d\n",ret);
		}
		else
		{
			nb++;
		}
	}

	return	nb;
}
#endif

static	u_int	WaitSendPacket(int board,int blo,time_t tms,uint8_t *txstatus)
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
		LGW_STATUS(board, TX_STATUS,txstatus);
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

static	int	SendPacketNow(int blo,t_lrr_pkt *downpkt,T_lgw_pkt_tx_t txpkt)
{
	uint8_t txstatus = 0;
	int	left;
	int	ret;
	int	diff	= 0;
	int	duration = 0;
	int	board;
	u_int	tms;
	int	tempgain;

	left	= LgwNbPacketWait;
	board = txpkt.rf_chain>>1;	// bit 0 = rf chain, bits 1-7 = board
	if (board < 0 || board >= LgwBoard)
	{
		RTL_TRDBG(1,"board out of bounds (%d), forced to 0\n", board);
		board = 0;
	}

	LGW_STATUS(board,TX_STATUS,&txstatus);
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

	tempgain	= LgwTempPowerGain();
	txpkt.rf_power	= txpkt.rf_power + tempgain;
	/* PT-1350: gateways with SAW filter may require compensation */
	txpkt.rf_power	= txpkt.rf_power + LgwTxPowerComp(IsmBand, txpkt.freq_hz);
	if	(txpkt.rf_power > LgwPowerMax)
		txpkt.rf_power	= LgwPowerMax;
	tms	= rtl_tmmsmono();
	diff	= (time_t)tms - (time_t)downpkt->lp_tms;
	if	((ret=lgw_send(txpkt)) != LGW_HAL_SUCCESS)
	{
		RTL_TRDBG(0,"PKT board%d send error=%d\n",board,ret);
		return	-2;
	}
	LgwNbPacketSend++;
	LGW_STATUS(board,TX_STATUS,&txstatus);
	if	(downpkt->lp_lgwdelay == 0 && txstatus != TX_EMITTING)
	{
		RTL_TRDBG(0,"PKT board%d SEND status(=%d) != TX_EMITTING\n",board,txstatus);
	}
	if	(0 && downpkt->lp_lgwdelay && txstatus != TX_SCHEDULED)	// TODO
	{
		RTL_TRDBG(0,"PKT board%d SEND status(=%d) != TX_SCHEDULED\n",board,txstatus);
	}

#ifdef LP_TP31
	// must be done only if packet was really sent
	DcTreatDownlink(downpkt);
#endif

RTL_TRDBG(1,"PKT SEND board%d tms=%09u/%d rfchain=%d status=%d sz=%d left=%d freq=%d mod=0x%02x bdw=%s spf=%s ecc=%s pr=%d nocrc=%d ivp=%d pw=%d temppw=%d\n",
	board,tms,diff,txpkt.rf_chain,txstatus,txpkt.size,left,txpkt.freq_hz,
	txpkt.modulation,BandWidthTxt(txpkt.bandwidth),
	SpreadingFactorTxt(txpkt.datarate),CorrectingCodeTxt(txpkt.coderate),
	txpkt.preamble,txpkt.no_crc,txpkt.invert_pol,txpkt.rf_power,tempgain);


	if	(downpkt->lp_lgwdelay == 0)
	{
		duration	= WaitSendPacket(board,blo,tms,&txstatus);
	}

	// the packet is now passed to the sx13, compute time added by all
	// treatements LRC/LRR, only if lgwdelay or non blocking mode

	diff	= 0;
	if	(downpkt->lp_lgwdelay || (!blo && !LgwWaitSending))
	{
		u_int	scheduled;	// time before sx13 schedule request

		diff	= ABS(rtl_tmmsmono() - downpkt->lp_tms);

		scheduled		= downpkt->lp_lgwdelay;
		LastTmoaBoard		= board;
		LastTmoaRequested	= ceil(downpkt->lp_tmoa/1000);
		LastTmoaRequested	= LastTmoaRequested +
					(LastTmoaRequested * 10)/100;
		RTL_TRDBG(1,"LGW DELAY tmao request=%dms + sched=%dms\n",
				LastTmoaRequested,scheduled);
		LastTmoaRequested	= LastTmoaRequested + scheduled + 70;
		CurrTmoaRequested	= LastTmoaRequested;
	}
	else
	{
	// TODO meme si no delay il faut compter la requete
		LastTmoaBoard		= board;
		LastTmoaRequested	= 0;
		CurrTmoaRequested	= 0;
	}

	if	(TraceLevel >= 1)
	{
		char	buff[1024];
		char	src[64];
		LoRaMAC_t	mf;
		u_char		*pt;

		memset	(&mf,0,sizeof(mf));
		// we assumed this is a loramac packet
		LoRaMAC_decodeHeader(txpkt.payload,txpkt.size,&mf);	
		pt	= (u_char *)&mf.DevAddr;
		sprintf	(src,"%02x%02x%02x%02x",*(pt+3),*(pt+2),*(pt+1),*pt);

		rtl_binToStr(txpkt.payload,txpkt.size,buff,sizeof(buff)-10);
		if	(downpkt->lp_lgwdelay == 0)
		{
			if	(blo || LgwWaitSending)
			{
RTL_TRDBG(1,"PKT SEND blocking dur=%ums data='%s' seq=%d devaddr=%s\n",
				duration,buff,mf.FCnt,src);
			}
			else
			{
RTL_TRDBG(1,"PKT SEND noblock dur=%fms diff=%ums data='%s' seq=%d devaddr=%s\n",
	downpkt->lp_tmoa/1000,diff,buff,mf.FCnt,src);
			}
		}
		else
		{
RTL_TRDBG(1,"PKT SEND async dur=%fms diff=%ums data='%s' seq=%d devaddr=%s\n",
	downpkt->lp_tmoa/1000,diff,buff,mf.FCnt,src);
		}
	}

	DoPcap((char *)txpkt.payload,txpkt.size);

	return	0;
}

static	void	SetTrigTarget(t_lrr_pkt *downpkt,T_lgw_pkt_tx_t *txpkt)
{
#ifdef	WITH_GPS
	struct timespec utc_time;
	struct timespec utc_time2;
#endif
	uint32_t	diffusraw = 0;
	uint32_t	diffus = 0;
	
	if	(downpkt->lp_lgwdelay == 0)
	{
		txpkt->tx_mode 	= IMMEDIATE;
		RTL_TRDBG(1,"LRR DELAY -> tx mode immediate\n");
		return;
	}

	txpkt->tx_mode 	= TIMESTAMPED;

#if	0	
	// do not use lgw_get_trigcnt() when possible it causes a lot of
	// troubles on some system (mainly with USB ...). So when we do not use
	// GPS correction is not necessary to call lgw_get_trigcnt()
	lgw_get_trigcnt(&(txpkt->count_us));	// now
	diffusraw	= ABS(txpkt->count_us - downpkt->lp_tus);
#endif

#ifdef	WITH_GPS
	if	(0 && UseGpsTime && Gps_ref_valid)
	{	// we have GPS time => trig correction
		LGW_GET_TRIGCNT(0,&(txpkt->count_us));	// now
		diffusraw	= ABS(txpkt->count_us - downpkt->lp_tus);
		utc_time.tv_sec		= downpkt->lp_gss;
		utc_time.tv_nsec	= downpkt->lp_gns;

		utc_time2.tv_sec	= 0;
		utc_time2.tv_nsec	= downpkt->lp_delay * 1000 * 1000;

		ADD_TIMESPEC(utc_time,utc_time2);
		if(lgw_utc2cnt(Time_reference_gps,utc_time,&(txpkt->count_us)) 
						!= LGW_GPS_SUCCESS)
		{
			RTL_TRDBG(0,"lgw_utc2cnt() error\n");
			return;
		}
	}
	else
#endif
	{	// no GPS time => no trig correction
		txpkt->count_us	= downpkt->lp_tus;
		diffusraw	= 0;
		txpkt->count_us	= txpkt->count_us + (downpkt->lp_delay * 1000);
		txpkt->count_us	= txpkt->count_us - diffusraw;
		txpkt->count_us	= txpkt->count_us - Sx13xxStartDelay;

	}
	diffus	= ABS(txpkt->count_us - downpkt->lp_tus);
RTL_TRDBG(3,"LGW DELAY trigtarget=%u trigorig=%u rawdelayus=%u gpsdelayus=%u\n",
	txpkt->count_us,downpkt->lp_tus,diffusraw,diffus);

	return;

#if	0
	if	(diffus > (uint32_t)4000000000U)
		diffus	= UINT_MAX - diffus;

	if	(diffus > LgwMxDiffTrig)
		LgwMxDiffTrig	= diffus;

	if	(diffusraw > (uint32_t)4000000000U)
		diffusraw	= UINT_MAX - diffusraw;
	if	(diffusraw > LgwMxDiffTrigRaw)
		LgwMxDiffTrigRaw	= diffusraw;

RTL_TRDBG(1,"LGW DELAY trigtarget=%u trigorig=%u delayus=%u maxdelayus=%u rawdelayus=%u rawmaxdelayus=%u\n",
	txpkt->count_us,downpkt->lp_tus,diffus,LgwMxDiffTrig,
					diffusraw,LgwMxDiffTrigRaw);
#endif
}

static	int	SendPacket(t_imsg  *msg)
{
	t_lrr_pkt	*downpkt;
	T_lgw_pkt_tx_t 	txpkt;
	t_channel	*chan;
	int	ret;
	int	blo	= 0;
	int	board;
	int	chain;
#ifndef USELIBLGW3
	bool	txenable[]	= LGW_RF_TX_ENABLE;
#endif

	downpkt	= msg->im_dataptr;

	if	(downpkt->lp_channel >= MaxChannel ||
		TbChannel[downpkt->lp_channel].name[0] == '\0')
	{
		LgwNbChanDownError++;	
		return	0;
	}
	chan	= &TbChannel[downpkt->lp_channel];

	memset	(&txpkt,0,sizeof(txpkt));

//	txpkt.freq_hz = 868100000;
//	txpkt.modulation = MOD_LORA;
//	txpkt.bandwidth = BW_125KHZ;
//	txpkt.datarate = DR_LORA_SF10;
//	txpkt.coderate = CR_LORA_4_5;

	board	= downpkt->lp_chain>>4;
	chain	= downpkt->lp_chain & 0x01;
	if	(chain >= LGW_RF_CHAIN_NB)
	{
		chain	= 0;
	}
#ifdef USELIBLGW3
	if	(LgwTxEnable[chain] == 0)
#else
	if	(txenable[chain] == 0)
#endif
	{
		RTL_TRDBG(1,"TX is disable on chain=%d => 0\n",
							downpkt->lp_chain);
		downpkt->lp_chain	= 0;
	}

	SetTrigTarget(downpkt,&txpkt);
	txpkt.rf_chain 		= (board<<1) | chain;
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
		ret	= SendPacketNow(blo=1,downpkt,txpkt);
		if	(ret < 0)
		{
			RTL_TRDBG(0,"SendPacketNow() error => %d\n",ret);
		}
		usleep(LgwAckData802Wait*1000);	// 10 ms
	}

	txpkt.size = downpkt->lp_size;
	memcpy	(txpkt.payload,downpkt->lp_payload,downpkt->lp_size);
	free	(downpkt->lp_payload);
	downpkt->lp_payload	= NULL;


	ret	= SendPacketNow(blo=0,downpkt,txpkt);
	if	(ret < 0)
	{
		RTL_TRDBG(0,"SendPacketNow() error => %d\n",ret);
	}

	return	1;
}

int	LgwDoSendPacket(time_t now)
{
	t_imsg	*msg;
	int	nbs	= 0;
	uint8_t txstatus = 0;
	int	left = 0;
	int	ret;
	int	board;

#if	0
	static	u_int	notfree	= 0;
	static	time_t	lastnotfree;

	if	(rtl_imsgCount(LgwSendQ) <= 0)
		continue;

	// do not pickup a new message if status != FREE
	LGW_STATUS(board,TX_STATUS,&txstatus);
	if	(txstatus != TX_FREE)
	{
//		RTL_TRDBG(4,"LGW busy status=%d do not peek packet\n",txstatus);
		if	(notfree == 0)
		{
			notfree	= 1;
			lastnotfree	= now;
			RTL_TRDBG(1,"BOARD%d LGW NOT FREE at=%u\n",board,now);
			return	0;
		}
		if	(ABS(now - lastnotfree) > 30000)
		{
			RTL_TRDBG(0,"BOARD%d LGW NOT FREE more than 30000ms status=%d\n"
				,board,txstatus);
			// ask main thread to restart LGW
			// does reset sx13 lgw_stop() correct the problem ?
			notfree	= 0;
			lgw_abort_tx(board);
			rtl_imsgAdd(MainQ,
				rtl_imsgAlloc(IM_DEF,IM_LGW_LINK_DOWN,NULL,0));
			LgwNbLinkDown++;
			return	0;
		}
		if	(ABS(now - lastnotfree) > 10000)
		{
			RTL_TRDBG(0,"BOARD%d LGW NOT FREE more than 10000ms status=%d\n"
				,board,txstatus);
			lgw_abort_tx(board);
			return	0;
		}
		return	0;
	}
	if	(notfree)
		RTL_TRDBG(1,"BOARD%d LGW FREE at=%u dur=%d\n",board,now,ABS(now-lastnotfree));
	notfree	= 0;
#endif
	if	((msg= rtl_imsgGet(LgwSendQ,IMSG_BOTH)) != NULL)
	{
		t_lrr_pkt       *downpkt;

		downpkt	= msg->im_dataptr;
		left	= rtl_imsgCount(LgwSendQ);
		if	(left > 0 && left > LgwNbPacketWait)
		{
			LgwNbPacketWait	= left;
		}
		if	(downpkt->lp_delay == 0 && CurrTmoaRequested)
		{	// mode immediate
			if	(ABS(now - downpkt->lp_tms) > MaxReportDnImmediat)
			{
		RTL_TRDBG(1,"PKT SEND NODELAY not sent after 3s => dropped\n");
				SetIndic(downpkt,0,LP_C1_MAXTRY,LP_C2_MAXTRY,-1);
				SendIndicToLrc(downpkt);
				rtl_imsgFree(msg);
				return	0;
			}
		RTL_TRDBG(1,"PKT SEND NODELAY avoid collision repostpone=%d\n",
						CurrTmoaRequested);
			rtl_imsgAddDelayed(LgwSendQ,msg,CurrTmoaRequested+3);
			return	0;
		}
#if	0
		LGW_STATUS(TX_STATUS,&txstatus);
		if	(txstatus != TX_FREE)
		{
			RTL_TRDBG(1,"PKT busy status=%d left=%d\n",txstatus,left-1);
			LgwNbBusySend++;
			// TODO dont free msg requeue it with nodelay ?
			rtl_imsgFree(msg);
			return	nbs;
		}
		notfree	= 0;
#endif
		ret	= SendPacket(msg);
		if	(ret > 0)
		{
			SetIndic(downpkt,1,-1,-1,-1);
			nbs	+= ret;
		}
		else
			SetIndic(downpkt,0,-1,-1,-1);
		SendIndicToLrc(downpkt);
		rtl_imsgFree(msg);
	}

	return	nbs;
}

