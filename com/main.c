
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
#include <sys/vfs.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <dirent.h>
#ifdef	WITH_GPS
#include <termios.h>
#endif

#include "rtlbase.h"
#include "rtlimsg.h"
#include "rtllist.h"
#include "rtlhtbl.h"
#include "rtlmd5.h"

#include "semtech.h"

#include "xlap.h"
#include "infrastruct.h"
#include "struct.h"

#include <resolv.h>

#include "headerloramac.h"
#include "networkfilter.h"

#ifndef	MIN
#define        MIN(a, b)       ((a) < (b) ? (a) : (b))
#endif

typedef unsigned char u8;
typedef unsigned short u16;
u16 crc_ccitt(u16 crc, const u8 *buffer, int len);
int	NtpdStarted();

#include "_whatstr.h"
#include "define.h"
#include "cproto.h"
#include "extern.h"

int 		SynchroForward	= 0;	// as "macro" GW do we forward frame

static	int	SendToLrc(t_lrr_pkt *uppkt,u_char *buff,int sz);
static	void	SendDtcSyncToLrc(t_xlap_link *lktarget);
static	int	SendStatToAllLrc(u_char *buff,int sz,int delay);
static	int	RecvRadioPacketStore(t_imsg *msg,t_lrr_pkt *uppkt,char *from);
static	void	LrcSaveStatus(int evt);
static	t_xlap_link	*LrcFirstStarted();
static	void	ReadWifiState();

char	*ExtLrrRestart		= "/tmp/lrrrestart";
char	*ExtRadioRestart	= "/tmp/lrrradiorestart";
char	*ExtConfigRefresh	= "/tmp/lrrconfigrefresh";

#ifdef	WITH_TRACK_TIC
#define	TRACKTICNB	100
u_int	TrackTicNb	= TRACKTICNB;
u_int	TrackTicCnt;
u_int	TrackTic[TRACKTICNB][2];
#endif

u_int	VersionMaj;
u_int	VersionMin;
u_int	VersionRel;
u_int	VersionFix;

u_int	VersionMajRff;
u_int	VersionMinRff;
u_int	VersionRelRff;
u_int	VersionFixRff;
time_t	RffTime;

int			PingRttPeriod	= 60;		// sec
int			AutoRebootTimer	= 14400;	// 4*3600
int			AutoRestoreNoLrc = 12;
int			AutoRebootTimerNoUplink	= 3600;	// 3600 (1 hour)
int			AutoRestartLrrMaxNoUplink = 3; 
int			AutoRevSshTimer	= 1200;		// 20 mn
int			IfaceDaemon	= 0;
int			LapTest		= 0;
int			LapTestDuration	= 43200;	// 12*3600

int			Sx13xxPolling	= 10;	// ms
#ifdef	USELIBLGW3
int			Sx13xxStartDelay= 0;	// ms
#else
int			Sx13xxStartDelay= SX13XX_START_DELAY;	// ms
#endif

#if defined(REF_DESIGN_V2)
char			*SpiDevice[SX1301AR_MAX_BOARD_NB];
#else

#if defined(WITH_MULTI_BOARD)
char            *SpiDevice[MAX_BOARD];
#else
char			*SpiDevice	= "/dev/spidev1.0";
#endif // WITH_MULTI_BOARD
#endif // REF_DESIGN_V2

#ifdef	WITH_TTY
char			*TtyDevice	= "/dev/ttyACM0";
#endif // WITH_TTY

int			CfgRadioState	= 0;	// config flag save CfgRadio
int			CfgRadioStop	= 0;	// config flag
int			CfgRadioDnStop	= 0;	// config flag

int			DownRadioStop	= 0;	// state of downlink radio
int			MainWantStop	= 0;
int			MainStopDelay	= 5;
// RDTP-12898
int			WifiStateReport	= 30;		// activate the feature of wifi state reporting, check each 30 s
char			WifiStateSsid[128];		// SSID
float			WifiStateRssi	= 0;		// Last average RSSI calculated
int			WifiStateRssiCount = 0;		// Last average RSSI calculated
int			WifiStateItf = 5;		// Itf state, 0=running, 5=down
float			*WSSample	= NULL;		// store values to make an average
int			WSSampleSize	= 10;		// max values that can be stored
int			WSSampleCount	= 0;		// number of values actually stored
int			WSSampleIdx	= 0;		// next index to use
int			WSSampleTimeout	= 120;		// expiration duration in seconds for samples
#define		WIFISTATESH	"wifistatecheck.sh"
#define		WIFISTATESHDIR	"lrr/com/shells"
#define		WIFISTATEFILE	"usr/data/lrr/_statewifi"
#define		FAILOVERSTATEFILE	"usr/data/lrr/_statefailover"
#define		FAILOVERSTATERESCUE	"usr/data/lrr/_statefailoverrescue"

float			AntennaGain[NB_ANTENNA];
float			CableLoss[NB_ANTENNA];

/* NFR590, RDTP-7849 */
char			* AutoVersion_Lrr;
char			* AutoVersion_LrrId;  
char			* AutoVersion_Hal;
char			* AutoVersion_Fpga;
char			* AutoVersion_Fw;
char 			* AutoVersion_Hw;
char 			* AutoVersion_Os;
char			* AutoVersion_Sn;
char			* AutoVersion_Sku;
char			* AutoVersion_ChipMsb;
char			* AutoVersion_ChipLsb;
char 			* CustomVersion_Build;
char 			* CustomVersion_Config;
char 			* CustomVersion_Custom1;
char 			* CustomVersion_Custom2;
char 			* CustomVersion_Custom3;

int			NbItf;
t_wan_itf		TbItf[NB_ITF_PER_LRR];

int			NbMfs;
t_mfs			TbMfs[NB_MFS_PER_LRR];

u_int			MemTotal;
u_int			MemFree;
u_int			MemBuffers;
u_int			MemCached;
u_int			MemUsed;

t_xlap_link		TbLapLrc[NB_LRC_PER_LRR];
struct	list_head	*LapList;
u_int			LapFlags	= LK_TCP_CLIENT|LK_SSP_SLAVE|
					LK_TCP_RECONN|LK_LNK_SAVEDNSENT;

t_lrc_link		TbLrc[NB_LRC_PER_LRR];
int			MasterLrc = -1;


t_lrr_config	ConfigIpInt;

t_avdv_ui	CmnLrcAvdvTrip;
t_avdv_ui	CpuAvdvUsed;
u_short		CpuLoad1;
u_short		CpuLoad5;
u_short		CpuLoad15;

int		PowerEnable	= 0;
char		*PowerDevice	= "";
int		PowerDownLev	= 0;
int		PowerUpLev	= 0;
int		PowerState	= '?';		// 'U' || 'D' : up || down
u_int		PowerDownCnt	= 0;
u_int		PowerDownCntP	= 0;

int		TempEnable	= 0;
int		TempPowerAdjust	= 0;
int		TempExtAdjust	= 0;
char		*TempDevice	= "";
int		CurrTemp	= -273;


int		CurrLrcOrder	= -1;		// current lrc/order index 

unsigned int	LrcNbPktTrip;
unsigned int	LrcAvPktTrip;
unsigned int	LrcDvPktTrip;
unsigned int	LrcMxPktTrip;
unsigned int	LrcNbDisc;

struct	timespec	Currtime;	// updated each sec in DoClockSc

struct timespec 	UptimeSyst;	// uptime system from /proc/uptime
char			UptimeSystStr[128];
struct timespec 	UptimeProc;	// uptime process
char			UptimeProcStr[128];

u_char	SickRestart	= 0;
int	GpsPositionOk	= 0;
float	GpsLatt		= 0;
float	GpsLong		= 0;
int	GpsAlti		= 0;
u_int	GpsUpdateCnt	= 0;
u_int	GpsUpdateCntP	= 0;
u_int	GpsDownCnt	= 0;
u_int	GpsDownCntP	= 0;
u_int	GpsUpCnt	= 0;
u_int	GpsMaxBadChecksum = 1000; /* PT-1008: 1000=~14mn before reset (1 RMC and i GGA processed at each loop */
u_int	GpsMaxBadRead = 300;      /* PT-1509: 300=~5mn before reset (1 read per second) */
u_int	GpsUpCntP	= 0;
u_char	GpsStatus	= '?';		/* 'U' || 'D' : up || down */
u_char	GpsUtcLeap	= 18;		// 18 = leap value the 2018/01/18
#if defined(KONA) && defined(WITH_GPS)
int	GpsNbCheckTektelic	= 0;
#endif
int			AdjustFineTimeStamp = 0; /* PT-1308: finetimestamp adjust to the end of the packet */

int	StatRefresh	= 3600;
int	RfCellRefresh	= 300;
int	SendRfCellNow	= 0;
int	ConfigRefresh	= 3600*24;
int	WanRefresh	= 300;
int	GpsStRefresh	= 30;
float	GpsStRateLow	= 0.85;
float	GpsStRateHigh	= 0.90;
char	*RootAct	= NULL;
char	*System		= NULL;
int	ServiceStopped	= 0;
int	LapState	= 0;

char    *TraceFile      = NULL;
char    *TraceStdout    = NULL;
int     TraceSize       = RTL_TRUNLIMITED; // no limit until conffiles are loaded
int     TraceLevel      = 0;
int     TraceLevelP	= -1;		// level persistent, -1 = not set => use value set in lrr.ini
int     TraceDebug      = 0;

char	*NtpDaemonIp    = NULL;

char	ConfigDefault[PATH_MAX];
char	ConfigCustom[PATH_MAX];
char	DirCommands[PATH_MAX];

char	*IsmBand		= "eu868";
char	*IsmBandAlter		= "";
int	IsmFreq			= 868;
int	IsmAsymDownlink		= 0;
int	IsmAsymDnModulo		= 8;
int	IsmAsymDnOffset		= 0;
char	*RfRegionId		= NULL;		// not "" can reallocated
u_int	RfRegionIdVers		= 0;

char	*ConfigFileParams	= "_parameters.sh";
char	*ConfigFileSystem	= "_system.sh";
char	*ConfigFileCustom	= "custom.ini";
char	*ConfigFileDynCalib	= "dyncalib.ini";
char	*ConfigFileDynLap	= "dynlap.ini";
char	*ConfigFileGps		= "gpsman.ini";
char	*ConfigFileDefine	= "defines.ini";
char	*ConfigFileLrr		= "lrr.ini";
char    *ConfigFileVersions     = "versions.ini";
char	*ConfigFileChannel	= "channels.ini";
char	*ConfigFileState	= "_state.ini";
#if	defined(WITH_SX1301_X1)
char	*ConfigFileLgw		= "lgw.ini";
#endif
#ifdef	WITH_SX1301_X8
char	*ConfigFileLgw		= "lgwx8.ini";
char	*ConfigFileLgwCustom	= "lgw.ini";
#endif

char	*ConfigFileLowLvLgw	= "lowlvlgw.ini";

char	*ConfigFileBootSrv	= "bootserver.ini";


pthread_t	MainThreadId;

void	*HtVarLrr;	// hash table for variables LRR lrr.ini
void	*HtVarLgw;	// hash table for variables LGW lgw.ini
void	*HtVarSys;	// hash table for variables "system"
void	*MainQ;		// command/timer/data messages queue for main thread
void	*LgwQ;		// command/timer messages queue for lora gateway thread
#ifdef REF_DESIGN_V2
void	*LgwSendQ[SX1301AR_MAX_BOARD_NB];	// data messages queue for lora gateway thread
#else
#define LGW_MAX_BOARD   2
void	*LgwSendQ[LGW_MAX_BOARD];	// data messages queue for lora gateway thread
#endif
void	*StoreQ;	// uplink radio packet storage
int	DiskSaveStoreQ	= 0;

void	*MainTbPoll;

#ifdef	LP_TP31
int	StorePktCount	= 30000;
#else
int	StorePktCount	= 0;
#endif
float	StoreMemUsed	= 50;	// 50%
int	ReStorePerSec	= 3;	// possible values 10,5,3,2,1 (DoClockMs())
int	ReStoreCtrlOutQ	= 1;
int	ReStoreCtrlAckQ	= 1;
//	FIX3323
int	MaxStorePktCount;	// max reach on wan refresh period
time_t	MaxStorePktTime;	// time of max

pthread_attr_t	LgwThreadAt;
pthread_t	LgwThread;
int		LgwThreadStarted;
int		LgwThreadStopped;
int		LgwUsbIndex	= 0;
pthread_attr_t	GpsThreadAt;
pthread_t	GpsThread;
#ifdef	WITH_USB	// check connexion with the board does not work
int		LgwLinkUp	= 0;
#else
int		LgwLinkUp	= 0;
#endif

int		MacWithBeacon	= 0;
int		MacWithFcsDown	= 0;
int		MacWithFcsUp	= 0;
unsigned int	MacNbFcsError;

pthread_attr_t	CellStateThreadAt;
pthread_t	CellStateThread;

u_int	LrcNbPktDrop;
u_int	LgwNbFilterDrop;
u_int	NwkfEnable		= 1;
u_int	NwkfReloadPeriod	= 300;
u_int	NwkfRequestPeriod	= 3600;
u_int	NwkfRequestStart	= 300;
u_int	NwkfFilterSize		= 0;
char	NwkfFilter[NF_MAX_SIZE+1];
u_int	NwkfFilterRefreshCmd	= 0;

int	OnConfigErrorExit	= 0;
int	OnStartErrorExit	= 0;


u_short			AutoRevSshPort	= 0;
int			AdjustDelay	= 0;
u_int			NbLrc		= 0;
u_int			LrrID		= 0;
u_short			LrrIDExt	= 0;
u_char			LrrIDPref	= 0;
u_char			LrrIDGetFromTwa	= 0;	// NFR684: get lrrid from TWA
u_int			LrrIDFromTwaGot	= 0;	// NFR684: lrrid got from TWA
u_int			LrrIDGetFromBS	= 0;	// NFR997
u_int			LrrIDFromBS	= 0;
float			LrrLat		= 90.0;
float			LrrLon		= 90.0;
int			LrrAlt		= 0;
u_int			UseGps		= 0;
u_int			UseGpsPosition	= 0;
u_int			UseGpsTime	= 0;
u_int			UseLgwTime	= 1;	// RDTP-6172 0 -> 1
u_int			DlShiftLc	= 0;	// RDTP-5911
u_int			Redundancy	= 0;
u_int			SimulN		= 0;
u_int			SimulationMode	= 0;	// Simulation mode activated
u_int			QosAlea		= 0;
u_int			TbLrrID[NB_LRC_PER_LRR];
float			TbLrrLat[NB_LRC_PER_LRR];
float			TbLrrLon[NB_LRC_PER_LRR];
int			TbLrrAlt[NB_LRC_PER_LRR];

u_int			TbDevAddrIncl[10];
int			NbDevAddrIncl;
u_int			TbDevAddrExcl[10];
int			NbDevAddrExcl;

int			MaxReportDnImmediat	= 60000;

int			LogUseRamDir	= 0;

static	int	RecvRadioPacket(t_lrr_pkt *uppkt,int late);
static	void	StartLgwThread();
static	void	CancelLgwThread();
#ifdef WITH_GPS
static	void	StartGpsThread();
static	void	CancelGpsThread();
#endif /* WITH_GPS */
static  void    DoHtmlFile();

static	int	DevAddrIncl(u_int   devaddr)
{
	int	i;

	if	(NbDevAddrIncl <= 0)
		return	1;

	for	(i = 0 ; i < NbDevAddrIncl ; i++)
	{
		if	(devaddr == TbDevAddrIncl[i])
			return	1;
	}
	return	0;
}

static	int	DevAddrExcl(u_int   devaddr)
{
	int	i;

	if	(NbDevAddrExcl <= 0)
		return	0;

	for	(i = 0 ; i < NbDevAddrExcl ; i++)
	{
		if	(devaddr == TbDevAddrExcl[i])
			return	1;
	}
	return	0;
}

int	OkDevAddr(char *dev)
{
	u_int	devaddr;

	devaddr	= (u_int)strtoul(dev,0,16);
	if	(DevAddrExcl(devaddr) == 1)	// excluded
	{
	RTL_TRDBG(1,"devaddr %08x excluded in configuration\n",devaddr);
	return	0;
	}
	if	(DevAddrIncl(devaddr) == 0)	// included
	{
	RTL_TRDBG(1,"devaddr %08x not included in configuration\n",devaddr);
	return	0;
	}
	return	1;
}

static void SendNetworkFilterRequest(t_xlap_link *lk)
{
	t_lrr_pkt	uppkt;

	if	(!lk)
		return;
	if	(LrrID == 0)
		return;
	memset	(&uppkt,0,sizeof(t_lrr_pkt));
	uppkt.lp_vers	= LP_LRX_VERSION;
	uppkt.lp_lrrid	= LrrID;
	uppkt.lp_flag	= LP_INFRA_PKT_INFRA;
	uppkt.lp_type	= LP_TYPE_LRR_INF_NETWORKFILTER_REQ;
	uppkt.lp_szh	= LP_PRE_HEADER_PKT_SIZE+sizeof(t_lrr_networkfilterReq);

	uppkt.lp_u.lp_networkfilterReq.nf_version	= NfVersion;
	uppkt.lp_u.lp_networkfilterReq.nf_netid		= NfNetwork;

	LapPutOutQueue(lk,(u_char *)&uppkt,uppkt.lp_szh);
	RTL_TRDBG(1,"NWF request version=%u netid=%06x\n", 
		NfVersion,NfNetwork);
}

void SendNetworkFilterRequestCmd()
{
	if	(NwkfEnable)
	{
		NfVersion	= 0;
		NfNetwork	= 0;
		NwkfFilterRefreshCmd	= 1;
		SendNetworkFilterRequest(LrcFirstStarted());
	}
}

// rdtp-6639
static	void DoLrrUIDAutoRevSshPort()
{
	char	uid[1024];
	char	*pt;

	uid[0]	= '\0';
	// get LRRGID environment variable
	pt = CfgStr(HtVarSys,"",-1,"LRROUI","");
	if (!pt || !*pt)
		return;
	strncat(uid,pt,sizeof(uid)-strlen(uid)-1);
	strncat(uid,"-",sizeof(uid)-strlen(uid)-1);
	// get LRROUI environment variable
	pt = CfgStr(HtVarSys,"",-1,"LRRGID","");
	if (!pt || !*pt)
		return;
	strncat(uid,pt,sizeof(uid)-strlen(uid)-1);

	unsigned char digest[32];	// 16 bytes
	unsigned int index = 0;
#if	0
#include <openssl/md5.h>
	MD5((unsigned char *)uid,strlen(uid),digest);
#else
	rtl_MD5_Digest((unsigned char *)uid,strlen(uid),digest);
#endif
	index = 
	(digest[0] << 24)|(digest[1] << 16)|(digest[2] << 8)|digest[3];
	AutoRevSshPort	= 50000 + (index % 10000);

	char hexadigest[128];
	rtl_binToStr(digest,16,hexadigest,sizeof(hexadigest)-1);

	RTL_TRDBG(1,"MD5 on LRRUID='%s'\n",uid);
	RTL_TRDBG(1,"MD5(LRRUID)='%s'\n",hexadigest);
	RTL_TRDBG(1,"MD5(LRRUID)/32=%08x => AutoReverseSshPort=%u\n",
							index,AutoRevSshPort);
}


static void SendLrrUID(t_xlap_link *lk)
{
	t_lrr_pkt	uppkt;
	t_lrc_link	*lrc;
	char		*pt = NULL;
	int		len;

	lrc	= lk->lk_userptr;
	memset	(&uppkt,0,sizeof(t_lrr_pkt));
	uppkt.lp_vers	= LP_LRX_VERSION;
	uppkt.lp_flag	= LP_INFRA_PKT_INFRA;
	// get LRRGID environment variable
	pt = CfgStr(HtVarSys,"",-1,"LRRGID","");
	if (pt && *pt)
	{
		len = MIN(sizeof(uppkt.lp_gwuid)-1, strlen(pt));
		strncpy((char *)uppkt.lp_gwuid, pt, len);
		uppkt.lp_gwuid[len] = '\0';
	}
	// get LRROUI environment variable
	pt = CfgStr(HtVarSys,"",-1,"LRROUI","");
	if (pt && *pt)
	{
		len = MIN(sizeof(uppkt.lp_oui)-1, strlen(pt));
		strncpy((char *)uppkt.lp_oui, pt, len);
		uppkt.lp_oui[len] = '\0';
	}

	uppkt.lp_type	= LP_TYPE_LRR_INF_LRRUID;
	uppkt.lp_szh	= LP_PRE_HEADER_PKT_SIZE+sizeof(t_lrr_lrruid);
	LapPutOutQueue(lk,(u_char *)&uppkt,uppkt.lp_szh);
	RTL_TRDBG(1,"LRR send lrruid='%s-%s' to lrc='%s' twa sz=%d idxlrc=%d\n",
	uppkt.lp_oui,uppkt.lp_gwuid, lk->lk_rhost,uppkt.lp_szh,lrc->lrc_idx);
	// set a timer in case lrc do not handle this message
	rtl_imsgAdd(MainQ,
	rtl_timerAlloc(IM_DEF,IM_TIMER_LRRUID_RESP,IM_TIMER_LRRUID_RESP_V,lrc,0));
	if	(lrc)
	{	//  RDTP-9756/7649
		lrc->lrc_twaresp	= 0;
		lrc->lrc_twalrridresp	= 0;
	}
}

void	DoLrrUIDConfigRefresh(int delay)
{
	t_lrr_pkt	uppkt;
	char		*pt = NULL;
	int		len;

	memset	(&uppkt,0,sizeof(t_lrr_pkt));
	uppkt.lp_vers	= LP_LRX_VERSION;
	uppkt.lp_flag	= LP_INFRA_PKT_INFRA;
	uppkt.lp_lrrid	= LrrID;
	// get LRRGID environment variable
	pt = CfgStr(HtVarSys,"",-1,"LRRGID","");
	if (pt && *pt)
	{
		len = MIN(sizeof(uppkt.lp_gwuid)-1, strlen(pt));
		strncpy((char *)uppkt.lp_gwuid, pt, len);
		uppkt.lp_gwuid[len] = '\0';
	}
	// get LRROUI environment variable
	pt = CfgStr(HtVarSys,"",-1,"LRROUI","");
	if (pt && *pt)
	{
		len = MIN(sizeof(uppkt.lp_oui)-1, strlen(pt));
		strncpy((char *)uppkt.lp_oui, pt, len);
		uppkt.lp_oui[len] = '\0';
	}

	uppkt.lp_type	= LP_TYPE_LRR_INF_CONFIG_LRRUID;
	uppkt.lp_szh	= LP_PRE_HEADER_PKT_SIZE+sizeof(t_lrr_config_lrruid);
	RTL_TRDBG(1,"LRR send config lrruid='%s-%s' sz=%d\n", 
		uppkt.lp_oui,uppkt.lp_gwuid, uppkt.lp_szh);

	SendStatToAllLrc((u_char *)&uppkt,uppkt.lp_szh,delay);
}

#if defined(KONA) && defined(WITH_GPS)
// prototype required to avoid a warning
char *strcasestr(const char *haystack, const char *needle);

// File "/tmp/position" contains:
//   Latitude: 4548.39528N
//   Longtitude: 00445.78207E	<= yes it is 'Longtitude' !
//   Altitude: 295.4
void GetGpsPositionTektelic()
{
	char	buf[80], *pt;
	char	*fn = "/tmp/position";
	FILE	*f;
	short	fp;
	double	sp;

	f = fopen(fn, "r");
	if (f)
	{
		/* PT-1714: /tmp/position is the only way to know BS has GPS, since serial connection is not opened on Tektelic */
		/* Set GpsFd to value different from -1 to report GPS present */
		GpsFd = 0;
		GpsPositionOk = 0;
		while (fgets(buf, sizeof(buf)-1, f))
		{
			if ((pt = strcasestr(buf, "latitude:")))
			{
				pt += 9;
				sscanf(pt, "%2hd%lf", &fp, &sp);
				GpsLatt = (double)fp + (sp/60);
				if (strchr(pt, 'S'))
					GpsLatt *= -1;
			}
			if ((pt = strcasestr(buf, "longitude:")))
			{
				pt += 11;
				sscanf(pt, "%3hd%lf", &fp, &sp);
				GpsLong = (double)fp + (sp/60);
				if (strchr(pt, 'W'))
					GpsLong *= -1;
			}
			if ((pt = strcasestr(buf, "altitude:")))
			{
				pt += 9;
				GpsAlti = atoi(pt);
			}
			if ((pt = strcasestr(buf, "lock:")))
			{
				pt += 5;
				if (strcasestr(pt, "yes"))
					GpsPositionOk = 1;
			}
		}
		fclose(f);
		RTL_TRDBG(3,"GetGpsPositionTektelic: lat=%.2f long=%.2f, alt=%d, ok=%d\n",
			GpsLatt, GpsLong, GpsAlti, GpsPositionOk);
	}
	else
	{
		GpsFd = -1;
		RTL_TRDBG(3,"GetGpsPositionTektelic: can not open '%s' file !\n", fn);
		GpsPositionOk = 0;
	}
	GpsNbCheckTektelic++;
	if (GpsPositionOk)
		GpsUpdateCnt += 1;
}
#endif

// RDTP-9756/7649
// check if both LRCs have returned the same LRRID
static uint32_t CheckLrrIdTwa(uint32_t lrrid)
{
	int	i;
	t_lrc_link	*lrc;

	for	(i = 0 ; i < NbLrc ; i++)
	{
		lrc	= &TbLrc[i];
		if	(lrc && lrc->lrc_twaresp && lrc->lrc_twalrridresp)
		{
			if	(lrc->lrc_twalrridresp != lrrid)
			{
		RTL_TRDBG(0,"Getting different lrrid from twa %08x!=%08x\n",
				lrc->lrc_twalrridresp,lrrid);
				return	0;
			}
		}
	}
	return	lrrid;
}

static void SendXlap_Cfg(t_xlap_link *lk)	// [2359]
{
	t_lrr_pkt	uppkt;
	u_int		t1;
	u_int		t2;
	u_int		t3;

	t1	= (lk->lk_t1 == LRR_DEFAULT_T1);
	t2	= (lk->lk_t2 == LRR_DEFAULT_T2);
	t3	= (lk->lk_t3 == LRR_DEFAULT_T3);

	if	(t1 && t2 && t3)
	{	// do not send timers values if default are set
		RTL_TRDBG(1,"LRR default xlap_cfg (%d,%d,%d) lrc='%s'\n",
			lk->lk_t1,lk->lk_t2,lk->lk_t3,lk->lk_rhost);
		return;
	}

	memset	(&uppkt,0,sizeof(t_lrr_pkt));
	uppkt.lp_vers	= LP_LRX_VERSION;
	uppkt.lp_flag	= LP_INFRA_PKT_INFRA;
	uppkt.lp_lrrid		= LrrID;
	uppkt.lp_lrridext	= LrrIDExt;
	uppkt.lp_lrridpref	= LrrIDPref;
	uppkt.lp_type		= LP_TYPE_LRR_INF_XLAP_CFG;
	uppkt.lp_szh	= LP_PRE_HEADER_PKT_SIZE+sizeof(t_lrr_xlap_cfg);
	uppkt.lp_u.lp_xlap_cfg.xc_t1	= lk->lk_t1;
	uppkt.lp_u.lp_xlap_cfg.xc_t2	= lk->lk_t2;
	uppkt.lp_u.lp_xlap_cfg.xc_t3	= lk->lk_t3;
	LapPutOutQueue(lk,(u_char *)&uppkt,uppkt.lp_szh);
	RTL_TRDBG(1,"LRR send xlap_cfg (%d,%d,%d) to lrc='%s'\n",
		lk->lk_t1,lk->lk_t2,lk->lk_t3,lk->lk_rhost);
}

static void SendLrrID(t_xlap_link *lk)
{
	int		locallrrid	= LrrID;
	float		locallatt	= LrrLat;
	float		locallong	= LrrLon;
	int		localalti	= LrrAlt;
	t_lrr_pkt	uppkt;

	if	(SimulN)
	{
		int	i;
		int	idx = -1;

		for	(i = 0 ; idx == -1 && i < NbLrc ; i++)
		{
			if	(&TbLapLrc[i] == lk)
			{
				idx	= i;
			}
		}
		if	(idx == -1)
			return;
		locallrrid	= TbLrrID[idx];
		locallatt	= TbLrrLat[idx];
		locallong	= TbLrrLon[idx];
		localalti	= TbLrrAlt[idx];
	}

	memset	(&uppkt,0,sizeof(t_lrr_pkt));
	uppkt.lp_vers	= LP_LRX_VERSION;
	uppkt.lp_flag	= LP_INFRA_PKT_INFRA;
	uppkt.lp_lrrid		= locallrrid;
	uppkt.lp_lrridext	= LrrIDExt;
	uppkt.lp_lrridpref	= LrrIDPref;
	uppkt.lp_type	= LP_TYPE_LRR_INF_LRRID;
	uppkt.lp_szh	= LP_PRE_HEADER_PKT_SIZE;
	uppkt.lp_gss	= (time_t)UptimeProc.tv_sec;
	uppkt.lp_gns	= (u_int)UptimeProc.tv_nsec;
	LapPutOutQueue(lk,(u_char *)&uppkt,uppkt.lp_szh);
	RTL_TRDBG(1,"LRR send lrrid=%02x-%04x-%08x to lrc='%s'\n",
	uppkt.lp_lrridpref,uppkt.lp_lrridext,uppkt.lp_lrrid,
	lk->lk_rhost);

#if defined(KONA) && defined(WITH_GPS)
	GetGpsPositionTektelic();
#endif
	if	(UseGpsPosition == 0)
	{	// use static GPS coordo
		uppkt.lp_u.lp_gpsco.li_gps	= 1;
		uppkt.lp_u.lp_gpsco.li_latt	= locallatt;
		uppkt.lp_u.lp_gpsco.li_long	= locallong;
		uppkt.lp_u.lp_gpsco.li_alti	= localalti;
	}
	else
	{	// dynamic coordo will sent later
		if	(UseGpsPosition && GpsPositionOk)
		{	// dynamic coordo are ready
			uppkt.lp_u.lp_gpsco.li_gps	= 2;
			uppkt.lp_u.lp_gpsco.li_latt	= GpsLatt;
			uppkt.lp_u.lp_gpsco.li_long	= GpsLong;
			uppkt.lp_u.lp_gpsco.li_alti	= GpsAlti;
		}
		else
		{	// dynamic coordo will sent later
			uppkt.lp_u.lp_gpsco.li_gps	= 0;
			uppkt.lp_u.lp_gpsco.li_latt	= locallatt;
			uppkt.lp_u.lp_gpsco.li_long	= locallong;
			uppkt.lp_u.lp_gpsco.li_alti	= localalti;
		}
	}
	uppkt.lp_type	= LP_TYPE_LRR_INF_GPSCO;
	uppkt.lp_szh	= LP_PRE_HEADER_PKT_SIZE+sizeof(t_lrr_gpsco);
	LapPutOutQueue(lk,(u_char *)&uppkt,uppkt.lp_szh);
RTL_TRDBG(1,"GPS send position (%f,%f,%d) mode=%d to lrc='%s'\n",
		uppkt.lp_u.lp_gpsco.li_latt,
		uppkt.lp_u.lp_gpsco.li_long,
		uppkt.lp_u.lp_gpsco.li_alti,
		uppkt.lp_u.lp_gpsco.li_gps,lk->lk_rhost);
	LrrLat	= uppkt.lp_u.lp_gpsco.li_latt;
	LrrLon	= uppkt.lp_u.lp_gpsco.li_long;
	LrrAlt	= uppkt.lp_u.lp_gpsco.li_alti;

	SendCapabToLrc(lk);
#ifdef	LP_TP31
	SendDtcSyncToLrc(lk);
#endif
	SendXlap_Cfg(lk);	// [2359]
	DoHtmlFile();
}

void	LoadDevAddr(char *lst)
{
	char	*delim	= " ,;|";
	char	*pt;
	char	*ctxt;
	u_int	devaddr;

	NbDevAddrIncl	= 0;
	NbDevAddrExcl	= 0;
	if	(!lst || !*lst)
		return;

	pt	= strtok_r(lst,delim,&ctxt);
	while	(pt && *pt)
	{
		if	(*pt == '!' || *pt == '-')
		{
			pt++;
			if	(sscanf(pt,"%x",&devaddr) == 1)
			{
				RTL_TRDBG(1,"devaddr %08x refused\n",devaddr);
				TbDevAddrExcl[NbDevAddrExcl]	= devaddr;
				NbDevAddrExcl++;
			}
		}	
		else
		{
			if	(sscanf(pt,"%x",&devaddr) == 1)
			{
				RTL_TRDBG(1,"devaddr %08x accepted\n",devaddr);
				TbDevAddrIncl[NbDevAddrIncl]	= devaddr;
				NbDevAddrIncl++;
			}
		}
		pt	= strtok_r(NULL,delim,&ctxt);
	}
}

static	void	ReadRffInfos()
{
	char	file[1024];
	char	line[1024];
	FILE	*f;
	char	*pt;

	VersionMajRff	= 0;
	VersionMinRff	= 0;
	VersionRelRff	= 0;
	VersionFixRff	= 0;
	RffTime	= 0;

	snprintf(file,sizeof(file),"%s/usr/etc/lrr/saverff_done",RootAct);
	f	= fopen(file,"r");
	if	(!f)
		return;

	while	(fgets(line,sizeof(line)-1,f))
	{
		if	((pt=strstr(line,"RFFVERSION=")))
		{
			pt	= strchr(line,'='); pt++;
			sscanf	(pt,"%d.%d.%d_%d",
			&VersionMajRff,&VersionMinRff,&VersionRelRff,&VersionFixRff);
			continue;
		}
		if	((pt=strstr(line,"RFFDATE=")))
		{
			if ((pt = strchr(line,'\n')))
				*pt = '\0';
			if ((pt = strchr(line,'\r')))
				*pt = '\0';
			pt	= strchr(line,'='); pt++;
			RffTime	= (time_t)rtl_iso8601_to_Unix(pt,1);
			continue;
		}
	}


	fclose(f);
}

void	MfsUsage(t_mfs *mfs)
{
	char		*path;
	struct	statfs	vfs;
	uint64_t	mega	= 1024*1024;
	uint64_t	bsize;
	uint64_t	info;

//	RTL_TRDBG(1,"MFS usage(%s)\n",path);

	mfs->fs_size	= 0;
	mfs->fs_used	= 0;
	mfs->fs_avail	= 0;
	mfs->fs_puse	= 0;
	if	(!mfs)
		return;
	path	= mfs->fs_name;
	if	(!path || !*path)
		return;


	if	(fstatfs(mfs->fs_fd,&vfs) != 0)
	{
		RTL_TRDBG(0,"cannot statfs(%s) fd=%d\n",path,mfs->fs_fd);
		return;
	}

	bsize	= vfs.f_bsize;
	info	= bsize * vfs.f_blocks;
	info	= info / mega;
	mfs->fs_size	= (u_int)info;

	if	(mfs->fs_size <= 0)
	{
		RTL_TRDBG(0,"cannot statfs(%s) size==0\n",path);
		return;
	}

	info	= bsize * vfs.f_bfree ;
	info	= info / mega;
	mfs->fs_avail	= (u_int)info;

	mfs->fs_used	= mfs->fs_size - mfs->fs_avail;
	mfs->fs_puse	= 
		(u_int)(100.0*(double)mfs->fs_used / (double)mfs->fs_size);

	RTL_TRDBG(3,"mfs=%s space=%u used=%u free=%u puse=%u\n",
		path,mfs->fs_size,mfs->fs_used,mfs->fs_avail,mfs->fs_puse);
}

void	CompAllMfsInfos(char shortlongtime)
{
	int	i;

	for	(i = 0 ; i < NB_MFS_PER_LRR ; i++)
	{
		if	(TbMfs[i].fs_enable == 0)	continue;
		if	(TbMfs[i].fs_exists == 0)	continue;
		MfsUsage(&TbMfs[i]);
	}
}

double	CompAllCpuInfos(char shortlongtime)
{
	static	u_int	tick	= 0;
	static	u_int	prev	= 0;
	FILE	*f;
	char	tmp[1024];
	int	ret;
	double	used	= 0.0;
	u_int	u,n,s,i;
	u_int	cpu;

	if	(shortlongtime == 'L')
	{
		float	l1,l5,l15;
		CpuLoad1	= 0;
		CpuLoad5	= 0;
		CpuLoad15	= 0;
		f	= fopen("/proc/loadavg","r");
		if	(!f)
			return	0.0;
		ret	= fscanf(f,"%f %f %f",&l1,&l5,&l15);
		fclose	(f);
		if	(ret != 3)
			return	0.0;
		CpuLoad1	= (u_short)(l1 * 100.0);
		CpuLoad5	= (u_short)(l5 * 100.0);
		CpuLoad15	= (u_short)(l15 * 100.0);
		return	0.0;
	}

	f	= fopen("/proc/stat","r");
	if	(!f)
		return	used;

	ret	= fscanf(f,"%s %u %u %u %u",tmp,&u,&n,&s,&i);
	fclose	(f);
	if	(ret != 5)
		return	used;
	s	= i;
	i	= ABS(i - prev);
	prev	= s;

	if	(tick == 0)
	{
		tick	= sysconf(_SC_CLK_TCK);
		cpu	= CfgInt(HtVarLrr,"versions",-1,"cpu_count",1);
		// /proc/stat cpu line returns sum of all idle ticks on all cores
		tick	= tick*cpu;
		return	used;			// first call return 0.0
	}

	used	= (double)i/(double)tick;	// idle
	used	= (1.0 - used)*100.0;		// used %
//	printf	("cpuused=%f\n",used);
	AvDvUiAdd(&CpuAvdvUsed,(u_int)used,Currtime.tv_sec);
	return	used;
}

void	CompAllMemInfos(char shortlongtime)
{
	int	fd;
	char	buff[1024];
	int	sz;
	char	*pt;

	MemTotal	= 0;
	MemFree		= 0;
	MemBuffers	= 0;
	MemCached	= 0;
	MemUsed		= 0;
	
	fd	= open("/proc/meminfo",0);
	if	(fd < 0)
		return;
	sz	= read(fd,buff,sizeof(buff)-1);
	close	(fd);
	if	(sz <= 0)
		return;
	buff[sz]	= '\0';

	pt	= buff;
	pt	= strstr(pt,"MemTotal:");
	if	(!pt || !*pt)
		return;
	pt	+= strlen("MemTotal:");
	MemTotal	= strtoul(pt,0,0);
	pt	= strstr(pt,"MemFree:");
	if	(!pt || !*pt)
		return;
	pt	+= strlen("MemFree:");
	MemFree	= strtoul(pt,0,0);
	pt	= strstr(pt,"Buffers:");
	if	(!pt || !*pt)
		return;
	pt	+= strlen("Buffers:");
	MemBuffers	= strtoul(pt,0,0);
	pt	= strstr(pt,"Cached:");
	if	(!pt || !*pt)
		return;
	pt	+= strlen("Cached:");
	MemCached	= strtoul(pt,0,0);

	MemUsed	= MemTotal - MemFree;
}

char	*DoConfigFileCustom(char *file)
{
	static	char	path[1024];

	snprintf(path,sizeof(path),"%s/%s",ConfigCustom,file);
	RTL_TRDBG(0,"search '%s'\n",path);
	if	(access(path,R_OK) == 0)
		return	path;
	RTL_TRDBG(0,"no custom configuration file '%s'\n",path);
	return	NULL;
}

char	*DoConfigFileDefault(char *file,char *suff)
{
	static	char	path[1024];

	if	(suff && *suff)
	{
		char	tmp[128];
		char	*pt;

		strcpy	(tmp,file);
		pt	= strstr(tmp,".ini");
		if	(pt)
		{
			*pt	= '\0';
			snprintf(path,sizeof(path),"%s/%s_%s.ini",ConfigDefault,tmp,suff);
		}
		else
			snprintf(path,sizeof(path),"%s/%s_%s",ConfigDefault,tmp,suff);
	}
	else
		snprintf(path,sizeof(path),"%s/%s",ConfigDefault,file);
	RTL_TRDBG(0,"search '%s'\n",path);
	if	(access(path,R_OK) == 0)
		return	path;
#if	0
	RTL_TRDBG(0,"cannot find default configuration file '%s'\n",path);
	sleep	(1);
	exit	(1);
#endif
	return	NULL;	// ism.band vs ism.bandlocal
}

void	SaveConfigFileState()
{
	char	path[1024];
	FILE	*f;

	snprintf(path,sizeof(path),"%s/%s",ConfigCustom,ConfigFileState);
	f	= fopen(path,"w");
	if	(!f)
		return;

	fprintf	(f,";;;; file generated by LRR process do not change it\n");
	fprintf	(f,"[lrr]\n");

	if	(CfgRadioState)
	{
		fprintf	(f,"\tradiostopped=%d\n",CfgRadioStop);
		fprintf	(f,"\tradiodnstopped=%d\n",CfgRadioDnStop);
	}
	else
	{
		fprintf	(f,";\tradiostopped=%d\n",CfgRadioStop);
		fprintf	(f,";\tradiodnstopped=%d\n",CfgRadioDnStop);
	}

	if	(LrrIDFromTwaGot != 0)
	{
		fprintf	(f,"\tuidfromtwa=0x%08x\n",LrrIDFromTwaGot);
	}

	fprintf	(f,"\tlrridused=0x%08x\n",LrrID);

	if	(TraceLevelP >= 0)
	{
		fprintf	(f,"[trace]\n");
		fprintf	(f,"\tlevelp=%d\n",TraceLevelP);
	}
	else
	{
		fprintf	(f,";[trace]\n");
		fprintf	(f,";\tlevelp=%d\n",TraceLevelP);
	}

	if	(LrrIDFromBS != 0)
		fprintf	(f,"\tuidfrombootsrv=0x%08x\n",LrrIDFromBS);

	fclose	(f);
}

char	*LrrPktFlagsTxt(unsigned int type)
{
	static	char	buf[64];

	buf[0]	= '\0';
	if	(type & LP_INFRA_PKT_INFRA)
	{
		strcat(buf,"I");
		return	buf;
	}

	strcat(buf,"R");
	if	(type & LP_RADIO_PKT_UP)
		strcat(buf,"U");
	if	(type & LP_RADIO_PKT_DOWN)
		strcat(buf,"D");
	if	(type & LP_RADIO_PKT_802154)
		strcat(buf,"8");
	if	(type & LP_RADIO_PKT_ACKMAC)
		strcat(buf,"A");
	if	(type & LP_RADIO_PKT_ACKDATA)
		strcat(buf,"+");
	if	(type & LP_RADIO_PKT_DELAY)
		strcat(buf,"Y");
	if	(type & LP_RADIO_PKT_PINGSLOT)
		strcat(buf,"B");
	if	(type & LP_RADIO_PKT_LORA_E)
		strcat(buf,"E");
	if	(type & LP_RADIO_PKT_RX2)
		strcat(buf,"2");

	return	buf;
}

static	void	CBHtDumpLrr(char *var,void *value)
{
	RTL_TRDBG(9,"var='%s' val='%s'\n",var,(char *)rtl_htblGet(HtVarLrr,var));
//	printf("var='%s' val='%s'\n",var,(char *)rtl_htblGet(HtVarLrr,var));
}

#ifdef CISCOMS

// Folowing functions csn* are used to get a LRR ID from a Cisco S/N
int csnCheckformat(char *str)
{
	char	*pt;
	int	i;

	pt = str;

	if (!pt || !*pt)
		return 0;

	// check LLL
	for (i=0; i<3; i++)
		if (!isupper(*pt++))
			return 0;
	// check YYWW
	for (i=0; i<4; i++)
		if (!isdigit(*pt++))
			return 0;
	// check SSSS
	for (i=0; i<4; i++)
	{
		if (!isupper(*pt) && !isdigit(*pt))
			return 0;
		pt++;
	}
	return 1;
}

// encode alphanum character
// '0' - '9' => 0 - 9
// 'A' - 'Z' => 10 - 36
int csnCodeAlNum(char c)
{
	int	res;
	if (isdigit(c))
		res = c - '0';
	else
		res = c - 'A' + 10;
	RTL_TRDBG(4, "csnCodeAlNum(%c) = %d\n", c, res);
	return res;
}

// get a LRR ID from a Cisco S/N
uint32_t csnCode(char *str)
{
	uint32_t	res;
	uint32_t	lll, yy, ww, ssss;
	char	*pt, tmp[10];



	if (!csnCheckformat(str))
	{
		RTL_TRDBG(0, "Incorrect format Cisco S/N '%s', must be LLLDDDDAAAA where L is an uppercase letter, D a digit, and A an uppercase letter or a digit\n", str);
		return -1;
	}

	pt = str;
	// keep L % 4
	lll = 	((*pt-'A') % 4) * 4*4 +
		((*(pt+1)-'A') % 4) * 4 +
		(*(pt+2)-'A') % 4;
	RTL_TRDBG(4, "csnCode(%s): lll=%d\n", str, lll);

	// keep YY % 3
	pt += 3;
	tmp[0] = *pt++;
	tmp[1] = *pt++;
	tmp[2] = '\0';
	yy = atoi(tmp) % 3;
	RTL_TRDBG(4, "csnCode(%s): yy=%d (%s)\n", str, yy, tmp);

	// keep WW % 26
	tmp[0] = *pt++;
	tmp[1] = *pt++;
	tmp[2] = '\0';
	ww = (atoi(tmp)-1) % 26;
	RTL_TRDBG(4, "csnCode(%s): ww=%d (%s)\n", str, ww, tmp);

	// keep S % 18 for the first one, and the full value for the others
	ssss = 	(csnCodeAlNum(*pt) % 18) * 36*36*36 +
	 	csnCodeAlNum(*(pt+1)) * 36*36 +
	 	csnCodeAlNum(*(pt+2)) * 36 +
	 	csnCodeAlNum(*(pt+3));
	RTL_TRDBG(4, "csnCode(%s): ssss=%d\n", str, ssss);

	res =	(lll * 3*26*18*36*36*36) +
		(yy * 26*18*36*36*36) +
		(ww * 18*36*36*36) +
		ssss;

	RTL_TRDBG(3, "csnCode(%s) = 0x%08x\n", str, res);
	return res;
}
#endif

static	u_int	DoLrrID(char *str,u_short *ext,u_char *pref)
{
	u_int	ret	= 0;

	if	(!str || !*str || !ext || !pref)
		return	0;

	*pref	= LP_LRRID_NO_PREF;
	*ext	= 0;

	if	(strcmp(str,"hostname_mac/32") == 0 )
	{
#ifdef	WIRMAV2
#if	0
		char	*pt;
		char	host[HOST_NAME_MAX+1];

		if	(gethostname(host,HOST_NAME_MAX) != 0)
		{
			RTL_TRDBG(0,"cannot get hostname\n");
			exit(1);
		}
		pt	= host;
		while	(*pt && *pt != '_')	pt++;
		if	(*pt == '_')
		{
			pt++;
			sscanf	(pt,"%x",&ret);
			return	ret;
		}
#endif
#endif
#if defined(RBPI_V1_0) || defined(NATRBPI_USB) || defined(SEMPICO)
		char	*pt;
		uint64_t	serial;
		char		line[1024];
		FILE		*f;
		f	= fopen("/proc/cpuinfo","r");
		if	(f)
		while	(fgets(line,sizeof(line)-1,f))
		{
			if	(!strstr(line,"Serial"))	continue;
			pt	= strchr(line,':');
			if	(!pt)				return	0;
			serial	= strtoull(pt+1, NULL, 16);
			ret	= (u_int)serial;
			*pref	= LP_LRRID_RBPI_CPUID;	// raspberry cpu id
			*ext	= 0;			// not used
			return	ret;
		}
#endif
#ifdef	CISCOMS
		char * pt = NULL;
		pt = CfgStr(HtVarSys,"",-1,"CISCOSN","");
		if (pt && *pt)
		{
			*pref	= LP_LRRID_CISCO_SN;	// cisco id
			*ext	= 0;			// not used
			ret = csnCode(pt);
			return ret;
		}
#endif
		*pref	= LP_LRRID_FULL_MAC;		// full mac address
		ret	= FindEthMac32(ext);
		if	(ret)
			return	ret;
		return	0;
	}

	*pref	= LP_LRRID_BY_CONFIG;			// set by configuration
	*ext	= 0x00;					// not used
	if	(*str == '0' && (*(str+1) == 'x' || *(str+1) == 'X') )
		sscanf	(str,"%x",&ret);
	else
		ret	= (u_int)atoi(str);
	return	ret;
}

// Test if getting lrrid from TWA is configured
static	void	DoLrrIDFromTwa(char *str)
{
	static	int	trace_done	= 0;
	int	l;

	// RDTP-9756/7649: now this function is recalled after each DISC on xlap
	// try to limit traces
	if	(trace_done == 0)
	{
		trace_done	= 1;
		l		= 1;
	}
	else
	{
		l		= 1000;
	}


	if (LrrIDGetFromBS)	// NFR997
	{
		RTL_TRDBG(l,"Getting lrrid from twa is disabled (got from bootserver)\n");
		return;
	}

	if	(!str || !*str || strcmp(str,"local") == 0)
	{
		RTL_TRDBG(l,"Getting lrrid from twa is disabled\n");
		return;
	}

	if	(strcmp(str,"fromtwa") == 0 || strcmp(str,"fromtwaonly") == 0)
	{
		char	*pt;
		int	ret	= 0;

		pt	= CfgStr(HtVarLrr,"lrr",-1,"uidfromtwa","0");
		if	(*pt == '0' && (*(pt+1) == 'x' || *(pt+1) == 'X') )
			sscanf	(pt,"%x",&ret);
		else
			ret	= (u_int)atoi(pt);
#if	0
		RTL_TRDBG(1,"Getting lrrid from twa is activated %08x\n",ret);
		if	(ret != 0)
		{	// twa already give us a lrrid
			LrrIDGetFromTwa	= 0;
			LrrIDFromTwaGot	= ret;
			LrrID		= ret;
		}
		else
		{	// request lrrid from twa
			LrrIDGetFromTwa = 1;
			if	(strcmp(str,"fromtwaonly") == 0)
				LrrIDGetFromTwa = 2;
		}
#else
		// RDTP-3332: renew LRRID because of TPE context
		// => forget last LRRID and get a new one in any cases
		RTL_TRDBG(l,"Getting lrrid from twa is activated last lrrid=%08x\n",ret);
		// request lrrid from twa
		LrrIDGetFromTwa = 1;
		LrrIDFromTwaGot	= 0;
		if	(strcmp(str,"fromtwaonly") == 0)
			LrrIDGetFromTwa = 2;
#endif
	}
	else
		RTL_TRDBG(l,"Getting lrrid from twa is disabled\n");
}

// the custom configuration can not overload defines "defines.ini"
// the defines are loaded in the 2 hash table HtVarLrr and HtVarLgw
static	void	LoadConfigDefine(int hot,int dumponly)
{
	int	err;
	char	*file;

	file	= DoConfigFileCustom(ConfigFileSystem);
	if	(file)
	{
		RTL_TRDBG(0,"load custom '%s'\n",file);
	}
	if	(file && (err=rtl_iniParse(file,CfgCBIniLoad,HtVarSys)) < 0)
	{
		RTL_TRDBG(0,"parse '%s' error=%d\n",file,err);
		exit(1);
	}

	file	= DoConfigFileCustom(ConfigFileParams);
	if	(file)
	{
		RTL_TRDBG(0,"load custom '%s'\n",file);
	}
	if	(file && (err=rtl_iniParse(file,CfgCBIniLoad,HtVarSys)) < 0)
	{
		RTL_TRDBG(0,"parse '%s' error=%d\n",file,err);
		exit(1);
	}

	file	= DoConfigFileDefault(ConfigFileDefine,NULL);
	if	(file)
	{
		RTL_TRDBG(0,"load default '%s'\n",file);
	}
	if	(file && (err=rtl_iniParse(file,CfgCBIniLoad,HtVarLrr)) < 0)
	{
		RTL_TRDBG(0,"parse '%s' error=%d\n",file,err);
		exit(1);
	}

	file	= DoConfigFileDefault(ConfigFileDefine,NULL);
	if	(file)
	{
		RTL_TRDBG(0,"load default '%s'\n",file);
	}
	if	(file && (err=rtl_iniParse(file,CfgCBIniLoad,HtVarLgw)) < 0)
	{
		RTL_TRDBG(0,"parse '%s' error=%d\n",file,err);
		exit(1);
	}

	LgwForceDefine(HtVarLrr);
	LgwForceDefine(HtVarLgw);
}

void AdjustDefaultIsmValues(void)
{
	if	(strlen(IsmBandAlter) == 0)
	{
#ifdef	LP_TP31
		if	(strcmp(IsmBand,"eu868") == 0)
			IsmBandAlter	= strdup("eu868_2015");
		else
			IsmBandAlter	= strdup(IsmBand);
#else
		IsmBandAlter	= strdup(IsmBand);
#endif
	}
	if	(strcmp(IsmBand,"eu868") == 0)
	{
		IsmFreq		= 868;
	}	else
	if	(strcmp(IsmBand,"in865") == 0)
	{
		IsmFreq		= 868;
	}	else
	if	(strcmp(IsmBand,"ru864") == 0)
	{
		IsmFreq		= 868;
	}	else
	if	(strcmp(IsmBand,"us915") == 0)
	{
		IsmAsymDownlink	= 1;
		IsmFreq		= 915;
	}	else
	if	(strcmp(IsmBand,"au915") == 0)
	{
		IsmAsymDownlink	= 1;
		IsmAsymDnModulo	= 8;
		IsmAsymDnOffset	= 0;
		IsmFreq		= 915;
	}	else
	if	(strcmp(IsmBand,"sg920") == 0)
	{
		IsmFreq		= 920;
	}	else
	if	(strcmp(IsmBand,"as923") == 0)
	{
		IsmFreq		= 920;
	}	else
	if	(strcmp(IsmBand,"kr920") == 0)
	{
		IsmFreq		= 920;
	}	else
	if	(strcmp(IsmBand,"cn779") == 0)
	{
		IsmFreq		= 779;
	}	else
	if	(strcmp(IsmBand,"cn470") == 0)
	{
		IsmAsymDownlink	= 1;
		IsmFreq		= 470;
	}	else
	{
		IsmFreq		= AdjustFreqForCalibration(IsmBand);
	}
}

int	AdjustFreqForCalibration(char *ism)
{
	char	*pt;
	int	f;

	pt	= ism;
	while	(*pt < '0' || *pt > '9') pt++;
	if	(!*pt)
		return	868;
	f	= atoi(pt);
	if	(f < 100 || f >= 1000)
		return	868;

	int	minDiff[3];
	int	nb = sizeof(minDiff)/sizeof(int);
	int	i,idxmin;
	int	min	= INT_MAX;

	// TODO we only have 3 LUT tables
	minDiff[0]	= 868;
	minDiff[1]	= 915;
	minDiff[2]	= 920;

	idxmin	= 0;
	for	(i = 0 ; i < nb ; i++)
	{
		int	ret;

		ret	= ABS(minDiff[i]-f);
		if	(ret < min)
		{
			min	= ret;
			idxmin	= i;
		}
	}

	return	minDiff[idxmin];
}

// Create or remove symbolic link on trace directory depending on the configuration
void LinkTraceDir()
{
	char	*trdir1, *trdir2;
#if 0
    char    *trdir, char    tmp[256];
#endif
	int	disabled = 0;

	// read lrr.ini:[trace].ramdir
	trdir1	= CfgStr(HtVarLrr,"trace",-1,"ramdir",NULL);
	// read default lrr.ini:[<system>].ramdir
	trdir2	= CfgStr(HtVarLrr,System,-1,"ramdir",NULL);

	// ramdir set to empty in lrr.ini -> feature disabled
	if (trdir1 && !*trdir1)
		disabled = 1;

	// ramdir not set in lrr.ini and also not set in default lrr.ini
	if (!trdir1 && (!trdir2 || !*trdir2))
		disabled = 1;

	if (disabled)
	{
		// ramdir function disabled
#if 0
		// remove link on var/log/lrr if it's a symbolic link
		system("[ -L \"$ROOTACT/var/log/lrr\" ] && rm \"$ROOTACT/var/log/lrr\"");
#endif
		LogUseRamDir = 0;
		return;
	}

	// ramdir function enabled
	LogUseRamDir = 1;
#if 0
	// trdir = ramdir of lrr.ini if set, or ramdir of default lrr.ini 
	trdir = trdir1 ? trdir1 : trdir2;

	// create target dir if needed
	snprintf(tmp,sizeof(tmp),"[ ! -d \"%s\" ] && mkdir -p \"%s\"", trdir, trdir);
	system(tmp);

	// remove var/log/lrr if it's a standard directory, in order to create a symbolic link
	system("[ -d \"$ROOTACT/var/log/lrr\" ] && rm -rf \"$ROOTACT/var/log/lrr\"");

	// remove var/log/lrr if it's a link, to create it correctly, because
	// it's difficult to check if an existing one is the good one
	system("[ -L \"$ROOTACT/var/log/lrr\" ] && rm \"$ROOTACT/var/log/lrr\"");

	// create link
	snprintf(tmp,sizeof(tmp),"ln -s \"%s\" \"$ROOTACT/var/log/lrr\"", trdir);
	system(tmp);
#endif
}

static	void	LoadConfigLrr(int hot,int dumponly)
{
	int	err;
	int	i;
	char	*file;
	char	*strid;
	char	*stridmode;
	char	*pt;
	char	section[128];
	char 	tmpstrvalue[8] = {0};

	file	= DoConfigFileDefault(ConfigFileLrr,NULL);
	if	(file)
	{
		RTL_TRDBG(0,"load default '%s'\n",file);
	}
	if	((err=rtl_iniParse(file,CfgCBIniLoad,HtVarLrr)) < 0)
	{
		RTL_TRDBG(0,"parse '%s' error=%d\n",file,err);
		exit(1);
	}
	file	= DoConfigFileCustom(ConfigFileLrr);
	if	(file)
	{
		RTL_TRDBG(0,"load custom '%s'\n",file);
	}
	if	(file && (err=rtl_iniParse(file,CfgCBIniLoad,HtVarLrr)) < 0)
	{
		RTL_TRDBG(0,"parse '%s' error=%d\n",file,err);
		exit(1);
	}
	/* gpsman.ini */
	file	= DoConfigFileCustom(ConfigFileGps);
	if	(file)
	{
		RTL_TRDBG(0,"load custom '%s'\n",file);
	}
	if	(file && (err=rtl_iniParse(file,CfgCBIniLoad,HtVarLrr)) < 0)
	{
		RTL_TRDBG(0,"parse '%s' error=%d\n",file,err);
		exit(1);
	}
	/* custom.ini */
	file	= DoConfigFileCustom(ConfigFileCustom);
	if	(file)
	{
		RTL_TRDBG(0,"load custom '%s'\n",file);
	}
	if	(file && (err=rtl_iniParse(file,CfgCBIniLoad,HtVarLrr)) < 0)
	{
		RTL_TRDBG(0,"parse '%s' error=%d\n",file,err);

		exit(1);
	}
	/* versions.ini */
	file	= DoConfigFileCustom(ConfigFileVersions);
	if	(file)
	{
		RTL_TRDBG(0,"load custom '%s'\n",file);
	}
	if	(file && (err=rtl_iniParse(file,CfgCBIniLoad,HtVarLrr)) < 0)
	{
		RTL_TRDBG(0,"parse '%s' error=%d\n",file,err);

		exit(1);
	}
	if (LrrIDGetFromBS)	// NFR997
	{
		file	= DoConfigFileCustom(ConfigFileDynLap);
		if	(file)
		{
			RTL_TRDBG(0,"load custom '%s'\n",file);
		}
		if	(file && (err=rtl_iniParse(file,CfgCBIniLoad,HtVarLrr)) < 0)
		{
			RTL_TRDBG(0,"parse '%s' error=%d\n",file,err);
			exit(1);
		}
	}
	file	= DoConfigFileCustom(ConfigFileState);
	if	(file)
	{
		RTL_TRDBG(0,"load custom '%s'\n",file);
	}
	if	(file && (err=rtl_iniParse(file,CfgCBIniLoad,HtVarLrr)) < 0)
	{
		RTL_TRDBG(0,"parse '%s' error=%d\n",file,err);
		exit(1);
	}

	rtl_htblDump(HtVarLrr,CBHtDumpLrr);

	if	(!dumponly)
		DecryptHtbl(HtVarLrr);

	IsmBand		= CfgStr(HtVarLrr,"ism",-1,"band",IsmBand);
	IsmBandAlter	= CfgStr(HtVarLrr,"ism",-1,"bandalter","");
	if	(!IsmBandAlter || !*IsmBandAlter)
		IsmBandAlter	= CfgStr(HtVarLrr,"ism",-1,"bandlocal","");
	AdjustDefaultIsmValues();
	IsmFreq		= CfgInt(HtVarLrr,"ism",-1,"freq",IsmFreq);
	IsmAsymDownlink	= CfgInt(HtVarLrr,"ism",-1,"asymdownlink", IsmAsymDownlink);
	IsmAsymDnModulo = CfgInt(HtVarLrr,"ism",-1,"asymdownlinkmodulo", IsmAsymDnModulo);
	IsmAsymDnOffset = CfgInt(HtVarLrr,"ism",-1,"asymdownlinkoffset", IsmAsymDnOffset);
	LinkTraceDir();		// see LogUseRamDir
	TraceLevel	= CfgInt(HtVarLrr,"trace",-1,"level",TraceLevel);
	TraceLevelP	= CfgInt(HtVarLrr,"trace",-1,"levelp",-1);
	if		(TraceLevelP >= 0)
			TraceLevel	= TraceLevelP;
	TraceDebug	= CfgInt(HtVarLrr,"trace",-1,"debug",TraceDebug);
	if		(dumponly)	TraceDebug	= 0;
	TraceSize	= CfgInt(HtVarLrr,"trace",-1,"size",TraceSize);
#ifdef	WIRMAV2
	TraceSize	= 10*1000*1000;
#endif

	TraceFile	= CfgStr(HtVarLrr,"trace",-1,"file","TRACE.log");
	TraceStdout	= CfgStr(HtVarLrr,"trace",-1,"stdout",NULL);
	StatRefresh	= CfgInt(HtVarLrr,"lrr",-1,"statrefresh",StatRefresh);
	RfCellRefresh	= CfgInt(HtVarLrr,"lrr",-1,"rfcellrefresh",RfCellRefresh);
	ConfigRefresh	= CfgInt(HtVarLrr,"lrr",-1,"configrefresh",ConfigRefresh);
	WanRefresh	= CfgInt(HtVarLrr,"lrr",-1,"wanrefresh",WanRefresh);
	GpsStRefresh	= CfgInt(HtVarLrr,"lrr",-1,"gpsstatusrefresh",GpsStRefresh);
	snprintf(tmpstrvalue,sizeof(tmpstrvalue), "%.2f", GpsStRateHigh);
	GpsStRateHigh	= atof(CfgStr(HtVarLrr,"lrr",-1,"gpsstatusratehigh", tmpstrvalue));
	snprintf(tmpstrvalue,sizeof(tmpstrvalue), "%.2f", GpsStRateLow);
	GpsStRateLow	= atof(CfgStr(HtVarLrr,"lrr",-1,"gpsstatusratelow", tmpstrvalue));
	GpsUtcLeap	= CfgInt(HtVarLrr,"lrr",-1,"gpsleapseconds", GpsUtcLeap);

	LrrLat		= atof(CfgStr(HtVarLrr,"lrr",-1,"lat","90.0"));
	LrrLon		= atof(CfgStr(HtVarLrr,"lrr",-1,"lon","90.0"));
	LrrAlt		= CfgInt(HtVarLrr,"lrr",-1,"alt",0);

	AdjustDelay	= CfgInt(HtVarLrr,"lrr",-1,"adjustdelay",0);
	NbLrc		= CfgInt(HtVarLrr,"lrr",-1,"nblrc",1);
	if	(NbLrc > NB_LRC_PER_LRR)	NbLrc	= NB_LRC_PER_LRR;
	GpsDevice	= CfgStr(HtVarLrr, System, -1, "gpsdevice", GpsDevice);
	GpsMaxBadChecksum	= CfgInt(HtVarLrr, "lrr", -1, "gpsmaxbadchecksum", GpsMaxBadChecksum);
	GpsMaxBadRead	= CfgInt(HtVarLrr, "lrr", -1, "gpsmaxbadread", GpsMaxBadRead);
	UseGpsPosition	= CfgInt(HtVarLrr,"lrr",-1,"usegpsposition",UseGpsPosition);
	UseGpsTime	= CfgInt(HtVarLrr,"lrr",-1,"usegpstime",UseGpsTime);
    // PT-1308
	AdjustFineTimeStamp = CfgInt(HtVarLrr, "lrr", -1, "adjustfinetimestamp", AdjustFineTimeStamp);
#if	0	// RDTP-6172
	UseLgwTime	= CfgInt(HtVarLrr,"lrr",-1,"uselgwtime",UseLgwTime);
#endif
	CellStateReport	= CfgInt(HtVarLrr,"lrr",-1,"cellstatereport",CellStateReport);
	CellStateDev	= CfgStr(HtVarLrr,"lrr",-1,"cellstatedev",CellStateDev);
	CellStateSampleSize	= CfgInt(HtVarLrr,"lrr",-1,"cellstatesamplesize",CellStateSampleSize);
	CellStateSampleTimeout	= CfgInt(HtVarLrr,"lrr",-1,"cellstatesampletimeout",CellStateSampleTimeout);
	// RDTP-12898
	WifiStateReport	= CfgInt(HtVarLrr,"lrr",-1,"wifistatereport",WifiStateReport);
	if (WifiStateReport == 0)
		strcpy(WifiStateSsid, "feature disabled");
	// RDTP-5911
	DlShiftLc	= CfgInt(HtVarLrr,"lrr",-1,"dlshiftlc",DlShiftLc);
	Redundancy	= CfgInt(HtVarLrr,"lrr",-1,"redundancy",Redundancy);
	CfgRadioState	= CfgInt(HtVarLrr,"lrr",-1,"radiostatesaved",0);
	CfgRadioStop	= CfgInt(HtVarLrr,"lrr",-1,"radiostopped",0);
	CfgRadioDnStop	= CfgInt(HtVarLrr,"lrr",-1,"radiodnstopped",0);
	// RDTP-5475
	SynchroForward	= CfgInt(HtVarLrr,"utcsynchro",-1,"forward",SynchroForward);
#ifndef	WITH_SX1301_X8
	SynchroPeriod	= CfgInt(HtVarLrr,"utcsynchro",-1,"period",SynchroPeriod);
	SynchroLC	= CfgInt(HtVarLrr,"utcsynchro",-1,"lc",SynchroLC);
	SynchroSF	= CfgInt(HtVarLrr,"utcsynchro",-1,"spfact",SynchroSF);
	SynchroMaxLrr	= CfgInt(HtVarLrr,"utcsynchro",-1,"maxlrr",SynchroMaxLrr);
#endif

	LgwThreadStopped= CfgRadioStop;


	// Simulation mode used for different things but no relation with the following param simulN
	SimulationMode	= CfgInt(HtVarLrr,"lrr",-1,"simulationmode",0);


	SimulN		= CfgInt(HtVarLrr,"lrr",-1,"simulN",0);
	QosAlea		= CfgInt(HtVarLrr,"lrr",-1,"qosalea",0);
	pt		= CfgStr(HtVarLrr,"lrr",-1,"devaddr","");
	LoadDevAddr(pt);

#ifdef	LP_TP31
	DcLtPeriod		= CfgInt(HtVarLrr,"lrr",-1,"dtcltperiod",168);
	StorePktCount	= 
		CfgInt(HtVarLrr,"uplinkstorage",-1,"pktcount",StorePktCount);
	pt	= CfgStr(HtVarLrr,"uplinkstorage",-1,"memused","");
	if	(pt && *pt)
		StoreMemUsed	= atof(pt);
	ReStorePerSec	= 
		CfgInt(HtVarLrr,"uplinkstorage",-1,"rstrpersec",ReStorePerSec);
	ReStoreCtrlOutQ	= 
		CfgInt(HtVarLrr,"uplinkstorage",-1,"ctrloutq",ReStoreCtrlOutQ);
	ReStoreCtrlAckQ	= 
		CfgInt(HtVarLrr,"uplinkstorage",-1,"ctrlackq",ReStoreCtrlAckQ);
	DiskSaveStoreQ = 
		CfgInt(HtVarLrr,"uplinkstorage",-1,"disksaveonexit",DiskSaveStoreQ);
#endif

	if	(UseGpsPosition || UseGpsTime)
		UseGps	= 1;

	for	(i = 0 ; i < NbLrc ; i++)
	{
		int	ipchk4;
		int	ipchk6;

		strcpy	(TbLapLrc[i].lk_name,
				CfgStr(HtVarLrr,"laplrc",i,"name","slave"));
		TbLapLrc[i].lk_addr = CfgStr(HtVarLrr,"laplrc",i,"addr","0.0.0.0");
		TbLapLrc[i].lk_port = CfgStr(HtVarLrr,"laplrc",i,"port","2404");

#if	0	// it is too risky to change the lap flags
		TbLapLrc[i].lk_type = (u_int)CfgInt(HtVarLrr,"laplrc",i,
					"type",LapFlags);
#endif
		TbLapLrc[i].lk_type	= LapFlags;
		ipchk4	= CfgInt(HtVarLrr,"laplrc",i,"ip4only",0);
		ipchk6	= CfgInt(HtVarLrr,"laplrc",i,"ip6only",0);
		if	(ipchk4 && ipchk6)
		{
RTL_TRDBG(0,"laplrc=%d addr='%s:%s' error both ip4/ip6 only => drop both\n",
				i,TbLapLrc[i].lk_addr,TbLapLrc[i].lk_port);
			ipchk4	= 0;
			ipchk6	= 0;
		}
		if	(ipchk4)
			TbLapLrc[i].lk_type	|= LK_TCP_IP4ONLY;
		if	(ipchk6)
			TbLapLrc[i].lk_type	|= LK_TCP_IP6ONLY;


		TbLapLrc[i].lk_t1 = (u_int)CfgInt(HtVarLrr,"laplrc",i,
					"iec104t1",LRR_DEFAULT_T1);
		TbLapLrc[i].lk_t2 = (u_int)CfgInt(HtVarLrr,"laplrc",i,
					"iec104t2",LRR_DEFAULT_T2);
		TbLapLrc[i].lk_t3 = (u_int)CfgInt(HtVarLrr,"laplrc",i,
					"iec104t3",LRR_DEFAULT_T3);


		if	(SimulN == 0)	// legacy case
		{
		TbLrrID[i] = LrrID;
		TbLrrLat[i] = LrrLat;
		TbLrrLon[i] = LrrLon;
		TbLrrAlt[i] = LrrAlt;
		}
		else			// simulate several lrr
		{
		TbLrrID[i] = (u_int)CfgInt(HtVarLrr,"laplrc",i,"uid",LrrID);
		TbLrrLat[i] = atof(CfgStr(HtVarLrr,"laplrc",i,"lat","90.0"));
		TbLrrLon[i] = atof(CfgStr(HtVarLrr,"laplrc",i,"lon","90.0"));
		TbLrrAlt[i] = CfgInt(HtVarLrr,"laplrc",i,"alt",0);
		}
RTL_TRDBG(1,"laplrc=%d addr='%s:%s' flg=%x t1=%d t2=%d t3=%d i4o=%d i6o=%d \n\t\t\tlrrid=%x lat=%f lon=%f alt=%d\n",
		i,TbLapLrc[i].lk_addr,TbLapLrc[i].lk_port,
		TbLapLrc[i].lk_type,TbLapLrc[i].lk_t1,
		TbLapLrc[i].lk_t2,TbLapLrc[i].lk_t3,ipchk4,ipchk6,
		TbLrrID[i],TbLrrLat[i],TbLrrLon[i],TbLrrAlt[i]);
	}

	for	(i = 0; i < NB_ITF_PER_LRR ; i++)
	{
		char	*pt;

		snprintf(section,sizeof(section),"%s/netitf",System);

		TbItf[i].it_enable	= CfgInt(HtVarLrr,section,i,"enable",0);
		TbItf[i].it_type	= CfgInt(HtVarLrr,section,i,"type",0);

		pt			= CfgStr(HtVarLrr,section,i,"name",0);
		if	(!pt || !*pt)
		{
			TbItf[i].it_enable	= 0;
			pt			= "";
		}
		TbItf[i].it_name	= strdup(pt);
		NbItf++;
	}
	// RDTP-4176
	for	(i = 0; i < NbItf ; i++)
	{
		char	*pt;

		if	(TbItf[i].it_enable == 0)	continue;
		snprintf(section,sizeof(section),"netitf");

		pt	= CfgStr(HtVarLrr,section,i,"pingaddr",0);
		if	(pt && *pt)
			TbItf[i].it_pingaddr	= strdup(pt);
		pt	= CfgStr(HtVarLrr,section,i,"pingaddrsrc",0);
		if	(pt && *pt)
			TbItf[i].it_pingaddrsrc	= strdup(pt);
	}
	FindIpInterfaces(&ConfigIpInt);

	for	(i = 0; i < NB_ITF_PER_LRR ; i++)
	{
		char	*pt	= TbItf[i].it_pingaddr;

		if	(!pt || !*pt)
			pt	= TbLapLrc[0].lk_addr;
RTL_TRDBG(1,"netitf=%d enable=%d name='%s' exists=%d type=%d pingaddr='%s'\n",i,
	TbItf[i].it_enable,TbItf[i].it_name,TbItf[i].it_exists,TbItf[i].it_type,
	pt);
	}

	for	(i = 0; i < NB_MFS_PER_LRR ; i++)
	{
		char	*pt;

		snprintf(section,sizeof(section),"%s/mfs",System);

		TbMfs[i].fs_enable	= CfgInt(HtVarLrr,section,i,"enable",0);
		pt			= CfgStr(HtVarLrr,section,i,"type","?");
		if	(!pt || !*pt)
			pt		= "?";
		TbMfs[i].fs_type	= *pt;

		pt			= CfgStr(HtVarLrr,section,i,"name",0);
		if	(!pt || !*pt)
		{
			TbMfs[i].fs_enable	= 0;
			pt			= "";
		}
		TbMfs[i].fs_name	= strdup(pt);
		TbMfs[i].fs_exists	= 1;
#ifndef __UCLIBC__ // OpenWRT gateway like Tracknet or Flexpico
		TbMfs[i].fs_fd		= open(TbMfs[i].fs_name,O_RDONLY|O_CLOEXEC);
#else
		/* O_CLOEXEC does not exist in flexpico toolchain. Instead, set the Close-on-Exec flag via fcntl */
		int flags;
		TbMfs[i].fs_fd = open(TbMfs[i].fs_name, O_RDONLY);
		flags = fcntl(TbMfs[i].fs_fd, F_GETFD);
		if ((flags = fcntl(TbMfs[i].fs_fd, F_GETFD)) < 0)
			flags = FD_CLOEXEC;
		else
			flags |= FD_CLOEXEC;
		fcntl(TbMfs[i].fs_fd, F_SETFD, flags);
#endif /* not FLEXPICO */
		if	(TbMfs[i].fs_fd < 0)
		{
			TbMfs[i].fs_exists	= 0;
		}
		NbMfs++;
		RTL_TRDBG(1,"mfs=%d mfs=%s enable=%d exists=%d type=%c\n",
		i,TbMfs[i].fs_name,TbMfs[i].fs_enable,TbMfs[i].fs_exists,
		TbMfs[i].fs_type);
	}

	snprintf(section,sizeof(section),"%s/power",System);
	PowerEnable	= CfgInt(HtVarLrr,section,-1,"enable",PowerEnable);
	pt		= CfgStr(HtVarLrr,section,-1,"device",PowerDevice);
	if	(!pt || !*pt)	PowerEnable	= 0;
	PowerDownLev	= CfgInt(HtVarLrr,section,-1,"down",PowerDownLev);
	PowerUpLev	= CfgInt(HtVarLrr,section,-1,"up",PowerUpLev);
	if	(PowerDownLev <= 0)		PowerEnable	= 0;
	if	(PowerUpLev <= 0)		PowerEnable	= 0;
	if	(PowerEnable)	PowerDevice	= strdup(pt);

	snprintf(section,sizeof(section),"%s/temperature",System);
	TempEnable	= CfgInt(HtVarLrr,section,-1,"enable",TempEnable);
	pt		= CfgStr(HtVarLrr,section,-1,"device",TempDevice);
	if	(!pt || !*pt)	TempEnable	= 0;
	if	(TempEnable)
	{
		TempDevice	= strdup(pt);
		TempPowerAdjust	= 
		CfgInt(HtVarLrr,section,-1,"poweradjust",TempPowerAdjust);
		TempExtAdjust	= 
		CfgInt(HtVarLrr,section,-1,"extadjust",TempExtAdjust);
	}
		

	Sx13xxPolling	= CfgInt(HtVarLrr,System,-1,"sx13xxpolling",
							Sx13xxPolling);
	Sx13xxStartDelay= CfgInt(HtVarLrr,System,-1,"sx13xxstartdelay",
							Sx13xxStartDelay);
#if defined(REF_DESIGN_V1)
#if defined(WITH_MULTI_BOARD)
	snprintf(section,sizeof(section),"%s/spidevice",System);
	for (i=0; i<LGW_MAX_BOARD; i++)
	{
		SpiDevice[i]	= CfgStr(HtVarLrr,section,i,"spidevice", "");
		// TODO: check why using setenv
		if (SpiDevice[i] && *SpiDevice[i])
		{
			char	tmp[40];
			if (i == 0)
				setenv("SPIDEVICE", strdup(SpiDevice[i]),1);
			snprintf(tmp,sizeof(tmp),"SPIDEVICE%d", i);
			setenv(tmp, strdup(SpiDevice[i]),1);
		}
		RTL_TRDBG(1,"SpiDevice[%d] '%s' (system=%s)\n", i, SpiDevice[i], System);
	}
#else
	SpiDevice	= CfgStr(HtVarLrr,System,-1,"spidevice", SpiDevice);
	if (SpiDevice && *SpiDevice)
	{
		setenv("SPIDEVICE", strdup(SpiDevice),1);
	}
	RTL_TRDBG(1,"SpiDevice '%s' (system=%s)\n", SpiDevice, System);
#endif // REF_DESIGN_V1
#elif defined(REF_DESIGN_V2)
	snprintf(section,sizeof(section),"%s/spidevice",System);
	for (i=0; i<SX1301AR_MAX_BOARD_NB; i++)
	{
		SpiDevice[i]	= CfgStr(HtVarLrr,section,i,"device", "");
		// TODO: check why using setenv
		if (SpiDevice[i] && *SpiDevice[i])
		{
			char	tmp[40];
			if (i == 0)
				setenv("SPIDEVICE", strdup(SpiDevice[i]),1);
			snprintf(tmp,sizeof(tmp),"SPIDEVICE%d", i);
			setenv(tmp, strdup(SpiDevice[i]),1);
		}
		RTL_TRDBG(1,"SpiDevice[%d] '%s' (system=%s)\n", i, SpiDevice[i], System);
	}
#endif /* defined(REF_DESIGN_V2) */

#ifdef	WITH_TTY
	TtyDevice	= CfgStr(HtVarLrr,System,-1,"ttydevice", TtyDevice);
	if (TtyDevice && *TtyDevice)
		setenv("TTYDEVICE", strdup(TtyDevice),1);
	RTL_TRDBG(1,"TtyDevice '%s' (system=%s)\n", TtyDevice, System);
#endif

	IfaceDaemon	= CfgInt(HtVarLrr,"ifacefailover",-1,"enable",
								IfaceDaemon);
	AutoRebootTimer	= CfgInt(HtVarLrr,"lrr",-1,"autoreboottimer",
							AutoRebootTimer);
	if	(AutoRebootTimer > 0 && AutoRebootTimer < 60)
		AutoRebootTimer	= 60;

	AutoRestoreNoLrc	= CfgInt(HtVarLrr,"lrr",-1,"autorestore_nolrc",
							AutoRestoreNoLrc);

	AutoRebootTimerNoUplink = CfgInt(HtVarLrr, "lrr", -1, "autoreboottimer_nouplink", AutoRebootTimerNoUplink);

	if	(AutoRebootTimerNoUplink > 0 && AutoRebootTimerNoUplink < 60)
		AutoRebootTimerNoUplink = 60;

	AutoRestartLrrMaxNoUplink = CfgInt(HtVarLrr, "lrr", -1, "autorestartlrrcnt_nouplink", AutoRestartLrrMaxNoUplink);

	AutoRevSshTimer	= CfgInt(HtVarLrr,"lrr",-1,"autoreversesshtimer",
							AutoRevSshTimer);
	if	(AutoRevSshTimer > 0 && AutoRevSshTimer < 60)
		AutoRevSshTimer	= 60;

	PingRttPeriod	= CfgInt(HtVarLrr,"lrr",-1,"pingrttperiod",
							PingRttPeriod);
	if	(PingRttPeriod > 0 && PingRttPeriod < 60)
		PingRttPeriod	= 60;

	strid		= CfgStr(HtVarLrr,"lrr",-1,"uid","0");
	LrrID		= DoLrrID(strid,&LrrIDExt,&LrrIDPref);
	// rdtp-6639
	AutoRevSshPort	= 50000 + (LrrID % 10000);
	// NFR684
	stridmode	= CfgStr(HtVarLrr,"lrr",-1,"uidmode","local");
	DoLrrIDFromTwa(stridmode);
	if	(LrrIDGetFromTwa == 2)
	{	// rdtp-6639
		DoLrrUIDAutoRevSshPort();
		// Use temporarily lrrid 0xffffffff to request the good lrrid
		if (LrrID == 0)
		{
			if ((LrrID = GetLrrIDIfCellular()))
				RTL_TRDBG(1,"Cellular interface, set temporarily lrrid=%08x\n", LrrID);
		}
	}

	if (LrrIDGetFromBS)	// NFR997
		LrrID	= (u_int) CfgInt(HtVarLrr,"lrr",-1,"uidfrombootsrv",0);

{
RTL_TRDBG(1,"LrrID '%s' lrrid=%u lrrid=%08x lrridext=%04x lrridpref=%02x\n",
		strid,LrrID,LrrID,LrrIDExt,LrrIDPref);
RTL_TRDBG(1,"ism='%s' ismused='%s'\n",
		IsmBand,IsmBandAlter);
RTL_TRDBG(1,"asym=%d asymmodulo=%d asymoffset=%d\n",
		IsmAsymDownlink,IsmAsymDnModulo,IsmAsymDnOffset);
RTL_TRDBG(1,"lat=%f lon=%f alt=%d Dev='%s' UseGpsPosition=%d\n",
		LrrLat,LrrLon,LrrAlt,GpsDevice,UseGpsPosition);
RTL_TRDBG(1,"UseGpsTime=%d UseLgwTime=%d GpsMaxBadChecksum=%d\n",
		UseGpsTime,UseLgwTime,GpsMaxBadChecksum);
RTL_TRDBG(1,"nblrc=%d redundancy=%d simulN=%d qosalea=%d\n",
		NbLrc,Redundancy,SimulN,QosAlea);
RTL_TRDBG(1,"sx13xxpolling=%d sx13xxstartdelay=%d\n",
		Sx13xxPolling,Sx13xxStartDelay);
RTL_TRDBG(1,"ifacedaemon=%d autoreboottimer=%d autorestore=%d autoreversesshtimer=%d\n",
		IfaceDaemon,AutoRebootTimer,AutoRestoreNoLrc,AutoRevSshTimer);
RTL_TRDBG(1,"adjustfinetimestamp=%d\n",
		AdjustFineTimeStamp);
}

	for	(i = 0 ; i < NB_ANTENNA ; i++)
	{
		AntennaGain[i] = atof(CfgStr(HtVarLrr,"antenna",i,"gain", "0"));
		CableLoss[i] = atof(CfgStr(HtVarLrr,"antenna",i,"cableloss", "0"));
		if (CableLoss[i] == 0.0)
			CableLoss[i] = atof(CfgStr(HtVarLrr,"antenna",i,"cablelost", "0"));
		if (AntennaGain[i] > ANTENNA_GAIN_MAX) {
			RTL_TRDBG(1, "Warning: use antenna gain max %.1f dBm\n", ANTENNA_GAIN_MAX);
			AntennaGain[i] = ANTENNA_GAIN_MAX;
		}
		if (CableLoss[i] > ANTENNA_GAIN_MAX) {
			RTL_TRDBG(1, "Warning: use antenna cable loss max %.1f dBm\n", ANTENNA_GAIN_MAX);
			CableLoss[i] = ANTENNA_GAIN_MAX;
		}
		RTL_TRDBG(1, "Antenna %d gain=%.1f dBm cableloss=%.1f dBm\n", i, AntennaGain[i], CableLoss[i]);
	}

	OnConfigErrorExit = CfgInt(HtVarLrr,"exitonerror",-1,"configure",0);
	OnStartErrorExit = CfgInt(HtVarLrr,"exitonerror",-1,"start",0);

	/* NFR590, RDTP-7849 */
	AutoVersion_Lrr       = CfgStr(HtVarLrr, "versions", -1, "lrr_version", "");
	AutoVersion_LrrId     = CfgStr(HtVarLrr, "versions", -1, "lrrid", "");
	AutoVersion_Hal       = CfgStr(HtVarLrr, "versions", -1, "hal_version", "");
	AutoVersion_Fpga      = CfgStr(HtVarLrr, "versions", -1, "fpga_version", "");
	AutoVersion_Fw        = CfgStr(HtVarLrr, "versions", -1, "firmware_version", "");
	AutoVersion_Hw        = CfgStr(HtVarLrr, "versions", -1, "hardware_version", "");
	AutoVersion_Os        = CfgStr(HtVarLrr, "versions", -1, "os_version", "");
	AutoVersion_Sn        = CfgStr(HtVarLrr, "versions", -1, "serial_number", "");
	AutoVersion_Sku       = CfgStr(HtVarLrr, "versions", -1, "sku", "");
	AutoVersion_ChipMsb   = CfgStr(HtVarLrr, "versions", -1, "chip_id_msb", "");
	AutoVersion_ChipLsb   = CfgStr(HtVarLrr, "versions", -1, "chip_id_lsb", "");
	CustomVersion_Build   = CfgStr(HtVarLrr, "versions", -1, "custom_build_version", "");
	CustomVersion_Config  = CfgStr(HtVarLrr, "versions", -1, "configuration_version", "");
	CustomVersion_Custom1 = CfgStr(HtVarLrr, "versions", -1, "custom1_version", "");
	CustomVersion_Custom2 = CfgStr(HtVarLrr, "versions", -1, "custom2_version", "");
	CustomVersion_Custom3 = CfgStr(HtVarLrr, "versions", -1, "custom3_version", "");


	/*
	RTL_TRDBG(1, "Versions:\n\tlrr=%d.%d.%d_%d\n\thardware=%s\n\tos=%s\n\thal=%s\n\tcustom_build=%s\n\tconfiguration=%s\n\tcustom1=%s\n\tcustom2=%s\n\tcustom3=%s\n", \
		VersionMaj, VersionMin, VersionRel, VersionFix, \
		AutoVersion_Hw, AutoVersion_Os, AutoVersion_Hal, CustomVersion_Build, \
		CustomVersion_Config, CustomVersion_Custom1, CustomVersion_Custom2, CustomVersion_Custom3);
	*/
	RTL_TRDBG(1, "Version:\n\tlrr=%s\n\thal=%s\n\tfpga=%s\n\tfw=%s\n\thw=%s\n\tos=%s\n\tsn=%s\n\tsku=%s\n\tchip_msb=%s\n\tchip_lsb=%s\n\tbuild=%s\n\tconfig=%s\n\tcustom1=%s\n\tcustom2=%s\n\tcustom3=%s\n\t\n",\
	AutoVersion_Lrr, AutoVersion_Hal, AutoVersion_Fpga, \
	AutoVersion_Fw, AutoVersion_Hw, \
	AutoVersion_Os, AutoVersion_Sn, AutoVersion_Sku, \
	AutoVersion_ChipMsb, AutoVersion_ChipLsb, \
	CustomVersion_Build, CustomVersion_Config, \
	CustomVersion_Custom1, CustomVersion_Custom2, CustomVersion_Custom3);


	// RDTP-2413
	NwkfEnable	= (u_int)CfgInt(HtVarLrr,
		"nwkfilter",-1,"enable",NwkfEnable);
	NwkfReloadPeriod= (u_int)CfgInt(HtVarLrr,
		"nwkfilter",-1,"reloadperiod",NwkfReloadPeriod);
	NwkfRequestPeriod= (u_int)CfgInt(HtVarLrr,
		"nwkfilter",-1,"requestperiod",NwkfRequestPeriod);
	NwkfRequestStart= (u_int)CfgInt(HtVarLrr,
		"nwkfilter",-1,"requeststart",NwkfRequestStart);

	if	(CfgInt(HtVarLrr,"labonly",-1,"justreadconfig",0)>=1)
		exit(1);
}

// search bootserver.ini
static	int	LoadConfigBootSrv(int dumponly)
{
	int	err;
	char	*file;

	file	= DoConfigFileCustom(ConfigFileBootSrv);
	if	(!file)
		return 0;

	RTL_TRDBG(0,"load custom '%s'\n",file);

	if	((err=rtl_iniParse(file,CfgCBIniLoad,HtVarLrr)) < 0)
	{
		RTL_TRDBG(0,"parse '%s' error=%d\n",file,err);
		exit(1);
	}

	rtl_htblDump(HtVarLrr,CBHtDumpLrr);

	if	(!dumponly)
		DecryptHtbl(HtVarLrr);

	return 1;
}

static	void	CBHtDumpLgw(char *var,void *value)
{
	RTL_TRDBG(9,"var='%s' val='%s'\n",var,(char *)rtl_htblGet(HtVarLgw,var));
}

static	void	LoadConfigLgw(int hot)
{
	int	err;
	char	*file;
	file	= DoConfigFileDefault(ConfigFileLowLvLgw,NULL);
	if	(file)
	{
		RTL_TRDBG(0,"load default '%s'\n",file);
		if	((err=rtl_iniParse(file,CfgCBIniLoad,HtVarLgw)) < 0)
		{
			RTL_TRDBG(0,"parse '%s' error=%d\n",file,err);
			exit(1);
		}
	}
	file	= DoConfigFileDefault(ConfigFileLowLvLgw,System);
	if	(file)
	{
		RTL_TRDBG(0,"load default '%s'\n",file);
		if	((err=rtl_iniParse(file,CfgCBIniLoad,HtVarLgw)) < 0)
		{
			RTL_TRDBG(0,"parse '%s' error=%d\n",file,err);
			exit(1);
		}
	}

	file	= DoConfigFileDefault(ConfigFileLgw,IsmBandAlter);
	if	(file)
	{
		RTL_TRDBG(0,"load default '%s'\n",file);
		if	((err=rtl_iniParse(file,CfgCBIniLoad,HtVarLgw)) < 0)
		{
			RTL_TRDBG(0,"parse '%s' error=%d\n",file,err);
			exit(1);
		}
	}
	else
	{
		file	= DoConfigFileDefault(ConfigFileLgw,IsmBand);
		if	(file)
		{
			RTL_TRDBG(0,"load default '%s'\n",file);
			if	((err=rtl_iniParse(file,CfgCBIniLoad,HtVarLgw)) < 0)
			{
				RTL_TRDBG(0,"parse '%s' error=%d\n",file,err);
				exit(1);
			}
		}
	}

	file	= DoConfigFileCustom(ConfigFileLowLvLgw);
	if	(file)
	{
		RTL_TRDBG(0,"load custom '%s'\n",file);
		if	((err=rtl_iniParse(file,CfgCBIniLoad,HtVarLgw)) < 0)
		{
			RTL_TRDBG(0,"parse '%s' error=%d\n",file,err);
			exit(1);
		}
	}


#if defined(REF_DESIGN_V2)
	file	= DoConfigFileCustom(ConfigFileLgwCustom);
#else
	file	= DoConfigFileCustom(ConfigFileLgw);
#endif
	if	(file)
	{
		RTL_TRDBG(0,"load custom '%s'\n",file);
	}
	if	(file && (err=rtl_iniParse(file,CfgCBIniLoad,HtVarLgw)) < 0)
	{
		RTL_TRDBG(0,"parse '%s' error=%d\n",file,err);
		exit(1);
	}

	file	= DoConfigFileCustom(ConfigFileDynCalib);
	if	(file)
	{
		RTL_TRDBG(0,"load custom '%s'\n",file);
		if	((err=rtl_iniParse(file,CfgCBIniLoad,HtVarLgw)) < 0)
		{
			RTL_TRDBG(0,"parse '%s' error=%d\n",file,err);
			exit(1);
		}
	}

	rtl_htblDump(HtVarLgw,CBHtDumpLgw);
}

static	void	LoadConfigChannel(int hot)
{
	int	err;
	char	*file;
	char	*pt;
	u_int	vers;

#if	0
	if	(RfRegionId && *RfRegionId)
#endif
	// here variable RfRegionId is not yet affected, but HtVarLgw is loaded
	// => test the key/ini instead of the variable
	pt	= CfgStr(HtVarLgw,"gen",-1,"rfregionid","");
	vers		= CfgInt(HtVarLgw,"gen",-1,"rfregionidvers",0);
	if	(pt && *pt)
	{
		RTL_TRDBG(0,"rfregionid='%s.%u' dont use default channels cfg\n",
			pt,vers);
		goto	onlycustom;
	}

	file	= DoConfigFileDefault(ConfigFileChannel,IsmBandAlter);
	if	(file)
	{
		RTL_TRDBG(0,"load default '%s'\n",file);
		if	((err=rtl_iniParse(file,CfgCBIniLoad,HtVarLgw)) < 0)
		{
			RTL_TRDBG(0,"parse '%s' error=%d\n",file,err);
			exit(1);
		}
	}
	else
	{
		file	= DoConfigFileDefault(ConfigFileChannel,IsmBand);
		if	(file)
		{
			RTL_TRDBG(0,"load default '%s'\n",file);
			if	((err=rtl_iniParse(file,CfgCBIniLoad,HtVarLgw)) < 0)
			{
				RTL_TRDBG(0,"parse '%s' error=%d\n",file,err);
				exit(1);
			}
		}
	}
onlycustom:
	file	= DoConfigFileCustom(ConfigFileChannel);
	if	(file)
	{
		RTL_TRDBG(0,"load custom '%s'\n",file);
	}
	if	(file && (err=rtl_iniParse(file,CfgCBIniLoad,HtVarLgw)) < 0)
	{
		RTL_TRDBG(0,"parse '%s' error=%d\n",file,err);
		exit(1);
	}

	rtl_htblDump(HtVarLgw,CBHtDumpLgw);

}

static	void	ServiceStatusResponse()
{
	FILE	*f;
	char	tmp[256];

	f	= fopen(SERVICE_STATUS_FILE,"w");
	if	(f)
	{
		snprintf(tmp,sizeof(tmp),"STATUS=RUNNING+SIG+IMSG");
		fprintf	(f,"%s",tmp);
		fclose	(f);
		RTL_TRDBG(0,"Service status '%s'\n",tmp);
	}
}

static	void	ServiceStatus(int sig)
{
	FILE	*f;
	char	tmp[256];

	// as lgw thread can have lock a mutex to write traces
	// do not write trace

//	signal	(SIGUSR1,SIG_IGN);
//	RTL_TRDBG(0,"Service status sig=%d\n",sig);

	f	= fopen(SERVICE_STATUS_FILE,"w");
	if	(f)
	{
		snprintf(tmp,sizeof(tmp),"STATUS=RUNNING+SIG");
		fprintf	(f,"%s",tmp);
		fclose	(f);
//		RTL_TRDBG(0,"Service status '%s'\n",tmp);
	}
	rtl_imsgAdd(MainQ,
		rtl_imsgAlloc(IM_DEF,IM_SERVICE_STATUS_RQST,NULL,0));

//	signal	(SIGUSR1,ServiceStatus);
}

static	pthread_attr_t	AbortThreadAt;
static	pthread_t	AbortThread;

static	void	*ForceAbort(void *param)
{
	sleep(15);
	abort();
	return	NULL;
}

static	void	ServiceStop(int sig)
{
	if	(MainThreadId != pthread_self())
		return;

	// as CancelXXXThread() or LgwStop() sometime block
	// start a other thread to force an abort()
	pthread_attr_init(&AbortThreadAt);
	pthread_create(&AbortThread,&AbortThreadAt,ForceAbort,NULL);

	// as lgw thread can have lock a mutex to write traces
	// do not write trace while lgw thread is not dead

	rtl_traceunlock();
	ServiceStopped	= 1;
	CancelLgwThread();
#ifdef WITH_GPS
	CancelGpsThread();
#endif
	RTL_TRDBG(0,"service stopping sig=%d ...\n",sig);

#ifndef	WITH_TTY
	if	(LgwStarted())
		LgwStop();
#endif

	RTL_TRDBG(0,"LgwNbPacketSend=%u\n",LgwNbPacketSend);
	RTL_TRDBG(0,"LgwNbPacketWait=%u\n",LgwNbPacketWait);
	RTL_TRDBG(0,"LgwNbPacketRecv=%u\n",LgwNbPacketRecv);

	RTL_TRDBG(0,"LgwNbStartOk=%u\n",LgwNbStartOk);
	RTL_TRDBG(0,"LgwNbStartFailure=%u\n",LgwNbStartFailure);
	RTL_TRDBG(0,"LgwNbConfigFailure=%u\n",LgwNbConfigFailure);
	RTL_TRDBG(0,"LgwNbLinkDown=%u\n",LgwNbLinkDown);


	RTL_TRDBG(0,"LgwNbFilterDrop=%u\n",LgwNbFilterDrop);
	RTL_TRDBG(0,"LgwNbBusySend=%u\n",LgwNbBusySend);
	RTL_TRDBG(0,"LgwNbSyncError=%u\n",LgwNbSyncError);
	RTL_TRDBG(0,"LgwNbCrcError=%u\n",LgwNbCrcError);
	RTL_TRDBG(0,"LgwNbSizeError=%u\n",LgwNbSizeError);
	RTL_TRDBG(0,"LgwNbChanUpError=%u\n",LgwNbChanUpError);
	RTL_TRDBG(0,"LgwNbChanDownError=%u\n",LgwNbChanDownError);
	RTL_TRDBG(0,"LgwNbDelayError=%u\n",LgwNbDelayError);
	RTL_TRDBG(0,"LgwNbDelayReport=%u\n",LgwNbDelayReport);
	RTL_TRDBG(0,"MacNbFcsError=%u\n",MacNbFcsError);
	RTL_TRDBG(0,"LrcNbPktDrop=%u\n",LrcNbPktDrop);
	RTL_TRDBG(0,"LrcNbDisc=%u\n",LrcNbDisc);

	rtl_htblDestroy(HtVarLrr);	
	rtl_htblDestroy(HtVarLgw);	

	sleep(1);
	exit(0);
}

static	char	*DoFilePid()
{
	static	char	file[128];

	snprintf(file,sizeof(file),"/var/run/%s.pid",SERVICE_NAME);
	return	file;
}

static	void	ServiceWritePid()
{
	FILE	*f;

	f	= fopen(DoFilePid(),"w");
	if	(f)
	{
		fprintf(f,"%d\n",getpid());
		fclose(f);
	}
}


#ifdef WITH_GPS
static void StartGpsThread()
{
// For tektelic no access to device, just use "/tmp/position" for coordinates
#ifdef  KONA
	return;
#endif
	int thr_status = -1;
	if ((thr_status = GpsThreadStatus()) == 1) {
		RTL_TRDBG(0, "gps thread already started\n");
		return;
	}
	if (pthread_attr_init(&GpsThreadAt)) {
		RTL_TRDBG(0, "cannot init gps thread attr err=%s\n", STRERRNO);
		exit(1);
	}

	if (pthread_create(&GpsThread, &GpsThreadAt, GpsRun, NULL)) {
		RTL_TRDBG(0, "cannot create gps thread err=%s\n", STRERRNO);
		exit(1);
	}
	RTL_TRDBG(0, "gps thread is started\n");

}
#endif /* WITH_GPS */

#ifdef WITH_GPS
static void CancelGpsThread()
{
// For tektelic no access to device, just use "/tmp/position" for coordinates
#ifdef  KONA
	return;
#endif
	int thr_status = -1;
	if ((thr_status = GpsThreadStatus()) == 1) {
		void * res;
		pthread_cancel(GpsThread);
		pthread_join(GpsThread, &res);
		if (res == PTHREAD_CANCELED) {
			RTL_TRDBG(0, "gps thread was canceled\n");
		}
		else {
			RTL_TRDBG(0, "gps thread was not canceled !!! (res=%d)\n", res);
		}
	}
	else
	{
		RTL_TRDBG(0, "gps thread already canceled\n");
	}
}
#endif /* WITH_GPS */

#ifdef WITH_GPS
void ReStartGpsThread()
{
	CancelGpsThread();
	StartGpsThread();
}
#endif /* WITH_GPS */

// check if there is this type of interface enabled
static int IsTypeInterface(int typ)
{
	t_wan_itf	*itf;
	int		i;

	// check if there is this type of interface enabled
	for	(i = 0; i < NB_ITF_PER_LRR ; i++)
	{
		itf	= &TbItf[i];
		if	(!itf)
			continue;
		if (itf->it_type == typ && itf->it_enable)
		{
			RTL_TRDBG(1,"Interface of type %d enabled found (num=%d name=%s)\n",
				typ, i, (itf->it_name?itf->it_name:"???"));
			return 1;
		}
	}
	RTL_TRDBG(4,"No interface of type %d enabled found\n", typ);
	return 0;
}

// start thread that gets informations about the cellular connection
static void StartCellStateThread()
{
	if (CellStateThreadStatus() == 1) {
		RTL_TRDBG(0, "CellState thread already started\n");
		return;
	}
	if (pthread_attr_init(&CellStateThreadAt)) {
		RTL_TRDBG(0, "cannot init CellState thread attr err=%s\n", STRERRNO);
		exit(1);
	}

	if (pthread_create(&CellStateThread, &CellStateThreadAt, CellStateRun, NULL)) {
		RTL_TRDBG(0, "cannot create CellState thread err=%s\n", STRERRNO);
		exit(1);
	}
	RTL_TRDBG(0, "CellState thread is started\n");

}

// cancel thread that gets informations about the cellular connection
static void CancelCellStateThread()
{
	if (CellStateThreadStatus() == 1) {
		void * res;
		pthread_cancel(CellStateThread);
		pthread_join(CellStateThread, &res);
		if (res == PTHREAD_CANCELED) {
			RTL_TRDBG(0, "CellState thread was canceled\n");
		}
		else {
			RTL_TRDBG(0, "CellState thread was not canceled !!! (res=%d)\n", res);
		}
	}
	else
	{
		RTL_TRDBG(0, "CellState thread already canceled\n");
	}
}

// restart thread that gets informations about the cellular connection
void ReStartCellStateThread()
{
	CancelCellStateThread();
	StartCellStateThread();
}

static	void	StartLgwThread()
{
	if	(LgwThreadStarted)
	{
		RTL_TRDBG(0,"lgw thread already started\n");
		return;
	}
	if	(pthread_attr_init(&LgwThreadAt))
	{
		RTL_TRDBG(0,"cannot init thread attr err=%s\n",STRERRNO);
		exit(1);
	}

	if	(pthread_create(&LgwThread,&LgwThreadAt,LgwRun,NULL))
	{
		RTL_TRDBG(0,"cannot create thread err=%s\n",STRERRNO);
		exit(1);
	}
	LgwThreadStarted	= 1;
	RTL_TRDBG(0,"lgw thread is started\n");
}

static	void	CancelLgwThread()
{
	if	(LgwThreadStarted)
	{
		void	*res;
		LgwThreadStarted	= 0;
#if	0
		// as lgw thread can have lock a mutex to write traces
		// do not write trace while lgw thread is not dead
		RTL_TRDBG(0,"sending cancel to lgw thread ...\n");
#endif
		pthread_cancel(LgwThread);
		pthread_join(LgwThread,&res);
		if	(res == PTHREAD_CANCELED)
		{
			RTL_TRDBG(0,"lgw thread was canceled\n");
		}
		else
		{
			RTL_TRDBG(0,"lgw thread was not canceled !!!\n");
		}
	}
	else
	{
		RTL_TRDBG(0,"lgw thread already canceled\n");
	}
}

void	ReStartLgwThread()
{
	CancelLgwThread();
	LgwStop();
	char * IsmBandBck = strdup(IsmBand);
	char * IsmBandAlterBck = strdup(IsmBandAlter);
	rtl_htblReset(HtVarLgw);

	if (strncmp(IsmBand, IsmBandBck, strlen(IsmBandBck)))
		IsmBand=strdup(IsmBandBck);
	if (strncmp(IsmBandAlter, IsmBandAlterBck, strlen(IsmBandAlterBck)))
		IsmBandAlter=strdup(IsmBandAlterBck);

	free(IsmBandBck);
	free(IsmBandAlterBck);

	LoadConfigDefine(1,1);
	LoadConfigLgw(1);
	LoadConfigChannel(1);
	LgwGenConfigure(1,1);
	ChannelConfigure(1,1);
	DownRadioStop		= 0;
	LgwThreadStopped	= 0;
	StartLgwThread();
}

#if	0
static	void	AveragePktTrip(u_int delta)
{
	static	int	cumDelta;
	static	u_short	count	= 1;

	if	(delta > LrcMxPktTrip)
		LrcMxPktTrip	= delta;

	cumDelta= cumDelta + delta;
	if	(cumDelta < 0)	// overflow => reset
	{
		LrcMxPktTrip	= 0;
		cumDelta	= delta;
		count		= 1;
	}
	if	(count == 0)	// loop => reset
	{
		LrcMxPktTrip	= 0;
		cumDelta	= delta;
		count	= 1;
	}

	LrcAvPktTrip	= cumDelta / count;
	count++;
}
#endif

#ifdef	LP_TP31
int	IsIndicSet(t_lrr_pkt *pkt)
{
	if	(pkt->lp_delivered > 0 || pkt->lp_cause1 > 0 || pkt->lp_cause2 > 0|| pkt->lp_cause3 > 0)
		return 1;
	return 0;
}

void	SetIndic(t_lrr_pkt *pkt,int delivered,int c1,int c2,int c3)
{
	if	(delivered >= 0)
		pkt->lp_delivered	= delivered;
	if	(c1 >= 0)
		pkt->lp_cause1		= c1;
	if	(c2 >= 0)
		pkt->lp_cause2		= c2;
	if	(c3 >= 0)
		pkt->lp_cause3		= c3;
}

void	SendIndicToLrc(t_lrr_pkt *pkt)
{
	u_char		buff[1024];
	int		szh;
	t_imsg		*msg;
	int		sz;

	if	(!pkt)
		return;
	if	(pkt->lp_beacon)
		return;
	if	(pkt->lp_synchro)	// RDTP-5475
		return;
	if	(pkt->lp_type != LP_TYPE_LRR_PKT_RADIO)
		return;
	if	(pkt->lp_deveui == 0)
		return;

	if	(pkt->lp_trip > 0xFFFF)
		pkt->lp_trip	= 0xFFFF;

	if	(MainThreadId == pthread_self())
	{
		int	rx2;

		// RDTP-15129
		if (pkt->lp_delivered > 0 && pkt->lp_cause3)
		{
			RTL_TRDBG(1,"force cause3=%02x to 00\n", pkt->lp_cause3);
			pkt->lp_cause3 = 0;
		}

		rx2	= (pkt->lp_flag & LP_RADIO_PKT_RX2) == LP_RADIO_PKT_RX2;
RTL_TRDBG(1,"PKT SEND INDIC deveui=%016llx fcnt=%u deliv=%u causes=(%02x,%02x,%02x) rtt=%u channel=%u spfact=%u rx2=%d bw=%d/%dK freq=%u\n",
		pkt->lp_deveui,pkt->lp_fcntdn,
		pkt->lp_delivered,pkt->lp_cause1,pkt->lp_cause2,pkt->lp_cause3,
		pkt->lp_trip,pkt->lp_channel,pkt->lp_spfact,rx2,
		pkt->lp_bandwidth,(int)(FreqBandWidth(pkt->lp_bandwidth)/1000),
		pkt->lp_freq);

		szh	 = LP_PRE_HEADER_PKT_SIZE+sizeof(t_lrr_pkt_sent_indic);
		pkt->lp_szh	= szh;
		pkt->lp_type	= LP_TYPE_LRR_PKT_SENT_INDIC;
		pkt->lp_u.lp_sent_indic.lr_u.lr_indic.lr_delivered	= 
							pkt->lp_delivered;
		pkt->lp_u.lp_sent_indic.lr_u.lr_indic.lr_cause1		= 
							pkt->lp_cause1;
		pkt->lp_u.lp_sent_indic.lr_u.lr_indic.lr_cause2		= 
							pkt->lp_cause2;
		pkt->lp_u.lp_sent_indic.lr_u.lr_indic.lr_cause3		= 
							pkt->lp_cause3;
		pkt->lp_u.lp_sent_indic.lr_u.lr_indic.lr_trip		= 
							pkt->lp_trip;

		memcpy	(buff,pkt,szh);
		SendToLrc(NULL,buff,szh+0);
	}
	else
	{
	RTL_TRDBG(3,"PKT POST INDIC deveui=%016llx fcnt=%u deliv=%u causes=(%02x,%02x,%02x)\n",
		pkt->lp_deveui,pkt->lp_fcntdn,
		pkt->lp_delivered,pkt->lp_cause1,pkt->lp_cause2,pkt->lp_cause3);

		msg	= rtl_imsgAlloc(IM_DEF,IM_LGW_SENT_INDIC,NULL,0);
		if	(!msg)
			return;
		sz	= sizeof(t_lrr_pkt);
		if	( rtl_imsgDupData(msg,pkt,sz) != msg)
		{
			rtl_imsgFree(msg);
			return;
		}
		rtl_imsgAdd(MainQ,msg);
	}
}
#else
int	IsIndicSet(t_lrr_pkt *pkt)
{
	return 0;
}

void	SetIndic(t_lrr_pkt *pkt,int delivered,int c1,int c2,int c3)
{
	return;
}
void	SendIndicToLrc(t_lrr_pkt *pkt)
{
	return;
}
#endif

#ifdef	WITH_TRACK_TIC
static	int	TrackTicCheck(t_lrr_pkt *downpkt)
{
	int	i;
	int	found	= 0;

	if	(downpkt->lp_classb || downpkt->lp_classc || downpkt->lp_classcmc)
	{
RTL_TRDBG(2,"MAC SEND tms=%u tus=%u no check in track b=%d c=%d mc=%d\n",
		downpkt->lp_tms,downpkt->lp_tus,
		downpkt->lp_classb,downpkt->lp_classc,downpkt->lp_classcmc);
		return	1;
	}

	for	(i = 0 ; !found && i < TrackTicNb ; i++)
	{
		if	(TrackTic[i][0] == downpkt->lp_tms
			&&	TrackTic[i][1] == downpkt->lp_tus)
		{
			found	= 1;
		}
	}
	if	(!found)
	{
RTL_TRDBG(0,"MAC SEND tms=%u tus=%u error not found in track\n",downpkt->lp_tms,
							downpkt->lp_tus);
	}
	else
	{
RTL_TRDBG(1,"MAC SEND tms=%u tus=%u retrieved in track\n",downpkt->lp_tms,
							downpkt->lp_tus);
	}
	return	found;
}
#endif

//
//	send a downlink packet to the radio thread
//
static	void	SendRadioPacket(t_lrr_pkt *downpkt,u_char *data,int sz)
{
	u_char	buff[1024];
	u_char	crcbuff[1024];
	u16	crc	= 0;

	char	dst[64];
#if	0
	char	src[64];
	int	ret;
#endif
	int	osz;
	int	ack = 0;
	int	m802 = 0;
	int	lrcdelay = 0;
	int	seqnum = 0;
	int	watteco;
	int	majv;
	int	minv;
	int	delay;
	int	ret;
	int	classb;

	ret	= LgwStarted();
	if	(LgwThreadStopped || !LgwThreadStarted || !ret)
	{
		RTL_TRDBG(1,"Radio thread KO start=%d cmdstart=%d cmdstop=%d\n",
			ret,LgwThreadStarted,LgwThreadStopped);
		SetIndic(downpkt,0,
		LP_C1_RADIO_STOP,LP_C2_RADIO_STOP,LP_CB_RADIO_STOP);
		SendIndicToLrc(downpkt);
		return;
	}

#if defined(KONA) && defined(WITH_GPS) 	// do not send anything before 3 min !
	if	(DownRadioStop || !LgwDlReady())
#else
	if	(DownRadioStop)
#endif
	{
		if (!DownRadioStop)
		{
			RTL_TRDBG(1,"Tektelic: downlink disabled for 3 minutes after radio startup\n");
		}
		else
		{
			RTL_TRDBG(1,"Radio stopped in downlink direction\n");
		}
		SetIndic(downpkt,0,
		LP_C1_RADIO_STOP_DN,LP_C2_RADIO_STOP_DN,LP_CB_RADIO_STOP_DN);
		SendIndicToLrc(downpkt);
		return;
	}

	if	(IsmAsymDownlink /* || strcmp(IsmBand,"us915") == 0 */)
	{
		if	(downpkt->lp_channel <= MAXUP_CHANNEL_IDX)
		{
			u_int	mindnchan	= (MAXUP_CHANNEL_IDX+1); // 127
			u_char	downchan;

			downchan = (downpkt->lp_channel%IsmAsymDnModulo);
			downchan += mindnchan + IsmAsymDnOffset;

			RTL_TRDBG(1,"MAC SEND channel conv %d -> %d (us915)\n",
				downpkt->lp_channel,downchan);
			downpkt->lp_channel	= downchan;
		}
	}

	ack	= (downpkt->lp_flag&LP_RADIO_PKT_ACKMAC)==LP_RADIO_PKT_ACKMAC;
	m802	= (downpkt->lp_flag&LP_RADIO_PKT_802154)==LP_RADIO_PKT_802154;
	lrcdelay= (downpkt->lp_flag&LP_RADIO_PKT_DELAYED)==LP_RADIO_PKT_DELAYED;
	classb	= (downpkt->lp_flag&LP_RADIO_PKT_PINGSLOT)==LP_RADIO_PKT_PINGSLOT;

	if	(classb)
	{	
		// retrieve slots parameters
		LgwInitPingSlot(downpkt);
		// set slots parameters
		LgwResetPingSlot(downpkt);
	}

	if	(downpkt->lp_postponed)
	{
#ifdef	WITH_TRACK_TIC
		TrackTicCheck(downpkt);
#endif
		RTL_TRDBG(2,"MAC SEND retrieve a postponed downlink packet\n");
		if	((ret=LgwSendPacket(downpkt,seqnum,m802,ack)) < 0)
		{
			RTL_TRDBG(1,"LgwSendPacket() error ret=%d\n",ret);
			SetIndic(downpkt,0,-1,-1,-1);
			SendIndicToLrc(downpkt);
		}
		return;
	}

	if 	(lrcdelay)	// NFR595
	{
		lrcdelay		= downpkt->lp_delay;
		downpkt->lp_tms		= 0;
		downpkt->lp_trip	= 0;
		downpkt->lp_delay	= 0;
		downpkt->lp_classc	= 1;
		downpkt->lp_classcmc	= 0;
	}

	if	(downpkt->lp_tms && downpkt->lp_lk)
	{
		t_xlap_link	*lk;
		t_lrc_link	*lrc;

		lk		= (t_xlap_link *)downpkt->lp_lk;
		lrc		= (t_lrc_link *)lk->lk_userptr;
		downpkt->lp_trip= ABS(rtl_tmmsmono() - downpkt->lp_tms);
#if	0
		AveragePktTrip(downpkt->lp_trip);
#endif
		RTL_TRDBG(9,"AvDvUiAdd p=%p d=%u t=%u\n",
			&lrc->lrc_avdvtrip,downpkt->lp_trip,Currtime.tv_sec);
		AvDvUiAdd(&lrc->lrc_avdvtrip,downpkt->lp_trip,Currtime.tv_sec);
		AvDvUiAdd(&CmnLrcAvdvTrip,downpkt->lp_trip,Currtime.tv_sec);
	}

	if      ((downpkt->lp_flag & LP_RADIO_PKT_DELAY) != LP_RADIO_PKT_DELAY)
	{
		downpkt->lp_delay	= 0;
		downpkt->lp_classc	= 1;
		downpkt->lp_classcmc	= 0;
	}


	osz	= sz;
	if	(m802 && MacWithFcsDown)
	{
		crc	= crc_ccitt(0,data,sz);
		memcpy	(crcbuff,data,sz);
		crcbuff[sz]	= crc & 0x00ff;
		crcbuff[sz+1]	= crc >> 8;
		sz	= sz + 2;
		data	= crcbuff;
	}

	downpkt->lp_size	= sz;
	downpkt->lp_payload	= (u_char *)malloc(sz);
	if	(!downpkt->lp_payload)
	{
RTL_TRDBG(0,"ERROR alloc payload %d\n",sz);
		return;
	}
	memcpy	(downpkt->lp_payload,data,sz);
	TmoaLrrPacket(downpkt);


	RTL_TRDBG(2,"MAC SEND sz=%d m802=%d ack=%d crcccitt=0x%04x tmoa=%fms\n",
			sz,m802,ack,crc,downpkt->lp_tmoa/1000.0);

	if	(lrcdelay)	// NFR595
	{
		t_imsg	*msg;
		int	sz;
		// replace the message in mainQ
		downpkt->lp_postponed	= lrcdelay;
		RTL_TRDBG(2,"MAC SEND LRC request a delayed downlink => postpone=%d\n",
			lrcdelay);
		sz	= sizeof(t_lrr_pkt);
		msg	= rtl_imsgAlloc(IM_DEF,IM_LGW_POST_DATA,NULL,0);
		if	(!msg)
		{
			return;
		}
		if	( rtl_imsgDupData(msg,downpkt,sz) != msg)
		{
			rtl_imsgFree(msg);
			return;
		}
		rtl_imsgAddDelayed(MainQ,msg,lrcdelay);
		return;
	}
	if	(m802 == 0)
	{
		LoRaMAC_t	mf;
		u_char		*pt;
		u_int64_t	deveui	= 0xFFFFFFFFFFFFFFFF;
		u_short		fcntdn	= 0xFFFF;

		memset	(&mf,0,sizeof(mf));
		LoRaMAC_decodeHeader(data,osz,&mf);
		seqnum	= mf.FCnt;
/*
		rtl_binToStr(mf.DevAddr,sizeof(mf.DevAddr),dst,sizeof(dst)-1);
*/

		pt	= (u_char *)&mf.DevAddr;

		snprintf(dst,sizeof(dst),"%02x%02x%02x%02x",*(pt+3),*(pt+2),*(pt+1),*pt);

		watteco	= (downpkt->lp_majorv & 0x80)?1:0;
		majv	= downpkt->lp_majorv & ~0x80;
		minv	= downpkt->lp_minorv;

#ifdef	LP_TP31
		deveui	= downpkt->lp_deveui;
		fcntdn	= downpkt->lp_fcntdn;
#endif

RTL_TRDBG(1,"MACLORA SEND seq=%d ack=%d devaddr=%s w=%d major=%d minor=%d len=%d tmoa=%fms flg='%s' deveui=%016llx/%u\n",
		mf.FCnt,ack,dst,watteco,majv,minv,
		downpkt->lp_size,downpkt->lp_tmoa/1000.0,
		LrrPktFlagsTxt(downpkt->lp_flag),deveui,fcntdn);

		if	(downpkt->lp_classb)
		{
RTL_TRDBG(1,"MACLORA SEND classb window=(%u,%09u) nb=%d slot1=%d slot2=%d c1=%u c2=%u\n",
			downpkt->lp_gss,downpkt->lp_gns,downpkt->lp_nbslot,
			downpkt->lp_firstslot,downpkt->lp_firstslot2,
			downpkt->lp_channel,downpkt->lp_classbchan2);
		}
		else	// RDTP-2543
		{
			char	*class	= "a";
			if	(downpkt->lp_classc)
				class	= "c";
			if	(downpkt->lp_classcmc)
				class	= "mc";
RTL_TRDBG(1,"MACLORA SEND class%s rx1=%u/%u rx2=%u/%u\n",class,
				downpkt->lp_channel,downpkt->lp_spfact,
				downpkt->lp_rx2channel,downpkt->lp_rx2spfact);
		}
	}
	else
	{

RTL_TRDBG(1,"MAC802 SEND tmoa=%fms flg='%s'\n",
		downpkt->lp_tmoa/1000.0,
		LrrPktFlagsTxt(downpkt->lp_flag));

#if	0
		mac802154_t	mf;
		memset	(&mf,0,sizeof(mf));
		ret	= mac802154_parse(data,sz,&mf);
		if	(ret <= 0)
		{
			rtl_binToStr(data,sz,(char *)buff,sizeof(buff)-10);
RTL_TRDBG(0,"ERROR MAC802 SEND data='%s'\n",buff);
			return;
		}
		seqnum	= mf.seq;
		rtl_binToStr(mf.dest_addr,sizeof(mf.dest_addr),dst,sizeof(dst)-1);
		rtl_binToStr(mf.src_addr,sizeof(mf.src_addr),src,sizeof(src)-1);
RTL_TRDBG(2,"MAC802 SEND seq=%d ack=%d dst=%04x/%s src=%04x/%s len=%d\n",
			mf.seq,ack,
			mf.dest_pid,dst,
			mf.src_pid,src,
			mf.payload_len);

RTL_TRDBG(2,"MAC802 SEND '%s'\n", mac802154_ip6typetxt(mac802154_ip6type(&mf)));
#endif
	}

	static	int	fmargin	= -1;
	if	(fmargin == -1)
	{
		fmargin	= 100;	// by default margin feature was activated
		fmargin	= CfgInt(HtVarLrr,"lrr",-1,"dnmargin",fmargin);
	}
	static	int	margin	= -1;
	if	(margin == -1)
	{
		margin	= fmargin;
		margin	= CfgInt(HtVarLrr,System,-1,"dnmargin",margin);
		if	(margin && margin < 100)
			margin	= 100;
		RTL_TRDBG(1,"down link dnmargin=%d\n",margin);
	}
	static	int	margincpu	= -1;
	if	(margin && margincpu == -1)
	{
		margincpu= 75;
		margincpu= CfgInt(HtVarLrr,System,-1,"dnmargincpu",margincpu);
		if	(margincpu < 75)
			margincpu	= 75;
		if	(margincpu > margin)
			margin	= margincpu;
		RTL_TRDBG(1,"down link dnmargin=%d dnmargincpu=%d\n",
							margin,margincpu);
	}

	delay	= downpkt->lp_delay - downpkt->lp_trip;
	if (/*!downpkt->lp_classb &&*/ margin && downpkt->lp_tms && downpkt->lp_delay && delay > margin)
	{
		t_imsg	*msg;
		int	sz;

		// SimulationMode used to force tus value
		if (SimulationMode)
		{
			int	forceTus;
			char	keyval[80];

			// force tus to current time + 'forcetus' ms
			snprintf(keyval,sizeof(keyval), "forcetus%s", dst);
			forceTus = (u_int)CfgInt(HtVarLrr, "lrr", -1, keyval, 0);
			if (forceTus > 0)
			{
				uint32_t	curmono;
				curmono = rtl_tmmsmono();
				RTL_TRDBG(1,"MAC SEND Simulation: forcetus=%d\n", forceTus);
				delay = forceTus;
				downpkt->lp_tus = 0;	// flag it to be set correctly in radio thread
				downpkt->lp_gns = curmono+forceTus;	// time to be transmited
				downpkt->lp_trip = downpkt->lp_delay-forceTus;
				if (downpkt->lp_trip > 10000 || downpkt->lp_trip < 0)
					downpkt->lp_trip = 500;
				downpkt->lp_tms = curmono - downpkt->lp_trip;
			}

		}

		// dont block sx1301 and give "more" chance to small packets
		delay	= delay - margincpu;
		if	(delay < 1)	delay	= 1;
RTL_TRDBG(1,"MAC SEND trip=%d avtrip=%u dvtrip=%u mxtrip=%u long delay postpone=%d in MainQ\n",
		downpkt->lp_trip,LrcAvPktTrip,LrcDvPktTrip,LrcMxPktTrip,delay);
		// replace the message in mainQ
		downpkt->lp_postponed	= delay;
		sz	= sizeof(t_lrr_pkt);
		msg	= rtl_imsgAlloc(IM_DEF,IM_LGW_POST_DATA,NULL,0);
		if	(!msg)
		{
			return;
		}
		if	( rtl_imsgDupData(msg,downpkt,sz) != msg)
		{
			rtl_imsgFree(msg);
			return;
		}
		rtl_imsgAddDelayed(MainQ,msg,delay);
		return;
	}

RTL_TRDBG(1,"MAC SEND trip=%d avtrip=%u dvtrip=%u mxtrip=%u no postpone delay=%d\n",
		downpkt->lp_trip,LrcAvPktTrip,LrcDvPktTrip,LrcMxPktTrip,delay);

	if	(TraceLevel > 3)
	{
		rtl_binToStr(data,sz,(char *)buff,sizeof(buff)-10);
		RTL_TRDBG(4,"MAC SEND\n>>>'%s'\n",buff);
	}

#ifdef	WITH_TRACK_TIC
	TrackTicCheck(downpkt);
#endif
	if	((ret=LgwSendPacket(downpkt,seqnum,m802,ack)) < 0)
	{
		RTL_TRDBG(1,"LgwSendPacket() error ret=%d\n",ret);
		SetIndic(downpkt,0,-1,-1,-1);
		SendIndicToLrc(downpkt);
	}
}


static	void	SendRadioBeacon(t_lrr_pkt *pktbeacon,u_char *data,int sz)
{
	t_lrr_pkt	downpkt;
	t_lrr_beacon_dn	*beacon	= &(pktbeacon->lp_u.lp_beacon_dn);

	struct	timeval	tv;
	char	when[128];
	int	ret;

	u_int	freqdn	= 0;

	memset	(&downpkt,0,sizeof(t_lrr_pkt));
	memcpy	(&downpkt,pktbeacon,LP_PRE_HEADER_PKT_SIZE);

	// as we use ON_GPS mode a beacon can only be sent on PPS
	beacon->be_gns	= 0;

	memset	(&tv,0,sizeof(tv));
	tv.tv_sec		= beacon->be_gss;
	tv.tv_usec		= beacon->be_gns / 1000;

	rtl_gettimeofday_to_iso8601date(&tv,NULL,when);

RTL_TRDBG(1,"MAC SEND beacon to send at utc='%s' (%u,%09u) lk=%p\n",
		when,beacon->be_gss,beacon->be_gns,pktbeacon->lp_lk);

	if	(LgwBeaconUtcTime.tv_sec == beacon->be_gss)
	{
RTL_TRDBG(1,"MAC SEND beacon already programmed at utc='%s' (%u,%09u) lk=%p\n",
		when,beacon->be_gss,beacon->be_gns,pktbeacon->lp_lk);
		LgwBeaconRequestedDupCnt++;
		return;
	}


	LgwBeaconRequestedCnt++;

	LgwBeaconUtcTime.tv_sec	= beacon->be_gss;
	LgwBeaconUtcTime.tv_nsec= beacon->be_gns;

	ret	= LgwStarted();
	if	(LgwThreadStopped || !LgwThreadStarted || !ret)
	{
		RTL_TRDBG(1,"Radio thread KO start=%d cmdstart=%d cmdstop=%d\n",
			ret,LgwThreadStarted,LgwThreadStopped);
		LgwBeaconLastDeliveryCause	= LP_CB_RADIO_STOP;
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
		LgwBeaconLastDeliveryCause	= LP_CB_RADIO_STOP_DN;
		return;
	}

#if	0	// duplicated requests must be checkec even if radio is stopped
	memset	(&downpkt,0,sizeof(t_lrr_pkt));
	memcpy	(&downpkt,pktbeacon,LP_PRE_HEADER_PKT_SIZE);

	// as we use ON_GPS mode a beacon can only be sent on PPS
	beacon->be_gns	= 0;

	memset	(&tv,0,sizeof(tv));
	tv.tv_sec		= beacon->be_gss;
	tv.tv_usec		= beacon->be_gns / 1000;

	rtl_gettimeofday_to_iso8601date(&tv,NULL,when);

	if	(LgwBeaconUtcTime.tv_sec == beacon->be_gss)
	{
RTL_TRDBG(1,"MAC SEND beacon already programmed at utc='%s' (%u,%09u) lk=%p\n",
		when,beacon->be_gss,beacon->be_gns,pktbeacon->lp_lk);
		LgwBeaconRequestedDupCnt++;
		return;
	}

RTL_TRDBG(1,"MAC SEND beacon to send at utc='%s' (%u,%09u) lk=%p\n",
		when,beacon->be_gss,beacon->be_gns,pktbeacon->lp_lk);


	LgwBeaconUtcTime.tv_sec	= beacon->be_gss;
	LgwBeaconUtcTime.tv_nsec= beacon->be_gns;
#endif

#ifndef KONA	// No need for tektelic, use directly utc time
#ifndef	WITH_SX1301_X8
	if	(SynchroPeriod == 0)	// RDTP-5475
	{
#endif
	if	(!UseGpsTime)
	{
	RTL_TRDBG(1,"beacon packets need use of GPS time (usegpstime=1)\n");
		LgwBeaconLastDeliveryCause	= LP_CB_NA;
		return;
	}
	if	(!Gps_ref_valid || GpsFd < 0)
	{
	RTL_TRDBG(1,"beacon packets need correct GPS time synchro(%d) fd=%d\n",
			Gps_ref_valid,GpsFd);
		LgwBeaconLastDeliveryCause	= LP_CB_NA;
		return;
	}
#ifndef	WITH_SX1301_X8
	}
#endif
#endif

	downpkt.lp_beacon	= 1;
	downpkt.lp_type		= LP_TYPE_LRR_PKT_RADIO;
	downpkt.lp_flag		= LP_RADIO_PKT_DOWN;
	downpkt.lp_gss		= beacon->be_gss;
	downpkt.lp_gns		= beacon->be_gns;
	downpkt.lp_tms		= rtl_tmmsmono();
	downpkt.lp_trip		= 0;
	downpkt.lp_delay	= 0;
	downpkt.lp_lgwdelay	= 1;
	downpkt.lp_lk		= pktbeacon->lp_lk;

	downpkt.lp_chain	= beacon->be_chain;
	downpkt.lp_channel	= beacon->be_channel;
	downpkt.lp_spfact	= beacon->be_spfact;
	if	(Rx2Channel && downpkt.lp_channel == UNK_CHANNEL)
	{
		downpkt.lp_channel	= Rx2Channel->channel;
		downpkt.lp_subband	= Rx2Channel->subband;
		downpkt.lp_spfact	= 
				CodeSpreadingFactor(Rx2Channel->dataraterx2);
	}
	downpkt.lp_correct	= CodeCorrectingCode(CR_LORA_4_5);

	// NFR1010 RDTP-1969
	if	(!strcmp(IsmBand,"us915") || !strcmp(IsmBand,"au915") 
						|| !strcmp(IsmBand,"cn470"))
	{

		// if the LRC requires a different DL freq => store it
		freqdn	= (beacon->be_freq[2] * 256 * 256)
			+ (beacon->be_freq[1] * 256)
			+ (beacon->be_freq[0]);
		freqdn	= freqdn * 100;
		if	(freqdn > 0)
		{
			downpkt.lp_freqdn[0]	= beacon->be_freq[0];
			downpkt.lp_freqdn[1]	= beacon->be_freq[1];
			downpkt.lp_freqdn[2]	= beacon->be_freq[2];
		}
	}

	downpkt.lp_size		= pktbeacon->lp_size;
	downpkt.lp_payload	= (u_char *)malloc(downpkt.lp_size);
	if	(!downpkt.lp_payload)
	{
RTL_TRDBG(0,"ERROR alloc payload %d\n",downpkt.lp_size);
		return;
	}
	memcpy	(downpkt.lp_payload,data,downpkt.lp_size);
	TmoaLrrPacket(&downpkt);

RTL_TRDBG(1,"MAC SEND beacon sz=%d tmoa=%fms chan=%u specific freqhz=%u sf=%d\n"
	,downpkt.lp_size,downpkt.lp_tmoa/1000.0,downpkt.lp_channel,freqdn,
	downpkt.lp_spfact);

	LgwSendPacket(&downpkt,0,0,0);
}

static	void	SendRadioClassCMultiCast(t_lrr_pkt *pktmulti,u_char *data,int sz)
{
	t_lrr_pkt		downpkt;
	t_lrr_multicast_dn	*multi	= &(pktmulti->lp_u.lp_multicast_dn);

	struct	timeval	tv;
	char	when[128];
	int	ret;

	LgwClassCRequestedCnt++;
	LgwClassCLastDeliveryCause	= LP_CB_NA;

	ret	= LgwStarted();
	if	(LgwThreadStopped || !LgwThreadStarted || !ret)
	{
		RTL_TRDBG(1,"Radio thread KO start=%d cmdstart=%d cmdstop=%d\n",
			ret,LgwThreadStarted,LgwThreadStopped);
		LgwClassCLastDeliveryCause	= LP_CB_RADIO_STOP;
		return;
	}

	if	(DownRadioStop)
	{
		RTL_TRDBG(1,"Radio stopped in downlink direction\n");
		LgwClassCLastDeliveryCause	= LP_CB_RADIO_STOP_DN;
		return;
	}

	memset	(&downpkt,0,sizeof(t_lrr_pkt));
	memcpy	(&downpkt,pktmulti,LP_PRE_HEADER_PKT_SIZE);

	// as we use ON_GPS mode a beacon can only be sent on PPS
	multi->mc_gns	= 0;

	memset	(&tv,0,sizeof(tv));
	tv.tv_sec		= multi->mc_gss;
	tv.tv_usec		= multi->mc_gns / 1000;
	rtl_gettimeofday_to_iso8601date(&tv,NULL,when);

	if	(LgwClassCUtcTime.tv_sec == multi->mc_gss)
	{
RTL_TRDBG(1,"MAC SEND classcmc already programmed at utc='%s' (%u,%09u) lk=%p\n",
		when,multi->mc_gss,multi->mc_gns,pktmulti->lp_lk);
		LgwClassCRequestedDupCnt++;
		return;
	}

RTL_TRDBG(1,"MAC SEND classcmc to send at utc='%s' (%u,%09u) lk=%p\n",
		when,multi->mc_gss,multi->mc_gns,pktmulti->lp_lk);


	LgwClassCUtcTime.tv_sec	= multi->mc_gss;
	LgwClassCUtcTime.tv_nsec= multi->mc_gns;

#ifndef KONA	// No need for tektelic, use directly utc time
#ifndef	WITH_SX1301_X8
	if	(SynchroPeriod == 0)	// RDTP-5475
	{
#endif
	if	(!UseGpsTime)
	{
	RTL_TRDBG(1,"Classcmc packets need use of GPS time (usegpstime=1)\n");
		return;
	}
	if	(!Gps_ref_valid || GpsFd < 0)
	{
	RTL_TRDBG(1,"Classcmc packets need correct GPS time synchro(%d) fd=%d\n",
			Gps_ref_valid,GpsFd);
		return;
	}
#ifndef	WITH_SX1301_X8
	}
#endif
#endif

	downpkt.lp_classc	= 0;
	downpkt.lp_classcmc	= 1;
	downpkt.lp_type		= LP_TYPE_LRR_PKT_RADIO;
	downpkt.lp_flag		= LP_RADIO_PKT_DOWN;
	downpkt.lp_gss		= multi->mc_gss;
	downpkt.lp_gns		= multi->mc_gns;
	downpkt.lp_tms		= rtl_tmmsmono();
	downpkt.lp_trip		= 0;
	downpkt.lp_delay	= 0;
	downpkt.lp_lgwdelay	= 1;
	downpkt.lp_lk		= pktmulti->lp_lk;

	downpkt.lp_deveui	= multi->mc_deveui;
	downpkt.lp_fcntdn	= multi->mc_fcntdn;

	downpkt.lp_chain	= multi->mc_chain;
	downpkt.lp_channel	= multi->mc_channel;
	downpkt.lp_spfact	= multi->mc_spfact;
	if	(Rx2Channel && downpkt.lp_channel == UNK_CHANNEL)
	{
		downpkt.lp_channel	= Rx2Channel->channel;
		downpkt.lp_subband	= Rx2Channel->subband;
		downpkt.lp_spfact	= 
				CodeSpreadingFactor(Rx2Channel->dataraterx2);
	}
	downpkt.lp_correct	= CodeCorrectingCode(CR_LORA_4_5);

	downpkt.lp_size		= pktmulti->lp_size;
	downpkt.lp_payload	= (u_char *)malloc(downpkt.lp_size);
	if	(!downpkt.lp_payload)
	{
RTL_TRDBG(0,"ERROR alloc payload %d\n",downpkt.lp_size);
		return;
	}
	memcpy	(downpkt.lp_payload,data,downpkt.lp_size);
	TmoaLrrPacket(&downpkt);

RTL_TRDBG(1,"MAC SEND classcmc sz=%d tmoa=%fms\n",downpkt.lp_size,downpkt.lp_tmoa/1000.0);

	LgwSendPacket(&downpkt,0,0,0);
}

// RDTP-2413
static	void	NfResponseFromLrc(t_lrr_pkt *downpkt)
{
	u_char	type;
	u_int	version;
	u_int	netid;
	u_char	partno;
	u_char	nbpart;
	char	*ff;
	int	sz;

	type	= downpkt->lp_u.lp_networkfilterResp.nf_type;
	version	= downpkt->lp_u.lp_networkfilterResp.nf_version;
	netid	= downpkt->lp_u.lp_networkfilterResp.nf_netid;
	ff	= downpkt->lp_u.lp_networkfilterResp.nf_filter;
	partno	= downpkt->lp_u.lp_networkfilterResp.nf_partno;
	nbpart	= downpkt->lp_u.lp_networkfilterResp.nf_nbpart;
	RTL_TRDBG(1,"NWF response version=%u netid=%06x type=%u\n",
						version,netid,type);
	switch	(type)
	{
	case	NF_TYPE_FILTER:
		if	(partno <= 1)
		{
			NwkfFilterSize	= 0;
			NwkfFilter[0]	= '\0';
		}
		sz	= strlen(ff);
		if	(NwkfFilterSize + sz < sizeof(NwkfFilter))
		{
			strcat(NwkfFilter+NwkfFilterSize,ff);	
			NwkfFilterSize	+= sz;
		}
		else
		{
			RTL_TRDBG(0,"NWF response filter to big=%u\n",
				NwkfFilterSize+sz);
		}
		RTL_TRDBG(1,"NWF response sz=%d/%d seq=%u/%u ff='%s'\n",
				sz,NwkfFilterSize,partno,nbpart,ff);
		if	(partno >= nbpart)
		{
			NfSave(version,netid,NwkfFilter);
			if	(NwkfFilterRefreshCmd)
			{
				NwkfFilterRefreshCmd	= 0;
				NfReload();
			}
		}
	break;
	case	NF_TYPE_FILENAME:
		RTL_TRDBG(1,"NWF response file='%s'\n",ff);
		RTL_TRDBG(0,"NWF update by file not supported\n");
	break;
	}
}

static	void	RecvInfraPacket(t_lrr_pkt *downpkt)
{
	char	version[256];
	char	cmd[1024];
	char	traceupgrade[1024];
	int	masterid;

	switch	(downpkt->lp_type)
	{
	case	LP_TYPE_LRR_INF_UPGRADE:
	{
		t_lrr_upgrade	*upgrade;
		char		*md5;

		upgrade	= &(downpkt->lp_u.lp_upgrade);
		snprintf(version,sizeof(version),"%u.%u.%u",
		upgrade->up_versMaj,upgrade->up_versMin,upgrade->up_versRel);
		RTL_TRDBG(1,"upgade requested version='%s' md5='%s'\n",
						version, upgrade->up_md5);
		md5	= (char *)upgrade->up_md5;

		snprintf(traceupgrade,sizeof(traceupgrade),"%s/var/log/lrr/UPGRADE.log",RootAct);
		if	(strlen(md5) && strcmp(md5,"*"))
		{
		snprintf(cmd,sizeof(cmd),"sh -c './upgrade.sh -V %s -M %s > %s 2>&1 &'",
			version,upgrade->up_md5,traceupgrade);
		}
		else
		{
		snprintf(cmd,sizeof(cmd),"sh -c './upgrade.sh -V %s > %s 2>&1 &'",
			version,traceupgrade);
		}
		DoSystemCmdBackGround(cmd);
		RTL_TRDBG(1,"%s\n",cmd);
	}
	break;

	case	LP_TYPE_LRR_INF_UPGRADE_V1:	// TODO a finir
	{
		t_lrr_upgrade_v1	*upgrade;
		char			*md5;

		upgrade	= &(downpkt->lp_u.lp_upgrade_v1);
		snprintf(version,sizeof(version),"%u.%u.%u",
		upgrade->up_versMaj,upgrade->up_versMin,upgrade->up_versRel);
		RTL_TRDBG(1,"upgade requested version='%s' md5='%s'\n",
						version, upgrade->up_md5);
		md5	= (char *)upgrade->up_md5;

		snprintf(traceupgrade,sizeof(traceupgrade),"%s/var/log/lrr/UPGRADE.log",RootAct);
		if	(strlen(md5) && strcmp(md5,"*"))
		{
		snprintf(cmd,sizeof(cmd),"sh -c './upgrade.sh -V %s -M %s > %s 2>&1 &'",
			version,upgrade->up_md5,traceupgrade);
		}
		else
		{
		snprintf(cmd,sizeof(cmd),"sh -c './upgrade.sh -V %s > %s 2>&1 &'",
			version,traceupgrade);
		}
		DoSystemCmdBackGround(cmd);
		RTL_TRDBG(1,"%s\n",cmd);
	}
	break;

	case	LP_TYPE_LRR_INF_UPGRADE_CMD:
	{
		t_lrr_upgrade_cmd	*upgrade;

		upgrade	= &(downpkt->lp_u.lp_upgrade_cmd);
		RTL_TRDBG(1,"upgade requested with cmd ...\n");

		snprintf(traceupgrade,sizeof(traceupgrade),"%s/var/log/lrr/UPGRADE.log",RootAct);
		snprintf(cmd,sizeof(cmd),"sh -c './upgrade.sh %s > %s 2>&1 &'",
			upgrade->up_cmd,traceupgrade);
		DoSystemCmdBackGround(cmd);
		RTL_TRDBG(1,"%s\n",cmd);
	}
	break;

	case	LP_TYPE_LRR_INF_RESTART_CMD:
	{
		RTL_TRDBG(1,"restartrequested with cmd ...\n");
		exit(0);
	}
	break;

	case	LP_TYPE_LRR_INF_SHELL_CMD:
	{
		DoShellCommand(downpkt);
	}
	break;

	//	start / stop radio without saving configuration flags
	case	LP_TYPE_LRR_INF_RADIO_STOP_CMD:
	{
		t_imsg	*msg;

		msg	= rtl_imsgAlloc(IM_DEF,IM_LGW_EXIT,NULL,0);
		if	(!msg)
			break;
		rtl_imsgAdd(LgwQ,msg);
		LgwThreadStopped	= 1;
	}
	break;

	case	LP_TYPE_LRR_INF_RADIO_START_CMD:
	{
		LgwThreadStopped	= 0;
		DownRadioStop		= 0;
	}
	break;

	case	LP_TYPE_LRR_INF_DNRADIO_STOP_CMD:
	{
		DownRadioStop	= 1;
	}
	break;
	case	LP_TYPE_LRR_INF_DNRADIO_START_CMD:
	{
		DownRadioStop	= 0;
	}
	break;
	case	LP_TYPE_LRR_PKT_DTC_SYNCHRO:
	{
#ifdef	LP_TP31
		SendDtcSyncToLrc(downpkt->lp_lk);
#endif
	}
	break;

	case	LP_TYPE_LRR_INF_TWA_LRRID:
	{
		t_lrc_link	*lrc;
		t_xlap_link	*lk;

		lk	= (t_xlap_link *)downpkt->lp_lk;
		lrc	= (t_lrc_link *)lk->lk_userptr;
		RTL_TRDBG(1,"get lrrid=%08x from twa lrcuid=%d idxlrc=%d\n",
			downpkt->lp_twalrrid,downpkt->lp_lrxid,lrc->lrc_idx);
		if	(lrc)
		{	// RDTP-9756/7649 store response result
			lrc->lrc_twaresp	= 1;
			lrc->lrc_twalrridresp	= downpkt->lp_twalrrid;
		}
		if	(LrrIDGetFromTwa == 0)
		{ // The response from the first lrc has already been received,
		  // this is the response from the second lrc
		  // just check if the second lrrid received is the same than 
		  // the first one
			if (downpkt->lp_twalrrid != LrrID)
			{
RTL_TRDBG(0,"ERROR: received 2 differents LrrID from twa (%08x!=%08x), big problems expected !\n",
				LrrID, downpkt->lp_twalrrid);
				break;
			}
	
		}
		if	(downpkt->lp_twalrruidns == 1)
		{ // The LRC understand the command but say it does not to
		  // support this request at all => use old mode
			LrrIDGetFromTwa	= 0;	// disable the feature even if 2
			LrrIDFromTwaGot	= 0;
			SendLrrID(downpkt->lp_lk);
			SaveConfigFileState();
			break;
		}
		if	(downpkt->lp_twalrrid == 0)
		{ // The LRC understand the command but has no response
		  // and ask to do not use old mode, retry later
			
RTL_TRDBG(0,"ERROR: received LrrID==0 from twa lrcid=%d idxlrc=%d ! => retry later\n",
				downpkt->lp_lrxid,lrc->lrc_idx);
			// RDTP-9756/7649 : wait a few sec
			// do nothing & wait timer expires to decide what to do
			break;
		}
		if	(LrrIDGetFromTwa>= 2 
			&& CheckLrrIdTwa(lrc->lrc_twalrridresp) == 0)
		{
			// RDTP-9756/7649 : wait a few sec
			// the 2 LRC do not return the same LRRID, ignore last
			// do nothing & wait timer expires to decide what to do
			lrc->lrc_twalrridresp	= 0;
			break;
		}
		LrrID		= downpkt->lp_twalrrid;
		LrrIDFromTwaGot	= LrrID;
		// RDTP-9756/7649 : reset feature only if <= 2
		if	(LrrIDGetFromTwa <= 1)
			LrrIDGetFromTwa = 0;	// disable the feature if != 2
		SaveConfigFileState();
		SendLrrID(downpkt->lp_lk);
	}
	break;

	// NFR997
	case	LP_TYPE_LRR_INF_PARTITIONID:
	{
		masterid = (u_int)CfgInt(HtVarLrr, "lrr", -1, "masterid", 0);
		RTL_TRDBG(1,"partitionid=%d for lrcid=%d (masterid=%d)\n",
					downpkt->lp_partitionid, downpkt->lp_lrxid, masterid);
		if	(LrrIDGetFromBS == 0)
		{
			RTL_TRDBG(0,"WARNING: received partitionid from lrc but NFR997 not activated\n");
			break;
		}

		// check if partitionid correspond to master id
		if (masterid == downpkt->lp_partitionid)
		{
			int	i;
			// search index of lrc
			RTL_TRDBG(3, "search lrc index ...\n");
			for	(i = 0 ; i < NbLrc ; i++)
			{
				if	(&TbLapLrc[i] == downpkt->lp_lk)
				{
					MasterLrc = i;
					RTL_TRDBG(1, "found master lrc: lrc %d (lrcid=%d)\n",
						MasterLrc, downpkt->lp_lrxid);
				}
			}
		}
		RTL_TRDBG(0, "master lrc is lrc %d\n", MasterLrc);
	}
	break;
	case	LP_TYPE_LRR_INF_NETWORKFILTER_RESP:	// RDTP-2413
	{
		NfResponseFromLrc(downpkt);
	}
	break;
	case	LP_TYPE_LRR_PKT_DLSYNC:			// RDTP-5475
	{
		t_lrr_sync_dn *sync	= &downpkt->lp_u.lp_sync_dn;

	RTL_TRDBG(1, "synchro #lrr=%d lrcgss=%09u lrcgns=%09u tus=%u fcnt=%u\n",
		sync->sy_macroCnt,sync->sy_lrcGss,sync->sy_lrcGns,
		sync->sy_microTus,sync->sy_microFcntdn);
#ifndef	WITH_SX1301_X8
		t_imsg	*msg;
		if	(SynchroPeriod == 0)
			break;
		if	(sync->sy_macroCnt <= 0)
			break;
		msg	= rtl_imsgAlloc(IM_DEF,IM_LGW_SYNC_TIME,NULL,0);
		if	(!msg)	break;
		if	(rtl_imsgDupData(msg,sync,sizeof(t_lrr_sync_dn)) != msg)
			rtl_imsgFree(msg);
		else
			rtl_imsgAdd(LgwQ,msg);
#endif
	}
	break;

	default :
	break;
	}
}

static	void	TcpKeepAliveHigh(int lrc,int fd)
{
	int	tcpKeepAlive	= 1;	// yes or not

	int	tcpKeepIdle	= 5;	// 5s
	int	tcpKeepIntvl	= 5;	// 5s
	int	tcpKeepCnt	= 20;	// 20 retries


	if	(fd < 0)
		return;


	tcpKeepAlive	= (u_int)CfgInt(HtVarLrr,"tcp",-1,
					"tcpkeepalive",tcpKeepAlive);
	tcpKeepIdle	= (u_int)CfgInt(HtVarLrr,"tcp",-1,
					"tcpkeepidle",tcpKeepIdle);
	tcpKeepIntvl	= (u_int)CfgInt(HtVarLrr,"tcp",-1,
					"tcpkeepintvl",tcpKeepIntvl);
	tcpKeepCnt	= (u_int)CfgInt(HtVarLrr,"tcp",-1,
					"tcpkeepcnt",tcpKeepCnt);

	if	(tcpKeepAlive <= 0)
		return;

RTL_TRDBG(1,"LAP LRC TCP KEEPALIVE HIGH lrc=%d fd=%d alive=%d idle=%d intvl=%d cnt=%d\n",
		lrc,fd,tcpKeepAlive,tcpKeepIdle,tcpKeepIntvl,tcpKeepCnt);

	setsockopt(fd,SOL_SOCKET,SO_KEEPALIVE,
				(char *)&tcpKeepAlive,sizeof(tcpKeepAlive));
	if	( tcpKeepIdle > 0 )
	{
		setsockopt(fd,IPPROTO_TCP,TCP_KEEPIDLE,
			(char *)&tcpKeepIdle,sizeof(tcpKeepIdle));
	}
	if	( tcpKeepIntvl > 0 )
	{
		setsockopt(fd,IPPROTO_TCP,TCP_KEEPINTVL,
			(char *)&tcpKeepIntvl,sizeof(tcpKeepIntvl));
	}
	if	( tcpKeepCnt > 0 )
	{
		setsockopt(fd,IPPROTO_TCP,TCP_KEEPCNT,
			(char *)&tcpKeepCnt,sizeof(tcpKeepCnt));
	}
}

static	void	TcpKeepAliveLow(int lrc,int fd)
{
	int	tcpKeepAlive	= 1;	// yes or not

	int	tcpKeepIdle	= 5;	// 5s
	int	tcpKeepIntvl	= 30;	// 30s
	int	tcpKeepCnt	= 3;	// 3 retries

	if	(fd < 0)
		return;



	tcpKeepAlive	= (u_int)CfgInt(HtVarLrr,"tcp",-1,
					"tcpkeepalivelow",tcpKeepAlive);
	tcpKeepIdle	= (u_int)CfgInt(HtVarLrr,"tcp",-1,
					"tcpkeepidlelow",tcpKeepIdle);
	tcpKeepIntvl	= (u_int)CfgInt(HtVarLrr,"tcp",-1,
					"tcpkeepintvllow",tcpKeepIntvl);
	tcpKeepCnt	= (u_int)CfgInt(HtVarLrr,"tcp",-1,
					"tcpkeepcntlow",tcpKeepCnt);

	if	(tcpKeepAlive <= 0)
		return;

RTL_TRDBG(1,"LAP LRC TCP KEEPALIVE LOW lrc=%d fd=%d alive=%d idle=%d intvl=%d cnt=%d\n",
		lrc,fd,tcpKeepAlive,tcpKeepIdle,tcpKeepIntvl,tcpKeepCnt);

	setsockopt(fd,SOL_SOCKET,SO_KEEPALIVE,
				(char *)&tcpKeepAlive,sizeof(tcpKeepAlive));
	if	( tcpKeepIdle > 0 )
	{
		setsockopt(fd,IPPROTO_TCP,TCP_KEEPIDLE,
			(char *)&tcpKeepIdle,sizeof(tcpKeepIdle));
	}
	if	( tcpKeepIntvl > 0 )
	{
		setsockopt(fd,IPPROTO_TCP,TCP_KEEPINTVL,
			(char *)&tcpKeepIntvl,sizeof(tcpKeepIntvl));
	}
	if	( tcpKeepCnt > 0 )
	{
		setsockopt(fd,IPPROTO_TCP,TCP_KEEPCNT,
			(char *)&tcpKeepCnt,sizeof(tcpKeepCnt));
	}
}

void	TcpKeepAliveNo(int lrc,int fd)
{
	int	tcpKeepAlive	= 0;	// yes or not

RTL_TRDBG(1,"LAP LRC TCP NOKEEPALIVE lrc=%d fd=%d alive=%d\n",
		lrc,fd,tcpKeepAlive);

	if	(fd < 0)
		return;


	setsockopt(fd,SOL_SOCKET,SO_KEEPALIVE,
				(char *)&tcpKeepAlive,sizeof(tcpKeepAlive));
}

void	SendCapabToLrc(t_xlap_link *lktarget)
{
	t_xlap_link	*lk;
	t_lrr_pkt	uppkt;
	int		i;

	memset	(&uppkt,0,sizeof(t_lrr_pkt));
	uppkt.lp_vers	= LP_LRX_VERSION;
	uppkt.lp_flag	= LP_INFRA_PKT_INFRA;
	uppkt.lp_lrrid	= LrrID;
	uppkt.lp_type	= LP_TYPE_LRR_INF_CAPAB;
	uppkt.lp_szh	= LP_PRE_HEADER_PKT_SIZE+sizeof(t_lrr_capab);
	uppkt.lp_u.lp_capab.li_nbantenna	= LgwAntenna;
	uppkt.lp_u.lp_capab.li_versMaj		= VersionMaj;
	uppkt.lp_u.lp_capab.li_versMin		= VersionMin;
	uppkt.lp_u.lp_capab.li_versRel		= VersionRel;
	uppkt.lp_u.lp_capab.li_versFix		= VersionFix;
	strncpy((char *)uppkt.lp_u.lp_capab.li_ismBand,IsmBand,9);
	strncpy((char *)uppkt.lp_u.lp_capab.li_system,System,32);
	strncpy((char *)uppkt.lp_u.lp_capab.li_ismBandAlter,IsmBandAlter,32);
	if	(RfRegionId && *RfRegionId)
		strncpy((char *)uppkt.lp_u.lp_capab.li_rfRegion,RfRegionId,32);
	uppkt.lp_u.lp_capab.li_ismVar		= RfRegionIdVers;
#ifdef	LP_TP31
	uppkt.lp_u.lp_capab.li_szPktRadioStruct	= sizeof(uppkt.lp_u.lp_radio);
	uppkt.lp_u.lp_capab.li_nbBoard		= LgwBoard;
	uppkt.lp_u.lp_capab.li_nbChan		= 8;
	uppkt.lp_u.lp_capab.li_nbSector		= LgwAntenna;	// TODO
	uppkt.lp_u.lp_capab.li_fpga		= 0;		// TODO
	uppkt.lp_u.lp_capab.li_nbChanUp		= 0;		// TODO
	uppkt.lp_u.lp_capab.li_nbChanDn		= 0;		// TODO
	uppkt.lp_u.lp_capab.li_iecExtFtr	= 1;
	uppkt.lp_u.lp_capab.li_dtcdnFtr		= 1;
	uppkt.lp_u.lp_capab.li_dtcupFtr		= 1;
	uppkt.lp_u.lp_capab.li_sentIndicFtr	= 1;
	uppkt.lp_u.lp_capab.li_pktStoreFtr	= StorePktCount;
	uppkt.lp_u.lp_capab.li_geoLocFtr	= 0;
	uppkt.lp_u.lp_capab.li_lbtFtr		= LgwLbtEnable;
#ifdef	WITH_GPS
	uppkt.lp_u.lp_capab.li_classBFtr	= 1;
	uppkt.lp_u.lp_capab.li_classCMcFtr	= 1;
#endif
	uppkt.lp_u.lp_capab.li_freqdnFtr	= 1;
#endif

#ifndef	WITH_LBT
	uppkt.lp_u.lp_capab.li_lbtFtr		= 0;
#endif
	uppkt.lp_u.lp_capab.li_shiftLcFtr	= DlShiftLc;	// RDTP-5911
	uppkt.lp_u.lp_capab.li_nwfFtr		= NwkfEnable;	// RDTP-2413
	uppkt.lp_u.lp_capab.li_multiRx2Ftr	= 1;		// RDTP-2543
	uppkt.lp_u.lp_capab.li_multiPingSlotFtr	= 1;		// RDTP-2543

	// RDTP-5475
#ifndef	WITH_SX1301_X8
	if	(SynchroPeriod)
	{
		uppkt.lp_u.lp_capab.li_classBFtr	= 1;
		uppkt.lp_u.lp_capab.li_classCMcFtr	= 1;
	}
#endif


	if	(lktarget)
	{
		LapPutOutQueue(lktarget,(u_char *)&uppkt,uppkt.lp_szh);
		return;
	}

	for	(i = 0 ; i < NbLrc ; i++)
	{
		lk	= &TbLapLrc[i];
		if	(lk->lk_state == SSP_STARTED)
		{
			LapPutOutQueue(lk,(u_char *)&uppkt,uppkt.lp_szh);
		}
	}
}

#ifdef	LP_TP31
#ifdef	__clang__
static	void	SendDtcSyncToLrc(t_xlap_link *lktarget) 
{
	RTL_TRDBG(0,"LRR compiled with clang !!!\n");
	abort(); 
}
#else
static	void	SendDtcSyncToLrc(t_xlap_link *lktarget)
{
	t_xlap_link	*lk;
	t_lrr_pkt	uppkt;
	int		i;

	memset	(&uppkt,0,sizeof(t_lrr_pkt));
	uppkt.lp_vers	= LP_LRX_VERSION;
	uppkt.lp_flag	= LP_RADIO_PKT_DTC;
	uppkt.lp_lrrid	= LrrID;
	uppkt.lp_type	= LP_TYPE_LRR_PKT_DTC_SYNCHRO;
	uppkt.lp_szh	= LP_PRE_HEADER_PKT_SIZE+sizeof(t_lrr_pkt_radio);

	inline	void dtcsync(void *pf,int ant,int c,int s,
                        float up,float dn,float upsub,float dnsub)
	{
		t_lrr_pkt_radio_dtc_ud	*dtc;

RTL_TRDBG(1,"DTC synchro a=%d c=%03d s=%03d uch=%f dch=%f usub=%f dsub=%f\n",
			ant,c,s,up,dn,upsub,dnsub);

		uppkt.lp_szh	= LP_PRE_HEADER_PKT_SIZE+sizeof(t_lrr_pkt_radio);
		uppkt.lp_size	= 0;

		uppkt.lp_chain	= ant<<4;
		uppkt.lp_channel= c;
		uppkt.lp_subband= s;

		dtc	= &uppkt.lp_u.lp_radio.lr_u2.lr_dtc_ud;
		dtc->lr_dtcchanneldn	= dn;
		dtc->lr_dtcsubbanddn	= dnsub;
		dtc->lr_dtcchannelup	= up;
		dtc->lr_dtcsubbandup	= upsub;

		if	(lktarget)
		{
			LapPutOutQueue(lktarget,(u_char *)&uppkt,uppkt.lp_szh);
			return;
		}

		for	(i = 0 ; i < NbLrc ; i++)
		{
			lk	= &TbLapLrc[i];
			if	(lk->lk_state == SSP_STARTED)
			{
				LapPutOutQueue(lk,(u_char *)&uppkt,uppkt.lp_szh);
			}
		}
	}

	DcWalkChanCounters(NULL,dtcsync);

	// send an invalid DTC synchro packet to signal end of synchro
	
	uppkt.lp_flag	= 0;
	uppkt.lp_type	= LP_TYPE_LRR_PKT_DTC_SYNCHRO;
	uppkt.lp_szh	= LP_PRE_HEADER_PKT_SIZE+sizeof(t_lrr_pkt_radio);
	dtcsync(NULL,0,0,0,0.0,0.0,0.0,0.0);
}
#endif
#endif

static	void	LapEventProceed(t_xlap_link *lk,int evttype,int evtnum,void *data,int sz)
{
	t_lrc_link	*lrc;

	lrc	= lk->lk_userptr;
	RTL_TRDBG(6,"LAP CB (%s,%s) st='%s' receive evttype=%d evtnum='%s'\n",
		lk->lk_addr,lk->lk_port,
		LapStateTxt(lk->lk_state),evttype,LapEventTxt(evtnum));

	LapState	= lk->lk_state;

	if	(LrrID == 0)	// sniffer
	{
		RTL_TRDBG(1,"LAP LRC evt='%s' with LrrId=0 !!!\n",LapEventTxt(evtnum));
		LrcSaveStatus(1);
		return;
	}

	switch	(evtnum)
	{
	case	EVT_LK_CREATED :	// tcp server only
	case	EVT_TCP_CONNECTED :
		RTL_TRDBG(1,"LAP LRC CNX\n");
		TcpKeepAliveHigh(-1,lk->lk_fd);
		DoHtmlFile();
#if defined(KEROS) && defined(WITH_LED_MGT)
		LedBackhaul(1);
#endif
		LrcSaveStatus(1);
	break;
	case	EVT_LK_DELETED :
		if	(!lrc)
			break;
		RTL_TRDBG(1,"LAP LRC DISC (%d)\n",lrc->lrc_stat.lrc_nbdisc);
		res_init();
		if	(lk->lk_state != SSP_INIT)
			lrc->lrc_stat.lrc_nbdisc++;
		DoHtmlFile();
		memset	(&lrc->lrc_stat_p,0,sizeof(lrc->lrc_stat_p));
#if defined(KEROS) && defined(WITH_LED_MGT)
		LedBackhaul(1);
#endif
		LrcSaveStatus(1);
		// RDTP-9756/7649 : 
		// restore LrrIDGetFromTwa & reset LrrIDFromTwaGot if necessary
		char *stridmode	= CfgStr(HtVarLrr,"lrr",-1,"uidmode","local");
		DoLrrIDFromTwa(stridmode);
		if	(lrc)
		{	//  RDTP-9756/7649
			lrc->lrc_twaresp	= 0;
			lrc->lrc_twalrridresp	= 0;
		}
	break;
	case	EVT_LK_STOPPED :
		RTL_TRDBG(1,"LAP LRC STOPPED\n");
		DoHtmlFile();
#if defined(KEROS) && defined(WITH_LED_MGT)
		LedBackhaul(1);
#endif
		LrcSaveStatus(1);
	break;
	case	EVT_LK_STARTED :
	{
#if defined(KEROS) && defined(WITH_LED_MGT)
		LedBackhaul(2);
#endif
		LrcSaveStatus(1);
		RTL_TRDBG(1,"LAP LRC STARTED\n");

		// if NFR684 activated
		RTL_TRDBG(1,"Get LrrID: from bootserver = %d, from twa = %d, lrrid=%08x\n", 
			LrrIDGetFromBS, LrrIDGetFromTwa, LrrIDFromTwaGot);
		if (!LrrIDGetFromBS && LrrIDGetFromTwa)
			SendLrrUID(lk);
		else
			SendLrrID(lk);
		SaveConfigFileState();
	}
	break;
	case	EVT_FRAMEI :
	{
		t_lrr_pkt	downpkt;
		int		szh	= LP_PRE_HEADER_PKT_SIZE;
		u_char		*frame	= data;

		if	(sz <= 0)	// ack forced by peer with frameI sz=0
			break;

		memset	(&downpkt,0,sizeof(t_lrr_pkt));
		memcpy	(&downpkt,data,szh);
		if	(downpkt.lp_vers != LP_LRX_VERSION)
		{
			RTL_TRDBG(0,"bad version %d/%d\n",downpkt.lp_vers,
						LP_LRX_VERSION);
#if	0		// TODO
			break;
#endif
		}
		szh	= downpkt.lp_szh;
		if	(szh > sizeof(t_lrr_pkt))
			szh	= sizeof(t_lrr_pkt);
		memcpy	(&downpkt,data,szh);
		downpkt.lp_lk		= lk;
		downpkt.lp_size		= sz - szh;
		downpkt.lp_payload	= frame + szh;

RTL_TRDBG(1,"LAP RECV sz=%d szh=%d psz=%d lrrid=%08x typ=%d/%d(%s) tms=%u rqtdelay=%u tus=%u\n",
	sz,szh,downpkt.lp_size,downpkt.lp_lrrid,downpkt.lp_type,
	downpkt.lp_flag,LrrPktFlagsTxt(downpkt.lp_flag),
	downpkt.lp_tms,downpkt.lp_delay, downpkt.lp_tus);

		if	((downpkt.lp_flag&LP_INFRA_PKT_INFRA))
		{
			RecvInfraPacket(&downpkt);
			break;
		}
		switch	(downpkt.lp_type)
		{
		case	LP_TYPE_LRR_PKT_RADIO:
			SendRadioPacket(&downpkt,frame + szh,sz - szh);
		break;
		case	LP_TYPE_LRR_INF_BEACON_DN:
			SendRadioBeacon(&downpkt,frame + szh,sz - szh);
		break;
		case	LP_TYPE_LRR_PKT_MULTICAST_DN:
			SendRadioClassCMultiCast(&downpkt,frame + szh,sz - szh);
		break;
		default :
		break;
		}
	}
	break;
	// Connection with LRC lost, get back non acked msg in order to store them and send them later
	case	EVT_FRAME_ACKLOST :
	{
		t_imsg		*imsg;
		t_lrr_pkt	uppkt;
		int		szh	= LP_PRE_HEADER_PKT_SIZE;
		u_char		*frame	= data;

		if (CfgInt(HtVarLrr,"lrr",-1,"treatacklost",0) == 0)
			break;

		if	(sz <= 0)	// ack forced by peer with frameI sz=0
			break;

		memset	(&uppkt,0,sizeof(t_lrr_pkt));
		memcpy	(&uppkt,data,szh);
		if	(uppkt.lp_vers != LP_LRX_VERSION)
		{
			RTL_TRDBG(0,"bad version %d/%d\n",uppkt.lp_vers,
						LP_LRX_VERSION);
			break;
		}
		szh	= uppkt.lp_szh;
		if	(szh > sizeof(t_lrr_pkt))
			szh	= sizeof(t_lrr_pkt);
		memcpy	(&uppkt,data,szh);
		uppkt.lp_lk		= lk;
		uppkt.lp_size		= sz - szh;
		uppkt.lp_payload	= frame + szh;

RTL_TRDBG(1,"LAP RECV ACK_LOST sz=%d lrrid=%08x typ=%d/%d(%s) tms=%u rqtdelay=%u\n",
	sz,uppkt.lp_lrrid,uppkt.lp_type,
	uppkt.lp_flag,LrrPktFlagsTxt(uppkt.lp_flag),
	uppkt.lp_tms,uppkt.lp_delay);

		// Store if it's an uplink
		if (uppkt.lp_flag & LP_RADIO_PKT_UP)
		{
			// need to alloc lp_payload, usually done by SendRadio*
			uppkt.lp_payload	= (u_char *)malloc(sz - szh);
			if	(!uppkt.lp_payload)
			{
				RTL_TRDBG(0,"malloc failed !\n");
				return;
			}
			memcpy	(uppkt.lp_payload,frame + szh,sz - szh);

			// RecvRadioPacketStore expect an imsg
			imsg	= rtl_imsgAlloc(IM_DEF,IM_LGW_RECV_DATA,NULL,0);
			if	(!imsg)
			{
				free(uppkt.lp_payload);
				RTL_TRDBG(0,"rtl_imsgAlloc failed !\n");
				return;
			}

			sz	= sizeof(t_lrr_pkt);
			if	( rtl_imsgDupData(imsg,&uppkt,sz) != imsg)
			{
				RTL_TRDBG(0,"rtl_imsgDupData(sz=%d) failed !\n", sz);
				rtl_imsgFree(imsg);
				return;
			}

//			rtl_imsgAdd(MainQ,msg);
			RecvRadioPacketStore(imsg,imsg->im_dataptr,"xlap");
		}

	}
	break;
	default :
	break;
	}
}

static	void	LapEventProceedTest(t_xlap_link *lk,int evttype,int evtnum,void *data,int sz)
{
	t_lrc_link	*lrc;
	time_t		rtt;
	int		nbu;

	lrc	= lk->lk_userptr;
	RTL_TRDBG(6,"LAP CB (%s,%s) st='%s' receive evttype=%d evtnum='%s'\n",
		lk->lk_addr,lk->lk_port,
		LapStateTxt(lk->lk_state),evttype,LapEventTxt(evtnum));

	if	(!lrc)
		return;

	if	(lrc->lrc_testinit == 0)
	{
		lrc->lrc_testlast	= Currtime.tv_sec;
		lrc->lrc_testinit	= 1;
	}

	LapState	= lk->lk_state;

	switch	(evtnum)
	{
	case	EVT_LK_CREATED :	// tcp server only
	case	EVT_TCP_CONNECTED :
		RTL_TRDBG(0,"LAP LRC CNX\n");
		TcpKeepAliveHigh(-1,lk->lk_fd);
		AvDvUiClear(&lrc->lrc_avdvtrip); // reset stats on new cnx
	break;
	case	EVT_LK_DELETED :
		RTL_TRDBG(0,"LAP LRC DISC (%d)\n",lrc->lrc_stat.lrc_nbdisc);
		lrc->lrc_testinit	= 0;
	break;
	case	EVT_LK_STOPPED :
		RTL_TRDBG(1,"LAP LRC STOPPED\n");
	break;
	case	EVT_LK_STARTED :
		RTL_TRDBG(1,"LAP LRC STARTED\n");
	break;
	case	EVT_FRAMEI :
		RTL_TRDBG(1,"LAP LRC FRAMEI\n");
	break;
	case	EVT_TESTFRcon :
		rtt	= ABS(rtl_tmmsmono() - lk->lk_tFrmSndAtMs);
		AvDvUiAdd(&lrc->lrc_avdvtrip,rtt,Currtime.tv_sec);
		if	(LapTest > 1)
		{
			printf("lrc=%s rtt=%d ucnt=%d\n",
				lk->lk_addr,(int)rtt,lk->lk_stat.st_nbsendu);
			fflush(stdout);
		}
		if	(ABS(Currtime.tv_sec - lrc->lrc_testlast) > 180)
		{
			nbu	= AvDvUiCompute(&lrc->lrc_avdvtrip,300,Currtime.tv_sec);
			printf("lrc=%s %d,%d,%d,%d,%d\n",lk->lk_addr,nbu,
			lrc->lrc_avdvtrip.ad_aver,
			lrc->lrc_avdvtrip.ad_sdev,
			lrc->lrc_avdvtrip.ad_aver+lrc->lrc_avdvtrip.ad_sdev,
			lrc->lrc_avdvtrip.ad_vmax);
			fflush(stdout);
			lrc->lrc_testlast	= Currtime.tv_sec;
		}
	break;
	default :
	break;
	}

}

static	void	DoClockMs()
{
	static	unsigned int errth;

	if	(!LgwThreadStopped && pthread_kill(LgwThread,0) != 0)
	{
		errth++;
		if	(errth >= 10)
		{
			errth	= 0;
			RTL_TRDBG(0,"lgw thread does not exist => restart\n");
			ReStartLgwThread();
			return;
		}
	}
	else
		errth	= 0;
}

static	int	SendToLrcRobin(t_lrr_pkt *uppkt,u_char *buff,int sz)
{
	static	unsigned	int	rr;
	t_xlap_link		*lk;
	int	ok	= 0;
	int	i;
	int	lrc;
	int	ret;
	float	sv_rssi;
	float	sv_snr;

	if	(!uppkt)
	{
		uppkt	= (t_lrr_pkt *)buff;
	}

	sv_rssi = uppkt->lp_rssi;
	sv_snr  = uppkt->lp_snr;

	RTL_TRDBG(6,"Robin on LRC available rr=%d\n",rr);
	for	(i = 0 ; i < NbLrc ; i++)
	{
		lrc	= rr % NbLrc;
		rr++;
		lk	= &TbLapLrc[lrc];
		uppkt->lp_lrrid	= TbLrrID[lrc];
		RTL_TRDBG(3,"try to send packet to LRC=%d lrrid=%08x\n",
					lrc,uppkt->lp_lrrid);

		if	(SimulN && QosAlea)
		{	// simulate several lrr and change qos
			uppkt->lp_rssi	= sv_rssi + (-5-(rand()%10));
			uppkt->lp_snr	= sv_snr + (-5-(rand()%10));
		}

		if	(lk->lk_state == SSP_STARTED 
					&& (ret=LapPutOutQueue(lk,buff,sz)) > 0)
		{
			ok++;
RTL_TRDBG(1,"packet sent to LRC=%d lrrid=%08x by robin rssi=%f snr=%f\n",
			lrc,uppkt->lp_lrrid,uppkt->lp_rssi,uppkt->lp_snr);
			return	ok;
		}
		else
			RTL_TRDBG(3,"LRC=%d not available\n",lrc);
	}

	return	ok;
}

static	int	SendToLrcOrder(t_lrr_pkt *uppkt,u_char *buff,int sz)
{
	t_xlap_link		*lk;
	int	ok	= 0;
	int	i;
	int	lrc;
	int	ret;

	if	(!uppkt)
	{
		uppkt	= (t_lrr_pkt *)buff;
	}

	RTL_TRDBG(6,"Order on LRC available nb=%d\n",NbLrc);
	for	(i = 0 ; i < NbLrc ; i++)
	{
		lrc	= i;
		if (LrrIDGetFromBS && MasterLrc >= 0)	// NFR997
		{
			// send to lrc master first
			if (i == 0)
				lrc = MasterLrc;
			else if (i == MasterLrc)	// send also to lrc 0, the master took its place
				lrc = 0;
		}

		lk	= &TbLapLrc[lrc];
		RTL_TRDBG(3,"try to send packet to LRC=%d lrrid=%08x\n",
					lrc,uppkt->lp_lrrid);

		if	(lk->lk_state == SSP_STARTED 
					&& (ret=LapPutOutQueue(lk,buff,sz)) > 0)
		{
			ok++;
			switch(uppkt->lp_type)
			{
			case	LP_TYPE_LRR_PKT_SENT_INDIC:
RTL_TRDBG(1,"indic sent to LRC=%d lrrid=%08x by order\n",
			lrc,uppkt->lp_lrrid);
			break;
			default :
RTL_TRDBG(1,"packet sent to LRC=%d lrrid=%08x by order rssi=%f snr=%f\n",
			lrc,uppkt->lp_lrrid,uppkt->lp_rssi,uppkt->lp_snr);
			break;
			}
			break;
		}
		else
			RTL_TRDBG(3,"LRC=%d not available\n",lrc);

		if (LrrIDGetFromBS && MasterLrc >= 0)	// NFR997
		{
			// restore lrc value for following keepalive treatment
			if (i == MasterLrc)
				lrc = i;
		}
	}

	// current LRC changes : restore keepalive on new LRC
	// reset keepalive on all others
	if	(ok && CurrLrcOrder != lrc)
	{
RTL_TRDBG(1,"LRC changes %d => %d\n",CurrLrcOrder,lrc);
		CurrLrcOrder	= lrc;
		for	(i = 0 ; i < NbLrc ; i++)
		{
			lk	= &TbLapLrc[i];
			if	(i == lrc)
			{
				TcpKeepAliveHigh(i,lk->lk_fd);
			}
			else
			{
				TcpKeepAliveLow(i,lk->lk_fd);
			}
		}
	}

	return	ok;
}

static	int	SendToLrc(t_lrr_pkt *uppkt,u_char *buff,int sz)
{
	int	ok	= 0;
	int	i;
	t_xlap_link	*lk;

	if	(!uppkt)
	{
		uppkt	= (t_lrr_pkt *)buff;
	}

	switch	(Redundancy)
	{
	case	1:	// round robin
		ok	=	SendToLrcRobin(uppkt,buff,sz);
	break;

	
	case	0:	// by order
	default	:
		ok	=	SendToLrcOrder(uppkt,buff,sz);
	break;
	}

	if	(ok == 0)
	{
		RTL_TRDBG(1,"no LRC available\n");
		for	(i = 0 ; i < NbLrc ; i++)
		{
			lk	= &TbLapLrc[i];
			if	(lk && lk->lk_state == SSP_STOPPED)
			{
				LapEventRequest(lk,EVT_TESTFRact,NULL,0);
			}
		}
		switch	(uppkt->lp_type)
		{
		case	LP_TYPE_LRR_PKT_RADIO :
			LrcNbPktDrop++;
		break;
		default :
		break;
		}
	}
	return	ok;
}

static	int	SendStatToAllLrc(u_char *buff,int sz,int delay)
{
	int	ret	= 0;
	int	lrc;

	if	(delay > 0)
	{
		t_imsg	*msg;

		msg	= rtl_imsgAlloc(IM_DEF,IM_LGW_DELAY_ALLLRC,NULL,0);
		if	(!msg)
		{
			return	-1;
		}
		if	( rtl_imsgDupData(msg,buff,sz) != msg)
		{
			rtl_imsgFree(msg);
			return	-1;
		}
		rtl_imsgAddDelayed(MainQ,msg,delay);
		RTL_TRDBG(3,"SendStatToAllLrc() delayed=%d\n",delay);
		return	0;
	}

	// stats are sent to all LRC
	for	(lrc = 0 ; lrc < NbLrc ; lrc++)
	{
		t_xlap_link		*lk;

		lk	= &TbLapLrc[lrc];
		if	(SimulN)
		{
			t_lrr_pkt	*uppkt;

			uppkt	= (t_lrr_pkt *)buff;
			uppkt->lp_lrrid	= TbLrrID[lrc];
		}
		if	(lk->lk_state == SSP_STARTED)
		{
			LapPutOutQueue(lk,(u_char *)buff,sz);
			ret++;
		}
	}

	return	ret;
}

static	void	DoHtmlFile()
{
	FILE	*f;
	char	file[512];
	char	tmp[512];

#ifdef	WIRMAV2
	snprintf(file,sizeof(file),"%s","/home/www/index.html");
#else
	snprintf(file,sizeof(file),"%s/var/log/lrr",RootAct);
	rtl_mkdirp(file);
	snprintf(file,sizeof(file),"%s/var/log/lrr/stat.html",RootAct);
#endif
	f	= fopen(file,"w");
	if	(!f)
		return;

	rtl_getCurrentIso8601date(tmp);

	fprintf(f,"<br>Update=%s</br>\n",tmp);
	fprintf(f,"<br>Uptime=%s</br>\n",UptimeProcStr);
	fprintf(f,"<br>Version=%s</br>\n",lrr_whatStr);
	fprintf(f,"<br>LrxVersion=%d</br>\n",LP_LRX_VERSION);
	fprintf(f,"<br>LrrId=%08x (%u) lat=%f lon=%f alt=%d</br>\n",
				LrrID,LrrID,LrrLat,LrrLon,LrrAlt);
	fprintf(f,"<br>LapState=%s</br>\n",LapStateTxt(LapState));

	fprintf(f,"<br>LgwNbPacketSend=%u</br>\n",LgwNbPacketSend);
	fprintf(f,"<br>LgwNbPacketWait=%u</br>\n",LgwNbPacketWait);
	fprintf(f,"<br>LgwNbPacketRecv=%u</br>\n",LgwNbPacketRecv);

	fprintf(f,"<br>LgwNbStartOk=%u</br>\n",LgwNbStartOk);
	fprintf(f,"<br>LgwNbStartFailure=%u</br>\n",LgwNbStartFailure);
	fprintf(f,"<br>LgwNbConfigFailure=%u</br>\n",LgwNbConfigFailure);
	fprintf(f,"<br>LgwNbLinkDown=%u</br>\n",LgwNbLinkDown);


	fprintf(f,"<br>LgwNbBusySend=%u</br>\n",LgwNbBusySend);
	fprintf(f,"<br>LgwNbSyncError=%u</br>\n",LgwNbSyncError);
	fprintf(f,"<br>LgwNbCrcError=%u</br>\n",LgwNbCrcError);
	fprintf(f,"<br>LgwNbSizeError=%u</br>\n",LgwNbSizeError);
	fprintf(f,"<br>LgwNbChanUpError=%u</br>\n",LgwNbChanUpError);
	fprintf(f,"<br>LgwNbChanDownError=%u</br>\n",LgwNbChanDownError);
	fprintf(f,"<br>LgwNbDelayError=%u</br>\n",LgwNbDelayError);
	fprintf(f,"<br>LgwNbDelayReport=%u</br>\n",LgwNbDelayReport);
	fprintf(f,"<br>MacNbFcsError=%u</br>\n",MacNbFcsError);
	fprintf(f,"<br>LrcNbPktTrip=%u</br>\n",LrcNbPktTrip);
	fprintf(f,"<br>LrcAvPktTrip=%u</br>\n",LrcAvPktTrip);
	fprintf(f,"<br>LrcDvPktTrip=%u</br>\n",LrcDvPktTrip);
	fprintf(f,"<br>LrcMxPktTrip=%u</br>\n",LrcMxPktTrip);
	fprintf(f,"<br>LrcNbPktDrop=%u</br>\n",LrcNbPktDrop);
	fprintf(f,"<br>LrcNbDisc=%u</br>\n",LrcNbDisc);


	fprintf(f,"<br>NbLrc=%u Adjust=%d Redundancy=%d</br>\n",
						NbLrc,AdjustDelay,Redundancy);
	fprintf(f,"<br>StatRefresh=%d RfCellRefresh=%d WanRefresh=%d</br>\n",
				StatRefresh,RfCellRefresh,WanRefresh);
	fprintf(f,"<br>InvertPol=%u NoCrc=%d NoHeader=%d SyncWord=%x</br>\n",
				LgwInvertPol,LgwNoCrc,LgwNoHeader,LgwSyncWord);
	fprintf(f,"<br>Preamble=%u PreambleAck=%d Power=%d AckData802=%u</br>\n",
			LgwPreamble,LgwPreambleAck,LgwPower,LgwAckData802Wait);
	fprintf(f,"<br>SimulN=%u QosAlea=%d</br>\n",
				SimulN,QosAlea);

	fclose(f);
}

static	int	LrcOkCount()
{
	int	lrc;
	int	nb	= 0;

	for	(lrc = 0 ; lrc < NbLrc ; lrc++)
	{
		t_xlap_link		*lk;

		lk	= &TbLapLrc[lrc];
		switch	(lk->lk_state)
		{
		case	SSP_STARTED:
		case	SSP_STOPPED:
			nb++;
		break;
		default	:
		break;
		}
	}
	return	nb;
}

static	int	LrcOkStarted()
{
	int	lrc;
	int	nb	= 0;

	for	(lrc = 0 ; lrc < NbLrc ; lrc++)
	{
		t_xlap_link		*lk;

		lk	= &TbLapLrc[lrc];
		switch	(lk->lk_state)
		{
		case	SSP_STARTED:
			nb++;
		break;
		default	:
		break;
		}
	}
	return	nb;
}

static	t_xlap_link	*LrcFirstStarted()
{
	int	lrc;

	for	(lrc = 0 ; lrc < NbLrc ; lrc++)
	{
		t_xlap_link		*lk;

		lk	= &TbLapLrc[lrc];
		switch	(lk->lk_state)
		{
		case	SSP_STARTED:
			return	lk;
		break;
		default	:
		break;
		}
	}
	return	NULL;
}


static	int	LrcOkStartedTestLoad(int out,int ack)
{
	int	lrc;
	int	nb	= 0;

	for	(lrc = 0 ; lrc < NbLrc ; lrc++)
	{
		t_xlap_link		*lk;

		lk	= &TbLapLrc[lrc];
		switch	(lk->lk_state)
		{
		case	SSP_STARTED:
			if	(out >= 0 && lk->lk_outcount > out)
				break;
			if	(ack >= 0 && lk->lk_ackcount > ack)
				break;
			nb++;
		break;
		default	:
		break;
		}
	}
	return	nb;
}

static	void	DoPowerRefresh(int delay,u_short lptype,u_char state,u_int raw,float volt)
{
	u_char		buff[1024];
	t_lrr_pkt	uppkt;
	int		szh	= LP_HEADER_PKT_SIZE_V0;
	int		szm	= 0;
	char		*data	= NULL;

	t_lrr_power	power;

	power.pw_gss	= (time_t)Currtime.tv_sec;
	power.pw_gns	= (u_int)Currtime.tv_nsec;
	power.pw_raw	= raw;
	power.pw_volt	= volt;
	power.pw_state	= state;

	delay	= 0;
	szm	= 0;
	data	= NULL;
	szh	= LP_PRE_HEADER_PKT_SIZE+sizeof(t_lrr_power);
	memset	(&uppkt,0,sizeof(uppkt));
	uppkt.lp_vers	= LP_LRX_VERSION;
	uppkt.lp_szh	= szh;
	uppkt.lp_flag	= LP_INFRA_PKT_INFRA;
	uppkt.lp_type	= lptype;
	uppkt.lp_lrrid	= LrrID;

	memcpy	(&uppkt.lp_u.lp_power,&power,sizeof(t_lrr_power));
	memcpy	(buff,&uppkt,szh);
	if	(szm > 0)
		memcpy	(buff+szh,data,szm);

	SendStatToAllLrc(buff,szh+szm,delay);
}

void	DoGpsRefresh(int delay, u_short lptype, u_char state, float srate_wma)
{
	u_char		buff[1024];
	t_lrr_pkt	uppkt;
	int		szh	= LP_HEADER_PKT_SIZE_V0;
	int		szm	= 0;
	char		*data	= NULL;

	t_lrr_gps_st	gps_st;

	gps_st.gps_gssupdate	= (time_t)Currtime.tv_sec;
	gps_st.gps_gnsupdate	= (u_int)Currtime.tv_nsec;
	gps_st.gps_state	= state;
	gps_st.gps_srate_wma = srate_wma;

	delay	= 0;
	szm	= 0;
	data	= NULL;
	szh	= LP_PRE_HEADER_PKT_SIZE+sizeof(t_lrr_gps_st);
	memset (&uppkt, 0, sizeof (uppkt));
	uppkt.lp_vers	= LP_LRX_VERSION;
	uppkt.lp_szh	= szh;
	uppkt.lp_flag	= LP_INFRA_PKT_INFRA;
	uppkt.lp_type	= lptype;
	uppkt.lp_lrrid	= LrrID;

	memcpy (&uppkt.lp_u.lp_gps_st, &gps_st, sizeof (t_lrr_gps_st));
	memcpy (buff, &uppkt, szh);
	if (szm > 0)
		memcpy (buff+szh, data, szm);

	SendStatToAllLrc(buff, szh+szm, delay);
}

void	DoStatRefresh(int delay)
{
	static	u_int	StatNbUpdate;

	u_char		buff[1024];
	t_lrr_pkt	uppkt;
	int		szh	= LP_HEADER_PKT_SIZE_V0;
	int		szm	= 0;
	char		*data	= NULL;

	t_lrr_stat_v1	*stat_v1 = &uppkt.lp_u.lp_stat_v1;



RTL_TRDBG(9,"AvDvUiCompute p=%p d=%u t=%u idxlrc=%d\n",
		&CmnLrcAvdvTrip,StatRefresh,Currtime.tv_sec,CurrLrcOrder);

	LrcNbPktTrip	= 
		AvDvUiCompute(&CmnLrcAvdvTrip,StatRefresh,Currtime.tv_sec);
	LrcAvPktTrip	= CmnLrcAvdvTrip.ad_aver;
	LrcDvPktTrip	= CmnLrcAvdvTrip.ad_sdev;
	LrcMxPktTrip	= CmnLrcAvdvTrip.ad_vmax;

	LrcNbDisc	= 0;
	if	(CurrLrcOrder >= 0 && CurrLrcOrder < NB_LRC_PER_LRR)
	{
		t_lrc_link	*lrc;

		lrc		= &TbLrc[CurrLrcOrder];
		LrcNbDisc	= lrc->lrc_stat.lrc_nbdisc;
	}

	DoHtmlFile();

	szm	= 0;
	data	= NULL;
	szh	= LP_PRE_HEADER_PKT_SIZE+sizeof(t_lrr_stat_v1);
	memset	(&uppkt,0,sizeof(uppkt));
	uppkt.lp_vers	= LP_LRX_VERSION;
	uppkt.lp_szh	= szh;
	uppkt.lp_flag	= LP_INFRA_PKT_INFRA;
	uppkt.lp_type	= LP_TYPE_LRR_INF_STATS_V1;
	uppkt.lp_lrrid	= LrrID;

	stat_v1->ls_LrcNbDisc		= LrcNbDisc;
	stat_v1->ls_LrcNbPktDrop	= LrcNbPktDrop;
	stat_v1->ls_LrcNbPktTrip	= LrcNbPktTrip;
	stat_v1->ls_LrcAvPktTrip	= LrcAvPktTrip;
	stat_v1->ls_LrcDvPktTrip	= LrcDvPktTrip;
	stat_v1->ls_LrcMxPktTrip	= LrcMxPktTrip;

	stat_v1->ls_LgwNbPacketSend	= LgwNbPacketSend;
	stat_v1->ls_LgwNbPacketWait	= LgwNbPacketWait;
	stat_v1->ls_LgwNbPacketRecv	= LgwNbPacketRecv;

	stat_v1->ls_LgwNbStartOk	= LgwNbStartOk;
	stat_v1->ls_LgwNbStartFailure	= LgwNbStartFailure;
	stat_v1->ls_LgwNbConfigFailure	= LgwNbConfigFailure;
	stat_v1->ls_LgwNbLinkDown	= LgwNbLinkDown;

	stat_v1->ls_LgwNbBusySend	= LgwNbBusySend;
	stat_v1->ls_LgwNbSyncError	= LgwNbSyncError;
	stat_v1->ls_LgwNbCrcError	= LgwNbCrcError;
	stat_v1->ls_LgwNbSizeError	= LgwNbSizeError;
	stat_v1->ls_LgwNbChanUpError	= LgwNbChanUpError;
	stat_v1->ls_LgwNbChanDownError	= LgwNbChanDownError;
	stat_v1->ls_LgwNbDelayReport	= LgwNbDelayReport;
	stat_v1->ls_LgwNbDelayError	= LgwNbDelayError;
	stat_v1->ls_LgwNbFilterDrop	= LgwNbFilterDrop;
// added in LRC 1.0.14, LRR 1.0.35
	stat_v1->ls_gssuptime		= (time_t)UptimeProc.tv_sec;
	stat_v1->ls_gnsuptime		= (u_int)UptimeProc.tv_nsec;
	stat_v1->ls_nbupdate		= ++StatNbUpdate;
	stat_v1->ls_statrefresh		= StatRefresh;
	stat_v1->ls_versMaj		= VersionMaj;
	stat_v1->ls_versMin		= VersionMin;
	stat_v1->ls_versRel		= VersionRel;
	stat_v1->ls_versFix		= VersionFix;
	stat_v1->ls_LgwInvertPol	= LgwInvertPol;
	stat_v1->ls_LgwNoCrc		= LgwNoCrc;
	stat_v1->ls_LgwNoHeader		= LgwNoHeader;
	stat_v1->ls_LgwPreamble		= LgwPreamble;
	stat_v1->ls_LgwPreambleAck	= LgwPreambleAck;
	stat_v1->ls_LgwPower		= LgwPower;
	stat_v1->ls_LgwAckData802Wait	= LgwAckData802Wait;
	stat_v1->ls_LrrUseGpsPosition	= UseGpsPosition;
	stat_v1->ls_LrrUseGpsTime	= UseGpsTime;
	stat_v1->ls_LrrUseLgwTime	= UseLgwTime;
	stat_v1->ls_LgwSyncWord		= LgwSyncWord;
	stat_v1->ls_rfcellrefresh	= RfCellRefresh;
	stat_v1->ls_gssupdate		= (time_t)Currtime.tv_sec;
	stat_v1->ls_gnsupdate		= (u_int)Currtime.tv_nsec;
	stat_v1->ls_gssuptimesys	= (time_t)UptimeSyst.tv_sec;
	stat_v1->ls_gnsuptimesys	= (u_int)UptimeSyst.tv_nsec;
	stat_v1->ls_sickrestart		= SickRestart;
	stat_v1->ls_configrefresh	= ConfigRefresh;
	stat_v1->ls_wanrefresh		= WanRefresh;

// added in LRC 1.0.31, LRR 1.4.4
	CompAllMfsInfos('L');
	{
	int	i;
	u_int	val;

	for	(i = 0 ; i < NB_MFS_PER_LRR ; i++)
	{
		if	(TbMfs[i].fs_enable == 0)	continue;
		if	(TbMfs[i].fs_exists == 0)	continue;

		val	= TbMfs[i].fs_used;
		stat_v1->ls_mfsUsed[i][0]	= (u_char)(val / (256*256));
		stat_v1->ls_mfsUsed[i][1]	= (u_char)(val / 256);
		stat_v1->ls_mfsUsed[i][2]	= (u_char)val;

		val	= TbMfs[i].fs_avail;
		stat_v1->ls_mfsAvail[i][0]	= (u_char)(val / (256*256));
		stat_v1->ls_mfsAvail[i][1]	= (u_char)(val / 256);
		stat_v1->ls_mfsAvail[i][2]	= (u_char)val;
	}
	}

	AvDvUiCompute(&CpuAvdvUsed,StatRefresh,Currtime.tv_sec);
	stat_v1->ls_cpuAv	= (u_char)CpuAvdvUsed.ad_aver;
	stat_v1->ls_cpuDv	= (u_char)CpuAvdvUsed.ad_sdev;
	stat_v1->ls_cpuMx	= (u_char)CpuAvdvUsed.ad_vmax;
	stat_v1->ls_cpuMxTime	= CpuAvdvUsed.ad_tmax;

	CompAllCpuInfos('L');
	stat_v1->ls_cpuLoad1	= CpuLoad1;
	stat_v1->ls_cpuLoad5	= CpuLoad5;
	stat_v1->ls_cpuLoad15	= CpuLoad15;

	CompAllMemInfos('L');
	stat_v1->ls_MemTotal[0]		= (u_char)(MemTotal / (256*256));
	stat_v1->ls_MemTotal[1]		= (u_char)(MemTotal / 256);
	stat_v1->ls_MemTotal[2]		= (u_char)(MemTotal);
	stat_v1->ls_MemFree[0]		= (u_char)(MemFree / (256*256));
	stat_v1->ls_MemFree[1]		= (u_char)(MemFree / 256);
	stat_v1->ls_MemFree[2]		= (u_char)(MemFree);
	stat_v1->ls_MemBuffers[0]	= (u_char)(MemBuffers / (256*256));
	stat_v1->ls_MemBuffers[1]	= (u_char)(MemBuffers / 256);
	stat_v1->ls_MemBuffers[2]	= (u_char)(MemBuffers);
	stat_v1->ls_MemCached[0]	= (u_char)(MemCached / (256*256));
	stat_v1->ls_MemCached[1]	= (u_char)(MemCached / 256);
	stat_v1->ls_MemCached[2]	= (u_char)(MemCached);


	stat_v1->ls_gpsUpdateCnt 	= ABS(GpsUpdateCnt - GpsUpdateCntP);
	GpsUpdateCntP		 	= GpsUpdateCnt;
#ifdef	KONA
	if	(GpsPositionOk)
		stat_v1->ls_gpsUpdateCnt	= StatRefresh;
	else
		stat_v1->ls_gpsUpdateCnt	= 0;
#endif

	stat_v1->ls_LrrTimeSync		= NtpdStarted();
	stat_v1->ls_LrrReverseSsh	= CmdCountOpenSsh();

	stat_v1->ls_powerState		= PowerState;
	stat_v1->ls_powerDownCnt	= ABS(PowerDownCnt - PowerDownCntP);
	PowerDownCntP			= PowerDownCnt;

	stat_v1->ls_gpsState		= GpsStatus;
	stat_v1->ls_gpsDownCnt		= ABS(GpsDownCnt - GpsDownCntP);
	GpsDownCntP			= GpsDownCnt;

	stat_v1->ls_gpsUpCnt		= ABS(GpsUpCnt - GpsUpCntP);
	GpsUpCntP			= GpsUpCnt;

	stat_v1->ls_rfScan		= CmdCountRfScan();

	stat_v1->ls_traceLevel		= TraceLevel + 1;// 0 means ? for LRC
	stat_v1->ls_useRamDir		= LogUseRamDir;

	stat_v1->ls_dtclt		= DcLt;

	memcpy	(buff,&uppkt,szh);
	if	(szm > 0)
		memcpy	(buff+szh,data,szm);

	SendStatToAllLrc(buff,szh+szm,delay);
}

void	GpsGetInfos(u_char *mode,float *lat,float *lon,short *alt,u_int *cnt)
{
	if (UseGpsPosition == 1) {
#if defined(KONA) && defined(WITH_GPS)
		GetGpsPositionTektelic();
#endif
		/* Real GPS data */
		if (GpsPositionOk && mode && lat && lon && alt) {
			*mode	= 2;	// gps
			*lat	= GpsLatt;
			*lon	= GpsLong;
			*alt	= GpsAlti;
		}
		if (cnt)
			*cnt	= GpsUpdateCnt;
	} else {
		/* Simulated GPS data */
		if (mode && lat && lon && alt) {
			*mode	= 1;
			*lat	= LrrLat;
			*lon	= LrrLon;
			*alt	= LrrAlt;
		}
		if (cnt)
			*cnt	= 0;
	}
}

#ifdef	SX1301AR_MAX_BOARD_NB
/*!
* \fn void DoLocKeyRefresh(int delay, char ** resp)
* \brief Refresh and send AES-128 keys for each board
* \param delay:
* \param resp: Adress of response string
* \return void
*
* This function reads the configuration hashtables to retrieve boards
* finetimestamping AES keys send the result to LRC(s)
* If resp is passed to NULL, IEC104/XML format is used.
* If not, the resp string is filled for using IEC104/Text format (Built-in shell cmd)
*
* resp string should be long enough to contains at least 160 bytes
*
*/
void DoLocKeyRefresh(int delay, char ** resp)
{
	u_char		buff[1024];
	char *		key = NULL;
	int		board, nbBoard;
	char		section[64];
	int		szh	= LP_HEADER_PKT_SIZE_V0;
	char		resp_tmp[40];

	t_lrr_pkt		uppkt;
	t_lrr_config_lockey	loc_key;

	RTL_TRDBG(1,"DoLocKeyRefresh\n");
	/* NFR622: Send configured boards finetimestamps AES-128 keys to LRC */
	nbBoard = CfgInt(HtVarLgw, "gen", -1, "board", 1);
	for (board = 0; board < nbBoard && board < SX1301AR_MAX_BOARD_NB; board++) {
		snprintf(section,sizeof(section),"board:%d", board);
		key = CfgStr(HtVarLrr, section ,-1, "aeskey", NULL);

		if (resp && *resp) {
			if (key && *key) {
			    snprintf(resp_tmp,sizeof(resp_tmp),"card%d=%s\n", board, key);
			} else {
			    snprintf(resp_tmp,sizeof(resp_tmp),"card%d=\n", board);
			}
			strcat(*resp, resp_tmp);
		}
		else {
			szh = LP_PRE_HEADER_PKT_SIZE + sizeof (t_lrr_config_lockey);
			memset(&loc_key, 0, sizeof (loc_key));
			memset(&uppkt, 0, sizeof (uppkt));
			uppkt.lp_vers	= LP_LRX_VERSION;
			uppkt.lp_szh	= szh;
			uppkt.lp_flag	= LP_INFRA_PKT_INFRA;
			uppkt.lp_type	= LP_TYPE_LRR_INF_CONFIG_LOCKEY;
			uppkt.lp_lrrid	= LrrID;

			loc_key.cf_idxboard = board;
			if (key && *key) {
				memcpy(&loc_key.cf_lockey, key, 2 * sizeof (u_char) * SX1301AR_BOARD_AES_KEY_SIZE/8);
			}

			memcpy (&uppkt.lp_u.lp_config_lockey, &loc_key, sizeof (t_lrr_config_lockey));
			memcpy (buff, &uppkt, szh);
			SendStatToAllLrc(buff, szh, delay);
		}
	}
}
#endif





void DoCustomVersionRefresh(int delay)
{
	u_char	buff[1024];
	int	szh = LP_HEADER_PKT_SIZE_V0;

	t_lrr_pkt	uppkt;
	t_lrr_custom_ver	cversions;
	RTL_TRDBG(1, "DoCustomVersionRefresh\n");
	/* NFR590, RDTP-7849 */
	szh = LP_PRE_HEADER_PKT_SIZE + sizeof (t_lrr_custom_ver);
	memset(&cversions, 0, sizeof (cversions));
	memset(&uppkt, 0, sizeof (uppkt));
	uppkt.lp_vers	= LP_LRX_VERSION;
	uppkt.lp_szh	= szh;
	uppkt.lp_flag	= LP_INFRA_PKT_INFRA;
	uppkt.lp_type	= LP_TYPE_LRR_INF_CUSTOM_VERSION;
	uppkt.lp_lrrid	= LrrID;
	/* LRR version */
	strncpy((char *)cversions.cf_cver_lrr, AutoVersion_Lrr, MAX_CUSTOM_VERSION_LEN);
	/* HAL version */
	strncpy((char *)cversions.cf_cver_hal, AutoVersion_Hal, MAX_CUSTOM_VERSION_LEN);
	/* Fpga version */
	strncpy((char *)cversions.cf_cver_fpga, AutoVersion_Fpga, MAX_CUSTOM_VERSION_LEN);
	/* Firmware version */
	strncpy((char *)cversions.cf_cver_fw, AutoVersion_Fw, 2 * MAX_CUSTOM_VERSION_LEN);    /* !!! 64-chars length instead 32 !!! */
	/* Hardware version */
	strncpy((char *)cversions.cf_cver_hw, AutoVersion_Hw, MAX_CUSTOM_VERSION_LEN);
	/* Os version */
	strncpy((char *)cversions.cf_cver_os, AutoVersion_Os, MAX_CUSTOM_VERSION_LEN);
	/* Serial number */
	strncpy((char *)cversions.cf_cver_sn, AutoVersion_Sn, MAX_CUSTOM_VERSION_LEN);
	/* SKU */
	strncpy((char *)cversions.cf_cver_sku, AutoVersion_Sku, MAX_CUSTOM_VERSION_LEN);
	/* Chip Id MSB/LSB */
	/*
	if (strlen(AutoVersion_ChipMsb) <= MAX_CUSTOM_VERSION_LEN)
		strcpy((char *)cversions.cf_cver_chipmsb, AutoVersion_ChipMsb);
	if (strlen(AutoVersion_ChipLsb) <= MAX_CUSTOM_VERSION_LEN)
		strcpy((char *)cversions.cf_cver_chiplsb, AutoVersion_ChipLsb);
	*/
	/* Custom build version */
	strncpy((char *)cversions.cf_cver_build, CustomVersion_Build, MAX_CUSTOM_VERSION_LEN);
	/* Configuration version */
	strncpy((char *)cversions.cf_cver_config, CustomVersion_Config, MAX_CUSTOM_VERSION_LEN);
	/* Custom1 version */
	strncpy((char *)cversions.cf_cver_custom1, CustomVersion_Custom1, MAX_CUSTOM_VERSION_LEN);
	/* Custom2 version */
	strncpy((char *)cversions.cf_cver_custom2, CustomVersion_Custom2, MAX_CUSTOM_VERSION_LEN);
	/* Custom3 version */
	strncpy((char *)cversions.cf_cver_custom3, CustomVersion_Custom3, MAX_CUSTOM_VERSION_LEN);

	memcpy (&uppkt.lp_u.lp_custom_ver, &cversions, sizeof (t_lrr_custom_ver));
	memcpy (buff, &uppkt, szh);

	RTL_TRDBG(3, " cf_cver_lrr: %s\n", cversions.cf_cver_lrr)
	RTL_TRDBG(3, " cf_cver_hal: %s\n", cversions.cf_cver_hal)
	RTL_TRDBG(3, " cf_cver_fpga: %s\n", cversions.cf_cver_fpga)
	RTL_TRDBG(3, " cf_cver_fw: %s\n", cversions.cf_cver_fw)
	RTL_TRDBG(3, " cf_cver_hw: %s\n", cversions.cf_cver_hw)
	RTL_TRDBG(3, " cf_cver_os: %s\n", cversions.cf_cver_os)
	RTL_TRDBG(3, " cf_cver_sn: %s\n", cversions.cf_cver_sn)
	RTL_TRDBG(3, " cf_cver_build: %s\n", cversions.cf_cver_build)
	RTL_TRDBG(3, " cf_cver_config: %s\n", cversions.cf_cver_config)
	RTL_TRDBG(3, " cf_cver_custom1: %s\n", cversions.cf_cver_custom1)
	RTL_TRDBG(3, " cf_cver_custom2: %s\n", cversions.cf_cver_custom2)
	RTL_TRDBG(3, " cf_cver_custom3: %s\n", cversions.cf_cver_custom3)

	SendStatToAllLrc(buff, szh, delay);

}

/*!
* \fn void DoAntsConfigRefresh(int delay, char ** resp)
* \brief Refresh and send antennas configuration, user-requested Tx EIRP and LUT-calibrated Tx EIRP
* \param delay:
* \param resp: Adress of response string
* \return void
*
* This function retrieve the antenna configuration and user-requested Tx EIRP then calculate the LUT-calibrated Tx EIRP.
* All these values are send to LRC(s).
*
* If resp argument is passed to NULL, IEC104/XML format is used.
* If not, the resp string is filled for using IEC104/Text format (Built-in shell cmd)
*
* resp string should be long enough to contains at least 160 bytes
*
*/
void DoAntsConfigRefresh(int delay, char ** resp)
{
	u_char		buff[1024];
	int		szh	= LP_HEADER_PKT_SIZE_V0;
	int 		i;
	char		resp_tmp[40];

	t_lrr_pkt		uppkt;
	t_lrr_config_ants	ants;

	RTL_TRDBG(1,"DoAntsConfigRefresh\n");
	/* NFR620 */
	for (i = 0; (i < LgwAntenna) && (i < NB_ANTENNA); i++)
	{
		if (resp && *resp) {
//			snprintf(resp_tmp,sizeof(resp_tmp),"chain%d=%d-%d\n", i, (int)roundf(AntennaGain[i]), (int)roundf(CableLoss[i]));
			snprintf(resp_tmp,sizeof(resp_tmp),"chain%d=%f-%f\n", i, AntennaGain[i], CableLoss[i]);
			strcat(*resp, resp_tmp);
		}
		else {
			szh = LP_PRE_HEADER_PKT_SIZE + sizeof (t_lrr_config_ants);
			memset(&ants, 0, sizeof (ants));
			memset(&uppkt, 0, sizeof (uppkt));
			uppkt.lp_vers	= LP_LRX_VERSION;
			uppkt.lp_szh	= szh;
			uppkt.lp_flag	= LP_INFRA_PKT_INFRA;
			uppkt.lp_type	= LP_TYPE_LRR_INF_CONFIG_ANTS;
			uppkt.lp_lrrid	= LrrID;

			ants.cf_idxant = i;
			if (AntennaGain[i] || CableLoss[i])
				ants.cf_use_float = 1;

			ants.cf_antgain = (int8_t)roundf(AntennaGain[i]);
			ants.cf_cableloss = (int8_t)roundf(CableLoss[i]);
			ants.cf_antgain_f = AntennaGain[i];
			ants.cf_cableloss_f = CableLoss[i];

			ants.cf_RX1_tx = TbChannel[1].power;
			ants.cf_RX1_eirp = GetTxCalibratedEIRP(TbChannel[1].power, AntennaGain[i], CableLoss[i], 0, 0);
			if	(Rx2Channel)
			{
			ants.cf_RX2_tx = Rx2Channel->power;
			ants.cf_RX2_eirp = GetTxCalibratedEIRP(Rx2Channel->power, AntennaGain[i], CableLoss[i], 0, 0);
			}
			//RTL_TRDBG(1, "Antenna config idx=%d, antgain=%d, cableloss=%d, rx1_tx=%d, rx1_eirp=%d, rx2_tx=%d, rx2_eirp=%d\n", i, ants.cf_antgain, ants.cf_cableloss, ants.cf_RX1_tx, ants.cf_RX1_eirp, ants.cf_RX2_tx, ants.cf_RX2_eirp);
			RTL_TRDBG(1, "Antenna config idx=%d, use_float=%d, antgain=%.1f=%d, cableloss=%.1f=%d, rx1_tx=%d, rx1_eirp=%d, rx2_tx=%d, rx2_eirp=%d\n", i, ants.cf_use_float, ants.cf_antgain_f, ants.cf_antgain, ants.cf_cableloss_f, ants.cf_cableloss, ants.cf_RX1_tx, ants.cf_RX1_eirp, ants.cf_RX2_tx, ants.cf_RX2_eirp);

			memcpy (&uppkt.lp_u.lp_config_ants, &ants, sizeof (t_lrr_config_ants));
			memcpy (buff, &uppkt, szh);
			SendStatToAllLrc(buff, szh, delay);
		}
	}

}


void	DoConfigRefresh(int delay)
{
	static	u_int		ConfigNbUpdate;

	u_char		buff[1024];
	t_lrr_pkt	uppkt;
	int		szh	= LP_HEADER_PKT_SIZE_V0;
	int		szm	= 0;
	char		*data	= NULL;

	int			idxlrc;
	t_lrr_config_lrc	cfglrc;
	t_lrr_config		config;
	int			mfs;
	int 			i;

	for	(idxlrc = 0 ; idxlrc < NbLrc ; idxlrc++)
	{
		int	port	= atoi(TbLapLrc[idxlrc].lk_port);
		char	*addr	= (char *)TbLapLrc[idxlrc].lk_addr;

		if	(port <= 0 || !addr || !*addr)
			continue;
		memset	(&cfglrc,0,sizeof(cfglrc));
		szm	= 0;
		data	= NULL;
		szh	= LP_PRE_HEADER_PKT_SIZE+sizeof(t_lrr_config_lrc);
		memset	(&uppkt,0,sizeof(uppkt));
		uppkt.lp_vers	= LP_LRX_VERSION;
		uppkt.lp_szh	= szh;
		uppkt.lp_flag	= LP_INFRA_PKT_INFRA;
		uppkt.lp_type	= LP_TYPE_LRR_INF_CONFIG_LRC;
		uppkt.lp_lrrid	= LrrID;

		cfglrc.cf_LrcCount	= NbLrc;
		cfglrc.cf_LrcIndex	= idxlrc;
		cfglrc.cf_LrcPort	= (u_short)port;
		strncpy((char *)cfglrc.cf_LrcUrl,addr,64);

		memcpy	(&uppkt.lp_u.lp_config_lrc,&cfglrc,
					sizeof(t_lrr_config_lrc));
		memcpy	(buff,&uppkt,szh);
		if	(szm > 0)
			memcpy	(buff+szh,data,szm);

		SendStatToAllLrc(buff,szh+szm,delay);
	}
#ifdef	SX1301AR_MAX_BOARD_NB
	DoLocKeyRefresh(delay, NULL);
#endif
	DoAntsConfigRefresh(delay, NULL);
	DoCustomVersionRefresh(delay);
	DoLrrUIDConfigRefresh(delay);

	memset	(&config,0,sizeof(config));
	szm	= 0;
	data	= NULL;
	szh	= LP_PRE_HEADER_PKT_SIZE+sizeof(t_lrr_config);
	memset	(&uppkt,0,sizeof(uppkt));
	uppkt.lp_vers	= LP_LRX_VERSION;
	uppkt.lp_szh	= szh;
	uppkt.lp_flag	= LP_INFRA_PKT_INFRA;
	uppkt.lp_type	= LP_TYPE_LRR_INF_CONFIG;
	uppkt.lp_lrrid	= LrrID;

	config.cf_nbupdate		= ++ConfigNbUpdate;
	config.cf_configrefresh		= ConfigRefresh;
	config.cf_gssupdate		= (time_t)Currtime.tv_sec;
	config.cf_gnsupdate		= (u_int)Currtime.tv_nsec;

	ReadRffInfos();

	config.cf_versMaj		= VersionMaj;
	config.cf_versMin		= VersionMin;
	config.cf_versRel		= VersionRel;
	config.cf_versFix		= VersionFix;
	config.cf_versMajRff		= VersionMajRff;
	config.cf_versMinRff		= VersionMinRff;
	config.cf_versRelRff		= VersionRelRff;
	config.cf_versFixRff		= VersionFixRff;
	config.cf_rffTime		= RffTime;
	config.cf_LrxVersion		= LP_LRX_VERSION;
	config.cf_LgwSyncWord		= LgwSyncWord;

#if	0
	config.cf_LrrLocMeth		= 0;	// unknown
	if	(UseGpsPosition == 0)
	{
		config.cf_LrrLocMeth		= 1;	// manual
		config.cf_LrrLocLat		= LrrLat;
		config.cf_LrrLocLon		= LrrLon;
		config.cf_LrrLocAltitude	= LrrAlt;
	}
	else
	{
		if	(GpsPositionOk)
		{
			config.cf_LrrLocMeth		= 2;	// gps
			config.cf_LrrLocLat		= GpsLatt;
			config.cf_LrrLocLon		= GpsLong;
			config.cf_LrrLocAltitude	= GpsAlti;
		}
		else
		{
		}
	}
#endif

	GpsGetInfos(&config.cf_LrrLocMeth,&config.cf_LrrLocLat,
		&config.cf_LrrLocLon,&config.cf_LrrLocAltitude,NULL);

	config.cf_LrrUseGpsPosition	= UseGpsPosition;
	config.cf_LrrUseGpsTime		= UseGpsTime;
	config.cf_LrrUseLgwTime		= UseLgwTime;
	if	(GpsFd != -1)
		config.cf_LrrGpsOk	= 1;

	config.cf_LrcDistribution	= Redundancy;

	memcpy	(config.cf_IpInt,ConfigIpInt.cf_IpInt,
					sizeof(config.cf_IpInt));
	memcpy	(config.cf_ItfType,ConfigIpInt.cf_ItfType,
					sizeof(config.cf_ItfType));
	// [2420]
	for (i=0; i<NB_ITF_PER_LRR; i++)
	{
		if (CellStateReport && ConfigIpInt.cf_ItfType[i] == ITF_TYPE_CELL)
		{
			// Update ConfigIpInt structure with last informations about cellular device
			// required because at startup these informations are not available yet when
			// the first LRR_config report is sent. As econd one is forced as soon as
			// the informations are available
			CellStateCopyIdent(&ConfigIpInt.cf_ItfIdentif_u[i].cf_ItfCellId);
			RTL_TRDBG(1,"Interface type cellular: imei='%s' imsi='%s' iccid='%s'\n",
				ConfigIpInt.cf_ItfIdentif_u[i].cf_ItfCellId.cf_ItfImei,
				ConfigIpInt.cf_ItfIdentif_u[i].cf_ItfCellId.cf_ItfImsi,
				ConfigIpInt.cf_ItfIdentif_u[i].cf_ItfCellId.cf_ItfIccid);
		}
		else if (WifiStateReport && ConfigIpInt.cf_ItfType[i] == ITF_TYPE_WIFI)
		{
			int ln;

			ln = sizeof(ConfigIpInt.cf_ItfIdentif_u[i].cf_ItfWifiId.cf_ItfSsid) - 1;
			strncpy((char *)ConfigIpInt.cf_ItfIdentif_u[i].cf_ItfWifiId.cf_ItfSsid, WifiStateSsid, ln);
			ConfigIpInt.cf_ItfIdentif_u[i].cf_ItfWifiId.cf_ItfSsid[ln] = '\0';
			RTL_TRDBG(1,"Interface type wifi: ssid='%s'\n",
				ConfigIpInt.cf_ItfIdentif_u[i].cf_ItfWifiId.cf_ItfSsid);
		}
	}
	memcpy	(config.cf_ItfIdentif_u,ConfigIpInt.cf_ItfIdentif_u,
					sizeof(config.cf_ItfIdentif_u));

	for	(mfs = 0 ; mfs < NB_MFS_PER_LRR ; mfs++)
	{
		if	(TbMfs[mfs].fs_enable == 0)	continue;
		if	(TbMfs[mfs].fs_exists == 0)	continue;
		strncpy	((char *)config.cf_Mfs[mfs],TbMfs[mfs].fs_name,16);
		config.cf_MfsType[mfs]	= TbMfs[mfs].fs_type;
	}


	memcpy	(&uppkt.lp_u.lp_config,&config,sizeof(t_lrr_config));
	memcpy	(buff,&uppkt,szh);
	if	(szm > 0)
		memcpy	(buff+szh,data,szm);

	SendStatToAllLrc(buff,szh+szm,delay);
}

void	DoRfcellRefresh(int delay)
{
	static	u_int		RfCellNbUpdate;
	static	t_lrr_stat_v1	StatSave;

	u_char		buff[1024];
	t_lrr_pkt	uppkt;
	int		szh	= LP_HEADER_PKT_SIZE_V0;
	int		szm	= 0;
	char		*data	= NULL;

	t_lrr_rfcell	rfcell;

	memset	(&rfcell,0,sizeof(rfcell));
	szm	= 0;
	data	= NULL;
	szh	= LP_PRE_HEADER_PKT_SIZE+sizeof(t_lrr_rfcell);
	memset	(&uppkt,0,sizeof(uppkt));
	uppkt.lp_vers	= LP_LRX_VERSION;
	uppkt.lp_szh	= szh;
	uppkt.lp_flag	= LP_INFRA_PKT_INFRA;
	uppkt.lp_type	= LP_TYPE_LRR_INF_RFCELL;
	uppkt.lp_lrrid	= LrrID;

	rfcell.rf_nbupdate		= ++RfCellNbUpdate;
	rfcell.rf_rfcellrefresh		= RfCellRefresh;
	rfcell.rf_gssupdate		= (time_t)Currtime.tv_sec;
	rfcell.rf_gnsupdate		= (u_int)Currtime.tv_nsec;

	rfcell.rf_LgwNbPacketRecv	= 
		ABS(LgwNbPacketRecv 	- StatSave.ls_LgwNbPacketRecv);
	rfcell.rf_LgwNbCrcError		= 
		ABS(LgwNbCrcError 	- StatSave.ls_LgwNbCrcError);
	rfcell.rf_LgwNbSizeError	= 
		ABS(LgwNbSizeError 	- StatSave.ls_LgwNbSizeError);
	rfcell.rf_LgwNbDelayReport	= 
		ABS(LgwNbDelayReport 	- StatSave.ls_LgwNbDelayReport);
	rfcell.rf_LgwNbDelayError	= 
		ABS(LgwNbDelayError 	- StatSave.ls_LgwNbDelayError);

	rfcell.rf_LgwNbPacketSend	= 
		ABS(LgwNbPacketSend 	- StatSave.ls_LgwNbPacketSend);
	rfcell.rf_LgwNbPacketWait	= 
		ABS(LgwNbPacketWait 	- 0);
	rfcell.rf_LgwNbBusySend		= 
		ABS(LgwNbBusySend 	- StatSave.ls_LgwNbBusySend);

	rfcell.rf_LgwNbStartOk		= 
		ABS(LgwNbStartOk 	- StatSave.ls_LgwNbStartOk);
	rfcell.rf_LgwNbStartFailure	= 
		ABS(LgwNbStartFailure 	- StatSave.ls_LgwNbStartFailure);
	rfcell.rf_LgwNbConfigFailure	= 
		ABS(LgwNbConfigFailure 	- StatSave.ls_LgwNbConfigFailure);
	rfcell.rf_LgwNbLinkDown		= 
		ABS(LgwNbLinkDown 	- StatSave.ls_LgwNbLinkDown);
	rfcell.rf_LgwNbFilterDrop		= 
		ABS(LgwNbFilterDrop 	- StatSave.ls_LgwNbFilterDrop);
	rfcell.rf_LgwFilterVersion	= NfVersion;
	rfcell.rf_LgwNetwork		= NfNetwork;

	rfcell.rf_LgwState		= 0;
	if	(!LgwThreadStopped && LgwStarted())
	{
		if	(DownRadioStop)
			rfcell.rf_LgwState		= 1;
		else
			rfcell.rf_LgwState		= 2;
	}

	rfcell.rf_LgwBeaconRequestedCnt	= LgwBeaconRequestedCnt;
	rfcell.rf_LgwBeaconSentCnt	= LgwBeaconSentCnt;
	rfcell.rf_LgwBeaconLastDeliveryCause	= LgwBeaconLastDeliveryCause;

	StatSave.ls_LgwNbPacketRecv	= LgwNbPacketRecv;
	StatSave.ls_LgwNbCrcError	= LgwNbCrcError;
	StatSave.ls_LgwNbSizeError	= LgwNbSizeError;
	StatSave.ls_LgwNbDelayReport	= LgwNbDelayReport;
	StatSave.ls_LgwNbDelayError	= LgwNbDelayError;

	StatSave.ls_LgwNbPacketSend	= LgwNbPacketSend;
	StatSave.ls_LgwNbBusySend	= LgwNbBusySend;

	StatSave.ls_LgwNbStartOk	= LgwNbStartOk;
	StatSave.ls_LgwNbStartFailure	= LgwNbStartFailure;
	StatSave.ls_LgwNbConfigFailure	= LgwNbConfigFailure;
	StatSave.ls_LgwNbLinkDown	= LgwNbLinkDown;
	StatSave.ls_LgwNbFilterDrop	= LgwNbFilterDrop;


	memcpy	(&uppkt.lp_u.lp_rfcell,&rfcell,sizeof(t_lrr_rfcell));
	memcpy	(buff,&uppkt,szh);
	if	(szm > 0)
		memcpy	(buff+szh,data,szm);

	SendStatToAllLrc(buff,szh+szm,delay);
}

// Read state file from failovermgr and return type of interface actually used
// return ITF_TYPE_ETHE | ITF_TYE_WIFI | ITF_TYPE_CELL | -1 on error
static	int	ReadItfUsed()
{
	FILE	*f;
	char	buf[128], fof[128], *pt;
	char	itused[40];
	int	ret;
	// states:
	// 0: up and used
	// 1: up and ready for backup
	// 2: service down
	// 3: no ip
	// 4: link down
	// 5: no used whatever the state

	itused[0] = '\0';
	snprintf(fof,sizeof(fof),"%s/" FAILOVERSTATEFILE, RootAct);
	f = fopen(fof, "r");
	if (!f)
	{
		RTL_TRDBG(0,"ReadItfUsed: can't read failover state (file='%s')\n", fof);
		return -1;
	}
	while (fgets(buf, sizeof(buf), f))
	{
		if ((pt = strstr(buf, "ITUSED=")))
		{
			pt += 7;
			strncpy(itused, pt, sizeof(itused)-1);
			itused[sizeof(itused)-1] = '\0';
			if ((pt = strchr(itused, '\n')))
				*pt = '\0';
			if ((pt = strchr(itused, '\r')))
				*pt = '\0';
			RTL_TRDBG(4,"ReadItfUsed: itused='%s'\n", itused);
		}
		/* reading ITWAIT is useless */
	}
	fclose(f);

	ret = -1;
	if (!strcmp(itused, "wifi"))
		ret = ITF_TYPE_WIFI;
	else if (!strcmp(itused, "cellular"))
		ret = ITF_TYPE_CELL;
	else
		ret = ITF_TYPE_ETHE;
	return ret;
}

static void cleanStr(char *s, int sz)
{
	int	i;

	if (!s || !*s)
		return;

	for (i=0; i<sz; i++)
	{
		if (s[i] == '\0')
			return;
		if (!isprint(s[i]))
			s[i] = ' ';
	}
}

static	void	DoWanRefresh(int delay)
{
	static	u_int	WanNbUpdate;

	u_char		buff[1024];
	t_lrr_pkt	uppkt;
	int		szh	= LP_HEADER_PKT_SIZE_V0;
	int		szm	= 0;
	char		*data	= NULL;
	int		deltadrop	= 0;
	int		nblrc	= 0;
	int		itfUsed = -1;

	t_lrr_wan	wan;
	int		i;

	for	(i = 0 ; i < NB_LRC_PER_LRR ; i++)
	{
		t_lrc_link	*lrc;
		t_xlap_link	*lk;
		t_lrr_wan_lrc	wanlrc;
		int		state;
		int		nbelem;

		memset	(&wanlrc,0,sizeof(wanlrc));

		lrc	= &TbLrc[i];
		lk	= lrc->lrc_lk;
		if	(!lk)		continue;
		nblrc++;

		nbelem = AvDvUiCompute(&lrc->lrc_avdvtrip,WanRefresh,Currtime.tv_sec);

		state	= (lk->lk_state == SSP_STARTED);
		lrc->lrc_stat.lrc_nbsendB	= lk->lk_stat.st_nbsendB;
		lrc->lrc_stat.lrc_nbsend	= lk->lk_stat.st_nbsendu
						+  lk->lk_stat.st_nbsends
						+  lk->lk_stat.st_nbsendi;

		lrc->lrc_stat.lrc_nbrecvB	= lk->lk_stat.st_nbrecvB;
		lrc->lrc_stat.lrc_nbrecv	= lk->lk_stat.st_nbrecvu
						+  lk->lk_stat.st_nbrecvs
						+  lk->lk_stat.st_nbrecvi;
		lrc->lrc_stat.lrc_nbdrop	= LrcNbPktDrop;

		wanlrc.wl_LrcCount		= NbLrc;
		wanlrc.wl_LrcIndex		= i;
		wanlrc.wl_LrcState		= state;
		wanlrc.wl_LrcNbPktTrip		= nbelem;
		wanlrc.wl_LrcAvPktTrip		= lrc->lrc_avdvtrip.ad_aver;
		wanlrc.wl_LrcDvPktTrip		= lrc->lrc_avdvtrip.ad_sdev;
		wanlrc.wl_LrcMxPktTrip		= lrc->lrc_avdvtrip.ad_vmax;
		wanlrc.wl_LrcMxPktTripTime	= lrc->lrc_avdvtrip.ad_tmax;

		wanlrc.wl_LrcNbIecUpByte	= ABS(lrc->lrc_stat.lrc_nbsendB
						- lrc->lrc_stat_p.lrc_nbsendB);
		wanlrc.wl_LrcNbIecUpPacket	= ABS(lrc->lrc_stat.lrc_nbsend
						- lrc->lrc_stat_p.lrc_nbsend);
		wanlrc.wl_LrcNbIecDownByte	= ABS(lrc->lrc_stat.lrc_nbrecvB
						- lrc->lrc_stat_p.lrc_nbrecvB);
		wanlrc.wl_LrcNbIecDownPacket	= ABS(lrc->lrc_stat.lrc_nbrecv
						- lrc->lrc_stat_p.lrc_nbrecv);
		wanlrc.wl_LrcNbDisc		= ABS(lrc->lrc_stat.lrc_nbdisc
						- lrc->lrc_stat_p.lrc_nbdisc);
		deltadrop			= ABS(lrc->lrc_stat.lrc_nbdrop
						- lrc->lrc_stat_p.lrc_nbdrop);

		memcpy	(&lrc->lrc_stat_p,&lrc->lrc_stat,sizeof(lrc->lrc_stat));

		szm	= 0;
		data	= NULL;
		szh	= LP_PRE_HEADER_PKT_SIZE+sizeof(t_lrr_wan_lrc);
		memset	(&uppkt,0,sizeof(uppkt));
		uppkt.lp_vers	= LP_LRX_VERSION;
		uppkt.lp_szh	= szh;
		uppkt.lp_flag	= LP_INFRA_PKT_INFRA;
		uppkt.lp_type	= LP_TYPE_LRR_INF_WAN_LRC;
		uppkt.lp_lrrid	= LrrID;

		memcpy	(&uppkt.lp_u.lp_wan_lrc,&wanlrc,sizeof(t_lrr_wan_lrc));
		memcpy	(buff,&uppkt,szh);
		if	(szm > 0)
			memcpy	(buff+szh,data,szm);

		SendStatToAllLrc(buff,szh+szm,delay);
	}

	memset	(&wan,0,sizeof(wan));
	szm	= 0;
	data	= NULL;
	szh	= LP_PRE_HEADER_PKT_SIZE+sizeof(t_lrr_wan);
	memset	(&uppkt,0,sizeof(uppkt));
	uppkt.lp_vers	= LP_LRX_VERSION;
	uppkt.lp_szh	= szh;
	uppkt.lp_flag	= LP_INFRA_PKT_INFRA;
	uppkt.lp_type	= LP_TYPE_LRR_INF_WAN;
	uppkt.lp_lrrid	= LrrID;

	wan.wl_nbupdate		= ++WanNbUpdate;
	wan.wl_wanrefresh	= WanRefresh;
	wan.wl_gssupdate	= (time_t)Currtime.tv_sec;
	wan.wl_gnsupdate	= (u_int)Currtime.tv_nsec;

	wan.wl_LrcCount		= nblrc;
	wan.wl_LrcNbPktDrop	= deltadrop;
	wan.wl_ItfCount 	= 0;

	// If failovermgr used, read information from its status file
	if (!IfaceDaemon)
	{
		itfUsed = ReadItfUsed();
		RTL_TRDBG(1,"Interface type actually used: %d\n", itfUsed);
	}

	for	(i = 0; i < NB_ITF_PER_LRR ; i++)
	{
		t_wan_itf	*itf;
		u_short		sent;
		u_short		lost;
		u_short		okay;

		itf	= &TbItf[i];
		if	(!itf)			continue;
		wan.wl_ItfCount++;
		wan.wl_ItfIndex[i]	= i;
		wan.wl_ItfType[i]	= itf->it_type;
		wan.wl_ItfState[i]	= StateItf(itf);

		wan.wl_ItfRssiNeverUsed[i]	= 0;
		// [2420][12898] dynamic data
		wan.wl_ItfKpisSet[i]	= 0;
		wan.wl_ItfRssi[i]		= 0.0;		// 1
		wan.wl_ItfRscp[i]		= 0.0;		// 2
		wan.wl_ItfEcio[i]		= 0.0;		// 4
		wan.wl_ItfRsrp[i]		= 0.0;		// 8
		wan.wl_ItfRsrq[i]		= 0.0;		// 16
		wan.wl_ItfSinr[i]		= 0.0;		// 32

		if	(!itf->it_enable)	continue;
		if	(!itf->it_name)		continue;
//		if	(!itf->it_exists)	continue;

		switch	(itf->it_type)
		{
		case	ITF_TYPE_ETHE:
		{
			strncpy	((char *)wan.wl_ItfTechno[i],"ethernet",
						sizeof(wan.wl_ItfTechno[i])-1);
			strncpy	((char *)wan.wl_ItfOper[i],"",	// ! applicable
						sizeof(wan.wl_ItfOper[i])-1);
			if (!IfaceDaemon && itfUsed == ITF_TYPE_ETHE)
			{
				wan.wl_ItfState[i] = 0;
			}
		}
		break;
		// cellular interface
		case	ITF_TYPE_CELL:
		{
			if (!CellStateReport)
				break;
			wan.wl_ItfKpisSet[i]	= 0;
			// if thread that interogates cellular device did not create the
			// structure, that means it failed to access the device
			if (!CellState)
			{
				RTL_TRDBG(1,"No cellular information available now (RSSI, mode ...)\n");
				break;
			}
			RTL_TRDBG(4,"Set cellular informations in lrr_wan ...\n");
			strncpy	((char *)wan.wl_ItfOper[i], CellState->ope,
						sizeof(wan.wl_ItfOper[i])-1);
			// clean ItfOper to remove unexpected characters that make problems to TWA
			cleanStr((char *)wan.wl_ItfOper[i], sizeof(wan.wl_ItfOper[i])-1);
			RTL_TRDBG(4,"ItfOper raw='%s' clean='%s'\n", CellState->ope, wan.wl_ItfOper[i]);
			if	(CellState->mode == CELL_MODE_GSM)
			{
				strncpy	((char *)wan.wl_ItfTechno[i], "3G",
						sizeof(wan.wl_ItfTechno[i])-1);
				wan.wl_ItfKpisSet[i]		|= 1;
				wan.wl_ItfRssi[i]		= CellState->indic.cont.gsm.rssi;
				wan.wl_ItfRssiUpdateCount[i]	= CellState->indic.update_count;
				CellState->indic.update_count = 0;
				RTL_TRDBG(4,"GSM: rssi=%.2f\n", wan.wl_ItfRssi[i]);
			}
			else if	(CellState->mode == CELL_MODE_WCDMA)
			{
				strncpy	((char *)wan.wl_ItfTechno[i], "3G",
						sizeof(wan.wl_ItfTechno[i])-1);
				wan.wl_ItfKpisSet[i]	|= 1;
				wan.wl_ItfRssi[i]		= CellState->indic.cont.wcdma.rssi;
				wan.wl_ItfRssiUpdateCount[i]	= CellState->indic.update_count;
				CellState->indic.update_count = 0;
				wan.wl_ItfKpisSet[i]	|= 2;
				wan.wl_ItfRscp[i]		= CellState->indic.cont.wcdma.rscp;
				wan.wl_ItfKpisSet[i]	|= 4;
				wan.wl_ItfEcio[i]		= CellState->indic.cont.wcdma.ecio;
				RTL_TRDBG(4,"WCDMA: rssi=%.2f rscp=%.2f ecio=%.2f\n",
					wan.wl_ItfRssi[i], wan.wl_ItfRscp[i], wan.wl_ItfEcio[i]);
			}
			else if	(CellState->mode == CELL_MODE_LTE)
			{
				strncpy	((char *)wan.wl_ItfTechno[i], "4G",
						sizeof(wan.wl_ItfTechno[i])-1);
				wan.wl_ItfKpisSet[i]	|= 1;
				wan.wl_ItfRssi[i]		= CellState->indic.cont.lte.rssi;
				wan.wl_ItfRssiUpdateCount[i]	= CellState->indic.update_count;
				CellState->indic.update_count = 0;

				wan.wl_ItfKpisSet[i]	|= 8;
				wan.wl_ItfRsrp[i]		= CellState->indic.cont.lte.rsrp;

				wan.wl_ItfKpisSet[i]	|= 16;
				wan.wl_ItfRsrq[i]		= CellState->indic.cont.lte.rsrq;

				wan.wl_ItfKpisSet[i]	|= 32;
				wan.wl_ItfSinr[i]		= CellState->indic.cont.lte.sinr;
				RTL_TRDBG(4,"LTE: rssi=%.2f rsrp=%.2f rsrq=%.2f sinr=%.2f\n",
					wan.wl_ItfRssi[i], wan.wl_ItfRsrp[i], wan.wl_ItfRsrq[i], wan.wl_ItfSinr[i]);
			}
			else
			{
				RTL_TRDBG(4, "Cellular device absent or no connection established\n");
			}
			RTL_TRDBG(4,"KpisSet[%d] = 0x%x RssiUpdateCount=%d\n", i, wan.wl_ItfKpisSet[i], wan.wl_ItfRssiUpdateCount[i]);

			if (!IfaceDaemon && itfUsed == ITF_TYPE_CELL)
			{
				wan.wl_ItfState[i] = 0;
			}
		}
		break;
		case	ITF_TYPE_WIFI:
		{
			wan.wl_ItfKpisSet[i]	= 0;
			strncpy	((char *)wan.wl_ItfTechno[i],"wifi",
						sizeof(wan.wl_ItfTechno[i])-1);
			strncpy	((char *)wan.wl_ItfOper[i],"",	// ! applicable
						sizeof(wan.wl_ItfOper[i])-1);

			if (WifiStateReport == 0)
			{
				RTL_TRDBG(1,"WifiStateReport disabled\n");
				break;
			}
			wan.wl_ItfKpisSet[i]	|= 1;
			wan.wl_ItfRssi[i]		= WifiStateRssi;
			wan.wl_ItfRssiUpdateCount[i]	= WifiStateRssiCount;
			WifiStateRssiCount = 0;
			wan.wl_ItfState[i]	= WifiStateItf;
			RTL_TRDBG(4,"Wifi: rssi=%.2f\n", wan.wl_ItfRssi[i]);

			if (!IfaceDaemon && itfUsed == ITF_TYPE_WIFI)
			{
				wan.wl_ItfState[i] = 0;
			}
		}
		break;
		default:
		{
			strncpy	((char *)wan.wl_ItfTechno[i],"unknown",
						sizeof(wan.wl_ItfTechno[i])-1);
			strncpy	((char *)wan.wl_ItfOper[i],"",	// ! applicable
						sizeof(wan.wl_ItfOper[i])-1);
		}
		break;
		}


		wan.wl_ItfActivityTime[i]	= itf->it_deftime;
		wan.wl_ItfAddr[i]		= itf->it_ipv4;

		sent	= ABS(itf->it_sentprtt - itf->it_sentprtt_p);
		lost	= ABS(itf->it_lostprtt - itf->it_lostprtt_p);
		okay	= ABS(itf->it_okayprtt - itf->it_okayprtt_p);
		if	(sent > 0 && okay)
		{
			wan.wl_ItfNbPktTrip[i]		= (u_short)sent;
			wan.wl_ItfLsPktTrip[i]		= (u_short)lost;
			wan.wl_ItfAvPktTrip[i]		= 
					(u_short)itf->it_avdvprtt.ad_aver;
			wan.wl_ItfDvPktTrip[i]		= 
					(u_short)itf->it_avdvprtt.ad_sdev;
			wan.wl_ItfMxPktTrip[i]		= 
					(u_short)itf->it_avdvprtt.ad_vmax;
			wan.wl_ItfMxPktTripTime[i]	= 
					(u_int)itf->it_avdvprtt.ad_tmax;
		}
		itf->it_sentprtt_p	= itf->it_sentprtt;
		itf->it_lostprtt_p	= itf->it_lostprtt;
		itf->it_okayprtt_p	= itf->it_okayprtt;

		wan.wl_ItfRxTraffic[i]		= itf->it_lgt.it_rxbytes_d;
		wan.wl_ItfRxAvgRate[i]		= itf->it_lgt.it_rxbytes_br;
		wan.wl_ItfRxMaxRate[i]		= itf->it_lgt.it_rxbytes_mbr;
		wan.wl_ItfRxMaxRateTime[i]	= itf->it_lgt.it_rxbytes_mbrt;

		wan.wl_ItfTxTraffic[i]		= itf->it_lgt.it_txbytes_d;
		wan.wl_ItfTxAvgRate[i]		= itf->it_lgt.it_txbytes_br;
		wan.wl_ItfTxMaxRate[i]		= itf->it_lgt.it_txbytes_mbr;
		wan.wl_ItfTxMaxRateTime[i]	= itf->it_lgt.it_txbytes_mbrt;
	}


	//	FIX3323
	if	(StorePktCount > 0)
	{
		int	nb	= rtl_imsgCount(StoreQ);
		if	(nb > 0)
		{
			wan.wl_LtqCnt		= nb;
		}
		if	(MaxStorePktCount > 0)
		{
			wan.wl_LtqMxCnt		= MaxStorePktCount;
			wan.wl_LtqMxCntTime	= MaxStorePktTime;
		}
		MaxStorePktCount= 0;
		MaxStorePktTime	= 0;
	}

	memcpy	(&uppkt.lp_u.lp_wan,&wan,sizeof(t_lrr_wan));
	memcpy	(buff,&uppkt,szh);
	if	(szm > 0)
		memcpy	(buff+szh,data,szm);

	SendStatToAllLrc(buff,szh+szm,delay);
}

static char * AutoRebootFile         = "usr/data/lrr/autorebootcnt.txt";
static char * AutoRebootFileNoUplink = "usr/data/lrr/autorebootcnt_nouplink.txt";
static char * AutoRestartLrrProcess  = "usr/data/lrr/autorestartlrrprocess.txt";

static  void AutoRebootGetFile(int cause, char (* file)[], int n)
{
	switch(cause) {
		case REBOOT_NO_UPLINK_RCV:
			snprintf(*file, n, "%s/%s", RootAct, AutoRebootFileNoUplink);
			break;
		case REBOOT_NO_LRC_COMM:
		default:
			snprintf(*file, n, "%s/%s", RootAct, AutoRebootFile);
			break;
	}
}

static	void	AutoRebootReset(int cause)
{
	char	file[1024];
	AutoRebootGetFile(cause, &file, sizeof(file));
	unlink	(file);
}

static	unsigned int	AutoRebootInc(int cause)
{
	int	inc	= 0;
	char	file[1024];
	FILE	*f;

	AutoRebootGetFile(cause, &file, sizeof(file));
	f	= fopen(file,"r");
	if	(f)
	{
		fscanf	(f,"%u",&inc);
		fclose	(f);
	}
	inc++;
	f	= fopen(file,"w");
	if	(f)
	{
		fprintf	(f,"%u",inc);
		fclose	(f);
	}
	return	inc;
}

static void AutoRestartLrrProcessReset(void)
{
	char file[1024];
	snprintf(file,sizeof(file),"%s/%s", RootAct, AutoRestartLrrProcess);
	unlink(file);
}

static unsigned int AutoRestartLrrProcessInc(void)
{
	int inc = 0;
	char file[1024];
	FILE * f = NULL;
	snprintf(file,sizeof(file),"%s/%s", RootAct, AutoRestartLrrProcess);
	f = fopen(file, "r");
	if (f) {
		fscanf(f, "%u", &inc);
		fclose(f);
	}
	inc++;
	f = fopen(file, "w");
	if (f) {
		fprintf(f, "%u", inc);
		fclose(f);
	}
	return inc;
}


static	void	NeedRevSsh(int nblrcok)
{
	static	int	revssh	= 0;
	static	time_t	tnocnx	= 0;
	time_t		now;
	int		tm	= AutoRevSshTimer;
	char		cmdStart[1024];
	char		cmdStop[1024];
	int		port;

#if	0
	port	= 50000 + (LrrID % 10000);
#endif
	port	= AutoRevSshPort;	// rdtp-6639
	snprintf(cmdStop,sizeof(cmdStop),"./shelllrr.sh closessh -Z 0 -P %d 2>&1 > /dev/null",
			port);
	snprintf(cmdStart,sizeof(cmdStart),"./shelllrr.sh openssh -Z 0 -a -K -P %d 2>&1 > /dev/null",
			port);

	if	(nblrcok > 0)
	{
		tnocnx	= 0;
		if	(revssh)
		{
			RTL_TRDBG(0,"LRC connection ok => kill revssh\n");
			revssh	= 0;
			DoSystemCmdBackGround(cmdStop);
		}
		return;
	}
	if	(tnocnx == 0)
	{
		tnocnx	= rtl_timemono(NULL);
		return;
	}
	now	= rtl_timemono(NULL);
	if	(ABS(now-tnocnx) <= tm)
	{
		return;
	}
	if	(revssh)
	{
		revssh++;
		if	(revssh <= 10)
		{
			return;
		}
		RTL_TRDBG(0,"LRC connection ko => kill&start revssh\n");
		revssh	= 0;
		DoSystemCmdBackGround(cmdStop);
		sleep(3);
	}
	// no LRC connection during more than N sec
	RTL_TRDBG(0,"no LRC connection during more than %dsec => revssh\n",tm);
	revssh	= 1;

	DoSystemCmdBackGround(cmdStart);
}


static	void	NeedReboot(int nblrcok)
{
	static	time_t	tnocnx	= 0;
	time_t		now;
	int		tm	= AutoRebootTimer;
	unsigned int	autorebootcnt;
	char		cmd[1024];
	FILE		*f;

	if	(nblrcok > 0)
	{
		AutoRebootReset(REBOOT_NO_LRC_COMM);
		tnocnx	= 0;
		return;
	}
	if	(tnocnx == 0)
	{
		tnocnx	= rtl_timemono(NULL);
		return;
	}
	now	= rtl_timemono(NULL);
	if	(ABS(now-tnocnx) <= tm)
	{
		return;
	}
	// no LRC connection during more than N sec
	autorebootcnt	= AutoRebootInc(REBOOT_NO_LRC_COMM);
	RTL_TRDBG(0,"no LRC connection during more than %dsec => reboot\n",tm);
	RTL_TRDBG(0,"auto reboot counter %d (%d)\n",autorebootcnt,AutoRestoreNoLrc);

	if	(AutoRestoreNoLrc && autorebootcnt > AutoRestoreNoLrc)
	{

		AutoRebootReset(REBOOT_NO_LRC_COMM);
		RTL_TRDBG(0,
		"too much auto reboot due to no LRC link => restore image ?\n");

#if	0	// not necessary because restoremgr tests it
		int	saved	= 0;
		snprintf(cmd,sizeof(cmd),"%s/usr/etc/lrr/saverff_done",RootAct);
		if	(access(cmd,F_OK) == 0)		saved	= 1;
		RTL_TRDBG(0,"'%s' exists = %d\n",cmd,saved);
#endif

		int	locked	= 0;
		snprintf(cmd,sizeof(cmd),"%s/usr/etc/lrr/execrff_locked",RootAct);
		if	(access(cmd,F_OK) == 0)		locked	= 1;
		RTL_TRDBG(0,"'%s' exists = %d\n",cmd,locked);

		int	oksh	= 0;
		snprintf(cmd,sizeof(cmd),"%s/lrr/com/cmd_shells/genericmgr.sh",RootAct);
		if	(access(cmd,X_OK) == 0)		oksh	= 1;
		RTL_TRDBG(0,"'%s' exists + x = %d\n",cmd,oksh);

		if	(!locked && oksh)
		{
			snprintf(cmd,sizeof(cmd),"%s/lrr/com/cmd_shells/genericmgr.sh restoremgr --restore 2>/dev/null",
				RootAct);
			RTL_TRDBG(0,"%s\n",cmd);
			system(cmd);
		}
	}

	snprintf(cmd,sizeof(cmd),"%s/usr/etc/lrr/autoreboot_last",RootAct);
	unlink(cmd);
	f	= fopen(cmd,"w");
	if	(f)
	{
		fprintf(f, "No LRC connection during more than %d s => reboot #%d\n", tm, autorebootcnt);
		fclose(f);
	}
	system("reboot");
}

static void NeedRebootNoUplink(int nbpktuplink)
{
	static int     nbpktuplink_last = 0;
	unsigned int   autorestartcnt = 0;
	unsigned int   autorebootcnt = 0;
	static time_t  tnocnx	= 0;
	time_t         now;
	int            tm = AutoRebootTimerNoUplink;
	int            cnt_max = AutoRestartLrrMaxNoUplink;
	char           cmd[1024];
	FILE         * f;

	if ((nbpktuplink > 0) && (nbpktuplink - nbpktuplink_last > 0))
	{
		AutoRebootReset(REBOOT_NO_UPLINK_RCV);
		AutoRestartLrrProcessReset();
		tnocnx = 0;
		nbpktuplink_last = nbpktuplink;
		return;
	}
	if (tnocnx == 0)
	{
		RTL_TRDBG(2, "NeedRebootNoUplink: no uplink received since the last function call. Start timer\n");
		tnocnx = rtl_timemono(NULL);
		return;
	}
	now	= rtl_timemono(NULL);
	RTL_TRDBG(2, "NeedRebootNoUplink: no uplink received since %u s\n", (unsigned int)(ABS(now-tnocnx)));
	if (ABS(now-tnocnx) < tm)
	{
		return;
	}

	autorestartcnt = AutoRestartLrrProcessInc();
	if (autorestartcnt <= cnt_max)
	{
		RTL_TRDBG(0, "NeedRebootNoUplink: no radio uplink packet received during %d s or more => restart (%d / %d)\n", tm, autorestartcnt, cnt_max);
		snprintf(cmd,sizeof(cmd),"%s/usr/etc/lrr/autorestart_last", RootAct);
		unlink(cmd);
		f = fopen(cmd, "w");
		if (f)
		{
			fprintf(f, "No radio uplink packet received during %d s => restart (%d / %d)\n", tm, autorestartcnt, cnt_max);
			fclose(f);
		}
		ServiceStop(0);
		return;
	}
	AutoRestartLrrProcessReset();
	autorebootcnt = AutoRebootInc(REBOOT_NO_UPLINK_RCV);
	RTL_TRDBG(0, "NeedRebootNoUplink: no radio uplink packet received during %d s or more => reboot (#%d)\n", tm, autorebootcnt);
	snprintf(cmd,sizeof(cmd),"%s/usr/etc/lrr/autoreboot_last", RootAct);
	unlink(cmd);
	f = fopen(cmd, "w");
	if (f)
	{
		fprintf(f, "No radio uplink packet received => reboot #%d\n", autorebootcnt);
		fclose(f);
	}
	system("reboot");
}

/* Number of values used to calculate the GPS status weighted moving average */
#define GPS_STATUS_WMA_NB 3
#ifdef WITH_GPS
static int CheckGpsStatus()
{
	static float 	srate[GPS_STATUS_WMA_NB] = {1.0, 1.0, 1.0};
	float 		srate_wma = 0.0;
	int 		i = 0;
	int 		ret = 0;
	int 		tektelic = 0;
	u_short		lptype;
	u_int 		cnt = GpsUpdateCnt;
	static u_int 	cnt_prev = 0;

#ifdef	WITH_SX1301_X1
	// RDTP-5475
	if	(SynchroPeriod)
		return	0;
#endif

#if defined(KONA) && defined(WITH_GPS)
	tektelic = 1;
#endif
	if ( tektelic
	     || (!cnt) 				/* if GPS is not working at all */
	     || (cnt >= GpsStRefresh)		/* Avoid first 30 NMEA frames */
	     || (cnt == cnt_prev))		/* if GPS starts working but fall down before the first 30 NMEA frames */
	{	 
		for (i = GPS_STATUS_WMA_NB - 1; i > 0; i--)
			srate[i] = srate[i-1];
#if defined(KONA) && defined(WITH_GPS)
		if (GpsNbCheckTektelic)
			srate[0] = (float)(cnt - cnt_prev) / (float)GpsNbCheckTektelic;
		else
			srate[0] = srate[1];
		RTL_TRDBG(4, "GPS Status cnt=(%u-%u)/%d (srate=%.2f)\n", cnt, cnt_prev, GpsNbCheckTektelic, srate[0]);
		GpsNbCheckTektelic = 0;
#else
		srate[0] = (float)(cnt - cnt_prev) / (float)GpsStRefresh;
		RTL_TRDBG(4, "GPS Status cnt=%u/%d (srate=%.2f)\n", cnt - cnt_prev, GpsStRefresh, srate[0]);
#endif
	}

	/* Calcul de la moyenne mobile pondérée */
	/* Weighted moving average */
	for (i=0; i < GPS_STATUS_WMA_NB; i++)
		srate_wma += srate[i] * (GPS_STATUS_WMA_NB - i);
	srate_wma /= GPS_STATUS_WMA_NB * (GPS_STATUS_WMA_NB + 1) / 2;
	cnt_prev = cnt;
	switch (GpsStatus) {
		default:
			GpsStatus = 'U';
		case 'U':
			if (srate_wma < GpsStRateLow) {
				GpsStatus = 'D';
				ret = 1;
				GpsDownCnt++;
				lptype = LP_TYPE_LRR_INF_GPS_DOWN;
			}
			break;
		case 'D':
			if (srate_wma > GpsStRateHigh) {
				GpsStatus = 'U';
				ret = 1;
				GpsUpCnt++;
				lptype = LP_TYPE_LRR_INF_GPS_UP;
			}
			break;
	}
	RTL_TRDBG(4, "GPS Status %c (srate_wma=%.2f ,sr0=%.2f, sr1=%.2f, sr2=%.2f)\n", GpsStatus, \
		srate_wma, srate[0], srate[1], srate[2]);
	if (ret > 0) {
		DoGpsRefresh(0, lptype, GpsStatus, srate_wma);
	}
	return ret;
}
#endif /* WITH_GPS */

static	int	Power()
{
	FILE		*f		= NULL;
	int		ret;
	int		value;
	u_short		lptype;

	if	(PowerEnable ==0)
		return	0;

	f	= fopen(PowerDevice,"r");
	if	(!f)
	{
		RTL_TRDBG(0,"cannot open '%s' => disable\n",PowerDevice);
		PowerEnable	= 0;
		return	0;
	}

	ret	= fscanf(f,"%d",&value);
	fclose	(f);
	if	(ret != 1)
	{
		RTL_TRDBG(0,"cannot read '%s' => disable\n",PowerDevice);
		PowerEnable	= 0;
		return	0;
	}

//RTL_TRDBG(1,"power read power=%d state=%c\n",value,PowerState);
	ret	= 0;
	switch	(PowerState)
	{
	default:
		PowerState	= 'U';
	case	'U':
		if	(value < PowerDownLev)
		{
			PowerState	= 'D';
			ret		= 1;
			lptype		= LP_TYPE_LRR_INF_POWER_DOWN;
		}
	break;
	case	'D':
		if	(value > PowerUpLev)
		{
			PowerState	= 'U';
			ret		= 1;
			lptype		= LP_TYPE_LRR_INF_POWER_UP;
			PowerDownCnt++;
		}
	break;
	}
	if	(ret > 0)
	{
		float	volt	= 0.0;

#ifdef	WIRMAV2
		if	(value > 1023)
			value	= 1023;
		volt	= (float)value;
		volt	= (volt * 456.0 / 10230.0) + 0.4;
#endif
		DoPowerRefresh(0,lptype,PowerState,(u_int)value,volt);
	}

	return	ret;
}

static	float	ConvTemperature(int X)
{
float	t;
t = (float)(((209310.0 - (float)((285000.0 * (X&0x3FF)) / 1023.0))) / 1083.0);
return	t;
}

static	int	Temperature()
{
	FILE		*f		= NULL;
	int		ret;
	int		value;
	float		fvalue;

	if	(TempEnable ==0)
		return	0;

	f	= fopen(TempDevice,"r");
	if	(!f)
	{
		RTL_TRDBG(0,"cannot open '%s' => disable\n",TempDevice);
		TempEnable	= 0;
		return	0;
	}

	ret	= fscanf(f,"%d",&value);
	fclose	(f);
	if	(ret != 1)
	{
		RTL_TRDBG(0,"cannot read '%s' => disable\n",TempDevice);
		TempEnable	= 0;
		return	0;
	}
	fvalue		= ConvTemperature(value);
	CurrTemp	= roundf(fvalue);

RTL_TRDBG(3,"temperature read raw=%d temp=%f (%d)\n",value,fvalue,CurrTemp);
	return	ret;
}

// RDTP-2574
static	void	LrcSaveStatus(int evt)
{
	static	time_t	lastupdate	= 0;
	static	int	PrevState[NB_LRC_PER_LRR];	// RDTP-5172
	char	*name	= "lrcstatuslink";
	char	file[PATH_MAX];
	char	filetmp[PATH_MAX];
	FILE	*f;
	int	i;
	int	chg	= 0;

	snprintf(file,sizeof(file),"%s/var/log/lrr/%s.txt",RootAct,name);
	snprintf(filetmp,sizeof(filetmp),"%s/var/log/lrr/%s.tmp",RootAct,name);
	unlink	(filetmp);
	f	= fopen(filetmp,"w");
	if	(!f)
	{
		RTL_TRDBG(1,"error cannot open/write '%s'\n",filetmp);
		return;
	}
	fprintf	(f,"LRRPID=%d\n",getpid());
	for	(i = 0 ; i < 2 && i < NB_LRC_PER_LRR ; i++)
	{
		t_lrc_link	*lrc;
		t_xlap_link	*lk;
		int		state;
		char		*name;
		char		*statetxt;

		name	= "LRCPRIMARY";
		state		= 0;
		statetxt	= "none";
		if	(i >= 1)
			name	= "LRCSECONDARY";
		lrc     = &TbLrc[i];
		if	(lrc && (lk=lrc->lrc_lk))
		{
			switch	(lk->lk_state)
			{
			case	SSP_STARTED:
			case	SSP_STOPPED:
				state		= 2;
				statetxt	= "ok";
			break;
			default:
				state		= 1;
				statetxt	= "ko";
			break;
			}
		}
		fprintf	(f,"%s=%s\n",name,statetxt);
		if	(PrevState[i] != state)
			chg	= 1;
		PrevState[i]	= state;
	}
	if	(chg || lastupdate == 0)
		time(&lastupdate);
	fprintf	(f,"LASTUPDATE=%u\n",(u_int)lastupdate);
	fclose	(f);
	unlink	(file);
	rename	(filetmp,file);
}

#define	PERIODICITY(cnt,per) ( ((per) && ((cnt) % (per)) == 0) ? 1 : 0 )
#define	TPWADELAY	10000

static	void	DoClockSc()
{
	static	unsigned	int	nbclock		= 0;
	static	unsigned	int	refreshclock	= 0;
	static			int	startedonce	= 0;
				int	tpwadelay	= TPWADELAY;
				int	nblrcok;
				int	ret;
#if	0
	u_char		buff[1024];
	t_lrr_pkt	uppkt;
	int		szh	= LP_HEADER_PKT_SIZE_V0;
	int		szm;
	char		*data	= "bonjour";
#endif

	clock_gettime(CLOCK_REALTIME,&Currtime);

	if	(LapTest)
	{
		goto	laponly;
	}

	ret	= Power();
	if	(ret)
	{
		RTL_TRDBG(0,"enter power state %c\n",PowerState);
		// TODO send a message to LRCs
		goto	laponly;
	}

	if	(access(ExtLrrRestart,R_OK) == 0)
	{
		RTL_TRDBG(1,"lrr restart '%s'\n",ExtLrrRestart);
		unlink	(ExtLrrRestart);
		exit(0);
		return;
	}
	if	(access(ExtRadioRestart,R_OK) == 0)
	{
		RTL_TRDBG(1,"lrr radiorestart '%s'\n",ExtRadioRestart);
		unlink	(ExtRadioRestart);
		DownRadioStop		= 0;
		LgwThreadStopped	= 0;
		ReStartLgwThread();
		return;
	}
	if	(access(ExtConfigRefresh,R_OK) == 0)
	{
		RTL_TRDBG(1,"lrr config refresh '%s'\n",ExtConfigRefresh);
		unlink	(ExtConfigRefresh);
		DoConfigRefresh(0);
		return;
	}

	CompAllItfInfos('S');
	CompAllCpuInfos('S');
	CompAllMemInfos('S');

	if	(startedonce == 0)
	{
		startedonce	= LrcOkStarted();
		/* PT-1432: no refresh of LRR version after an upgrade */
		/* refresh the config first time LRC are seen after starting */
		if (startedonce) 
		{
			RTL_TRDBG(1,"DoConfigRefresh forced (LRC reachable)\n");
			DoConfigRefresh(0);
		}
	}


	if	(startedonce)
	{
		ret	= Power();
		if	(ret)
		{
			RTL_TRDBG(0,"enter power state %c\n",PowerState);
			// TODO send a message to LRCs
			goto	laponly;
		}
	}

	tpwadelay = TPWADELAY;
	if	(startedonce && PERIODICITY(refreshclock,StatRefresh))
	{
		RTL_TRDBG(1,"DoStatRefresh\n");
		DoStatRefresh(tpwadelay);
		tpwadelay += TPWADELAY;
	}

	if	(startedonce && WifiStateReport && PERIODICITY(refreshclock,WifiStateReport))
	{
		RTL_TRDBG(1,"ReadWifiState\n");
		ReadWifiState();
	}

	if	(startedonce && PERIODICITY(refreshclock,ConfigRefresh))
	{
		RTL_TRDBG(1,"DoConfigRefresh\n");
		DoConfigRefresh(tpwadelay);
		tpwadelay += TPWADELAY;
	}

	if	(startedonce && PERIODICITY(refreshclock,WanRefresh))
	{
		CompAllItfInfos('L');
		RTL_TRDBG(1,"DoWanRefresh\n");
		DoWanRefresh(tpwadelay);
		tpwadelay += TPWADELAY;
	}

	if	(startedonce && (PERIODICITY(refreshclock,RfCellRefresh) || SendRfCellNow))
	{
		RTL_TRDBG(1,"DoRfcellRefresh\n");
		DoRfcellRefresh(tpwadelay);
		tpwadelay += TPWADELAY;
		SendRfCellNow = 0;
	}
	
#ifdef WITH_GPS
	if (startedonce && PERIODICITY(refreshclock, GpsStRefresh))
	{
#if defined(KONA)
		GetGpsPositionTektelic();
#endif
		ret = CheckGpsStatus();
		if (ret) {
			RTL_TRDBG(1, "GPS Status changed to %c\n", GpsStatus);
		}
	}
	/* Check if Gps thread is still running. If not restart it. */
	if (UseGps && ((nbclock % 30) == 0)) {
		int gps_thr_status = -1;
		if ((gps_thr_status = GpsThreadStatus()) == 0)
			ReStartGpsThread();

	}
#endif

	/* Check if CellState thread is still running. If not restart it. */
	if (CellStateReport && ((nbclock % 30) == 0))
	{
		if (IsTypeInterface(ITF_TYPE_CELL) && CellStateThreadStatus() == 0)
			ReStartCellStateThread();
	}

	if	(startedonce)
	{
		refreshclock++;
	}

	nbclock++;
	if	((nbclock % 30) == 0)
	{
		nblrcok	= LrcOkCount();
		if	(AutoRebootTimer > 0)
		{
			NeedReboot(nblrcok);
		}
		if	((AutoRebootTimerNoUplink > 0) && (CfgRadioStop == 0))
		{
			NeedRebootNoUplink(LgwNbPacketRecv);
		}
		if	(AutoRevSshTimer > 0)
		{
			NeedRevSsh(nblrcok);
		}
	}
	if	((nbclock % 30) == 0)
	{
		Temperature();
	}

	// RDTP-2413
	if	(NwkfEnable && (nbclock % NwkfReloadPeriod) == 0)
	{
		NfReload();
	}
	// RDTP-2413
	if	(startedonce && NwkfEnable && NwkfRequestPeriod > 0
			&& (refreshclock % NwkfRequestPeriod) == 0)
	{
		if	(LrrID)
			SendNetworkFilterRequest(LrcFirstStarted());
	}


	laponly :
	if	(NbLrc <= 1)
	{
		LapDoClockSc();
	}
	else
	{	// period of TCP reconnect depends of number of LRC connected
		int	lrc;
		lrc	= LrcOkCount();
		LapDoClockScPeriod(3 + (lrc*10));
	}
	if	((nbclock % 120) == 0)
	{	// RDTP-2574
		LrcSaveStatus(0);
	}

}

static	void	DoClockHh()
{
	static	unsigned int nbclock = 0;
	char	exe[1024];
	char	root[1024];
	char	cmd[1024];
	char	trace[1024];
	int	dayopen;
    int cr;


	RTL_TRDBG(1,"DoClockHh()\n");

	// backup traces
	dayopen	= rtl_tracedayopen();
#if	defined(KEROS) || !defined(WIRMAV2)
	if	(dayopen != -1 && LogUseRamDir)
#else
	if	(dayopen != -1)
#endif
	{
        snprintf(root,sizeof(root),"%s/lrr/com/shells", RootAct);
        cr = SystemGetFilePath(root, "backuptraces.sh", exe, sizeof(exe));
		if	((cr == SYSTEM_OK) && (access(exe,X_OK) == 0))
		{
			strcpy	(trace,"/dev/null");
			snprintf(cmd,sizeof(cmd),"%s %d 2>&1 > %s &",exe,dayopen,trace);
			DoSystemCmdBackGround(cmd);
			RTL_TRDBG(1,"backuptraces started day=%d\n",dayopen);
		}
		else
		{
			RTL_TRDBG(1,"backuptraces does not exist\n");
		}
	}
	nbclock++;
}

int	ClearStoreQ()
{
	int		nb	= 0;
	t_imsg		*msg;
	t_lrr_pkt	*uppkt;

	while	((msg = rtl_imsgGet(StoreQ,IMSG_MSG)))
	{
		uppkt	= (t_lrr_pkt *)msg->im_dataptr;
		if	(uppkt->lp_payload)
			free(uppkt->lp_payload);
		uppkt->lp_payload	= NULL;
		rtl_imsgFree(msg);
		nb++;
	}
	RTL_TRDBG(1,"clear stored messages nb=%d\n",nb);
	return	nb;
}

static	void	RunStoreQMs()	// function called 10 times per sec
{
	static	u_int	count;
	int		nb;
	t_imsg		*msg;
	t_lrr_pkt	*uppkt;

	count++;

	if	(StorePktCount <= 0)
		return;
	if	(StoreQ == NULL)
		return;
	if	(LrrID == 0)
		return;
	// NFR997: Master lrc not yet identified
	if	(LrrIDGetFromBS && MasterLrc < 0)
		return;

//	count % (ceil(10.0 / (float)ReStorePerSec))

	switch	(ReStorePerSec)
	{
	case	10:	// each time no control
	break;
	case	5:
		if	( (count % 2) != 0)
			return;
	break;
	case	3:
		if	( (count % 3) != 0)
			return;
	break;
	case	2:
		if	( (count % 5) != 0)
			return;
	break;
	case	1:
		if	( (count % 10) != 0)
			return;
	break;
	default:
		ReStorePerSec	= 1;
	break;
	}

	nb	= rtl_imsgCount(StoreQ);
	if	(nb <= 0)
		return;

	//	FIX3323
	if	(nb > MaxStorePktCount)
	{
		MaxStorePktCount= nb;
		MaxStorePktTime	= (time_t)Currtime.tv_sec;
	}

	if	(LrcOkStartedTestLoad(ReStoreCtrlOutQ,ReStoreCtrlAckQ) <= 0)
		return;

	msg	= rtl_imsgGet(StoreQ,IMSG_MSG);
	if	(!msg)
		return;

	uppkt	= (t_lrr_pkt *)msg->im_dataptr;
	RecvRadioPacket(uppkt,1);

	if (TraceLevel >= 3)
	{
		LoRaMAC_t	mf;
		u_char		*pt;
		char	dst[64];
		LoRaMAC_decodeHeader(uppkt->lp_payload,uppkt->lp_size,&mf);
		pt	= (u_char *)&mf.DevAddr;

		snprintf(dst,sizeof(dst),"%02x%02x%02x%02x",*(pt+3),*(pt+2),*(pt+1),*pt);
		RTL_TRDBG(3,"unstored msg: seq=%d devaddr=%s sz=%d\n", mf.FCnt, dst, uppkt->lp_size);
	}

	if	(uppkt->lp_payload)
		free(uppkt->lp_payload);
	uppkt->lp_payload	= NULL;
	rtl_imsgFree(msg);
	nb	= rtl_imsgCount(StoreQ);
	RTL_TRDBG(1,"unstore message nb=%d\n",nb);
}

static	int	RecvRadioPacketStore(t_imsg *msg,t_lrr_pkt *uppkt,char *from)
{
	static	u_int	localdrop;	// TODO
	int	nb;
	float	memused	= 0.0;

	if	(StorePktCount <= 0)
		return	0;
	if	(StoreQ == NULL)
		return	0;

	nb	= rtl_imsgCount(StoreQ);
	if	(!msg || !uppkt)
		return	nb;
	if	(!from)
		from	= "?";

	if	(MemTotal)
	memused	= ((float)(MemUsed-MemBuffers-MemCached)/(float)MemTotal)*100.0;

	uppkt->lp_type	= LP_TYPE_LRR_PKT_RADIO_LATE;
	uppkt->lp_flag	= uppkt->lp_flag | LP_RADIO_PKT_LATE;
	if	( (nb >= StorePktCount) || (memused && memused > StoreMemUsed))
	{
		t_imsg		*drop;
		t_lrr_pkt	*dppkt;

		drop	= rtl_imsgGet(StoreQ,IMSG_MSG);
		if	(drop)
		{
			dppkt	= (t_lrr_pkt *)drop->im_dataptr;
			if	(dppkt->lp_payload)
				free(dppkt->lp_payload);
			dppkt->lp_payload	= NULL;
			rtl_imsgFree(drop);
			localdrop++;
		}
	}
	rtl_imsgAdd(StoreQ,msg);
	nb	= rtl_imsgCount(StoreQ);

RTL_TRDBG(1,"store message nb=%d max=%d drop=%u memused=%f from %s (%d,%d)\n",
	nb,StorePktCount,localdrop,memused,from,uppkt->lp_szh,uppkt->lp_size);
	if (TraceLevel >= 3)
	{
		LoRaMAC_t	mf;
		u_char		*pt;
		char	dst[64];
		LoRaMAC_decodeHeader(uppkt->lp_payload,uppkt->lp_size,&mf);
		pt	= (u_char *)&mf.DevAddr;

		snprintf(dst,sizeof(dst),"%02x%02x%02x%02x",*(pt+3),*(pt+2),*(pt+1),*pt);
		RTL_TRDBG(3,"  stored msg: seq=%d devaddr=%s sz=%d\n", mf.FCnt, dst, uppkt->lp_size);
	}

	return	nb;
}

// RDTP-2266
static	int	SaveStoreQ(int callatexit)
{
	char	file[PATH_MAX];
	int	nb;
	int	fd;
	int	err;
	t_imsg	*imsg;

	if	(DiskSaveStoreQ <= 0)
		return	0;
	nb	= rtl_imsgCount(StoreQ);
	snprintf(file,sizeof(file),"%s/usr/data/lrr/storeq",RootAct);
	RTL_TRDBG(1,"save stored messages nb=%d to '%s'\n",nb,file);
	if	(nb <= 0)
		return	0;
	fd	= open(file,O_WRONLY|O_CREAT|O_TRUNC,S_IRUSR|S_IWUSR);
	if	(fd < 0)
	{
		RTL_TRDBG(0,"cannot open '%s'\n",file);
		return	-1;
	}

	err	= 0;
	nb	= 0;
	while	((imsg = rtl_imsgGet(StoreQ,IMSG_MSG)))
	{
		t_lrr_pkt	*pkt;
		u_char		*data;
		int		szh;
		int		szm;
		int		ret;

		err	= 0;
		pkt	= (t_lrr_pkt *)imsg->im_dataptr;
		szh	= LP_PRE_HEADER_PKT_SIZE+sizeof(t_lrr_pkt_radio);
		szm	= pkt->lp_size;
		data	= pkt->lp_payload;
RTL_TRDBG(3,"save stored message nb=%d (%d,%d)\n",nb,szh,szm);
		ret	= write(fd,(void *)&szh,sizeof(szh));
		if	(ret != sizeof(szh))	{ err = __LINE__ ; break; }
		ret	= write(fd,(void *)pkt,szh);
		if	(ret != szh)		{ err = __LINE__ ; break; }
		ret	= write(fd,(void *)&szm,sizeof(szm));
		if	(ret != sizeof(szm))	{ err = __LINE__ ; break; }
		ret	= write(fd,(void *)data,szm);
		if	(ret != szm)		{ err = __LINE__ ; break; }
		nb++;
	}
	close	(fd);

	RTL_TRDBG(1,"save stored messages nb=%d err=%d\n",nb,err);
	return	nb;
}

// RDTP-2266
static	int	LoadStoreQ()
{
	char	file[PATH_MAX];
	int	nb	= 0;
	int	fd;
	int	sz	= 0;
	int	err	= 0;
	struct	stat	st;
	if	(DiskSaveStoreQ <= 0)
		return	0;
	snprintf(file,sizeof(file),"%s/usr/data/lrr/storeq",RootAct);
	RTL_TRDBG(1,"load stored messages from '%s'\n",file);
	fd	= open(file,O_RDWR);
	if	(fd < 0)
	{
		RTL_TRDBG(0,"cannot open '%s'\n",file);
		return	-1;
	}

	if	(fstat(fd,&st) < 0)
	{
		RTL_TRDBG(0,"cannot stat '%s'\n",file);
		return	-1;
	}

	CompAllMemInfos('S');

	err	= 0;
	nb	= 0;
	sz	= 0;
	while	(sz < st.st_size)
	{
		t_imsg		*imsg;
		t_lrr_pkt	pkt;
		u_char		data[1024];
		int		szh;
		int		szm;
		int		ret;

		err	= 0;
		imsg	= rtl_imsgAlloc(IM_DEF,IM_LGW_RECV_DATA,NULL,0);
		if	(!imsg)			{ err = __LINE__ ; break; }

		ret	= read(fd,(void *)&szh,sizeof(szh));
		if	(ret != sizeof(szh))	{ err = __LINE__ ; break; }
		sz	+= ret;
		if	(szh > sizeof(data))	{ err = __LINE__ ; break; }
		ret	= read(fd,(void *)data,szh);
		if	(ret != szh)		{ err = __LINE__ ; break; }
		sz	+= ret;
		if	(szh > LP_PRE_HEADER_PKT_SIZE+sizeof(t_lrr_pkt_radio))
			szh = LP_PRE_HEADER_PKT_SIZE+sizeof(t_lrr_pkt_radio);
		memset	(&pkt,0,sizeof(pkt));
		memcpy	(&pkt,data,szh);
		pkt.lp_szh	= szh;

		ret	= read(fd,(void *)&szm,sizeof(szm));
		if	(ret != sizeof(szm))	{ err = __LINE__ ; break; }
		sz	+= ret;

		pkt.lp_size	= szm;
		pkt.lp_payload	= (u_char *)malloc(szm);
		if	(!pkt.lp_payload)	{ err = __LINE__ ; break; }

		ret	= read(fd,(void *)pkt.lp_payload,szm);
		if	(ret != szm)		{ err = __LINE__ ; break; }
		sz	+= ret;

		if	( rtl_imsgDupData(imsg,&pkt,sizeof(pkt)) != imsg)
						{ err = __LINE__ ; break; }

		RecvRadioPacketStore(imsg,&pkt,"disk");
		nb++;
		if	((nb % 10) == 0)
			CompAllMemInfos('S');
	}
	close	(fd);
	unlink	(file);
	RTL_TRDBG(1,"load stored messages nb=%d err=%d sz=%d tot=%d\n",
		nb,err,sz,st.st_size);

	return	nb;
}

static	int	RecvRadioPacket(t_lrr_pkt *uppkt,int late)
{
	u_char	buff[1024];
	int	szh	= LP_HEADER_PKT_SIZE_V0;
	int	szm	= uppkt->lp_size;
	u_char	*data	= uppkt->lp_payload;
	u16	crcccitt= 0;
	u16	crcnode = 0;
	u_char	mtype;

	if	(late)
		goto	lateonly;

	// RDTP-2413 : check if devaddr OK, return 1 to drop
	// RDTP-5475 : still never drop join:0 and now never drop proprietary:7
	mtype	= *data >> 5;
	if	(NwkfEnable && szm >= 5 && mtype != 0 && mtype != 7)
	{
		u_int	devaddr;

		devaddr	= (*(data+4) << 24) | (*(data+3) << 16);
		devaddr	|= (*(data+2) << 8) | (*(data+1));
		if	(NfCheck(devaddr) <= 0)
		{
			LgwNbFilterDrop++;
			RTL_TRDBG(1,"NWF devaddr=%08x dropped\n",devaddr);
			return	1;
		}
		RTL_TRDBG(1,"NWF devaddr=%08x forwarded\n",devaddr);
	}

	if	(MacWithFcsUp && szm >= 2)
	{
		crcnode	= data[szm-2] + (data[szm-1] << 8);
		crcccitt= crc_ccitt(0,data,szm-2);
	}

	RTL_TRDBG(2,"MAC RECV sz=%d crcccitt=0x%04x crcnode=0x%04x\n",
			szm,crcccitt,crcnode);

	if	(crcnode != crcccitt)
	{
		MacNbFcsError++;
		return	1;		// drop
	}
	if	(MacWithFcsUp)
		szm	= szm - 2;	// suppress FCS

RTL_TRDBG(2,"MAC RECV\n");
	if	(TraceLevel > 3)
	{
		rtl_binToStr(data,szm,(char *)buff,sizeof(buff)-10);
		RTL_TRDBG(4,"MAC RECV\n<<<'%s'\n",buff);
	}

lateonly:
	szh		= LP_PRE_HEADER_PKT_SIZE+sizeof(t_lrr_pkt_radio);
	uppkt->lp_vers	= LP_LRX_VERSION;
	uppkt->lp_flag	|= LP_RADIO_PKT_UP;	// keeps flags comming from lgw
	uppkt->lp_lrrid	= LrrID;
	uppkt->lp_type	= LP_TYPE_LRR_PKT_RADIO;
	uppkt->lp_szh	= szh;
	if	(late)
	{
		uppkt->lp_type	= LP_TYPE_LRR_PKT_RADIO_LATE;
		uppkt->lp_flag	= uppkt->lp_flag | LP_RADIO_PKT_LATE;
	}

	memcpy	(buff,uppkt,szh);
	memcpy	(buff+szh,data,szm);

#ifdef	WITH_TRACK_TIC
	TrackTic[TrackTicCnt % TrackTicNb][0]	= uppkt->lp_tms;
	TrackTic[TrackTicCnt % TrackTicNb][1]	= uppkt->lp_tus;
	TrackTicCnt++;
#endif

	return	SendToLrc(NULL,buff,szh+szm);
}

static	void	DoInternalEvent(t_imsg *imsg,int *freemsg)
{
	static	int	startFailure;

	*freemsg	= 1;
	RTL_TRDBG(9,"receive event cl=%d ty=%d\n",imsg->im_class,imsg->im_type);
	switch(imsg->im_class)
	{
	case	IM_DEF :
		switch(imsg->im_type)
		{
		case	IM_SERVICE_STATUS_RQST :
			ServiceStatusResponse();
		break;
		case	IM_LGW_RECV_DATA :	// uplink packet from radio
		{
			t_lrr_pkt	*uppkt;
			int		ret;

			uppkt	= (t_lrr_pkt *)imsg->im_dataptr;
			// NFR997: store messages until MasterLrc is identified
			if (LrrIDGetFromBS && MasterLrc == -1)
			{
				RTL_TRDBG(3,"Master lrc not yet identified, message will be stored\n");
				ret = 0;
			}
			else
				ret	= RecvRadioPacket(uppkt,0);
			if	(ret == 0)
			{
				*freemsg	= 0;
				RecvRadioPacketStore(imsg,uppkt,"radio");
				break;
			}
			if	(uppkt->lp_payload)
				free(uppkt->lp_payload);
			uppkt->lp_payload	= NULL;
		}
		break;
		case	IM_LGW_SENT_INDIC :
		{
			t_lrr_pkt	*uppkt;

			uppkt	= (t_lrr_pkt *)imsg->im_dataptr;
			SendIndicToLrc(uppkt);
		}
		break;
		case	IM_LGW_POST_DATA :	// postponed downlink packet
		{
			t_lrr_pkt	*dnpkt;

			dnpkt	= (t_lrr_pkt *)imsg->im_dataptr;
			SendRadioPacket(dnpkt,NULL,0);
		}
		break;
		case	IM_LGW_DELAY_ALLLRC  :	// delayed stats to all lrc
		{
			u_char	*buff;
			int	sz;

			buff	= (u_char *)imsg->im_dataptr;
			sz	= imsg->im_datasz;
			SendStatToAllLrc(buff,sz,0);
			RTL_TRDBG(3,"SendStatToAllLrc() retrieved\n");
		}
		break;
		case	IM_CMD_RECV_DATA :	// uplink data from commands
		{
			t_lrr_pkt	*resppkt;
			int		sz;
			t_xlap_link	*lk;
			u_char		buff[1024];

			resppkt	= (t_lrr_pkt *)imsg->im_dataptr;
			sz	= imsg->im_datasz;
			lk	= (t_xlap_link *)resppkt->lp_lk;
			RTL_TRDBG(3,"Retrieve command data lk=%p sz=%d\n",
							lk,sz);

			memcpy	(buff,resppkt,resppkt->lp_szh);
			LapPutOutQueue(lk,(u_char *)buff,resppkt->lp_szh);
		}
		break;
		case	IM_LGW_CONFIG_FAILURE :
			if	(OnConfigErrorExit)
			{
				ServiceStop(0);
				exit(1);
			}
			ReStartLgwThread();
		break;
		case	IM_LGW_START_FAILURE :
		{
			char	cmd[1024];

			if	(OnStartErrorExit)
			{
				ServiceStop(0);
				exit(1);
			}
			startFailure++;
			if	(startFailure < 10)
			{	// just restart the radio thread
				ReStartLgwThread();
				break;
			}
			// try to power off on the radio board
			// and restart the radio thread
			startFailure	= 0;
			snprintf(cmd,sizeof(cmd),"%s/lrr/com/cmd_shells/%s/",
								RootAct,System);
			strcat	(cmd,"radiopower.sh");
			if	(access(cmd,X_OK) != 0)
			{
			RTL_TRDBG(0,"RADIO power on/off not provided on '%s'\n",
									System);
				ReStartLgwThread();
				break;
			}
			strcat	(cmd," 2>&1 > /dev/null &");
			DoSystemCmdBackGround(cmd);
			RTL_TRDBG(0,"RADIO power on/off\n");
			ReStartLgwThread();
		}
		break;
		case	IM_LGW_STARTED :
			startFailure	= 0;
			if	(CfgRadioDnStop)
			{
				DownRadioStop	= 1;
				RTL_TRDBG(0,"RADIO downlink stopped by config\n");
			}
		break;
		case	IM_LGW_LINK_DOWN :
			ReStartLgwThread();
		break;
		}
	break;
	}
}

static	void	DoInternalTimer(t_imsg *imsg)
{
	u_int	szmalloc;
#ifndef	MACOSX
	struct	mallinfo info;
	info	= mallinfo();
	szmalloc= info.uordblks;
#else
	szmalloc= 0;
#endif
	t_lrc_link	*lrc;

	lrc	= imsg->im_to;
	RTL_TRDBG(3,"receive timer cl=%d ty=%d vsize=%ld alloc=%u lrc=%p\n",
		imsg->im_class,imsg->im_type,rtl_vsize(getpid()),szmalloc,lrc);

	switch(imsg->im_class)
	{
	case	IM_DEF :
		switch(imsg->im_type)
		{
		case	IM_TIMER_GEN :
		rtl_imsgAdd(MainQ,
		rtl_timerAlloc(IM_DEF,IM_TIMER_GEN,IM_TIMER_GEN_V,NULL,0));
		break;
		case	IM_TIMER_LRRUID_RESP :	// NFR684
		{
			char	*ok = "NOK";
			if (LrrIDGetFromTwa <= 1)
			{
				RTL_TRDBG(1,"On timer failed to get lrrid from twa => lrrid '%08x' used\n", LrrID);
				break;
			}
			// RDTP-9756/7649 : LrrIDGetFromTwa == 2 ie fromtwaonly
			if (!lrc)		// error no LRC ???
			{
			RTL_TRDBG(1,"Timer lrrid from twa lrc=NULL => %s\n",ok);
				break;
			}
			if (lrc->lrc_lk && lrc->lrc_twaresp && lrc->lrc_twalrridresp)
				ok	= "OK";
RTL_TRDBG(1,"Timer lrrid from twa idxlrc=%d lk=%p resp=%d lrrid=%08x => %s\n",
	lrc->lrc_idx,lrc->lrc_lk,lrc->lrc_twaresp,lrc->lrc_twalrridresp,ok);
			if (!lrc->lrc_lk)	// error no link
				break;
			if (lrc->lrc_twaresp && lrc->lrc_twalrridresp)	
			{
				// a postive response was received from this LRC
				break;
			}
			LapDisc(lrc->lrc_lk,"");
		}
		break;
		}
	break;
	}
}

static	void	MainLoop()
{
	time_t	lasttimems	= 0;
	time_t	lasttimesc	= 0;
	time_t	lasttimehh	= 0;
	time_t	now		= 0;
	int	ret;
	int	freemsg;

	t_imsg	*msg;


	unlink	(ExtLrrRestart);
	unlink	(ExtRadioRestart);

	rtl_imsgAdd(MainQ,
	rtl_timerAlloc(IM_DEF,IM_TIMER_GEN,IM_TIMER_GEN_V,NULL,0));

	while	(1)
	{
		// internal events
		while ((msg= rtl_imsgGet(MainQ,IMSG_MSG)) != NULL)
		{
			freemsg	= 1;
			DoInternalEvent(msg,&freemsg);
			if	(freemsg)
				rtl_imsgFree(msg);
		}

		// external events
		ret	= rtl_poll(MainTbPoll,MSTIC);
		if	(ret < 0)
		{
		}

		// clocks
		now	= rtl_tmmsmono();
		if	(ABS(now-lasttimems) >= 100)
		{
			RunStoreQMs();
			DoClockMs();
			lasttimems	= now;
		}
		if	(ABS(now-lasttimesc) >= 1000)
		{
			if	(MainWantStop && --MainStopDelay <= 0)
			{
				RTL_TRDBG(0,"auto stopped\n");
				break;
			}
			if	(LapTest && --LapTestDuration <= 0)
			{
				RTL_TRDBG(0,"lap test auto stopped\n");
				break;
			}
			DoClockSc();
			lasttimesc	= now;
		}
		if	(ABS(now-lasttimehh) >= 3600*1000)
		{
			DoClockHh();
			lasttimehh	= now;
		}

		// internal timer
		while ((msg= rtl_imsgGet(MainQ,IMSG_TIMER)) != NULL)
		{
			DoInternalTimer(msg);
			rtl_imsgFree(msg);
		}
	}
}

static	void	SigHandler(int sig)
{
	switch(sig)
	{
	case	SIGTERM:
		ServiceStop(sig);
	break;
	case	SIGUSR1:
		ServiceStatus(sig);
	break;
	case	SIGUSR2:
		ReStartLgwThread();
	break;
	}
}

#ifdef	WITH_GPS
void	GpsPositionUpdated(LGW_COORD_T *loc_from_gps)
{
	t_xlap_link	*lk;
	t_lrr_pkt	uppkt;
	int		i;

	RTL_TRDBG(1,"GPS update position (%f,%f,%d) -> (%f,%f,%d)\n",
			LrrLat,LrrLon,LrrAlt,
			loc_from_gps->lat,loc_from_gps->lon,loc_from_gps->alt);

	memset	(&uppkt,0,sizeof(t_lrr_pkt));
	uppkt.lp_vers	= LP_LRX_VERSION;
	uppkt.lp_flag	= LP_INFRA_PKT_INFRA;
	uppkt.lp_lrrid	= LrrID;
	uppkt.lp_type	= LP_TYPE_LRR_INF_GPSCO;
	uppkt.lp_szh	= LP_PRE_HEADER_PKT_SIZE+sizeof(t_lrr_gpsco);

	uppkt.lp_u.lp_gpsco.li_gps	= 2;
	uppkt.lp_u.lp_gpsco.li_latt	= loc_from_gps->lat;
	uppkt.lp_u.lp_gpsco.li_long	= loc_from_gps->lon;
	uppkt.lp_u.lp_gpsco.li_alti	= loc_from_gps->alt;

	for	(i = 0 ; i < NbLrc ; i++)
	{
		
		lk	= &TbLapLrc[i];
		if	(lk->lk_state == SSP_STARTED)
		{
			LapPutOutQueue(lk,(u_char *)&uppkt,uppkt.lp_szh);
		}
	}
}
#endif

#define	u_int_sizeof(X)	(unsigned int)sizeof((X))
void	StructSize()
{
t_lrr_pkt	pkt;

printf("LP_HEADER_PKT_SIZE_V0=%d old ...\n",LP_HEADER_PKT_SIZE_V0);

printf("LP_IEC104_SIZE_MAX=%d\n",LP_IEC104_SIZE_MAX);
printf("LP_PRE_HEADER_PKT_SIZE=%d\n",LP_PRE_HEADER_PKT_SIZE);
printf("LP_MACLORA_SIZE_MAX=%d (IEC104-PREHEADER-sizeof(lp_u.lp_radio)-10)\n",
				LP_MACLORA_SIZE_MAX);
printf("LP_DATA_CMD_LEN=%d (IEC104-PREHEADER-10)\n",
				LP_DATA_CMD_LEN);

printf("sizeof(t_lrr_pkt.lp_u)=%d\n",u_int_sizeof(pkt.lp_u));
printf("sizeof(t_lrr_pkt.lp_u.lp_radio)=%d\n",u_int_sizeof(pkt.lp_u.lp_radio));
#ifdef	LP_TP31
printf("sizeof(t_lrr_pkt.lp_u.lp_radio_p1)=%d\n",u_int_sizeof(pkt.lp_u.lp_radio_p1));
printf("sizeof(t_lrr_pkt.lp_u.lp_sent_indic)=%d\n",u_int_sizeof(pkt.lp_u.lp_sent_indic));
#endif
printf("sizeof(t_lrr_pkt.lp_u.lp_stat_v1)=%d\n",u_int_sizeof(pkt.lp_u.lp_stat_v1));
printf("sizeof(t_lrr_pkt.lp_u.lp_config)=%d\n",u_int_sizeof(pkt.lp_u.lp_config));
printf("sizeof(t_lrr_pkt.lp_u.lp_config_lrc)=%d\n",u_int_sizeof(pkt.lp_u.lp_config_lrc));
printf("sizeof(t_lrr_pkt.lp_u.lp_wan)=%d\n",u_int_sizeof(pkt.lp_u.lp_wan));
printf("sizeof(t_lrr_pkt.lp_u.lp_wan_lrc)=%d\n",u_int_sizeof(pkt.lp_u.lp_wan_lrc));
printf("sizeof(t_lrr_pkt.lp_u.lp_rfcell)=%d\n",u_int_sizeof(pkt.lp_u.lp_rfcell));

printf("sizeof(t_lrr_pkt.lp_u.lp_shell_cmd)=%d\n",u_int_sizeof(pkt.lp_u.lp_shell_cmd));
printf("sizeof(t_lrr_pkt.lp_u.lp_cmd_resp)=%d\n",u_int_sizeof(pkt.lp_u.lp_cmd_resp));

printf("max network part size of t_lrr_pkt=%d\n",(u_int)offsetof(t_lrr_pkt,lp_payload));
printf("sizeof(t_lrr_pkt)=%d (memory only)\n",u_int_sizeof(pkt));

printf("static data size of t_imsg=%d\n",IMSG_DATA_SZ);
}

int	NtpdStartedGeneric(char *file)
{
	int	ret	= 0;
	FILE	*f;

	if	(!file || !*file)
		return	0;
	f	= fopen(file,"r");
	if	(f)
	{
		pid_t	pid;

		pid	= 0;
		if	(fscanf(f,"%d",&pid) == 1 && pid > 0)
		{
			if	(kill(pid,0) == 0)
				ret	= 1;
		}
		fclose	(f);
	}

	return	ret;
}

#define C_NTP_PORT  123

#define	C_NTP_PAYLOAD_MAX	468U
#define C_NTP_AUTHENTICATOR	96U

#define C_NTP_H1_USED_DATA	0x3FU
#define C_NTP_PROT_VERSION_2	0x10U
#define C_NTP_MODE_CONTROL	0x06U
#define C_NTP_H2_OPCODE		0x02U

#define C_NTP_H2_RESP_MASK	0x80U
#define C_NTP_H2_ERROR_MASK	0x40U
#define C_NTP_H2_MORE_MASK	0x20U

#define C_NTP_H1_VAL		( C_NTP_PROT_VERSION_2 | C_NTP_MODE_CONTROL )
#define C_NTP_H2_VAL		( C_NTP_H2_OPCODE )

#define C_NTP_S1_CLOCK_OFFSET	6
#define C_NTP_S1_SOURCE_MASK	0x3FU
#define C_NTP_S2_STATUS_MASK	0x0FU


int	NtpdRequestService(void)
{
	fd_set			fds;
	struct timeval		tv;
	struct sockaddr_in	sock;
	struct in_addr		address;
	int			s; /* socket fd */
	int			n; /* data lenght */

	/* RFC-1305 NTP control message format */
	struct  {
		unsigned char header1;	/*	Version Number: 3-5	*/
					/*	Mode: bits 0-2	*/
		unsigned char header2;	/*	Response: 7	*/
					/*	Error: 6;	*/
					/*	More: 5;	*/
					/*	OpCode: 0-4	*/
		unsigned short	sequence;
		unsigned char	status1;  /* LI and clock source */
		unsigned char	status2;  /* count and code */
		unsigned short	associationID;
		unsigned short	offset;
		unsigned short	count;
		char		payload[C_NTP_PAYLOAD_MAX];
		char		authenticator[C_NTP_AUTHENTICATOR];
	} ntpMsg;

	char str[C_NTP_PAYLOAD_MAX];

	/* static int ignore_leap_alarm = -1; */

	memset( &ntpMsg, 0, sizeof(ntpMsg) );

	tv.tv_sec = 1;
	tv.tv_usec = 0;

	FD_ZERO(&fds);

	inet_aton(NtpDaemonIp, &address);
	sock.sin_family = AF_INET;
	sock.sin_addr = address;
	sock.sin_port = htons(C_NTP_PORT);

	/* Send request to local ntp service */
	ntpMsg.header1 = C_NTP_H1_VAL;
	ntpMsg.header2 = C_NTP_H2_VAL;
	ntpMsg.sequence = htons(1);

	if ((s = socket (PF_INET, SOCK_DGRAM, 0)) < 0)
	{
		RTL_TRDBG(3, "NTP : can not open socket to NTP daemon\n");
		return 0;
	}

	if (connect(s, (struct sockaddr *)&sock, sizeof(sock)) < 0)
	{
		RTL_TRDBG(3, "NTP : can not connect socket to NTP daemon\n");
		close(s);
		return 0;
	}
  
	FD_SET(s, &fds);

	if (send(s, &ntpMsg, sizeof(ntpMsg), 0) < 0)
	{
		RTL_TRDBG(3, "NTP : can not connect send data to NTP daemon\n");
		close(s);
		return 0;
	}

	/* Read response */
	if (select (s+1, &fds, (fd_set *)0, (fd_set *)0 , &tv) <= 0)
	{
		RTL_TRDBG(3, "NTP : no answer received from NTP daemon\n");
		close(s);
		return 0;
	}

	if ((n = recv (s, &ntpMsg, sizeof(ntpMsg), 0)) < 0)
	{
		RTL_TRDBG(3, "NTP : no answer received from NTP daemon\n");
		close(s);
		return 0;
	}

	RTL_TRDBG(5, "NTP server dump message:\n");
	RTL_TRDBG(5, "   header1:  0x%02x\n", ntpMsg.header1);
	RTL_TRDBG(5, "   header2:  0x%02x\n", ntpMsg.header2);
	RTL_TRDBG(5, "   sequence: 0x%04x\n", ntpMsg.sequence);
	RTL_TRDBG(5, "   status1:  0x%02x\n", ntpMsg.status1);
	RTL_TRDBG(5, "   status2:  0x%02x\n", ntpMsg.status2);
	RTL_TRDBG(5, "   assoc_id: 0x%04x\n", ntpMsg.associationID);
	RTL_TRDBG(5, "   offset:   0x%04x\n", ntpMsg.offset);
	RTL_TRDBG(5, "   count:    0x%04x\n", ntpMsg.count);

	if ( !((ntpMsg.header1 & C_NTP_H1_USED_DATA) == C_NTP_H1_VAL) || !((ntpMsg.header2 & ~C_NTP_H2_MORE_MASK) == (C_NTP_H2_VAL|C_NTP_H2_RESP_MASK)) )
	{
		RTL_TRDBG(3, "NTP : Invalid status word received from NTP daemon (0x%02x%02x)\n", ntpMsg.header1, ntpMsg.header2);
		close(s);
		return 0;
	}

	if (!(ntpMsg.header2 | C_NTP_H2_ERROR_MASK))
	{
		RTL_TRDBG(3, "NTP : error bit set in answer from NTP daemon (0x%02x%02x)\n", ntpMsg.header1, ntpMsg.header2);
		close(s);
		return 0;
	}

	if (!(ntpMsg.header2 | C_NTP_H2_MORE_MASK))
	{
		RTL_TRDBG(3, "NTP : unexpected bit set in answer from NTP daemon (0x%02x%02x)\n", ntpMsg.header1, ntpMsg.header2);
		close(s);
		return 0;
	}

/*

    PT-1803: leap_alarm issue happens on all models. So no more checked: no negative impact detected

 	if ((ntpMsg.status1 >> C_NTP_S1_CLOCK_OFFSET) == 3)
	{
		if (ignore_leap_alarm == -1) {
			ignore_leap_alarm = CfgInt(HtVarLrr, System, -1, "ntpignoreleapalarm", 0);
		}
		if (ignore_leap_alarm) {
			RTL_TRDBG(3, "NTP : warning leap_alarm detected\n");
		} else {
			RTL_TRDBG(3, "NTP : ntp unsynchronized\n");
			close(s);
			return 0;
		}
	}
*/

	if ((ntpMsg.status2 & C_NTP_S2_STATUS_MASK) == 1)
	{
		RTL_TRDBG(3, "NTP : ntp server re-starting\n");
		close(s);
		return 0;
	}

	if ((ntpMsg.status1 & C_NTP_S1_SOURCE_MASK)  == 6) /* Sync to another NTP server, check IP is valid */
	{
		strncpy(str, ntpMsg.payload, sizeof(str));
		if (!strstr (str, "refid="))
		{
			RTL_TRDBG(3, "NTP : ntp sync server IP unknown\n");
			close(s);
			return 0;
		}
	}

	/* check if stratum is properly returned */
	strncpy(str, ntpMsg.payload, sizeof(str));
	if (!strstr (str, "stratum="))
	{
		RTL_TRDBG(3, "NTP : ntp stratum unknown\n");
		close(s);
		return 0;
	}

	/* check if accuracy is properly returned */
	strncpy(str, ntpMsg.payload, sizeof(str));
	if (   ! strstr (str, "rootdisp=")
            || ! strstr (str, "rootdelay=") )
	{
		RTL_TRDBG(3, "NTP : accuracy unknown\n");
		close(s);
		return 0;
	}

	/* check if poll interval is properly returned */
	strncpy(str, ntpMsg.payload, sizeof(str));
	if (!strstr (str, "tc="))
	{
		RTL_TRDBG(3, "NTP : poll interval unknown\n");
		close(s);
		return 0;
	}

	RTL_TRDBG(3, "NTP : synchronized\n");
	close(s);
	return 1;
}

int NtpdStartedCiscoCorsica(char *file)
{

 int ret = 0;
 char * buffer = NULL;
 long length;

 FILE *f;

 if (!file || !*file){
  return 0;
 }
 f = fopen(file,"r");
 if (f){
  fseek (f, 0, SEEK_END);
  length = ftell (f);
  fseek (f, 0, SEEK_SET);
  buffer = (char *)malloc(length * sizeof (char));
  if (buffer) {
   fread(buffer, 1, length, f);
  }
  fclose(f);
 }

 if (buffer) {
  if (strchr(buffer, '*')){
   ret = 1;
  } else {
   ret = 0;
  }
 }

 if (buffer) {
  free(buffer);
 }
 return ret;
}

#if	defined(WITH_USB_PROTECTION)

char	*UsbProtect	= "/etc/usbkey.txt";
char	*UsbProtectBkp	= "/etc/usbkey.txt.bkp";

static	void	UsbProtectBackup()
{
	char	cmd[512];

	snprintf(cmd,sizeof(cmd),"cp %s %s >/dev/null 2>&1 ",UsbProtect,UsbProtectBkp);
	system	(cmd);
}

static	void	UsbProtectRestore()
{
	char	cmd[512];

	snprintf(cmd,sizeof(cmd),"cp %s %s >/dev/null 2>&1",UsbProtectBkp,UsbProtect);
	system	(cmd);
}

static	void	UsbProtectCreate()
{
	FILE	*f;

	f	= fopen(UsbProtect,"w");
	if	(f)
	{
		// TODO use secret here
		fprintf	(f,"usb_auto_protect\n");
		fclose	(f);
	}
}

static	void	UsbProtectInit(int init)
{
	// try to restore file protect if exists
	UsbProtectRestore();
	if	(access(UsbProtect,R_OK) == 0)
	{	// file protect already exists, just backup it
		UsbProtectBackup();
		return;
	}
	// no file to protect automatic update with USB => create one
	UsbProtectCreate();
	UsbProtectBackup();
}

int	UsbProtectOff()
{
	UsbProtectBackup();
	unlink	(UsbProtect);
	return	0;
}

int	UsbProtectOn()
{
	UsbProtectInit(0);
	return	0;
}
#endif

#if defined(SYSTEM_NAME)
#define SYSTEM_DEFINED
void	InitSystem()
{
	System = getenv("SYSTEM");
	if (!System)
	{
		RTL_TRDBG(0,"SYSTEM not defined\n");
		exit(1);
	}
	RootAct = getenv("ROOTACT");
	if (!RootAct)
	{
		RTL_TRDBG(0,"ROOTACT not defined\n");
		exit(1);
	}
#if defined(WITH_USB_PROTECTION)
	UsbProtectInit(1);
#endif
}

int	NtpdStarted()
{
#if !defined(WITH_NTP_FILE)
	NtpDaemonIp = CfgStr(HtVarLrr, "lrr", -1, "ntpdaemonip", "127.0.0.1");
	RTL_TRDBG(0, "NTP daemon IP=%s\n", NtpDaemonIp);
	return NtpdRequestService();
#else
	char *NtpPidFile;

	NtpPidFile = getenv("NTP_PID_FILE");
	if (!NtpPidFile)
	{
		RTL_TRDBG(0,"NTP_PID_FILE not defined\n");
		exit(1);
	}

#if defined(CISCOMS)
	return	NtpdStartedCiscoCorsica(NtpPidFile);
#else
	return	NtpdStartedGeneric(NtpPidFile);
#endif
#endif
}

#else // SYSTEM_NAME

#ifdef NTP_PID_FILE
#undef NTP_PID_FILE
#endif

#ifdef NTP_USE_CISCO_SPECIFIC
#undef NTP_USE_CISCO_SPECIFIC
#endif

#if defined(NATRBPI_USB)
#define	SYSTEM_DEFINED
void	InitSystem()
{
	System	= "natrbpi_usb_v1.0";
}
#endif

#if defined(RBPI_V1_0)
#define	SYSTEM_DEFINED
void	InitSystem()
{
	System	= "rbpi_v1.0";
}
#endif

#if defined(SEMPICO)
#define	SYSTEM_DEFINED
void	InitSystem()
{
	System	= "sempico";
}
#endif

#ifdef	IR910
#define	SYSTEM_DEFINED
void	InitSystem()
{
	System	= "ir910";
}
#endif

#ifdef	CISCOMS
#define	SYSTEM_DEFINED
void	InitSystem()
{
	System	= "ciscoms";
}

#define NTP_USE_CISCO_SPECIFIC
#define NTP_PID_FILE	"/var/run/ntp_status"
#endif

#ifdef  FCMLB
#define	SYSTEM_DEFINED
void    InitSystem()
{
        System  = "fcmlb";
}
#endif

#ifdef	FCPICO
#define	SYSTEM_DEFINED
void    InitSystem()
{
        System  = "fcpico";
}
#endif

#ifdef  FCLAMP
#define	SYSTEM_DEFINED
void    InitSystem()
{
        System  = "fclamp";
}
#endif

#ifdef  FCLOC
#define	SYSTEM_DEFINED
void    InitSystem()
{
        System  = "fcloc";
}
#endif

#ifdef	RFILR
#define	SYSTEM_DEFINED
void	InitSystem()
{
	System	= "rfilr";
}
#endif

#ifdef  OIELEC
#define	SYSTEM_DEFINED
void    InitSystem()
{
        System  = "oielec";
}
#endif

#ifdef	FLEXPICO
#define	SYSTEM_DEFINED
void    InitSystem()
{
        System  = "flexpico";
}
#endif

#ifdef	TRACKNET // TRACKNET: to be removed
#define	SYSTEM_DEFINED
void    InitSystem()
{
        System  = "tracknet";
}
#endif

int     NtpdStarted()
{
#ifdef NTP_USE_CISCO_SPECIFIC
	return	NtpdStartedCiscoCorsica(NTP_PID_FILE);
#else
	NtpDaemonIp = CfgStr(HtVarLrr, "lrr", -1, "ntpdaemonip", "127.0.0.1");
	RTL_TRDBG(0, "NTP daemon IP=%s\n", NtpDaemonIp);
	return	NtpdRequestService();
#endif
}

#endif // SYSTEM_NAME

#ifndef	SYSTEM_DEFINED
void    InitSystem()
{
#warning "you are compiling the LRR for linux 32/64bits generic target system"
#warning "this implies the use of a Semtech Picocell connected with ttyACMx"
        System  = "linux";
}
#endif

void	DoUptime()
{
	FILE	*f;
	float	sec;
	struct	timeval	tv;

	// absolute process uptime ~ now
	clock_gettime(CLOCK_REALTIME,&UptimeProc);
	rtl_getCurrentIso8601date(UptimeProcStr);


	// relative system uptime in sec
	f	= fopen("/proc/uptime","r");
	if	(!f)
		return;

	fscanf	(f,"%f",&sec);

	// as process is newer than system :) ...
	UptimeSyst.tv_sec	= UptimeProc.tv_sec - (time_t)sec;
	UptimeSyst.tv_nsec	= 0;

	tv.tv_sec		= UptimeSyst.tv_sec;
	tv.tv_usec		= 0;
	rtl_gettimeofday_to_iso8601date(&tv,NULL,UptimeSystStr);


	fclose	(f);

}

static	void	StopIfaceDaemon()
{
	char	cmd[1024];

	if	(IfaceDaemon == 0)
	{
		RTL_TRDBG(0,"IfaceDaemon disabled\n");
		return;
	}
	snprintf(cmd,sizeof(cmd),"killall %s 2>&1 > /dev/null &","ifacefailover.sh");
	DoSystemCmdBackGround(cmd);
	snprintf(cmd,sizeof(cmd),"killall %s 2>&1 > /dev/null &","ifacerescuespv.sh");
	DoSystemCmdBackGround(cmd);
	RTL_TRDBG(0,"IfaceDaemon stopped\n");
}

#ifdef	__clang__
static	void	ConfigIfaceDaemon(FILE *f)
{
	RTL_TRDBG(0,"LRR compiled with clang !!!\n");
	abort(); 
}
#else
static	void	ConfigIfaceDaemon(FILE *f)
{
	char	*section	= "ifacefailover.";
	int	lg;
	int	routesfound	= 0;
	int	rescuespvfound	= 0;

	inline	void	CBHtWalk(char *var,void *value)
	{
//		fprintf(out,"'%s' '%s'\n",var,(char *)value);
		if	(strncmp(var,section,lg) == 0)
		{
			char	*pt;
			char	maj[256];
			char	*val;

			val	= (char *)value;
			strcpy	(maj,var+lg);
			pt	= maj;
			while	(*pt)
			{
				*pt	= toupper(*pt);
				pt++;
			}
			if	(*val != '"')
			fprintf	(f,"%s=\"%s\"\n",maj,(char *)val);
			else
			fprintf	(f,"%s=%s\n",maj,(char *)val);
			if	(strcmp(maj,"ROUTES") == 0 && strlen(val))
			{
				routesfound	= 1;
			}
			if	(strcmp(maj,"RESCUESPV") == 0 && strlen(val))
			{
				rescuespvfound	= 1;
			}
		}
	}

	lg	= strlen(section);
	rtl_htblDump(HtVarLrr,CBHtWalk);

	// routes not declared use lrc1 .. lrcN
	if	(routesfound == 0 && NbLrc > 0)
	{
		int	i;

		fprintf	(f,"# no routes declared use lrc list\n");
		fprintf	(f,"ROUTES=\"");
		for	(i = 0 ; i < NbLrc ; i++)
		{
			fprintf	(f,"%s",TbLapLrc[i].lk_addr);
			if	(i != NbLrc - 1)
				fprintf	(f," ");
		}
		fprintf	(f,"\"\n");
	}
	if	(rescuespvfound == 0)
	{
		fprintf	(f,"# key rescuespv not declared => forced\n");
		fprintf	(f,"RESCUESPV=\"1\"\n");
	}
}
#endif

static	void	StartIfaceDaemon()
{
	char	exe[1024];
	char	cmd[1024];
	char	trace[1024];
	char	config[1024];
	FILE	*f;

	if	(IfaceDaemon == 0)
	{
		RTL_TRDBG(0,"IfaceDaemon disabled\n");
		return;
	}

	snprintf(exe,sizeof(exe),"%s/lrr/com/shells/%s/",RootAct,System);
	strncat	(exe,"ifacefailover.sh",sizeof(exe)-strlen(exe)-1);
	if	(access(exe,X_OK) != 0)
	{
		RTL_TRDBG(0,"IfaceDaemon not provided on '%s'\n",System);
		return;
	}
	snprintf(config,sizeof(config),"%s/usr/data/ifacefailover.cfg",RootAct);
	f	= fopen(config,"w");
	if	(f)
	{
		ConfigIfaceDaemon(f);
		fclose(f);
	}

	snprintf(trace,sizeof(trace),"%s/var/log/lrr/IFACEDAEMON.log",RootAct);
	snprintf(cmd,sizeof(cmd),"%s 2>&1 > %s &",exe,trace);
	DoSystemCmdBackGround(cmd);
	RTL_TRDBG(0,"IfaceDaemon started\n");
}

static	void	ReadWifiState()
{
	FILE	*f;
	char	state[256], buf[128], *pt;
	int	i, idx, tot;
	// states:
	// 0: up and used
	// 1: up and ready for backup
	// 2: service down
	// 3: no ip
	// 4: link down
	// 5: no used whatever the state
	int	itfstate=5;

	snprintf(state,sizeof(state),"%s/" WIFISTATEFILE, RootAct);
	f = fopen(state, "r");
	if (!f)
	{
		RTL_TRDBG(0,"ReadWifiState: can't read wifi state (file='%s')\n", state);
		return;
	}
	while (fgets(buf, sizeof(buf), f))
	{
		if ((pt = strstr(buf, "SSID=")))
		{
			pt += 5;
			strncpy(WifiStateSsid, pt, sizeof(WifiStateSsid)-1);
			WifiStateSsid[sizeof(WifiStateSsid)-1] = '\0';
			if ((pt = strchr(WifiStateSsid, '\n')))
				*pt = '\0';
			if ((pt = strchr(WifiStateSsid, '\r')))
				*pt = '\0';
			RTL_TRDBG(4,"ReadWifiState: SSID='%s'\n", WifiStateSsid);
		}
		else if ((pt = strstr(buf, "RSSI=")))
		{
			itfstate = 1;
			pt += 5;
			WSSample[WSSampleIdx] = atoi(pt);
			RTL_TRDBG(4,"ReadWifiState: add RSSI WSSample[%d]=%f\n",
					WSSampleIdx, WSSample[WSSampleIdx]);
			WSSampleIdx = (WSSampleIdx + 1) % WSSampleSize;
			WSSampleCount = WSSampleCount>=WSSampleSize ? WSSampleCount : WSSampleCount+1;
			WifiStateRssiCount++;
		}
		else if ((pt = strstr(buf, "STATE=")))	// unknown, connected, disconnected
		{
			pt += 6;
			if (!strncmp(pt, "unknown", 7))
				itfstate = 2;
			else if (!strncmp(pt, "disconnected", 12))
				itfstate = 2;
			else if (!strncmp(pt, "down", 4))
				itfstate = 5;
			else if (!strncmp(pt, "noip", 4))
				itfstate = 4;
			else if (!strncmp(pt, "pingitfail", 10))
				itfstate = 3;
		}
	}
	fclose(f);
	WifiStateItf = itfstate;

	// Calculate average value
	tot = 0;
	for (i=0; i<WSSampleCount; i++)
	{
		// starts with last value stored
		idx = (WSSampleIdx - 1 - i + WSSampleSize) %WSSampleSize;
		tot += WSSample[idx];
	}
	if (WSSampleCount > 0)
		WifiStateRssi = (float) tot / WSSampleCount;
	else
		WifiStateRssi = 0;
	RTL_TRDBG(1,"WifiState: average RSSI=%f\n", WifiStateRssi);
}

static	void	StopWifiStateReport()
{
	char	cmd[1024];

	if	(WifiStateReport == 0)
	{
		RTL_TRDBG(0,"WifiStateReport disabled\n");
		return;
	}
	snprintf(cmd,sizeof(cmd),"killall %s 2>&1 > /dev/null &", WIFISTATESH);
	DoSystemCmdBackGround(cmd);
	if (WSSample)
	{
		free(WSSample);
		WSSample = NULL;
	}
	RTL_TRDBG(0,"WifiStateReport stopped\n");
}

static	void	StartWifiStateReport()
{
	char	exe[256];
	char	cmd[1024];
	char	trace[256];
	char	state[256];

	if	(WifiStateReport == 0)
	{
		RTL_TRDBG(0,"WifiStateReport disabled\n");
		return;
	}
	else if (!IsTypeInterface(ITF_TYPE_WIFI))
	{
		WifiStateReport = 0;
		RTL_TRDBG(0,"no wifi interface enabled, WifiStateReport disabled\n");
		return;
	}

	if (WifiStateReport > 0 && WifiStateReport < 10)
	{
		RTL_TRDBG(0,"Warning: wifistatereport=%d is too small => increased to 10\n", WifiStateReport);
		WifiStateReport = 10;
	}

	if (WSSample == NULL)
	{
		WSSampleCount = 0;
		WSSampleIdx = 0;
		WSSample = (float *) malloc(WSSampleSize * sizeof(float));
		if (!WSSample)
		{
				RTL_TRDBG(0, "ERROR: failed to allocate size for %d samples !\n", WSSampleSize);
				return;
		}
	}

	snprintf(exe,sizeof(exe),"%s/%s/%s", RootAct,  WIFISTATESHDIR, WIFISTATESH);
	snprintf(trace,sizeof(trace),"%s/var/log/lrr/statewifi.log",RootAct);
	snprintf(state,sizeof(state),"%s/" WIFISTATEFILE, RootAct);
	snprintf(cmd,sizeof(cmd),"%s %d > %s 2> %s &", exe, WifiStateReport, state, trace);
	DoSystemCmdBackGround(cmd);
	RTL_TRDBG(0,"WifiStateReport started\n");
}

static	void	ServiceExit()
{
	StopIfaceDaemon();
	StopWifiStateReport();
#if defined(KEROS) && defined(WITH_LED_MGT)
	LedBackhaul(0);
#endif
	SaveStoreQ(1);
	unlink	(DoFilePid());
	RTL_TRDBG(0,"service stopped\n");
}

static	void	DoSigAction(int nointr)
{
	struct sigaction sigact;

	sigemptyset(&sigact.sa_mask);
	sigact.sa_flags = 0;
	sigact.sa_handler = SigHandler;
	sigaction(SIGTERM,&sigact,NULL);
	sigaction(SIGUSR1,&sigact,NULL);
	sigaction(SIGUSR2,&sigact,NULL);

	sigact.sa_handler = SIG_IGN;
	sigaction(SIGINT,&sigact,NULL);
	sigaction(SIGQUIT,&sigact,NULL);
	sigaction(SIGHUP,&sigact,NULL);
	sigaction(SIGPIPE,&sigact,NULL);

	sigact.sa_handler = SIG_DFL;
	sigaction(SIGCHLD,&sigact,NULL);
	if	(nointr)
	sigaction(SIGINT,&sigact,NULL);
}

static	void	DoLap()
{
	int	i;
#ifdef	LP_TP31
	LapEnableLongMode();
#endif
	LapList		= LapInit(0,MainTbPoll);
	for	(i = 0 ; i < NbLrc ; i++)
	{
		TbLapLrc[i].lk_cbEvent	= (void *)LapEventProceed;
		TbLapLrc[i].lk_userptr	= &TbLrc[i];
		LapAddLink(&TbLapLrc[i]);

		TbLrc[i].lrc_lk		= &TbLapLrc[i];
		TbLrc[i].lrc_idx	= i;

		RTL_TRDBG(1,"idxlrc=%d lk=%p lrc=%p avdv=%p\n",i,
			&TbLapLrc[i],&TbLrc[i],&TbLrc[i].lrc_avdvtrip);
	}

	LapBindAll();
	LapConnectAll();
}

static	void	DoLapTest()
{
	int	i;
#ifdef	LP_TP31
	LapEnableLongMode();
#endif
	LapList		= LapInit(0,MainTbPoll);
	for	(i = 0 ; i < NbLrc ; i++)
	{
		TbLapLrc[i].lk_cbEvent	= (void *)LapEventProceedTest;
		TbLapLrc[i].lk_userptr	= &TbLrc[i];
		TbLapLrc[i].lk_t3	= 3;
		LapAddLink(&TbLapLrc[i]);

		TbLrc[i].lrc_lk		= &TbLapLrc[i];

		RTL_TRDBG(1,"idxlrc=%d lk=%p lrc=%p avdv=%p\n",i,
			&TbLapLrc[i],&TbLrc[i],&TbLrc[i].lrc_avdvtrip);
	}

	LapBindAll();
	LapConnectAll();
}

#ifdef	__clang__
void	DecryptHtbl(void *htbl) 
{ 
	RTL_TRDBG(0,"LRR compiled with clang !!!\n");
	abort(); 
}
#else
void	DecryptHtbl(void *htbl)
{
	inline	int	CBHtWalk(void *htbl,char *pvar,void *pvalue,void *p)
	{
		char	*value	= (char *)pvalue;
		char	var[512];
		char	varext[512];
		char	plaintext[1024];
		char	*pt;
		int	keynum;
		int	ret;
		int	lg;
		u_char	*keyvalhex;

		if	(!value)
			return	0;
		strcpy	(var,pvar);
		snprintf(varext,sizeof(varext),"%s_crypted_k",var);
		pt	= rtl_htblGet(htbl,varext);
		if	(!pt || !*pt)
			return	0;
		// this variable is declared as crypted 
		lg	= strlen(value);
		if	(lg == 0 || *value != '[' || value[lg-1] != ']')
		{
			// but the crypted value is "" or not between [] 
			// => considered as not crypted
RTL_TRDBG(1,"'%s' declared as _crypted_k=%s but empty or not [...]\n",
				var,pt);
			rtl_htblRemove(htbl,varext);
			return	0;
		}
		value[lg-1]	= '\0';
		value++;
		keynum		= atoi(pt);
#ifdef NOSSL
		keyvalhex	= NULL;
#else
		keyvalhex	= BuildHex(keynum);
#endif
		if	(!keyvalhex)
			return	0;

RTL_TRDBG(1,"decrypt '%s' key=%d '%s'\n",var,keynum,value);
		plaintext[0]	= '\0';
#ifdef NOSSL
		ret = -1;
#else
		ret	= lrr_keyDec((u_char *)value,-1,
				keyvalhex,NULL,(u_char *)plaintext,1000,0);
#endif
		if	(ret >= 0)	// "" is a possible decrypted value
		{
			plaintext[ret]	= '\0';
#if	0
RTL_TRDBG(1,"decrypt'%s' ret=%d '%s'\n",var,ret,plaintext);
#endif
			rtl_htblRemove(htbl,varext);
			rtl_htblRemove(htbl,var);
			rtl_htblInsert(htbl,var,(void *)strdup(plaintext));
		}
		else
		{
RTL_TRDBG(1,"decrypt '%s' ret=%d ERROR\n",var,ret);
		}

		free(keyvalhex);
		return	0;
	}

	rtl_htblWalk(htbl,CBHtWalk,NULL);
}
#endif

void GetConfigFromBootSrv()
{
	char	buf[256];
	int	ret = 1;
	char	*addr[2] = { 0 }, *port[2] = { 0 }, *user[2] = { 0 }, *pwd[2] = { 0 };
	int i = 0;
	int current_BS = 0;
	int nb_BS = 0;

	LrrIDGetFromBS = 1;

	/* Retrieve Configurations */
	for (i = 0; i < 2; i++) {
		char *ad, *po, *us, *pw;
		int ind = nb_BS;

		ad = CfgStr(HtVarLrr, "bootserver", i, "addr", NULL);
		po = CfgStr(HtVarLrr, "bootserver", i, "port", NULL);
		us = CfgStr(HtVarLrr, "bootserver", i, "user", NULL);
		pw = CfgStr(HtVarLrr, "bootserver", i, "pass", NULL);

		if (ad != NULL) {
			addr[ind] = ad;
			port[ind] = po;
			user[ind] = us;
			pwd[ind]  = pw;
			nb_BS++;
		}
	}

	if (nb_BS == 0) {
		RTL_TRDBG(0,"No bootserver configuration ...\n");
		/* NFO : Do something smarter Here */
		while(1)
			sleep(30);
	}

	/* Alternate through available configurations ... */
	/*   pause if all failed and try again            */ 
	while (ret)
	{
		int ind = current_BS;
		current_BS = (current_BS + 1) % nb_BS;

		// execute shell to get config from bootserver
		RTL_TRDBG(0,"Get config from bootserver '%s:%s'...\n", addr[ind], port[ind]);

		snprintf(buf,sizeof(buf),"$ROOTACT/lrr/com/cmd_shells/treatbootsrv.sh -A %s -P %s -U %s -W %s",
				addr[ind], port[ind], user[ind], pwd[ind]);
		ret = system(buf);
		if (ret)
		{
			RTL_TRDBG(0,"Getting config from bootserver failed, retry in 30 seconds\n");
			if (current_BS == 0) {
				sleep(30);
			}
		}
	}
}

// RDTP-1910
void	SupLogSetPermission()
{
	char	buf[256];
	int	ret;

	buf[0]	= '\0';
	snprintf(buf,sizeof(buf),"chmod u+s $ROOTACT/lrr/suplog/suplog.x");
	ret = system(buf);
	if (ret)
	{
		RTL_TRDBG(0,"cannot chmod u+s for suplog.x\n");
	}
}

int	main(int argc,char *argv[])
{
	int	i;
	char	*pt = lrr_whatStr+18;	// TODO
	char	* lrr_base_version = NULL;
	char	deftrdir[256];

	MainThreadId	= pthread_self();

	InitSystem();

	if	(!System)
		System	= "generic";
	if	(!RootAct)
		RootAct	= getenv("ROOTACT");
	if	(!RootAct)
	{
		printf	("$ROOTACT not set\n");
		setenv	("ROOTACT",strdup("/home/actility"),1);
		RootAct	= getenv("ROOTACT");
	}

	clock_gettime(CLOCK_REALTIME,&Currtime);

	sscanf(pt,"%d.%d.%d_%d",&VersionMaj,&VersionMin,&VersionRel,&VersionFix);

	lrr_base_version = calloc(128, sizeof (char));
	snprintf(lrr_base_version,128,"%d.%d.%d",VersionMaj, VersionMin, VersionRel);
	setenv("LRR_VERSION", strdup(lrr_base_version), 1);
	free(lrr_base_version);
	lrr_base_version = calloc(128, sizeof (char));
	snprintf(lrr_base_version,128,"%d",VersionFix);
	setenv("LRR_FIXLEVEL", strdup(lrr_base_version), 1);
	free(lrr_base_version);

#ifdef _HALV_COMPAT
	setenv("HAL_VERSION", strdup(hal_version), 1);
#endif

	LP_VALID_SIZE_PKT();
	if	(argc == 2 && strcmp(argv[1],"--size") == 0)
	{
		StructSize();
		exit(0);
	}
	if	(argc == 2 && (strcmp(argv[1],"--version") == 0
			|| strcmp(argv[1],"--versions") == 0))
	{
		printf	("%s\n",rtl_version());
		printf	("%s\n",lrr_whatStr);
		printf	("%s\n",LgwVersionInfo());
#ifdef _HALV_COMPAT
		printf	("HAL %s\n",hal_version);
#endif
#ifdef	WITH_GPS
		printf	("gps=yes\n");
#endif
#ifdef	WITH_USB
		printf	("usb=yes\n");
#endif
#ifdef	WITH_SPI
		printf	("spi=yes\n");
#endif
#ifdef	WITH_TTY
		printf	("tty=yes\n");
#endif
#ifdef	WITH_SX1301_X1
		printf	("x1=yes\n");
#endif
#ifdef	WITH_SX1301_X8
		printf	("x8=yes\n");
#endif
		exit(0);
	}

	if	(argc == 2 && strcmp(argv[1],"--stpicoid") == 0)
	{
#ifdef	WITH_TTY
		uint8_t uid[8];  //unique id
		if	(lgw_connect(TtyDevice) == LGW_REG_SUCCESS)
		{
                	lgw_mcu_get_unique_id(&uid[0]);
			printf("%.2x%.2x%.2x%.2x%.2x%.2x%.2x%.2x\n",
				uid[0], uid[1], uid[2], uid[3], 
				uid[4], uid[5], uid[6], uid[7]);
			exit(0);
		}
		fprintf(stderr,"--stpicoid chipid error on tty link %s\n",
			TtyDevice);
		exit(1);
#endif
		fprintf(stderr,"--stpicoid not supported\n");
		exit(1);
	}
	if	(argc == 2 && strcmp(argv[1],"--avdv") == 0)
	{
		t_avdv_ui	avdvTrip;
		time_t		now;
		int		nb,n;
		int		t0,t1;

		time	(&now);
		for	(i = 0 ; i < AVDV_NBELEM ; i++)
		{
			AvDvUiAdd(&avdvTrip,360+(rand()%50),now+i);
		}
		t0	= rtl_tmmsmono();
		for	(n = 0 ; n < 100 ; n++)
			nb = AvDvUiCompute(&avdvTrip,now+AVDV_NBELEM,now+i);
		t1	= rtl_tmmsmono();
		printf	("nb=%d tms=%f av=%u dv=%u mx=%u\n",nb,
			(float)(t1-t0)/(float)nb,
			avdvTrip.ad_aver,avdvTrip.ad_sdev,avdvTrip.ad_vmax);
		exit(0);
	}

	if	(argc == 2 && strcmp(argv[1],"--cpu") == 0)
	{
		u_int	nb	= 0;
		while	(1)
		{
			double	used;
			used	= CompAllCpuInfos('S');
			printf	("cpuused=%f\n",used);
			if	(nb && nb%30==0)
			{
				CompAllCpuInfos('L');
				AvDvUiCompute(&CpuAvdvUsed,30,Currtime.tv_sec);
				printf("cpu av=%u dv=%u vm=%u\n",
					CpuAvdvUsed.ad_aver,
					CpuAvdvUsed.ad_sdev,
					CpuAvdvUsed.ad_vmax);
				printf("l1=%f l5=%f l15=%f\n",
					(float)CpuLoad1/100.0,
					(float)CpuLoad5/100.0,
					(float)CpuLoad15/100.0);
			}
			Currtime.tv_sec++;
			nb++;
			sleep(1);
		}
		exit(0);
	}
	if	(argc == 2 && strcmp(argv[1],"--mem") == 0)
	{
		while	(1)
		{
			CompAllMemInfos('S');
			printf	("MemTotal :%u ",MemTotal);
			printf	("MemFree :%u ",MemFree);
			printf	("Buffers :%u ",MemBuffers);
			printf	("Cached :%u ",MemCached);
			printf	("Used :%u\n",MemUsed);
			sleep(5);
		}
		exit(0);
	}

	DoUptime();


	snprintf(ConfigDefault,sizeof(ConfigDefault),"%s/%s",RootAct,"lrr/config");
	snprintf(ConfigCustom,sizeof(ConfigCustom),"%s/%s",RootAct,"usr/etc/lrr");
	rtl_mkdirp(ConfigCustom);
	snprintf(DirCommands,sizeof(DirCommands),"%s/%s",RootAct,"usr/data/lrr/cmd_shells");
	rtl_mkdirp(DirCommands);

	printf	("$ROOTACT %s\n",RootAct);
	printf	("ConfigDefault '%s'\n",ConfigDefault);
	printf	("ConfigCustom '%s'\n",ConfigCustom);

	rtl_init();

	HtVarLrr	= rtl_htblCreateSpec(25,NULL,
						HTBL_KEY_STRING|HTBL_FREE_DATA);

	HtVarLgw	= rtl_htblCreateSpec(25,NULL,
						HTBL_KEY_STRING|HTBL_FREE_DATA);

	HtVarSys	= rtl_htblCreateSpec(25,NULL,
						HTBL_KEY_STRING|HTBL_FREE_DATA);

	if	(!HtVarLrr || !HtVarLgw || !HtVarSys)
	{
		RTL_TRDBG(0,"cannot alloc internal resources (htables)\n");
		exit(1);
	}


	if	(argc == 2 && strcmp(argv[1],"--config") == 0)
	{
		TraceDebug	= 0;
		TraceLevel	= 3;
		rtl_tracelevel(TraceLevel);
		LoadConfigDefine(0,1);
		LoadConfigBootSrv(1);
		LoadConfigLrr(0,1);
		LoadConfigLgw(0);
		LoadConfigChannel(0);
		LgwConfigure(0,0);
		ChannelConfigure(0,0);
		exit(0);
	}

	if	(argc == 2 && strcmp(argv[1],"--ntp") == 0)
	{
		int	ret;

		TraceDebug	= 0;
		TraceLevel	= 0;
		rtl_tracelevel(TraceLevel);
		LoadConfigDefine(0,1);
		LoadConfigBootSrv(1);
		LoadConfigLrr(0,1);
		LoadConfigLgw(0);
		LoadConfigChannel(0);
		LgwConfigure(0,0);
		ChannelConfigure(0,0);

		TraceLevel	= 5;
		rtl_tracelevel(TraceLevel);
		ret	= NtpdStarted();
#if !defined(WITH_NTP_FILE)
		printf("ntp check by request ret=%d\n",ret);
		exit(0);
#else
		printf("ntp check by pid file ret=%d\n",ret);
		exit(0);
#endif
	}
#ifndef	__clang__
	if	(argc == 2 && strcmp(argv[1],"--ini") == 0)
	{
		int		i;
		int		nbv	= 0;
		t_ini_var	tbvar[1024*4];

		inline	int	compar(const void *arg1, const void *arg2)
		{
			t_ini_var	*v1	= (t_ini_var *)arg1;
			t_ini_var	*v2	= (t_ini_var *)arg2;

			return	strcmp(v1->in_name,v2->in_name);
		}

		inline	void	CBHtDumpCli(char *var,void *value)
		{
//			printf("'%s' '%s'\n",var,(char *)value);
			if	(nbv < sizeof(tbvar)/sizeof(t_ini_var))
			{
				tbvar[nbv].in_name	= var;
				tbvar[nbv].in_val	= value;
				nbv++;
			}
		}
		TraceDebug	= 0;
		TraceLevel	= 0;
		rtl_tracelevel(TraceLevel);
		LoadConfigDefine(0,1);
		LoadConfigBootSrv(1);
		LoadConfigLrr(0,1);
		LoadConfigLgw(0);
		LoadConfigChannel(0);

		nbv	= 0;
		rtl_htblDump(HtVarSys,CBHtDumpCli);
		qsort(tbvar,nbv,sizeof(t_ini_var),compar);
		for	(i = 0 ; i < nbv ; i++)
			printf("'%s' '%s'\n",tbvar[i].in_name,
							tbvar[i].in_val);
		nbv	= 0;
		rtl_htblDump(HtVarLrr,CBHtDumpCli);
		qsort(tbvar,nbv,sizeof(t_ini_var),compar);
		for	(i = 0 ; i < nbv ; i++)
			printf("'%s' '%s'\n",tbvar[i].in_name,
							tbvar[i].in_val);
		nbv	= 0;
		rtl_htblDump(HtVarLgw,CBHtDumpCli);
		qsort(tbvar,nbv,sizeof(t_ini_var),compar);
		for	(i = 0 ; i < nbv ; i++)
			printf("'%s' '%s'\n",tbvar[i].in_name,
							tbvar[i].in_val);


		exit(0);
	}
#else
	RTL_TRDBG(0,"LRR compiled with clang !!!\n");
#endif

	if	(argc == 2 && strcmp(argv[1],"--rtt") == 0)
	{
		TraceDebug	= 0;
		TraceLevel	= 3;
		rtl_tracelevel(TraceLevel);
		LoadConfigDefine(0,1);
		LoadConfigLrr(0,1);
		StartItfThread();
		while	(1)
		{
			clock_gettime(CLOCK_REALTIME,&Currtime);
			sleep(1);
		}
		exit(0);
	}

	if	(argc == 2 && strcmp(argv[1],"--itf") == 0)
	{
		int	nb	= 0;

		TraceDebug	= 0;
		TraceLevel	= 3;
		rtl_tracelevel(TraceLevel);
		LoadConfigDefine(0,1);
		LoadConfigLrr(0,1);
		while	(1)
		{
			CompAllItfInfos('S');
			if	(nb && nb%30==0)
			{
				CompAllItfInfos('L');
			}
			sleep(1);
			nb++;
		}
		exit(0);
	}

	if	(argc == 2 && strcmp(argv[1],"--mfs") == 0)
	{
		int	nb	= 0;

		TraceDebug	= 0;
		TraceLevel	= 3;
		rtl_tracelevel(TraceLevel);
		LoadConfigDefine(0,1);
		LoadConfigLrr(0,1);
		while	(1)
		{
//			CompAllMfsInfos('S');
			if	(nb && nb%30==0)
			{
				CompAllMfsInfos('L');
			}
			sleep(1);
			nb++;
		}
		exit(0);
	}
	if	(argc == 2 
		&& (!strcmp(argv[1],"--xlap") || !strcmp(argv[1],"--lap")))
	{
		LapTest		= 1;
		TraceDebug	= 0;
		TraceLevel	= 0;
		if	(!strcmp(argv[1],"--xlap"))
		{
			LapTest		= 2;
			TraceLevel	= 1;
		}
		rtl_tracelevel(TraceLevel);
		LoadConfigDefine(0,0);
		LoadConfigLrr(0,0);
		DoSigAction(1);
		MainTbPoll	= rtl_pollInit();
		MainQ		= rtl_imsgInitIq();
		LgwQ		= rtl_imsgInitIq();
#ifdef REF_DESIGN_V2
		for (i=0; i<SX1301AR_MAX_BOARD_NB; i++)
#else
		for (i=0; i<LGW_MAX_BOARD; i++)
#endif
		{
			LgwSendQ[i]	= rtl_imsgInitIq();
			if	(!LgwSendQ[i])
			{
				RTL_TRDBG(0,"cannot alloc internal resources (queues)\n");
				exit(1);
			}
		}
		if	(!MainTbPoll || !MainQ || !LgwQ)
		{
			RTL_TRDBG(0,"cannot alloc internal resources (queues)\n");
			exit(1);
		}

		LgwThreadStopped= 1;
		DoLapTest();
		MainLoop();
		exit(0);
	}

	// Activate traces before reading tracelevel in config to see startup messages
	rtl_tracemutex();
	rtl_tracelevel(1);
	rtl_tracesizemax(TraceSize);
	snprintf(deftrdir,sizeof(deftrdir)-1,"%s/%s",RootAct,"var/log/lrr/TRACE.log");
	deftrdir[sizeof(deftrdir)-1] = '\0';
        rtl_tracerotate(deftrdir);
	RTL_TRDBG(0,"Traces activated for startup\n");

	if	(access(DoFilePid(),R_OK) == 0)
	{
		SickRestart	= 1;
	}

	atexit	(ServiceExit);
	ServiceWritePid();

	DoSigAction(0);

#if defined(KEROS) && defined(WITH_LED_MGT)
	LedConfigure();
#endif

	SupLogSetPermission();

	// if bootserver.ini found, activate NFR997
	if (LoadConfigBootSrv(0))
	{
		LoadConfigDefine(0,0);
		GetConfigFromBootSrv();
	}

	MainTbPoll	= rtl_pollInit();
	MainQ		= rtl_imsgInitIq();
	LgwQ		= rtl_imsgInitIq();
	StoreQ		= rtl_imsgInitIq();
#ifdef REF_DESIGN_V2
	for (i=0; i<SX1301AR_MAX_BOARD_NB; i++)
#else
	for (i=0; i<LGW_MAX_BOARD; i++)
#endif
	{
		LgwSendQ[i]	= rtl_imsgInitIq();
		if	(!LgwSendQ[i])
		{
			RTL_TRDBG(0,"cannot alloc internal resources (queues)\n");
			exit(1);
		}
	}
	if	(!MainTbPoll || !MainQ || !LgwQ || !StoreQ)
	{
		RTL_TRDBG(0,"cannot alloc internal resources (queues)\n");
		exit(1);
	}

	LoadConfigDefine(0,0);
	LoadConfigLrr(0,0);
	rtl_tracemutex();
	rtl_tracelevel(TraceLevel);
        if      (strcmp(TraceFile,"stderr") != 0)
        {
		rtl_tracesizemax(TraceSize);
		rtl_tracecommitsizemax();
                rtl_tracerotate(TraceFile);
        }
#if	0
	DecryptHtbl(HtVarLrr);
#endif

	RTL_TRDBG(0,"start lrr.x/main th=%lx pid=%d sickrestart=%d\n",
			(long)pthread_self(),getpid(),SickRestart);
	RTL_TRDBG(0,"%s\n",lrr_whatStr);
	RTL_TRDBG(0,"%s\n",LgwVersionInfo());
#ifdef _HALV_COMPAT
	RTL_TRDBG(0,"HAL %s\n",hal_version);
#endif
	RTL_TRDBG(0,"lrrid=%08x lrridext=%04x lrridpref=%02x\n",
		LrrID,LrrIDExt,LrrIDPref);

	if (TraceStdout && *TraceStdout)
	{
		if (!freopen(TraceStdout, "w", stdout))
		{
			RTL_TRDBG(0,"stdout can't be redirected to '%s' !\n", TraceStdout);
		}
		else
		{
			RTL_TRDBG(0,"stdout redirected to '%s'\n", TraceStdout);
		}
	}
	AddGlobalScopeIpv6();

	LoadConfigLgw(0);
	LoadConfigChannel(0);
	// very special case , because channels.ini needs info from lgw.ini
	LgwGenConfigure(0,1);
	ChannelConfigure(0,1);

	StartIfaceDaemon();
	StartWifiStateReport();
#ifdef	WITH_GPS
	if (UseGps)
		StartGpsThread();
#else
	UseGps		= 0;
	UseGpsTime	= 0;
	UseGpsPosition	= 0;
#endif
	StartLgwThread();
	StartCmdThread();
	StartItfThread();

	// RDTP-2420
	if (CellStateReport && IsTypeInterface(ITF_TYPE_CELL))
		StartCellStateThread();

#if	0
	LapSizeFrame(2,256);
#endif

	// RDTP-2413
	if	(NwkfEnable)
		NfInit();

	LoadStoreQ();

	DoLap();
	RTL_TRDBG(0,"TRACE LEVEL IS SET TO %d\n", TraceLevel);
	MainLoop();
	exit(0);
}
