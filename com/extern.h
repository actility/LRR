
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


/* use xstr("MACRO") to get value of the MACRO defined with -DMACRO=<val> */
#define xstr(s) str(s)
#define str(s) #s

#include "system_api.h"


#ifdef	WITH_TRACK_TIC
extern	u_int	TrackTicNb;
extern	u_int	TrackTicCnt;
extern	u_int	TrackTic[][2];
#endif

extern	u_int	VersionMaj;
extern	u_int	VersionMin;
extern	u_int	VersionRel;

extern	int	Sx13xxPolling;
extern	int	Sx13xxStartDelay;

/*
    determine max number of possible radio boards and chips
    - 1 by default on V1.x except if WITH_MULTI_BOARD is present
    - SX1301AT_MAX_BOARD on V2.x
*/

#define USE_SPI_ARRAY   1
#if defined(REF_DESIGN_V1)
/* Ref design v1.x */
#if defined(WITH_MULTI_BOARD)
#define MAX_BOARD               2
#else
#define MAX_BOARD               1
#undef USE_SPI_ARRAY
#endif /* WITH_MULTI_BOARD */
#define MAX_CHIPS_PER_BOARD     2
#else
/* Ref design V2.x */
#define MAX_BOARD               SX1301AR_MAX_BOARD_NB
#define MAX_CHIPS_PER_BOARD     SX1301AR_BOARD_CHIPS_NB
#endif

#if defined(USE_SPI_ARRAY)
extern	char	*SpiDevice[MAX_BOARD];
#else
extern  char    *SpiDevice;
#endif

#ifdef	WITH_TTY
extern	char	*TtyDevice;
#endif
extern	int	CfgRadioStop;
extern	int	CfgRadioDnStop;
extern	int	DownRadioStop;
extern	int	MainWantStop;
extern	int	MainStopDelay;
extern int	SendRfCellNow;

extern	float	AntennaGain[];
extern	float	CableLoss[];
extern	char	*RfRegionId;
extern	u_int	RfRegionIdVers;

extern	char	DirCommands[];

#ifdef REF_DESIGN_V2
extern	sx1301ar_tref_t Gps_time_ref[SX1301AR_MAX_BOARD_NB];
#else
extern	struct tref 	Gps_time_ref[MAX_BOARD];
#endif
extern	int			Gps_ref_valid; 
extern	struct timespec		LgwCurrUtcTime;
extern	struct timespec		LgwBeaconUtcTime;
extern	struct timespec		LgwClassCUtcTime;
extern	time_t			LgwTmmsUtcTime;

extern	struct	timespec	Currtime;

extern	t_wan_itf		TbItf[NB_ITF_PER_LRR];
extern	int			NbItf;

extern	t_lrc_link		TbLrc[NB_LRC_PER_LRR];
extern	u_int			NbLrc;

extern	char			UptimeSystStr[];
extern	char			UptimeProcStr[];


extern	char	*RootAct;
extern	char	*System;
extern	char	*IsmBand;
extern	char	*IsmBandAlter;
extern	int	IsmFreq;
extern	int	IsmAsymDownlink;
extern	int	IsmAsymDnModulo;
extern	int	IsmAsymDnOffset;

extern	t_lrr_config	ConfigIpInt;
extern	char	*ExtConfigRefresh;

extern	int	LgwLinkUp;
extern	int	MacWithBeacon;
extern	int	ServiceStopped;
extern	int	LgwThreadStopped;
extern	int	GpsThreadStopped;
extern	char    *TraceFile;
extern	int     TraceSize;
extern	int     TraceLevel;
extern	int     TraceDebug;
extern	void	*HtVarLrr;
extern	void	*HtVarLgw;
extern	void	*HtVarSys;
extern	void	*MainQ;
extern	void	*LgwQ;
extern	void	*LgwSendQ[];
extern	void	*MainTbPoll;

extern	int	PingRttPeriod;

extern	unsigned	int	UseLgwTime;
extern	int			AdjustDelay;
extern	unsigned	int	LrrID;
extern	unsigned	short	AutoRevSshPort;

extern int      GpsFd;
extern char   * GpsDevice;
extern u_int    UseGps;
extern u_int    UseGpsPosition;
extern u_int    UseGpsTime;
extern u_int    GpsUpdateCnt;
extern u_int    GpsMaxBadChecksum;
extern u_int    GpsMaxBadRead;
extern float    GpsLatt;
extern float    GpsLong;
extern int      GpsAlti;
extern int      GpsPositionOk;
extern float    LrrLat;
extern float    LrrLon;
extern int      LrrAlt;
extern u_char	GpsUtcLeap;
extern u_char	GpsStatus;

#ifndef CELLULAR_C
extern t_cell_infos	*CellState;
extern int		CellStateReport;
extern char		*CellStateDev;
extern int		CellStateSampleSize;
extern int		CellStateSampleTimeout;
#endif
extern char		WifiStateSsid[];

extern int		AdjustFineTimeStamp;

extern	unsigned	int	LrcAvPktTrip;
extern	unsigned	int	LrcMxPktTrip;
extern	unsigned 	int	LrxAvPktProcess;
extern	unsigned 	int	LrxMxPktProcess;

extern	unsigned	int	LgwNbPacketSend;
extern	unsigned	int	LgwNbPacketWait;
extern	unsigned	int	LgwNbPacketLoop;
extern	unsigned	int	LgwNbPacketRecv;

extern	unsigned	int	LgwNbStartOk;
extern	unsigned	int	LgwNbStartFailure;
extern	unsigned	int	LgwNbConfigFailure;
extern	unsigned	int	LgwNbLinkDown;
extern	unsigned	int	LgwNbBusySend;
extern	unsigned	int	LgwNbSyncError;
extern	unsigned	int	LgwNbCrcError;
extern	unsigned	int	LgwNbSizeError;
extern	unsigned	int	LgwNbChanUpError;
extern	unsigned	int	LgwNbChanDownError;
extern	unsigned	int	LgwNbDelayError;
extern	unsigned	int	LgwNbDelayReport;

extern	unsigned	int	MacNbFcsError;

extern			int	LgwInvertPol;
extern			int	LgwInvertPolBeacon;
extern			int	LgwNoCrc;
extern			int	LgwNoHeader;
extern			int	LgwPreamble;
extern			int	LgwPreambleAck;
extern			int	LgwPower;
extern			int	LgwPowerMax;
extern			int	LgwAckData802Wait;
extern			u_int	LgwSyncWord;
extern			char	*LgwSyncWordStr;
extern			int	LgwBoard;
extern			int	LgwChipsPerBoard;
extern			int	LgwAntenna;
extern			int	LgwForceRX2;
extern			int	LgwLbtEnable;
extern			int	LgwLbtRssi;
extern			int	LgwLbtRssiOffset;
extern			int	LgwLbtScantime;
extern			int	LgwLbtTransmitTime;
extern			int	LgwLbtNbChannel;
extern			int	LgwLbtClasscDelay;

extern			t_channel	TbChannel[NB_CHANNEL];
extern			t_channel	* Rx2Channel;
extern			int	NbChannel;
extern			int	MaxChannel;
extern			t_channel_entry	TbChannelEntry[NB_CHANNEL];
extern			int	NbChannelEntry;
extern			t_channel_entry	TbChannelEntryDn[NB_CHANNEL];
extern			int	NbChannelEntryDn;

extern			int	LastTmoaRequested[];
extern			int	CurrTmoaRequested[];

extern			int	WanRefresh;
extern			int	TempEnable;
extern			int	TempPowerAdjust;
extern			int	TempExtAdjust;
extern			int	CurrTemp;

extern			int	MaxReportDnImmediat;
extern			int	LogUseRamDir;

extern			u_int	LgwBeaconRequestedCnt;
extern			u_int	LgwBeaconRequestedDupCnt;
extern			u_int	LgwBeaconRequestedLateCnt;
extern			u_int	LgwBeaconSentCnt;
extern			u_char	LgwBeaconLastDeliveryCause;

extern			u_int	LgwClassCRequestedCnt;
extern			u_int	LgwClassCRequestedDupCnt;
extern			u_int	LgwClassCRequestedLateCnt;
extern			u_int	LgwClassCSentCnt;
extern			u_char	LgwClassCLastDeliveryCause;

extern			int	MasterLrc;	// NFR997

extern			float	DcLt;		// long term dl dutycycle
extern			int	DcLtPeriod;	// long term dl dutycycle period, default is 168hours = 7 days
extern			u_int	DlShiftLc;

extern			u_int	NfVersion;
extern			u_int	NfNetwork;
extern			u_int	NbLocRjTot;

// RDTP-5475
extern	int 		SynchroMaxLrr;
extern	int 		SynchroForward;
extern	int 		SynchroPeriod;
extern	int 		SynchroLC;	
extern	int 		SynchroSF;
extern	t_channel	*SynchroChannel;
extern	double		SynchroDrift;
extern	double		SynchroDriftMin;
extern	double		SynchroDriftMax;
extern	int		SynchroForced;
extern	int		SynchroReset;

extern	u_char		LrrIDGetFromTwa;
extern	u_int		LrrIDFromTwaGot;

extern	u_int		SimulationMode;	// Simulation mode activated
