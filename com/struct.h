
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


typedef	struct
{
	int8_t	ta_temp;
	int8_t	ta_power;
}	t_temp_adap;

typedef struct s_gpstime
{
	struct timespec utc;
	struct timespec ubx;
}	t_gpstime;

typedef	struct
{
	char	*in_name;
	char	*in_val;
}	t_ini_var;

typedef	struct	s_define
{
	char	*name;
	int	value;
}	t_define;

typedef	struct	s_channel
{
uint32_t	channel;
uint32_t	freq_hz;
uint8_t		subband;
uint8_t		modulation;
uint8_t		bandwidth;
uint16_t	datarate;
int8_t		power;
uint8_t		usedforrx2;
uint16_t	dataraterx2;
uint32_t	lbtscantime;	// 0:no 1:default scantime X:scantime in us
uint32_t	lbttransmit_time; // 0:no 1:default transmit time X:transmit time in ms
uint8_t		name[64];
}	t_channel;

typedef	struct	s_channel_entry
{
uint32_t	freq_hz;
uint32_t	index;			// index in TbChannel
}	t_channel_entry;

/*!
 * Radio LoRa modem parameters to compute time on air
 */
typedef struct
{
    int8_t   Power;
    uint32_t Bandwidth;
    uint32_t Datarate;
    uint8_t  LowDatarateOptimize;
    uint8_t  Coderate;
    uint16_t PreambleLen;
    uint8_t  FixLen;
    uint8_t  CrcOn;
    uint8_t  IqInverted;
    uint8_t  RxContinuous;
    uint32_t TxTimeout;
}	t_RadioLoRaSettings;

#define	AVDV_NBELEM	1024
typedef	struct	s_avdv_ui
{
	time_t		ad_time[AVDV_NBELEM];
	uint32_t	ad_hist[AVDV_NBELEM];
	uint32_t	ad_slot;	// next slot
	uint32_t	ad_vmax;	// value max
	time_t		ad_tmax;	// time of value max
	uint32_t	ad_aver;
	uint32_t	ad_sdev;
}	t_avdv_ui;

typedef	struct	s_lrc_link_stat
{
	uint32_t	lrc_nbdisc;
	uint32_t	lrc_nbsend;	// pkt
	uint32_t	lrc_nbsendB;	// byte
	uint32_t	lrc_nbrecv;	// pkt
	uint32_t	lrc_nbrecvB;	// byte
	uint32_t	lrc_nbdrop;
}	t_lrc_link_stat;

typedef	struct	s_lrc_link
{
	void		*lrc_lk;	// t_xlap_link *
	int		lrc_idx;
	t_avdv_ui	lrc_avdvtrip;
	t_lrc_link_stat	lrc_stat;
	t_lrc_link_stat	lrc_stat_p;
	int		lrc_testinit;
	time_t		lrc_testlast;
	int		lrc_twaresp;	// response received RDTP-9756/7649
	uint32_t	lrc_twalrridresp;// response may be 0 RDTP-9756/7649
}	t_lrc_link;

typedef	struct	s_wan_stat
{
	time_t	it_tmms;
	double	it_rxbytes_p;	// previous rx bytes on itf
	double	it_txbytes_p;	// previous tx bytes on itf
	double	it_rxbytes_d;	// delta rx bytes on period
	double	it_txbytes_d;	// delta tx bytes on period

	double	it_rxbytes_br;	// bytes rate rx on period
	double	it_txbytes_br;	// bytes rate tx on period
	double	it_rxbytes_mbr;	// max bytes rate rx on period
	double	it_txbytes_mbr;	// max bytes rate tx on period
	time_t	it_rxbytes_mbrt;// max bytes rate rx time
	time_t	it_txbytes_mbrt;// max bytes rate tx time
}	t_wan_stat;

// type of cellular connection
#define CELL_MODE_NOSERV	0
#define CELL_MODE_LTE		1
#define CELL_MODE_WCDMA		2
#define CELL_MODE_GSM		3

typedef	struct {
	uint8_t		mode; 	// CELL_MODE_LTE, CELL_MODE_WCDMA, CELL_MODE_GSM, CELL_MODE_NOSERV
	time_t		tm;	// timestamp
	uint32_t  	update_count;	// number of times the informations were updated
	union {
		struct {
			 // [0,96], 0: rssi<-120 dBm, 96: rssi>=-25 dBm, 255: unknown or undetectable
			float     rssi; // [0,96], 0: rssi<-120 dBm, 96: rssi>=-25 dBm, 255: unknown or undetectable
			// count number of reports during the period
			uint32_t  rssi_count;
			// [0,97], 0: rsrp<-140 dBm, 97: rsrp>=-44 dBm, 255: unknown or undetectable
			float     rsrp;
			// [0,251], 0: sinr<-20 dB, 1: -20<=sinr-<-19.8 dB, 255: unknown or undetectable
			float     sinr;
			 // [0,34], 0: rsrq<-19.5 dB, 1: -19.5<=rsrq<-19 dB, 255: unknown or undetectable
			float     rsrq;
		} lte;
		struct {
			// [0,96], 0: rssi<-120 dBm, 96: rssi>=-25 dBm, 255: unknown or undetectable
			float     rssi; 
			// count number of reports during the period
			uint32_t  rssi_count;
			// [0,96], 0: rscp<-120 dBm, 96: rscp>=-25 dBm, 255: unknown or undetectable
			float     rscp;
			// [0,65], 0: ecio<-32 dB, 1: -32<=ecio<31.5 dB, 255: unknown or undetectable
			float     ecio;
		} wcdma;
		struct {
			float     rssi; // [0,96], 0: rssi<-120 dBm, 96: rssi>=-25 dBm, 255: unknown or undetectable
			uint32_t  rssi_count;  // count number of reports during the period
		} gsm;
	} cont;
} t_cell_indic; 

typedef struct {
	char		ope[32+1];
	uint8_t		mode; 	// the mode is also present in indic. This one is the one reported to TWA
				// CELL_MODE_LTE, CELL_MODE_WCDMA, CELL_MODE_GSM, CELL_MODE_NOSERV
	char    	imei[30];
	char    	imsi[30];
	char    	iccid[30];
	t_cell_indic	indic;
} t_cell_infos;

typedef	struct	s_wan_itf
{
	int	it_enable;
	char	*it_name;
	int	it_exists;
	int	it_type;	// by configuration


	int		it_l2;		// carrier==1 low level link
	int		it_up;		// operstate=="up"
	uint32_t	it_ipv4;	// ipv4 ok
	int		it_def;		// default route uses this interface

	double		it_rxbytes_c;	// current rx bytes on itf
	double		it_txbytes_c;	// current tx bytes on itf

	t_wan_stat	it_sht;		// short period stats on itf
	t_wan_stat	it_lgt;		// long period stats on itf

	uint32_t	it_readcnt;
	uint32_t	it_l2cnt;	// # itf was up
	double		it_l2time;	// time itf was up
	uint32_t	it_upcnt;	// # itf was up
	double		it_uptime;	// time itf was up
	uint32_t	it_defcnt;	// # itf was "default itf"
	double		it_deftime;	// time itf was "default itf"
	unsigned int	it_lostprtt_p;	// previous it_lostprtt
	unsigned int	it_sentprtt_p;	// previous it_sentprtt
	unsigned int	it_okayprtt_p;	// previous it_okayprtt

	int		it_idx;
	pthread_t	it_thread;
	// Round trip time to LRC with ping
	t_avdv_ui	it_avdvprtt;	// avdv for ping rtt
	int		it_nbpkprtt;	// nb ping in avdv
	unsigned int	it_lostprtt;	// tot ping lost for ping rtt
	unsigned int	it_sentprtt;	// tot ping sent for ping rtt
	unsigned int	it_okayprtt;	// tot ping okay for ping rtt (ack)
	time_t		it_lastprtt;	// last ping for ping rtt
	time_t		it_lastavdv;

	// Check full connectivity on interface with ping
	time_t		it_lastpcnx;	// last ping for ping cnx

	// RDTP-4176
	char		*it_pingaddr;	// default is addr of primary LRC
	char		*it_pingaddrsrc;// default is to use -Iitfname option
}	t_wan_itf;


typedef	struct	s_mfs
{
	int	fs_enable;
	char	*fs_name;
	u_char	fs_type;	// by configuration lrr.ini
	int	fs_exists;
	int	fs_fd;

	u_int	fs_size;
	u_int	fs_used;
	u_int	fs_avail;
	u_int	fs_puse;
}	t_mfs;


enum reboot_causes_codes {
    REBOOT_NO_LRC_COMM,
    REBOOT_NO_UPLINK_RCV,
    REBOOT_CAUSES_MAX
};

