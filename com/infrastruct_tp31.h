
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

#ifndef		_INFRASTRUCT_H_
#define		_INFRASTRUCT_H_
#include	<stddef.h>

// LRR version >= 2.0.0
#define		LP_LRRID_NO_PREF	0x00	// old LRRs with 4Bytes mac@
#define		LP_LRRID_FULL_MAC	0x01	// full mac address 6B
#define		LP_LRRID_BY_CONFIG	0x02	// set manually by configuration
#define		LP_LRRID_RBPI_CPUID	0x03	// raspberry CPU serial
#define		LP_LRRID_CISCO_SN	0x04	// cisco serial number

#define		LP_LRRID_MTC		0xfe	// prefixe for MTC
#define		LP_LRRID_RESERVED	0xff	// prefixe for LRC peer ?


#define		NB_LRC_PER_LRR		5 // LRC declared by LRR
#define		NB_ITF_PER_LRR		2 // network interfaces by LRR
#define		NB_MFS_PER_LRR		5 // mounted file systems by LRR
#define		NB_MAX_BOARD_PER_LRR    6 // max board by LRR (LRC side)
#define		NB_MAX_ANT_PER_LRR    	8 // max antenna by LRR (LRC side)

//	Causes for downlink transmission failures on RX1/RX2
//	Coded on 1 byte:
//		- first hexa digit is the type of failure
//		- 2nd hexa is an index for the type:
//			- index 0..9 are used by LRR
//			- index A..F are used by LRC
// causes for RX1 also called cause1
#define	LP_C1_OK		0
#define	LP_C1_RADIO_STOP	0xA0	// Radio stopped
#define	LP_C1_RADIO_STOP_DN	0xA1	// Downlink radio stopped
#define	LP_C1_NA		0xA2	// rx1 not available (n/a)
#define	LP_C1_BUSY		0xA3	// Radio busy tx
#define	LP_C1_LBT		0xA4	// listen before talk
#define	LP_C1_DELAY		0xB0	// Too late for rx1
#define	LP_C1_PAYLOAD_SIZE	0xB1	// Too big for rx1/rx2
#define	LP_C1_LRC		0xC0	// LRC chooses RX2
#define	LP_C1_DTC_LRR		0xD0	// DTC detected by LRR
#define	LP_C1_DTC_LRC		0xDA	// DTC detected by LRC
#define LP_C1_DWELL_LRC		0xDB	// Max DM dwell time constraint detected by the LRC
#define	LP_C1_NO_LRR_LRC	0xDD	// No LRR connected detected by LRC
#define	LP_C1_DTC_PEER		0xDE	// DTC not allowed by the peering operator ???
#define	LP_C1_NETID_LRC		0xDF	// NETID
#define	LP_C1_MAXTRY		0xE0	// Tx immediate failure classC
#define	LP_C1_NO_MATCH_FREQ	0xF0	// No matching channel for multicast frequency

// causes for RX2 also called cause2
#define	LP_C2_OK		0
#define	LP_C2_RADIO_STOP	0xA0	// Radio stopped
#define	LP_C2_RADIO_STOP_DN	0xA1	// Downlink radio stopped
#define	LP_C2_NA		0xA2	// rx2 not available
#define	LP_C2_BUSY		0xA3	// Radio busy tx
#define	LP_C2_LBT		0xA4	// listen before talk
#define	LP_C2_DELAY		0xB0	// Too late for rx1
#define	LP_C2_DTC_LRR		0xD0	// DTC detected by LRR
#define	LP_C2_DTC_LRC		0xDA	// DTC detected by LRC
#define	LP_C2_DWELL_LRC		0xDB	// Max DM dwell time constraint detected by the LRC
#define	LP_C2_NO_LRR_LRC	0xDD	// No LRR connected detected by LRC
#define	LP_C2_DTC_PEER		0xDE	// DTC not allowed by the peering operator ???
#define	LP_C2_NETID_LRC		0xDF	// NETID
#define	LP_C2_LRC		0xC0	// LRC chooses RX2
#define	LP_C2_MAXTRY		0xE0	// Tx immediate failure classC (n/a)
#define	LP_C2_NO_MATCH_FREQ	0xF0	// No matching channel for multicast frequency
#define	LP_C2_MC_INVALID_MULTI_RX2	0xF2	// Multiple RX2 channels and no configured multicast frequency

// causes for classB also called cause3 ...
#define	LP_CB_OK		0
#define	LP_CB_RADIO_STOP	0xA0	// Radio stopped
#define	LP_CB_RADIO_STOP_DN	0xA1	// Downlink radio stopped
#define	LP_CB_NA		0xA2	// classB not available (GPS,sync,...)
#define	LP_CB_BUSY		0xA3	// Radio busy tx
#define	LP_CB_LBT		0xA4	// listen before talk
#define	LP_CB_BOARDERR		0xA5	// radio board error
#define	LP_CB_DELAY		0xB0	// Too late for rx1
#define	LP_CB_DTC_LRR		0xD0	// DTC detected by LRR
#define	LP_CB_DTC_LRC		0xDA	// DTC detected by LRC
#define LP_CB_DWELL_LRC		0xDB	// Max DM dwell time constraint detected by the LRC
#define	LP_CB_NA_GPS_DOWN	0xDC	// classB not available because LRR GPS is down
#define	LP_CB_NO_LRR_LRC	0xDD	// No LRR connected detected by LRC
#define	LP_CB_DTC_PEER		0xDE	// DTC not allowed by the peering operator ???
#define	LP_CB_NETID_LRC		0xDF	// NETID
#define	LP_CB_LRC		0xC0	// LRC chooses RX2
#define	LP_CB_MAXTRY		0xE0	// Tx immediate failure classC (n/a)
#define	LP_CB_NO_MATCH_FREQ	0xF0	// No matching channel for multicast frequency
#define	LP_CB_MC_INVALID_MULTI_SLOT	0xF1	// Multiple Ping Slot channels and no configured multicast frequency

#define	LP_STRUCTPACKED	__attribute__ ((packed))

// lp_flag
#define	LP_RADIO_PKT_UP		1
#define	LP_RADIO_PKT_DOWN	2
#define	LP_RADIO_PKT_ACKMAC	4	// downlink only
#define	LP_RADIO_PKT_DELAY	8	// downlink only
#define	LP_RADIO_PKT_PINGSLOT	16	// downlink Class B
#define	LP_RADIO_PKT_LORA_E	32
#define	LP_RADIO_PKT_802154	64	// downlink only
#define	LP_RADIO_PKT_NOCRC	128	// uplink only
#define	LP_RADIO_PKT_SHIFTLC	256	// downlink only
#define	LP_RADIO_PKT_ACKDATA	512	// downlink only + LP_RADIO_PKT_802154
#define	LP_RADIO_PKT_RX2	1024	// second window
#define	LP_RADIO_PKT_DTC	2048	// dutycycle info present
#define	LP_RADIO_PKT_LATE	4096	// uplink only
#define	LP_RADIO_PKT_DELAYED	8192	// downlink only
#define	LP_RADIO_PKT_FINETIME	16384	// uplink only fine timestamp
#define	LP_INFRA_PKT_INFRA	32768	// packet LRR<->LRC or LRC<->LRC infra

// lp_flagm
#define	LP_RADIO_PKT_GPSTIME	1	// uplink only
#define	LP_RADIO_PKT_ASWAIT	2	// packet delayed due to ASWAIT feature


// for each uplink packet we keep LP_MAX_LRR_TRACKING (max) LRR best sources
// for each source we keep rssi,snr,gpsco,timestamp, see : t_lrr_qos
#define	LP_MAX_LRR_TRACKING	10	// NFR688 from 8 to 10
#define	LP_MAX_ANT_TRACKING	3


#define	LP_PRE_HEADER_PKT_SIZE	20
#define	LP_DATA_CMD_LEN		192
#define	MAX_CUSTOM_VERSION_LEN	32

#define	LP_DATA_LRRUID_LEN	256

#if	0	// old TP30 values
#define	LP_IEC104_SIZE_MAX	249
#define	LP_MACLORA_SIZE_MAX	(LP_IEC104_SIZE_MAX - LP_PRE_HEADER_PKT_SIZE \
				- sizeof(t_lrr_pkt_radio) - 10)

#define	LP_UNION_SIZE_MAX	(LP_IEC104_SIZE_MAX - LP_PRE_HEADER_PKT_SIZE \
				- 10)
#endif

#define	LP_IEC104_SIZE_MAX	512
#define	LP_MACLORA_SIZE_MAX	(256)
#define	LP_UNION_SIZE_MAX	(LP_IEC104_SIZE_MAX - LP_PRE_HEADER_PKT_SIZE)


// protocole LRX version 0 : lp_vers and lp_szh are always set to 0
// and the size before the mac payload is limited and fixed to 49 bytes
#define	LP_HEADER_PKT_SIZE_V0	49
//
// protocole LRX version 1 : lp_vers>=1, lp_szh == the size of the member used
// in lp_u for the current packet. Now the size before the mac payload is not
// fixe, ie [LP_PRE_HEADER_PKT_SIZE .. LP_PRE_HEADER_PKT_SIZE+sizeof(lp_u)]

// size of t_lrr_pkt_radio before LRR 2.6.x
#define	LP_T_LRR_PKT_RADIO_BEFORE_2_6_X	46

#define	LP_VALID_SIZE_PKT()						\
{									\
	t_lrr_pkt	pkt;						\
	if	(offsetof(t_lrr_pkt,lp_u) > LP_PRE_HEADER_PKT_SIZE)	\
	{								\
printf	("&lp_u could not be > %d\n",LP_PRE_HEADER_PKT_SIZE);		\
		exit(1);						\
	}								\
	if	(offsetof(t_lrr_pkt,lp_payload) > LP_IEC104_SIZE_MAX)	\
	{								\
printf	("&lp_u could not be > %d\n",LP_IEC104_SIZE_MAX);		\
		exit(1);						\
	}								\
	if	(sizeof(t_lrr_pkt_radio_var_up) != 8)			\
	{								\
printf	("s_lrr_pkt_radio_var_up != 8\n");				\
		exit(1);						\
	}								\
	if	(sizeof(t_lrr_pkt_radio_var_dn) != 8)			\
	{								\
printf	("s_lrr_pkt_radio_var_dn != 8\n");				\
		exit(1);						\
	}								\
	if	(sizeof(t_lrr_pkt_radio_802_dn) != 8)			\
	{								\
printf	("s_lrr_pkt_radio_802_dn != 8\n");				\
		exit(1);						\
	}								\
	if	(sizeof(t_lrr_pkt_indic_status) != 8)			\
	{								\
printf	("t_lrr_pkt_indic_status != 8\n");				\
		exit(1);						\
	}								\
	if	(sizeof(t_lrr_pkt_radio_dtc_ud) != 16)			\
	{								\
printf	("t_lrr_pkt_radio_dtc_ud != 16\n");				\
		exit(1);						\
	}								\
	if	(sizeof(pkt.lp_u.lp_radio.lr_u3) != 16)			\
	{								\
printf	("pkt.lp_u.lp_radio.lr_u3 != 16\n");				\
		exit(1);						\
	}								\
	if	(sizeof(pkt.lp_u) > LP_UNION_SIZE_MAX)		\
	{								\
printf	("lp_u could not be > %d\n",LP_UNION_SIZE_MAX);		\
		exit(1);						\
	}								\
}


#define		li_alti		li_u._li_alti_i
#define		li_alti_i	li_u._li_alti_i
#define		li_alti_f	li_u._li_alti_f

typedef	struct	LP_STRUCTPACKED	s_lrr_gpsco
{
	float	li_latt;
	float	li_long;
	u_char	li_gps;		// 0 not received, 1 set manualy, 2 true GPS
	union
	{
		int	_li_alti_i;	// sent by LRR HAL/API/GPS
		float	_li_alti_f;	// retrieved from local LRR declaration
	}	li_u;
}	t_lrr_gpsco;


#ifdef	LP_MODULE_LRC
#include "sNS_pkt.h"

typedef	struct	LP_STRUCTPACKED	s_lrr_qos
{
	u_int		qs_lrrid;
	float		qs_rssi;
	float		qs_snr;
	float		qs_esp;		// estimated signal power NFR182
	u_int		qs_gss;		// time sec (LINUX or GPS)
	u_int		qs_gns;		// time nsec (LINUX or GPS)
	u_int		qs_tms;		// time msec (LINUX)
	u_int		qs_tus;		// time usec (SEMTECH)
	u_char		qs_chain;	// rf_chain or board number
	t_lrr_gpsco	qs_gps;
	double		qs_instantPER;
	u_char		qs_invalid;   	// 0=Valid !0 = Invalid
	u_char		qs_ftq;		// fine time qualifier discared if ...
	u_int		qs_cookie;	// sNS.c store a uniq Id, used to retrieve context
	u_char		qs_lrrftq;	// fine time qualifier as said by LRR
	u_char		qs_gpsq;	// time qualifier is GPS one
	u_char		qs_channel;
	FNSInfo_t	*FNSInfo;
	short		qs_foffset;	// freq offset
	u_char		qs_ftstatus;	// fine timestamp status
	short		qs_ftdelta;	// fine timestamp delta
}	t_lrr_qos;
#endif

typedef	struct	LP_STRUCTPACKED	s_lrr_capab
{
	u_char	li_nbantenna;
	u_short	li_versMaj;		// LRR v M.m.r
	u_short	li_versMin;
	u_short	li_versRel;
	u_char	li_ismBand[9+1];	// eu868,sg920,us915
	u_int	li_ismVar;		// now rfRegion version
	u_char	li_system[32+1];
	u_char	li_ismBandAlter[32+1];	// the real ism band used
	u_char	li_rfRegion[32+1];	// the rfregion used
	u_char  li_szPktRadioStruct;	// sizeof(t_lrr_pkt_radio) used
	u_char	li_nbBoard;		// 0 => 1 board with 1 SX (basic arch)
	u_char	li_nbChan;		// number of channels per board
	u_char	li_nbSector;		// 0 => 1 sector
	u_char  li_fpga;                // radio board with fpga
	u_char	li_nbChanUp;		// number of channels Up
	u_char	li_nbChanDn;		// number of channels Down
	u_char	li_iecExtFtr;		// support IEC extended
	u_char	li_dtcdnFtr;		// support down duty cycle feature
	u_char	li_dtcupFtr;		// support up duty cycle feature
	u_char	li_sentIndicFtr;	// support sent indication feature
	u_int	li_pktStoreFtr;		// support storage feature and #msg
	u_char	li_geoLocFtr;		// support geolocaziation feature
	u_char	li_lbtFtr;		// support LBT feature
	u_char	li_classBFtr;		// support Class B feature
	u_char	li_freqdnFtr;		// support of dynamic downlink frequency
	u_char	li_classCMcFtr;		// support Class C multicast (on PPS)
	u_short	li_versFix;		// level fix version
	u_char	li_shiftLcFtr;		// support of downlink shift LC
	u_char	li_nwfFtr;		// support of network filtering RDTP-2413
	u_char	li_multiRx2Ftr;		// support of N RX2 RDTP-2543
	u_char	li_multiPingSlotFtr;	// support of N classB slot RDTP-2543
}	t_lrr_capab;

typedef	struct	LP_STRUCTPACKED	s_lrr_pkt_radio_var_up
{
	float	lr_rssi;		// 
	float	lr_snr;			//
}	t_lrr_pkt_radio_var_up;


typedef	struct	LP_STRUCTPACKED	s_lrr_pkt_radio_802_dn
{
	u_char	lr_szpreamb;		// 
	u_char	lr_opaque[7];		// only for watteco 802154 ACK+DATA
}	t_lrr_pkt_radio_802_dn;	

typedef	struct	LP_STRUCTPACKED	s_lrr_pkt_radio_var_dn
{
	u_char	lr_shiftlc;		// shift the downlink logical channel 
	u_char	lr_majorv;		// major version of LORMAC bit7=>watteco
	u_char	lr_minorv;		// minor version of LORMAC
	u_char	lr_rx2spfact;		// SF to use on RX2
	u_char	lr_freqdn[3];		// dynamic downlink frequency
	u_char	lr_rx2channel;		// channel to use on RX2
}	t_lrr_pkt_radio_var_dn;	

typedef	struct	LP_STRUCTPACKED	s_lrr_pkt_radio_p1
{
	u_int	lr_tms;			// time msec (LINUX)
	u_int	lr_tus;			// time usec (SEMTECH)
	u_int	lr_gss;			// time sec (LINUX or GPS)
	u_int	lr_gns;			// time nsec (LINUX or GPS)

	u_char	lr_chain;		// rf_chain
	union
	{
		t_lrr_pkt_radio_var_up	lr_var_up;	// rssi, snr ...
		t_lrr_pkt_radio_802_dn	lr_802_dn;	// ack + data
		t_lrr_pkt_radio_var_dn	lr_var_dn;	// version
	}	lr_u;
	u_char	lr_channel;		// channel number
	u_char	lr_subband;		// subband number G1,G2,G3
	u_char	lr_spfact;		// spreading factor up
	u_char	lr_correct;		// error correcting type
}	t_lrr_pkt_radio_p1;

typedef	struct	LP_STRUCTPACKED	s_lrr_pkt_radio_dtc_ud	// NFR326
{
	float	lr_dtcchanneldn;
	float	lr_dtcsubbanddn;
	float	lr_dtcchannelup;
	float	lr_dtcsubbandup;
}	t_lrr_pkt_radio_dtc_ud;

typedef	struct	LP_STRUCTPACKED	s_lrr_pkt_radio_ref_dn	// NFR329
{
	u_int64_t	lr_deveui;
	u_short		lr_fcntdn;
	u_int32_t	lr_devaddr;
}	t_lrr_pkt_radio_ref_dn;

typedef	struct	LP_STRUCTPACKED	t_lrr_pkt_indic_status
{
	u_char	lr_unused;
	u_char	lr_delivered;		// 0 or 1
	u_char	lr_cause1;		// 
	u_char	lr_cause2;		// 
	u_short	lr_trip;
	u_char	lr_cause3;
	u_char	lr_rfu;
}	t_lrr_pkt_indic_status;

typedef	struct	LP_STRUCTPACKED	s_lrr_pkt_radio
{
	u_int	lr_tms;			// time msec (LINUX)
	u_int	lr_tus;			// time usec (SEMTECH)
	u_int	lr_gss;			// time sec (LINUX or GPS)
	u_int	lr_gns;			// time nsec (LINUX or GPS)

	u_char	lr_chain;		// rf_chain
	union
	{
		t_lrr_pkt_radio_var_up	lr_var_up;	// rssi, snr ...
		t_lrr_pkt_radio_802_dn	lr_802_dn;	// ack + data
		t_lrr_pkt_radio_var_dn	lr_var_dn;	// version
		t_lrr_pkt_indic_status	lr_indic;
	}	lr_u;
	u_char	lr_channel;		// channel number
	u_char	lr_subband;		// subband number G1,G2,G3
	u_char	lr_spfact;		// spreading factor up
	u_char	lr_correct;		// error correcting type

//	added to t_lrr_pkt_radio_p1
	u_char	lr_bandwidth;		// bandwidth 0:? 1:125k 2:250k 3:500k
	union
	{
		t_lrr_pkt_radio_dtc_ud	lr_dtc_ud;	// up	(LRR -> LRC)
		t_lrr_pkt_radio_ref_dn	lr_ref_dn;	// down (LRC -> LRR)
	}	lr_u2;
//	added to t_lrr_pkt_radio from 2.6.0
	union
	{
		// lr_xxx_xx is not use in DL but is overwriten by LRRs in 
		// t_lrr_pkt_sent_indic in UL
		// => we have sizeof(t_lrr_pkt_radio_dtc_ud) free bytes in DL
		t_lrr_pkt_radio_dtc_ud	lr_xxx_xx;
		u_char			lr_dlnuse[16];
	}	lr_u3;
	u_short	lr_flagr;		// more radio flags (not used yet)
	u_short	lr_ie_size;		// size of IE (not used yet)
	// RDTP-10520
	u_int	lr_fq_freq;		// frequency center
	short	lr_fq_offset;		// frequency offset in Hz
	u_char	lr_ft_status;		// fine timestamp status
	short	lr_ft_delta;		// fine timestamp delta
}	t_lrr_pkt_radio;


typedef	struct	LP_STRUCTPACKED	s_lrr_pkt_sent_indic
{
	u_int	lr_tms;			// time msec (LINUX)
	u_int	lr_tus;			// time usec (SEMTECH)
	u_int	lr_gss;			// time sec (LINUX or GPS)
	u_int	lr_gns;			// time nsec (LINUX or GPS)

	u_char	lr_chain;		// rf_chain
	union				// unused here
	{
		t_lrr_pkt_radio_var_up	lr_var_up;	// rssi, snr ...
		t_lrr_pkt_radio_802_dn	lr_802_dn;	// ack + data
		t_lrr_pkt_radio_var_dn	lr_var_dn;	// version
		t_lrr_pkt_indic_status	lr_indic;	// sent_indic status
	}	lr_u;
	u_char	lr_channel;		// channel number
	u_char	lr_subband;		// subband number G1,G2,G3
	u_char	lr_spfact;		// spreading factor up
	u_char	lr_correct;		// error correcting type
	u_char	lr_bandwidth;		// bandwidth 0:? 1:125k 2:250k 3:500k
	union
	{
		t_lrr_pkt_radio_dtc_ud	lr_xxx_xx;	// not possible here
		t_lrr_pkt_radio_ref_dn	lr_ref_dn;	// down (LRC -> LRR)
	}	lr_u2;
	t_lrr_pkt_radio_dtc_ud	lr_dtc_ud;
	u_short	lr_flagr;		// more radio flags (not used yet)
	u_short	lr_ie_size;		// size of IE (not used yet)
	// RDTP-10520
	u_int	lr_fq_freq;		// frequency center
	short	lr_fq_offset;		// frequency offset in Hz
	u_char	lr_ft_status;		// fine timestamp status
	short	lr_ft_delta;		// fine timestamp delta
}	t_lrr_pkt_sent_indic;

typedef	struct	LP_STRUCTPACKED	s_lrr_stat
{
	u_int	ls_LrcNbDisc;
	u_int	ls_LrcNbPktDrop;

	u_int	ls_LgwNbPacketSend;
	u_int	ls_LgwNbPacketRecv;

	u_int	ls_LgwNbCrcError;
	u_int	ls_LgwNbDelayError;
}	t_lrr_stat;

// since LRC 1.0.14, it is possible to add attributs at the end of this struture
// without consequence for previous LRR versions
typedef	struct	LP_STRUCTPACKED	s_lrr_stat_v1
{
	u_int	ls_LrcNbDisc;
	u_int	ls_LrcNbPktDrop;
	u_int	ls_LrcMxPktTrip;
	u_int	ls_LrcAvPktTrip;

	u_int	ls_LgwNbPacketSend;
	u_int	ls_LgwNbPacketWait;
	u_int	ls_LgwNbPacketRecv;

	u_int	ls_LgwNbStartOk;
	u_int	ls_LgwNbStartFailure;
	u_int	ls_LgwNbConfigFailure;
	u_int	ls_LgwNbLinkDown;

	u_int	ls_LgwNbBusySend;
	u_int	ls_LgwNbSyncError;
	u_int	ls_LgwNbCrcError;
	u_int	ls_LgwNbSizeError;
	u_int	ls_LgwNbChanUpError;
	u_int	ls_LgwNbChanDownError;
	u_int	ls_LgwNbDelayError;
	u_int	ls_LgwNbDelayReport;

// added in LRC 1.0.14, LRR 1.0.35
	u_int	ls_gssuptime;		// uptime sec LRR linux process
	u_int	ls_gnsuptime;		// uptime nsec LRR linux process
	u_int	ls_nbupdate;		// update count since uptime
	u_short	ls_statrefresh;		// update period
	u_short	ls_versMaj;		// LRR v M.m.r
	u_short	ls_versMin;
	u_short	ls_versRel;
	u_char	ls_LgwInvertPol;
	u_char	ls_LgwNoCrc;
	u_char	ls_LgwNoHeader;
	u_char	ls_LgwPreamble;
	u_char	ls_LgwPreambleAck;
	u_char	ls_LgwPower;
	u_char	ls_LgwAckData802Wait;
	u_char	ls_LrrUseGpsPosition;
	u_char	ls_LrrUseGpsTime;
	u_char	ls_LrrUseLgwTime;

// added in LRC 1.0.18, LRR 1.0.59
	u_int	ls_LgwSyncWord;
	u_short	ls_rfcellrefresh;
	u_int	ls_gssupdate;		// time sec (LINUX or GPS)
	u_int	ls_gnsupdate;		// time nsec (LINUX or GPS)
	u_int	ls_gssuptimesys;	// uptime sec LRR linux	/proc/uptime
	u_int	ls_gnsuptimesys;	// uptime nsec LRR linux
	u_char	ls_sickrestart;		// process restart due to crash
	u_int	ls_configrefresh;

// added in LRC 1.0.26, LRR 1.2.6
	u_int	ls_LrcNbPktTrip;
	u_int	ls_LrcDvPktTrip;
	u_int	ls_wanrefresh;

// added in LRC 1.0.31, LRR 1.4.4
	u_char	ls_mfsUsed[NB_MFS_PER_LRR][3];		// in Mega Bytes
	u_char	ls_mfsAvail[NB_MFS_PER_LRR][3];		// in Mega Bytes

	u_char	ls_cpuAv;
	u_char	ls_cpuDv;
	u_char	ls_cpuMx;
	u_int	ls_cpuMxTime;

	u_short	ls_cpuLoad1;
	u_short	ls_cpuLoad5;
	u_short	ls_cpuLoad15;

	u_char	ls_MemTotal[3];			// in Kilo Bytes
	u_char	ls_MemFree[3];			// in Kilo Bytes
	u_char	ls_MemBuffers[3];		// in Kilo Bytes
	u_char	ls_MemCached[3];		// in Kilo Bytes

	u_short	ls_gpsUpdateCnt;
	u_char	ls_LrrTimeSync;
	u_char	ls_LrrReverseSsh;

	u_char	ls_powerState;			// current power state
	u_short	ls_powerDownCnt;		// count of power down period

	u_char	ls_rfScan;			// rfscan running

	u_char	ls_traceLevel;	// TraceLevel + 1, 0 means unknown for LRC
	u_char	ls_useRamDir;	// ramdir activated

	u_char	ls_gpsState;			// current gps state
	u_short	ls_gpsDownCnt;			// count of gps down period
	u_short	ls_gpsUpCnt;			// count of gps up period
	u_short	ls_versFix;
	float   ls_dtclt;       // long term downlink dutycycle

	// RDTP-2413
	u_int	ls_LgwNbFilterDrop;
}	t_lrr_stat_v1;

typedef	struct	LP_STRUCTPACKED	s_lrr_power
{
	u_int	pw_gss;			// time sec (LINUX or GPS)
	u_int	pw_gns;			// time nsec (LINUX or GPS)
	u_int	pw_raw;
	float	pw_volt;
	u_char	pw_state;		// 'U' or 'D'
}	t_lrr_power;

typedef	struct	LP_STRUCTPACKED	s_lrr_gps_st
{
	u_int	gps_gssupdate;		// time sec (LINUX or GPS)
	u_int	gps_gnsupdate;		// time nsec (LINUX or GPS)
	u_char	gps_state;		// 'U' or 'D'
	float	gps_srate_wma;		// WMA Rate
}	t_lrr_gps_st;

typedef	struct	LP_STRUCTPACKED	s_lrr_rfcell
{
// added in LRC 1.0.18, LRR 1.0.59
	u_int	rf_nbupdate;		// update count since uptime
	u_short	rf_rfcellrefresh;	// update period, all fields reset to 0
	u_int	rf_gssupdate;		// time sec (LINUX or GPS)
	u_int	rf_gnsupdate;		// time nsec (LINUX or GPS)


	u_short	rf_LgwNbPacketRecv;
	u_short	rf_LgwNbCrcError;
	u_short	rf_LgwNbSizeError;
	u_short	rf_LgwNbDelayError;
	u_short	rf_LgwNbDelayReport;

	u_short	rf_LgwNbPacketSend;
	u_short	rf_LgwNbPacketWait;
	u_short	rf_LgwNbBusySend;

	u_short	rf_LgwNbStartOk;
	u_short	rf_LgwNbStartFailure;
	u_short	rf_LgwNbConfigFailure;
	u_short	rf_LgwNbLinkDown;

	u_char	rf_LgwState;
	u_int	rf_LgwBeaconRequestedCnt;
	u_int	rf_LgwBeaconSentCnt;
	u_char	rf_LgwBeaconLastDeliveryCause;

	// RDTP-2413
	u_int	rf_LgwNbFilterDrop;
	u_int	rf_LgwFilterVersion;
	u_int	rf_LgwNetwork;
}	t_lrr_rfcell;

// [2420][12898]
#define	ITF_TYPE_ETHE	0
#define	ITF_TYPE_CELL	1
#define	ITF_TYPE_WIFI	2
typedef	struct	LP_STRUCTPACKED	s_lrr_cellid
{
	u_char	cf_ItfImei[16+1];
	u_char	cf_ItfImsi[16+1];
	u_char	cf_ItfIccid[20+1];
}	t_lrr_cellid;

typedef	struct	LP_STRUCTPACKED	s_lrr_wifiid
{
	u_char	cf_ItfSsid[32+1];
}	t_lrr_wifiid;

typedef	struct	LP_STRUCTPACKED	s_lrr_config
{
// added in LRC 1.0.18, LRR 1.0.60
	u_int	cf_nbupdate;		// update count since uptime
	u_int	cf_configrefresh;	// update period, all fields reset to 0
	u_int	cf_gssupdate;		// time sec (LINUX or GPS)
	u_int	cf_gnsupdate;		// time nsec (LINUX or GPS)

	u_short	cf_versMaj;		// LRR v M.m.r
	u_short	cf_versMin;
	u_short	cf_versRel;
	u_short	cf_LrxVersion;		// LP_LRX_VERSION
	u_int	cf_LrrSMN[4];		// serial mkt number 128bits
	u_int	cf_LgwSyncWord;
	float	cf_LrrLocLat;
	float	cf_LrrLocLon;
	short	cf_LrrLocAltitude;
	u_char	cf_LrrLocMeth;
	u_char	cf_unsed;

	u_char	cf_LrrUseGpsPosition;
	u_char	cf_LrrUseGpsTime;
	u_char	cf_LrrUseLgwTime;
	u_char	cf_LrcDistribution;

	u_char	cf_IpInt[NB_ITF_PER_LRR][9+1];	// list of IP/interfaces
	u_char	cf_unsed_1[3][9+1];
	u_char	cf_ItfType[NB_ITF_PER_LRR];	// type of IP/interfaces
	u_char	cf_unsed_2[3];

// added in LRC 1.0.31, LRR 1.4.4
	u_char	cf_Mfs[NB_MFS_PER_LRR][16+1];	// mounted file system name
	u_char	cf_MfsType[NB_MFS_PER_LRR];	// root,tmp,user,boot,Backup

	u_char	cf_LrrGpsOk;			// if GpsFd opened

// added in LRC 1.0.31, LRR 1.4.12
	u_short	cf_versMajRff;		// LRR v M.m.r
	u_short	cf_versMinRff;
	u_short	cf_versRelRff;
	u_int	cf_rffTime;
	u_short	cf_versFix;
	u_short	cf_versFixRff;

	// [2420][12898]
	union
	{
		t_lrr_cellid	cf_ItfCellId;
		t_lrr_wifiid	cf_ItfWifiId;
	}	cf_ItfIdentif_u[NB_ITF_PER_LRR];
}	t_lrr_config;

typedef	struct	LP_STRUCTPACKED	s_lrr_config_lrc
{
// added in LRC 1.0.22, LRR 1.2.4
	u_short	cf_LrcCount;		// number of LRC declared
	u_short	cf_LrcIndex;		// [0..LrcCount-1]

	u_short	cf_LrcPort;
	u_char	cf_LrcUrl[64+1];
}	t_lrr_config_lrc;

typedef struct LP_STRUCTPACKED	s_lrr_config_lockey
{
	u_char	cf_idxboard;		// board index (starting from 0)
	u_char	cf_lockey[32+1];	// fine TS AES-128 key string-formatted
}	t_lrr_config_lockey;

typedef struct LP_STRUCTPACKED	s_lrr_config_ants
{
	int8_t	cf_idxant;		// antenna index (starting from 0)
	int8_t	cf_antgain;		// antenna gain (dBm)
	int8_t	cf_cableloss;		// cable loss (dBm)
	int8_t	cf_RX1_tx;		// Requested Tx EIRP for RX1 (dBm)
	int8_t	cf_RX1_eirp;		// Effective Tx EIRP for RX1 (dBm)
	int8_t	cf_RX2_tx;		// Requested Tx EIRP for RX2 (dBm)
	int8_t	cf_RX2_eirp;		// Effective Tx EIRP for RX2 (dBm)
	int8_t	cf_use_float;		// Set to 1 if floats values non-null
	float	cf_antgain_f;		// antenna gain (dBm) floating point value
	float	cf_cableloss_f;		// cable loss (dBm) floating point value
}	t_lrr_config_ants;

typedef struct LP_STRUCTPACKED s_lrr_custom_ver
{
	/* Add in LRR 2.2.x */
	u_char cf_cver_hw[MAX_CUSTOM_VERSION_LEN+1];		// hardware custom version string
	u_char cf_cver_os[MAX_CUSTOM_VERSION_LEN+1];		// os custom version string
	u_char cf_cver_hal[MAX_CUSTOM_VERSION_LEN+1];		// hal custom version string
	u_char cf_cver_build[MAX_CUSTOM_VERSION_LEN+1];		// build custom version string
	u_char cf_cver_config[MAX_CUSTOM_VERSION_LEN+1];	// config custom version string
	u_char cf_cver_custom1[MAX_CUSTOM_VERSION_LEN+1];	// custom1 custom version string
	u_char cf_cver_custom2[MAX_CUSTOM_VERSION_LEN+1];	// custom2 custom version string
	u_char cf_cver_custom3[MAX_CUSTOM_VERSION_LEN+1];	// custom3 custom version string
	// RDTP-7849 + cf_cver_lrr,cf_cver_fpga,cf_cver_fw,cf_cver_sn,cf_cver_sku
	u_char cf_cver_lrr[MAX_CUSTOM_VERSION_LEN+1];		// auto filled lrr version string
	u_char cf_cver_fpga[MAX_CUSTOM_VERSION_LEN+1];		// auto filled fpga version string
	u_char cf_cver_fw[2*MAX_CUSTOM_VERSION_LEN+1];		// auto filled firmware version string !! 64-bytes long only here
	u_char cf_cver_sn[MAX_CUSTOM_VERSION_LEN+1];		// auto filled serial number string
	u_char cf_cver_sku[MAX_CUSTOM_VERSION_LEN+1];		// auto filled sku string
}	t_lrr_custom_ver;

typedef	struct	LP_STRUCTPACKED	s_lrr_wan
{
// added in LRC 1.0.26, LRR 1.2.6
	u_int	wl_nbupdate;		// update count since uptime
	u_int	wl_wanrefresh;		// update period
	u_int	wl_gssupdate;		// time sec (LINUX or GPS)
	u_int	wl_gnsupdate;		// time nsec (LINUX or GPS)

	u_char	wl_ItfCount;		// number of ITF declared
	u_char	wl_LrcCount;		// number of LRC declared

	u_char	wl_ItfIndex[NB_ITF_PER_LRR];
	u_char	wl_ItfType[NB_ITF_PER_LRR];	// 0 ether, 1 gprs, 2 wifi
	u_char	wl_ItfState[NB_ITF_PER_LRR];	// 0 is up/running/used by def
	u_int	wl_ItfActivityTime[NB_ITF_PER_LRR];
	u_int	wl_ItfAddr[NB_ITF_PER_LRR];
	// [2420][12898]
	u_char	wl_ItfRssiNeverUsed[NB_ITF_PER_LRR];
	// [2420][12898]
	u_char	wl_ItfTechno[NB_ITF_PER_LRR][9+1];	// GPRS,3G,4G,5G,wifi
	u_short	wl_ItfNbPktTrip[NB_ITF_PER_LRR];	// ping/icmp pkt sent
	u_short	wl_ItfLsPktTrip[NB_ITF_PER_LRR];	// ping/icmp pkt lost
	u_short	wl_ItfAvPktTrip[NB_ITF_PER_LRR];
	u_short	wl_ItfDvPktTrip[NB_ITF_PER_LRR];
	u_short	wl_ItfMxPktTrip[NB_ITF_PER_LRR];
	u_int	wl_ItfMxPktTripTime[NB_ITF_PER_LRR];

	float	wl_ItfTxTraffic[NB_ITF_PER_LRR];
	float	wl_ItfTxAvgRate[NB_ITF_PER_LRR];
	float	wl_ItfTxMaxRate[NB_ITF_PER_LRR];
	u_int	wl_ItfTxMaxRateTime[NB_ITF_PER_LRR];

	float	wl_ItfRxTraffic[NB_ITF_PER_LRR];
	float	wl_ItfRxAvgRate[NB_ITF_PER_LRR];
	float	wl_ItfRxMaxRate[NB_ITF_PER_LRR];
	u_int	wl_ItfRxMaxRateTime[NB_ITF_PER_LRR];

	u_int	wl_LrcNbPktDrop;

	u_int	wl_LtqCnt;	// FIX3323: current number of packets stored
	u_int	wl_LtqMxCnt;	// FIX3323: max in report period
	u_int	wl_LtqMxCntTime;// FIX3323: time of max

	// [2420][12898]
	u_char	wl_ItfOper[NB_ITF_PER_LRR][32+1];	// current operator name
	u_char	wl_ItfKpisSet[NB_ITF_PER_LRR];
	float	wl_ItfRssi[NB_ITF_PER_LRR];	// wl_ItfKpisSet.bit1
	float	wl_ItfRscp[NB_ITF_PER_LRR];	// wl_ItfKpisSet.bit2
	float	wl_ItfEcio[NB_ITF_PER_LRR];	// wl_ItfKpisSet.bit3
	float	wl_ItfRsrp[NB_ITF_PER_LRR];	// wl_ItfKpisSet.bit4
	float	wl_ItfRsrq[NB_ITF_PER_LRR];	// wl_ItfKpisSet.bit5
	float	wl_ItfSinr[NB_ITF_PER_LRR];	// wl_ItfKpisSet.bit6
	u_int	wl_ItfRssiUpdateCount[NB_ITF_PER_LRR];
}	t_lrr_wan;

typedef	struct	LP_STRUCTPACKED	s_lrr_wan_lrc
{
// added in LRC 1.0.26, LRR 1.2.6
	u_char	wl_LrcCount;		// number of LRC declared
	u_char	wl_LrcIndex;
	u_char	wl_LrcState;
	u_int	wl_LrcNbIecUpByte;
	u_int	wl_LrcNbIecUpPacket;
	u_int	wl_LrcNbIecDownByte;
	u_int	wl_LrcNbIecDownPacket;
	u_int	wl_LrcNbDisc;
	u_short	wl_LrcNbPktTrip;
	u_short	wl_LrcAvPktTrip;
	u_short	wl_LrcDvPktTrip;
	u_short	wl_LrcMxPktTrip;
	u_int	wl_LrcMxPktTripTime;	// time of max
}	t_lrr_wan_lrc;

typedef	struct	LP_STRUCTPACKED	s_lrr_upgrade
{
	u_short	up_versMaj;		// LRR v M.m.r
	u_short	up_versMin;
	u_short	up_versRel;
	u_char	up_md5[32+1];
}	t_lrr_upgrade;

typedef	struct	LP_STRUCTPACKED	s_lrr_upgrade_v1
{
	u_short	up_versMaj;		// -V M.m.r
	u_short	up_versMin;
	u_short	up_versRel;
	u_char	up_md5[32+1];		// -M
	u_char	up_ipv4[16+1];		// -A
	u_short	up_port;		// -P
	u_char	up_user[16+1];		// -U
	u_char	up_password[16+1];	// -W
	u_char	up_forced;		// -F
}	t_lrr_upgrade_v1;

typedef	struct	LP_STRUCTPACKED	s_lrr_cmd
{
	u_int	cm_serial;	// serial number, if != 0 ACK is required
	u_int	cm_cref;	// user cross ref data
	u_int	cm_itf;		// interface to return data (1 CLI)
	u_int	cm_tms;		// time of command LRC side
}	t_lrr_cmd;

typedef	struct	LP_STRUCTPACKED	s_lrr_upgrade_cmd
{
	u_char		up_cmd[LP_DATA_CMD_LEN+1];
	t_lrr_cmd	cm_cmd;
}	t_lrr_upgrade_cmd;

typedef	struct	LP_STRUCTPACKED	s_lrr_restart_cmd
{
	u_char		re_cmd[LP_DATA_CMD_LEN+1];
	t_lrr_cmd	cm_cmd;
}	t_lrr_restart_cmd;

typedef	struct	LP_STRUCTPACKED	s_lrr_shell_cmd
{
	u_char		sh_cmd[LP_DATA_CMD_LEN+1];
	t_lrr_cmd	cm_cmd;
}	t_lrr_shell_cmd;

typedef	struct	LP_STRUCTPACKED	s_lrr_cmd_resp
{
	u_char		rp_cmd[LP_DATA_CMD_LEN+1];
	u_char		rp_fmt;		// 0 means rp_cmd is null term string
	u_char		rp_code;	// 0 OK
	u_char		rp_codeemb;	// 0 OK
	u_short		rp_type;	// type of initial command
	u_int		rp_serial;	// serial of initial command
	u_int		rp_cref;	// user cross ref data
	u_int		rp_tms;		// request time msec
	u_int		rp_seqnum;	// sequence number 0..N
	u_char		rp_itf;		// interface (1 CLI)
	u_char		rp_more;	// more data follow
	u_char		rp_continue;	// not a finale answer
}	t_lrr_cmd_resp;

typedef	struct	LP_STRUCTPACKED	s_lrr_beacon_dn
{
	u_int		be_gss;
	u_int		be_gns;
	u_char		be_spfact;
	u_char		be_freq[3];	
	u_char		be_chain;	// antenna indice
	u_char		be_channel;
}	t_lrr_beacon_dn;

typedef	struct	LP_STRUCTPACKED	s_lrr_lrruid
{
	u_char		lu_oui[7];	// IEEE OUI: 3bytes => 6 hexa + 1 end
	u_char		lu_gwuid[LP_DATA_LRRUID_LEN+1];	// gateway ID for OUI
}	t_lrr_lrruid, t_lrr_config_lrruid;

typedef	struct	LP_STRUCTPACKED	s_lrr_twa_lrrid
{
	u_int		tl_nosupport;	// no support of LP_TYPE_LRR_INF_LRRUID
	u_int		tl_lrrid;
}	t_lrr_twa_lrrid;

typedef	struct	LP_STRUCTPACKED	s_lrr_multicast_dn
{
	u_int		mc_gss;
	u_int		mc_gns;
	u_char		mc_spfact;
	u_char		mc_freq[3];
	u_char		mc_chain;	// antenna indice
	u_char		mc_channel;
	u_int64_t	mc_deveui;
	u_short		mc_fcntdn;
}	t_lrr_multicast_dn;

typedef	struct	LP_STRUCTPACKED	s_lrr_sync_inf
{
	u_int		sy_macroLrr;
	u_int		sy_macroGss;
	u_int		sy_macroGns;
	u_char		sy_macroTyp;	// 0 unix time, 1 GPS time, 2 fine timestamp
}	t_lrr_sync_inf;

typedef	struct	LP_STRUCTPACKED	s_lrr_sync_dn
{
	u_int		sy_lrcGss;
	u_int		sy_lrcGns;
	u_int		sy_microTus;
	u_short		sy_microFcntdn;
	u_short		sy_macroCnt;
	u_char		sy_rfu[8];
	t_lrr_sync_inf	sy_macroInf[LP_MAX_LRR_TRACKING];
}	t_lrr_sync_dn;

#define	lp_lrrid	lp_lrxid
#define	lp_lrridext	lp_lrxidext
#define	lp_lrridpref	lp_lrxidpref
#define	lp_lrcid	lp_lrxid
#define	lp_tms		lp_u.lp_radio.lr_tms
#define	lp_tus		lp_u.lp_radio.lr_tus
#define	lp_gss		lp_u.lp_radio.lr_gss
#define	lp_gns		lp_u.lp_radio.lr_gns
#define	lp_chain	lp_u.lp_radio.lr_chain
#define	lp_rssi		lp_u.lp_radio.lr_u.lr_var_up.lr_rssi
#define	lp_snr		lp_u.lp_radio.lr_u.lr_var_up.lr_snr
#define	lp_shiftlc	lp_u.lp_radio.lr_u.lr_var_dn.lr_shiftlc
#define	lp_majorv	lp_u.lp_radio.lr_u.lr_var_dn.lr_majorv
#define	lp_minorv	lp_u.lp_radio.lr_u.lr_var_dn.lr_minorv
#define	lp_rx2spfact	lp_u.lp_radio.lr_u.lr_var_dn.lr_rx2spfact
#define	lp_freqdn	lp_u.lp_radio.lr_u.lr_var_dn.lr_freqdn
#define	lp_opaque	lp_u.lp_radio.lr_u.lr_802_dn.lr_opaque
#define	lp_rx2channel	lp_u.lp_radio.lr_u.lr_var_dn.lr_rx2channel

#define	lp_channel	lp_u.lp_radio.lr_channel
#define	lp_subband	lp_u.lp_radio.lr_subband
#define	lp_spfact	lp_u.lp_radio.lr_spfact
#define	lp_correct	lp_u.lp_radio.lr_correct
#define	lp_bandwidth	lp_u.lp_radio.lr_bandwidth
#define	lp_freq		lp_u.lp_radio.lr_fq_freq
#define	lp_freqoffset	lp_u.lp_radio.lr_fq_offset

#define	lp_deveui	lp_u.lp_radio.lr_u2.lr_ref_dn.lr_deveui
#define	lp_fcntdn	lp_u.lp_radio.lr_u2.lr_ref_dn.lr_fcntdn
#define	lp_devaddr	lp_u.lp_radio.lr_u2.lr_ref_dn.lr_devaddr

#define	lp_oui		lp_u.lp_send_lrruid.lu_oui
#define	lp_gwuid	lp_u.lp_send_lrruid.lu_gwuid
#define	lp_twalrrid	lp_u.lp_twa_lrrid.tl_lrrid
#define	lp_twalrruidns	lp_u.lp_twa_lrrid.tl_nosupport

#define	lp_partitionid	lp_u.lp_partition_id.pi_partition_id

#define	LP_TYPE_LRR_PKT_RADIO			0
#define	LP_TYPE_LRR_INF_LRRID			1
#define	LP_TYPE_LRR_INF_GPSCO			2
#define	LP_TYPE_LRR_INF_CAPAB			3
#define	LP_TYPE_LRR_INF_STATS			4
#define	LP_TYPE_LRR_INF_STATS_V1		5
#define	LP_TYPE_LRR_INF_UPGRADE			6
#define	LP_TYPE_LRR_INF_UPGRADE_V1		7	// LRR >= 1.0.51
#define	LP_TYPE_LRR_INF_UPGRADE_CMD		8	// LRR >= 1.0.51
#define	LP_TYPE_LRR_INF_RESTART_CMD		9	// LRR >= 1.0.51
#define	LP_TYPE_LRR_INF_SHELL_CMD		10	// LRR >= 1.0.55
#define	LP_TYPE_LRR_INF_RFCELL			11	// LRR >= 1.0.59
#define	LP_TYPE_LRR_INF_CONFIG			12	// LRR >= 1.0.59
#define	LP_TYPE_LRR_INF_CONFIG_LRC		13	// LRR >= 1.2.4
#define	LP_TYPE_LRR_INF_WAN			14	// LRR >= 1.2.7
#define	LP_TYPE_LRR_INF_WAN_LRC			15	// LRR >= 1.2.7
#define	LP_TYPE_LRR_INF_RADIO_STOP_CMD		16	// LRR >= 1.2.7
#define	LP_TYPE_LRR_INF_RADIO_START_CMD		17	// LRR >= 1.2.7
#define	LP_TYPE_LRR_INF_DNRADIO_STOP_CMD	18	// LRR >= 1.2.7
#define	LP_TYPE_LRR_INF_DNRADIO_START_CMD	19	// LRR >= 1.2.7
#define	LP_TYPE_LRR_INF_CMD_RESPONSE		20	// LRR >= 1.2.7
#define	LP_TYPE_LRR_PKT_RADIO_TEST		21	// LRR == 255.255.255
#define	LP_TYPE_LRR_INF_POWER_UP		22	// LRR >= 1.4.18
#define	LP_TYPE_LRR_INF_POWER_DOWN		23	// LRR >= 1.4.18
#define	LP_TYPE_LRR_PKT_SENT_INDIC		24	// LRR >= 1.8.x
#define	LP_TYPE_LRR_PKT_RADIO_LATE		25	// LRR >= 1.8.x
#define	LP_TYPE_LRR_PKT_DTC_SYNCHRO		26	// LRR >= 1.8.x
#define LP_TYPE_LRR_INF_CONFIG_LOCKEY		27	// LRR >= 2.0.x
#define LP_TYPE_LRR_INF_CONFIG_ANTS		28	// LRR >= 2.0.x
#define LP_TYPE_LRR_INF_BEACON_DN		29	// LRR => 2.2.x
#define LP_TYPE_LRR_INF_CUSTOM_VERSION		30	// LRR >= 2.2.x
#define	LP_TYPE_LRR_INF_LRRUID			31	// LRR >= 2.2.x
#define	LP_TYPE_LRR_INF_TWA_LRRID		32	// LRR >= 2.2.x
#define	LP_TYPE_LRR_INF_CONFIG_LRRUID		33	// LRR >= 2.2.x
#define LP_TYPE_LRR_INF_GPS_UP			34	// LRR >= 2.2.31
#define LP_TYPE_LRR_INF_GPS_DOWN		35	// LRR >= 2.2.31
#define LP_TYPE_LRR_PKT_MULTICAST_DN		36	// LRR => 2.2.40
#define	LP_TYPE_LRR_INF_PARTITIONID		37	// LRR => 2.2.44
#define	LP_TYPE_LRR_INF_NETWORKFILTER_REQ	38	// LRR => 2.4.8
#define	LP_TYPE_LRR_INF_NETWORKFILTER_RESP	39	// LRR => 2.4.8
#define LP_TYPE_LRR_PKT_DLSYNC			40	// LRR => 2.4.61
#define	LP_TYPE_LRR_INF_BACKUP_DISC		41	// LRC => LRC
#define	LP_TYPE_LRR_INF_BACKUP_RF_STATE		42	// LRC => LRC
#define	LP_TYPE_LRR_INF_BACKUP_RESET_PKT_STAT	43	// LRC => LRC
#define LP_TYPE_LRR_INF_XLAP_CFG		44	// LRR => 2.6.5 [2359]
#define	LP_TYPE_LRR_INF_MASTER_DISC		45	// LRC => LRC [13474]
#define	LP_TYPE_LRR_INF_BACKUP_CREATED_OK	46	// LRC => LRC [13474]
#define	LP_TYPE_LRR_INF_BACKUP_CREATED_ERR	47	// LRC => LRC [13474]

#ifdef	LP_MODULE_LRC

#define	LP_TYPE_MTC_PKT_UP			1000	// MTC
#define	LP_TYPE_MTC_PKT_DOWN			1001	// MTC

#include	"MTC_msg.h"

#endif

// [2359]
typedef struct  LP_STRUCTPACKED s_lrr_xlap_cfg
{
	u_int	xc_t1;
	u_int	xc_t2;
	u_int	xc_t3;
}	t_lrr_xlap_cfg;

typedef struct  LP_STRUCTPACKED s_lrr_partition_id
{
	u_char          pi_partition_id;
}	t_lrr_partition_id;

// LRR => LRC
typedef struct  LP_STRUCTPACKED s_lrr_networkfilterReq
{
	u_int	nf_version;
	u_int	nf_netid;
}	t_lrr_networkfilterReq;

// LRC => LRR
typedef struct  LP_STRUCTPACKED s_lrr_networkfilterResp
{
	u_int	nf_version;
	u_int	nf_netid;
#define NF_TYPE_FILTER		0
#define NF_TYPE_FILENAME	1
	u_char	nf_type;
	u_char	nf_partno;
	u_char	nf_nbpart;
#define NF_FILTER_PART_SIZE	256
	char	nf_filter[NF_FILTER_PART_SIZE+1];
}	t_lrr_networkfilterResp;


typedef	struct	LP_STRUCTPACKED	s_lrr_pkt
{
	// following members are part of LAP/LRC/LRR network messages
	// we assume that : 
	// 	- short/int are coded in little endian 
	// 	- float are coded with IEEE754/32bits
	// 	- structures are packed (see use of LP_STRUCTPACKED)
	u_char	lp_vers;	// protocole version (0|1)
	u_char	lp_rfu;		// not used
	u_short	lp_szh;		// size before lp_payload : (20) + union(lp_u)
	u_short	lp_flag;	// various flags
	u_short	lp_type;	// type of message/nature of union(lp_u)
	u_int	lp_lrxid;	// LRC or LRR or MTC id 
	u_int	lp_delay;	// waiting time on the destination msec
	u_short	lp_lrxidext;	// +2B to extend LRRID (ie:mac@) useless now
	u_char	lp_lrxidpref;	// prefixe for type of LRRID (Lora|MTC)
	u_char	lp_flagm;

	union
	{
		t_lrr_gpsco		lp_gpsco;
		t_lrr_capab		lp_capab;
		t_lrr_pkt_radio_p1	lp_radio_p1;
		t_lrr_pkt_radio		lp_radio;
		t_lrr_pkt_sent_indic	lp_sent_indic;
		t_lrr_stat		lp_stat;
		t_lrr_stat_v1		lp_stat_v1;
		t_lrr_rfcell		lp_rfcell;
		t_lrr_config		lp_config;
		t_lrr_config_lrc	lp_config_lrc;
		t_lrr_config_lockey	lp_config_lockey;
		t_lrr_config_ants	lp_config_ants;
		t_lrr_config_lrruid	lp_config_lrruid;
		t_lrr_custom_ver	lp_custom_ver;
		t_lrr_wan		lp_wan;
		t_lrr_wan_lrc		lp_wan_lrc;
		t_lrr_upgrade		lp_upgrade;
		t_lrr_upgrade_v1	lp_upgrade_v1;
		t_lrr_upgrade_cmd	lp_upgrade_cmd;
		t_lrr_restart_cmd	lp_restart_cmd;
		t_lrr_shell_cmd		lp_shell_cmd;
		t_lrr_cmd_resp		lp_cmd_resp;
		t_lrr_power		lp_power;
#ifdef	LP_MODULE_LRC
		t_mtc_pkt_up		lp_mtc_pkt_up;
		t_mtc_pkt_down		lp_mtc_pkt_down;
#endif
		t_lrr_beacon_dn		lp_beacon_dn;
		t_lrr_lrruid		lp_send_lrruid;
		t_lrr_twa_lrrid		lp_twa_lrrid;
		t_lrr_gps_st		lp_gps_st;
		t_lrr_multicast_dn	lp_multicast_dn;
		t_lrr_partition_id	lp_partition_id;
		t_lrr_networkfilterReq	lp_networkfilterReq;
		t_lrr_networkfilterResp	lp_networkfilterResp;
		t_lrr_sync_dn		lp_sync_dn;
		t_lrr_xlap_cfg		lp_xlap_cfg;	// [2359]
	}	lp_u;

	// following members are not part of LAP/LRC/LRR network messages
	// but are used by all modules LRC LRR LRR simulator ...
	// lp_payload must be the first of these members
	u_char	*lp_payload;
	u_short	lp_size;		// size of lp_payload
	u_char	lp_optqos;		// has been optimized
	void	*lp_lk;			// (t_xlap_link *)

	// following members are used by LRC modules only
#ifdef	LP_MODULE_LRC
	struct s_lrr_link	*lp_lrr;	// (t_lrr_link *)
	u_int64_t	lp_lrriid; // (lrxidpref<<48)+(lrxidext<<32)+(lp_lrxid)
	u_int		lp_qosid;
	u_int		lp_nbqos;
	t_lrr_qos	lp_qos[LP_MAX_LRR_TRACKING];
	u_int		lp_nbtrk;
	t_lrr_qos	lp_trk[LP_MAX_LRR_TRACKING*LP_MAX_ANT_TRACKING];
	u_short		FCntUp;
	u_short		FCntDn;
	u_char		lp_samepayload;
	u_char		lp_samepacket;
	u_char		lp_repeated;
	float		lp_esp;		// estimated signal power NFR182
	u_int32_t	lp_devaddrv2; // only used when LRCv2. RAN passes devaddr to DEV with this
	u_char		lp_mongoChecked; // only used when LRCv2.
	FNSInfo_t	*FNSInfo;
	u_char		lp_src_backup;	// comes from LRC backup RDTP-11708
#endif

	// following members are used by LRR modules only
#ifdef	LP_MODULE_LRR
	u_int	lp_postponed;		// postponded in mainQ
	u_int	lp_trip;		// round trip LRR -> LRC -> LRR
	float	lp_tmoa;		// time on air
	u_int	lp_lgwdelay;		// delayed by lgw/sx13xx
	char	*lp_cmdname;		// command name to execute 
	char	*lp_cmdfull;		// full command to execute name+params
	int	lp_cmdtype;
	u_char	lp_delivered;
	u_char	lp_cause1;
	u_char	lp_cause2;
	u_char	lp_cause3;
	u_char	lp_beacon;
	u_char	lp_classb;
	u_short	lp_firstslot;
	u_short	lp_firstslot2;
	u_short	lp_nbslot;
	u_short	lp_currslot;
	u_int	lp_gss0;
	u_int	lp_gns0;
	u_short	lp_idxtry;
	u_short	lp_nbtry;
	u_short	lp_maxtry;		
	u_char	lp_period;		// current period for classb (1 or 2)
	u_int	lp_sidx;
	float	lp_sdur;
	u_char	lp_bypasslbt;		// classC are sent in TIMESTAMPED mode
	u_char	lp_stopbylbt;
	u_char	lp_rx2lrr;		// LRR chooses RX2 windows
	u_char	lp_classc;		// ~ lp_delay == 0
	u_char	lp_classcmc;		// classc multicast => on pps
	u_char	lp_classbchan2;		// chan for second period RDTP-2983
	u_char	lp_synchro;		// uplink packet to synchronise gateways
	int32_t	lp_misswin;		// delay in ms we missed the sending window
#endif
}	t_lrr_pkt;


#ifdef	LP_MODULE_LRR_SIMUL

#define	STAT_NO_CRC	0
#define	STAT_CRC_BAD	1
#define	STAT_CRC_OK	2


struct lgw_pkt_rx_s {
uint32_t	freq_hz;	/*!> central frequency of the IF chain */
uint8_t		if_chain;	/*!> by which IF chain was packet received */
uint8_t		status;		/*!> status of the received packet */
uint8_t		modulation; /*!> modulation used by the packet */
uint8_t		bandwidth;	/*!> modulation bandwidth (Lora only) */
uint16_t	datarate;	/*!> RX datarate of the packet */
uint8_t		coderate;	/*!> error-correcting code of the packet */
uint32_t	count_us;	/*!> internal gateway counter for timestamping*/
float		rssi;		/*!> average packet RSSI in dB */
float		snr;		/*!> average packet SNR, in dB (Lora only) */
float		snr_min;	/*!> minimum packet SNR, in dB (Lora only) */
float		snr_max;	/*!> maximum packet SNR, in dB (Lora only) */
uint16_t	crc;		/*!> CRC that was received in the payload */
uint16_t	size;		/*!> payload size in bytes */
uint8_t		payload[256];	/*!> pointer to the payload */
};
struct lgw_pkt_tx_s {
uint32_t	freq_hz;	/*!> center frequency of TX */
uint8_t		tx_mode;	/*!> select on what event/time the TX is */
uint32_t	count_us;	/*!> timestamp or delay in microseconds igger */
uint8_t		rf_chain;	/*!> through which RF chain will the packet */
int8_t		rf_power;	/*!> TX power, in dBm */
uint8_t		modulation;	/*!> modulation to use for the packet */
uint8_t		bandwidth;	/*!> modulation bandwidth (Lora only) */
uint8_t		invert_pol;	/*!> invert signal polarity(Lora only) */
uint16_t	f_dev;		/*!> frequency deviation (FSK only) */
uint16_t	datarate;	/*!> TX datarate */
uint8_t		coderate;	/*!> error-correcting code of the packet */
uint16_t	preamble;	/*!> set the preamble length, 0 for default */
uint8_t		no_crc;		/*!> if true, do not send a CRC in the packet */
uint8_t		no_header;	/*!> if true, enable implicit header mode */
uint16_t	size;		/*!> payload size in bytes */
uint8_t		payload[256];	/*!> pointer to the payload */
};

#endif
#endif
