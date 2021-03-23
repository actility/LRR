
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

/* contains information collected with "configure" */
#include "autoconfig.h"

#define	STRERRNO	strerror(errno)
#define	MSTIC		1
#define ABS(x)		((x) > 0 ? (x) : -(x))
#ifndef MIN
#define	MIN(a, b)	((a) < (b) ? (a) : (b))
#endif


#ifndef WEXITSTATUS
#define WEXITSTATUS(s) (((s) & 0xff00) >> 8)
#endif

#define	LP_LRX_VERSION	1

#define	SERVICE_NAME		"lora-relay"
#define	SERVICE_STATUS_FILE	"/tmp/lora-relay.status"

// [2359]
#define	LRR_DEFAULT_T1	60		// DEFAULT_T1 == 15
#define	LRR_DEFAULT_T2	DEFAULT_T2	// DEFAULT_T2 == 10
#define	LRR_DEFAULT_T3	DEFAULT_T3	// DEFAULT_T3 == 20

#define	NB_ANTENNA	3
#define	ANTENNA_GAIN_MAX 127

#define	NB_CHANNEL		255	//	0..254
#define	UNK_CHANNEL		255
#define	MAXUP_CHANNEL_IDX	126	// >= 127 => downklink channels

#define	LGW_RECV_PKT	4

#define	IM_DEF			1000
#define	IM_TIMER_GEN		1001
#define					IM_TIMER_GEN_V		10000	// ms

#define	IM_TIMER_BEACON		1002
#define					IM_TIMER_BEACON_V	60000	// ms

#define	IM_TIMER_LRRUID_RESP	1003
#define					IM_TIMER_LRRUID_RESP_V	5000	// ms

#define	IM_SERVICE_STATUS_RQST	1100

#define	IM_LGW_CONFIG_FAILURE	1200	// LGW -> SPV
#define	IM_LGW_START_FAILURE	1201	// LGW -> SPV
#define	IM_LGW_STARTED		1202	// LGW -> SPV
#define	IM_LGW_LINK_UP		1203	// LGW -> SPV
#define	IM_LGW_LINK_DOWN	1204	// LGW -> SPV
#define	IM_LGW_SEND_DATA	1205	// SPV -> LGW
#define	IM_LGW_RECV_DATA	1206	// LGW -> SPV
#define	IM_LGW_GPS_TIME		1207	// SPV -> LGW
#define	IM_LGW_POST_DATA	1208	// SPV -> SPV then IM_LGW_SEND_DATA
#define	IM_LGW_DELAY_ALLLRC	1209	// SPV -> SPV
#define	IM_LGW_EXIT		1210	// SPV -> LGW
#define	IM_LGW_SENT_INDIC	1211	// LGW -> SPV
#define	IM_LGW_SYNC_TIME	1212	// SPV -> LGW	RDTP-5475

#define	IM_CMD_RECV_DATA	1300	// CMD -> SPV

#define	IM_DC_LIST		1400	// Duty cycle

#define	MAX_BOARDS		4	// Maximum of boards

#if !defined(SYSTEM_NAME)

/* Semtech Ref Design version, based on gw platform. New: changing to base it on configured Semtech ref design*/
#if defined(CISCOMS) || defined(FCLOC) || defined(SEMTECH_V2_1)
#define REF_DESIGN_V2 1
#undef  REF_DESIGN_V1
#else
#define REF_DESIGN_V1 1
#undef  REF_DESIGN_V2
#endif

#else /* SYSTEM_NAME */

#if defined(SEMTECH_V2_1)
#define REF_DESIGN_V2 1
#undef  REF_DESIGN_V1
#else
#define REF_DESIGN_V1 1
#undef  REF_DESIGN_V2
#endif

#endif /* SYSTEM_NAME */

#ifdef REF_DESIGN_V2
#define LGW_GPS_SUCCESS  0
#define LGW_GPS_ERROR   -1
#define	LGW_COORD_T	sx1301ar_coord_t
#define	LGW_GPSMSG_T	sx1301ar_nmea_msg_t
#ifdef HAL_VERSION_5
#define LGW_UBXMSG_T	sx1301ar_ubx_msg_t
#endif /* HAL_VERSION_5 */
#else
#define	LGW_COORD_T	struct coord_s
#define	LGW_GPSMSG_T	enum gps_msg
#endif /* REF_DESIGN_V2 */

#define GPS_REF_MAX_AGE 30

/* LBT Feature availability
 *    - For GW_V2, LBT feature comes since HAL >= 4.0.0 (SX1301AR_LBT_CHANNEL_NB_MAX)
 */

/* for new mechanism: do not change the setting done by configuration */
#if ! defined(SYSTEM_NAME)

#if defined(LGW_LBT_ISSUE) || defined(SX1301AR_LBT_CHANNEL_NB_MAX)
#define WITH_LBT 1
#else
/* gateway V1.0 or with old HAL */
#if ((defined(FCPICO) && !defined(HAL_VERSION_5)) ||defined(FCLAMP) || defined(NATRBPI)) && defined(WITH_LBT)
#undef	WITH_LBT
#endif
#endif

#endif /* SYSTEM_NAME */

#ifdef	STM32FWVERSION	// pico lora lib => no lbt no gps
#undef	WITH_LBT
#undef	WITH_GPS
#endif

#ifdef	WITH_TTY	// pico lora lib => no lbt no gps
#undef	WITH_LBT
#undef	WITH_GPS
#endif

#ifdef REF_DESIGN_V1
#ifdef	WIRMAMS
#define	LGW_BOARD_SETCONF(p1,...)	lgw_board_setconf(p1,__VA_ARGS__)
#define	LGW_GET_TRIGCNT(p1,...)		lgw_get_trigcnt(p1,__VA_ARGS__)
#define	LGW_RECEIVE(p1,...)		lgw_receive(p1,__VA_ARGS__)
#define	LGW_REG_CHECK(p1,...)		lgw_reg_check(p1,__VA_ARGS__)
#define	LGW_REG_RB(p1,...)		lgw_reg_rb(p1,__VA_ARGS__)
#define	LGW_REG_W(p1,...)		lgw_reg_w(p1,__VA_ARGS__)
#define	LGW_RXIF_SETCONF(p1,...)	lgw_rxif_setconf(p1,__VA_ARGS__)
#define	LGW_RXRF_SETCONF(p1,...)	lgw_rxrf_setconf(p1,__VA_ARGS__)
#define	LGW_START(p1)			lgw_start(p1)
#define	LGW_STATUS(p1,...)		lgw_status(p1,__VA_ARGS__)
#define	LGW_STOP(p1)			lgw_stop(p1)
#define	LGW_TXGAIN_SETCONF(p1,...)	lgw_txgain_setconf(p1,__VA_ARGS__)
#else
#define	LGW_BOARD_SETCONF(p1,...)	lgw_board_setconf(__VA_ARGS__)
#define	LGW_GET_TRIGCNT(p1,...)		lgw_get_trigcnt(__VA_ARGS__)
#define	LGW_RECEIVE(p1,...)		lgw_receive(__VA_ARGS__)
#define	LGW_REG_CHECK(p1,...)		lgw_reg_check(__VA_ARGS__)
#define	LGW_REG_RB(p1,...)		lgw_reg_rb(__VA_ARGS__)
#define	LGW_REG_W(p1,...)		lgw_reg_w(__VA_ARGS__)
#define	LGW_RXIF_SETCONF(p1,...)	lgw_rxif_setconf(__VA_ARGS__)
#define	LGW_RXRF_SETCONF(p1,...)	lgw_rxrf_setconf(__VA_ARGS__)
#define	LGW_START(p1)			lgw_start()
#define	LGW_STATUS(p1,...)		lgw_status(__VA_ARGS__)
#define	LGW_STOP(p1)			lgw_stop()
#define	LGW_TXGAIN_SETCONF(p1,...)	lgw_txgain_setconf(__VA_ARGS__)
#endif /* WIRMAMS */
#endif /* REF_DESIGN_V1 */
