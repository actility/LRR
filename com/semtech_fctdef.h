
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

/* loragw_hal.h */

int (*tlgw_board_setconf[MAX_BOARD])(struct lgw_conf_board_s conf);
int (*tlgw_lbt_setconf[MAX_BOARD])(struct lgw_conf_lbt_s conf);
int (*tlgw_rxrf_setconf[MAX_BOARD])(uint8_t rf_chain, struct lgw_conf_rxrf_s conf);
int (*tlgw_rxif_setconf[MAX_BOARD])(uint8_t if_chain, struct lgw_conf_rxif_s conf);
int (*tlgw_txgain_setconf[MAX_BOARD])(struct lgw_tx_gain_lut_s *conf);
#if defined(TRACKNET) || defined(HAL_HAVE_LGW_START_WITH_DEVICE) // TRACKNET: to be removed
int (*tlgw_start[MAX_BOARD])(void *path);
#else
int (*tlgw_start[MAX_BOARD])(void);
#endif
int (*tlgw_stop[MAX_BOARD])(void);
int (*tlgw_receive[MAX_BOARD])(uint8_t max_pkt, struct lgw_pkt_rx_s *pkt_data);
int (*tlgw_send[MAX_BOARD])(struct lgw_pkt_tx_s pkt_data);
int (*tlgw_status[MAX_BOARD])(uint8_t select, uint8_t *code);
int (*tlgw_abort_tx[MAX_BOARD])(void);
int (*tlgw_get_trigcnt[MAX_BOARD])(uint32_t* trig_cnt_us);
const char* (*tlgw_version_info[MAX_BOARD])(void);
uint32_t (*tlgw_time_on_air[MAX_BOARD])(struct lgw_pkt_tx_s *packet);

/* loragw_reg.h */

#ifdef HAL_HAVE_LGW_CONNECT_WITH_DEVICE
int (*tlgw_connect[MAX_BOARD])(bool spi_only, uint32_t tx_notch_freq, void *spi_path);
#else
int (*tlgw_connect[MAX_BOARD])(bool spi_only, uint32_t tx_notch_freq);
#endif
int (*tlgw_disconnect[MAX_BOARD])(void);
int (*tlgw_soft_reset[MAX_BOARD])(void);
int (*tlgw_reg_check[MAX_BOARD])(FILE *f);
int (*tlgw_reg_w[MAX_BOARD])(uint16_t register_id, int32_t reg_value);
int (*tlgw_reg_r[MAX_BOARD])(uint16_t register_id, int32_t *reg_value);
int (*tlgw_reg_wb[MAX_BOARD])(uint16_t register_id, uint8_t *data, uint16_t size);
int (*tlgw_reg_rb[MAX_BOARD])(uint16_t register_id, uint8_t *data, uint16_t size);

#ifdef WITH_GPS
/* loragw_gps.h */
#ifdef HAL_VERSION_5
int (*tlgw_gps_sync[MAX_BOARD])(struct tref* ref, uint32_t count_us, struct timespec utc, struct timespec gps_time);
#else
int (*tlgw_gps_sync[MAX_BOARD])(struct tref* ref, uint32_t count_us, struct timespec utc);
#endif
int (*tlgw_cnt2utc[MAX_BOARD])(struct tref ref, uint32_t count_us, struct timespec* utc);
int (*tlgw_utc2cnt[MAX_BOARD])(struct tref ref,struct timespec utc, uint32_t* count_us);
#endif
