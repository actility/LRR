
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


#ifndef LORAMAC_H
#define LORAMAC_H

#include <stdint.h>
#include <stdbool.h>




typedef struct {
	uint8_t		MType;		// 3-bits
	uint8_t		RFU;		// 3-bits
	uint8_t		Major;		// 2-bits
} LoRaMAC_MHDR_t;

typedef struct {
	uint8_t		ADR;		// 1-bit
	uint8_t		ADRACKReq;	// 1-bit
	uint8_t		ACK;		// 1-bit
	uint8_t		FPending;	// 1-bit
	uint8_t		FOptsLen;	// 4-bits
} LoRaMAC_FCtrl_t;

typedef struct {
	LoRaMAC_MHDR_t	MHDR;
	uint8_t		MIC[4];
	uint8_t		DevAddr[4];
	LoRaMAC_FCtrl_t		FCtrl;
	uint32_t	FCnt;		// 16-bits sent but 32 bits used for MIC calculation and cryptage
#define	LORA_MAX_OPTS	15
	uint8_t		FOpts[LORA_MAX_OPTS];
	uint8_t		FPort;
	bool		updown;
#define	LORA_UpLink	0
#define	LORA_DownLink	1
	bool		payload_encrypted;
	uint8_t		*payload;
	uint8_t		payload_len;
	uint8_t		flags;
#define LORA_FPort_Absent	0x1
	uint32_t	Delay;
#define LORAMAC_R2	2
#define LORAMAC_R3	3
	int specificationVersion;
} LoRaMAC_t;


int LoRaMAC_decodeHeader(uint8_t *data, uint32_t len, LoRaMAC_t *lora);

#endif
