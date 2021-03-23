
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


#include	"mac802154.h"
#include	"macip6.h"


int	mac802154_ip6type(mac802154_t *mf)
{
	unsigned char	*l6;

	if	(mf->fcf.frame_type != FRAME802154_DATAFRAME)
		return	-1;

	l6	= mf->payload;
	if	(mf->payload_len >= 6
			&& l6[0] == 0x7b && l6[1] == 0x3b
			&& l6[2] == 0x3a && l6[3] == 0x02
			&& l6[4] == 0x85)
	{
		return	MACIP6_ROUTER_SOLI;
	}
	if	(mf->payload_len >= 6 
			&& l6[0] == 0x7a && l6[1] == 0x3b
			&& l6[2] == 0x3a && l6[3] == 0x1a
			&& l6[4] == 0x9b && l6[5] == 0x00)
	{
		return	MACIP6_RPL_DIS;
	}
	if	(mf->payload_len >= 6
			&& l6[0] == 0x7a && l6[1] == 0x3b
			&& l6[2] == 0x3a && l6[3] == 0x1a
			&& l6[4] == 0x9b && l6[5] == 0x01)
	{
		return	MACIP6_RPL_DIO;
	}
	if	(mf->payload_len >= 6
			&& l6[0] == 0x7a && l6[1] == 0x33
			&& l6[2] == 0x3a && l6[3] == 0x9b 
			&& l6[4] == 0x02)
	{
		return	MACIP6_RPL_DAO;
	}
	if	(mf->payload_len >= 6
			&& l6[0] == 0x7a && l6[1] == 0x33
			&& l6[2] == 0x3a && l6[3] == 0x9b 
			&& l6[4] == 0x03)
	{
		return	MACIP6_RPL_DAOACK;
	}
	if	(mf->payload_len >= 6
			&& l6[0] == 0x7a && l6[1] == 0x33
			&& l6[2] == 0x3a && l6[3] == 0x80)
	{
		return	MACIP6_PING_REQUEST;
	}
	if	(mf->payload_len >= 6
			&& l6[0] == 0x7a && l6[1] == 0x33
			&& l6[2] == 0x3a && l6[3] == 0x81)
	{
		return	MACIP6_PING_REPLY;
	}
	return	0;
}

char	*mac802154_ip6typetxt(int type)
{
	switch	(type)
	{
	case	-1	:
		return	"MACIP6 not a mac data frame";
	break;
	case	MACIP6_ROUTER_SOLI	:
		return	"MACIP6_ROUTER_SOLI";
	break;
	case	MACIP6_RPL_DIS	:
		return	"MACIP6_RPL_DIS";
	break;
	case	MACIP6_RPL_DIO	:
		return	"MACIP6_RPL_DIO";
	break;
	case	MACIP6_RPL_DAO	:
		return	"MACIP6_RPL_DAO";
	break;
	case	MACIP6_RPL_DAOACK:
		return	"MACIP6_RPL_DAOACK";
	break;

	case	MACIP6_PING_REQUEST:
		return	"MACIP6_PING_REQUEST";
	break;
	case	MACIP6_PING_REPLY:
		return	"MACIP6_PING_REPLY";
	break;

	default :
		return	"MACIP6 type ???";
	break;
	}
	return	"";
}
