
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

/*! @file leds.c
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>

#include "rtlbase.h"

#if defined(KEROS) && defined(WITH_LED_MGT)

#define LED_TX_LORA		"/sys/class/leds/led3:red:lora"
#define LED_RX_LORA		"/sys/class/leds/led3:green:lora"
#define LED_NOK_BACKHAUL	"/sys/class/leds/led2:red:backhaul"
#define LED_BACKHAUL		"/sys/class/leds/led2:green:backhaul"
#define BUFFER_MAX_SIZE		128

int LedWrite(char *path, char *str)
{
	int ret = 0;
	int fd = -1;

	fd = open(path, O_WRONLY);
	if (fd >= 0)
	{
		if (write(fd, str, strlen(str)) < 0)
		{
			RTL_TRDBG(0, "Unable to write in '%s'\n", path);
			ret = -1;
		}
		close(fd);
	}
	else
	{
		RTL_TRDBG(0, "Unable to open '%s' (%s)\n", path, strerror(errno));
		ret = -1;
	}
	return ret;
}

int LedConfigure()
{
	LedWrite(LED_TX_LORA"/trigger", "oneshot");
	LedWrite(LED_RX_LORA"/trigger", "oneshot");

	return 0;
}

int LedShotTxLora()
{
	LedWrite(LED_TX_LORA"/shot", "1");
	return 0;
}

int LedShotRxLora()
{
	LedWrite(LED_RX_LORA"/shot", "1");
	return 0;
}

int LedBackhaul(int val)
{
	static int	previous = -1;

	if (val == previous)
		return 0;

	switch (val)
	{
		case 0:
			LedWrite(LED_BACKHAUL"/trigger", "none");
			LedWrite(LED_BACKHAUL"/brightness", "0");
			LedWrite(LED_NOK_BACKHAUL"/trigger", "none");
			LedWrite(LED_NOK_BACKHAUL"/brightness", "0");
			previous = 0;
			break;
		case 1:
			LedWrite(LED_NOK_BACKHAUL"/trigger", "timer");
			LedWrite(LED_BACKHAUL"/brightness", "0");
			previous = 1;
			break;
		case 2:
			LedWrite(LED_BACKHAUL"/brightness", "1");
			LedWrite(LED_NOK_BACKHAUL"/trigger", "none");
			LedWrite(LED_NOK_BACKHAUL"/brightness", "0");
			previous = 2;
			break;
	}
	return 0;
}

#endif	// KEROS and LED Mgt
