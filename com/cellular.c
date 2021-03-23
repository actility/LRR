
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

/*! @file cellular.c
 *
 * This thread is only for managing AT commands on cellular device, to get informations
 * It controls open, read, decode and close operation.
 * Once the data are read, they are available in the other threads to be used.
 */

#define  CELLULAR_C
#ifdef  CELLULAR_C
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <pthread.h>
#include <errno.h>
#include <ctype.h>
#ifndef MACOSX
#include <malloc.h>
#endif

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <termios.h>

#include "semtech.h"
#include "rtlbase.h"

#include "define.h"
#include "infrastruct.h"
#include "struct.h"
#include "extern.h"

t_cell_infos	*CellState=NULL;
char			*CellStateDev = "/dev/ttyUSB2";		/* Default cellular device that report state of the connection */
int				CellStateReport = 1;				// activate the feature of cellular infos reporting
t_cell_indic	*CellStateSample = NULL;			// store values to make an average
int				CellStateSampleSize = 10;			// max values stored
int				CellStateSampleCount = 0;			// number of values actually stored
int				CellStateSampleIdx = 0;				// next index to use
int				CellStateSampleTimeout = 120;		// expiration duration in seconds for samples


static int		CSFd = -1;			// fd on device that reports informations continously (each 10 s)

static int		_CSDeviceStatus = 0;
static int		_CSThreadStatus = 0;

static int		count_thread_cs;

static struct   termios cs_ttyopt_restore; /* cellular serial port options used for saving/restoring */

static void		cleanup_thread_cs(void * a);
static void		CSMainLoop();
static void		CSUpdate();
static void		CSDebugTermios(struct termios ttyopt, unsigned int verbose_lvl);

enum {
	CS_DEVICE_STATUS_STOPPED = 0,
	CS_DEVICE_STATUS_STARTED = 1,
	CS_DEVICE_MAX
};

// dump a buffer in hexa
static void DebugFrameHexdump(char * buf, int bufsize) {
	static char hex[2048] = {0};
	static char tmp[4] = {0};
	int i;
	if (buf) {
		memset(hex, 0, sizeof (hex));
		memset(tmp, 0, sizeof (tmp));
		sprintf(hex, "frame hexdump [%02d bytes]: ", bufsize);
		for (i = 0; i < bufsize; i++) {
			sprintf(tmp, "%02X ", *(buf + i));
			strcat(hex, tmp);
		}
		RTL_TRDBG(4, "%s\n", hex);
	}
}

// display termio options
static void CSDebugTermios(struct termios ttyopt, unsigned int verbose_lvl)
{
	RTL_TRDBG(verbose_lvl, "CellState tty flags CLOCAL = %d\n", (ttyopt.c_cflag & CLOCAL)?1:0);
	RTL_TRDBG(verbose_lvl, "CellState tty flags CREAD  = %d\n", (ttyopt.c_cflag & CREAD)?1:0);
	RTL_TRDBG(verbose_lvl, "CellState tty flags CS8	= %d\n", (ttyopt.c_cflag & CS8)?1:0);
	RTL_TRDBG(verbose_lvl, "CellState tty flags PARENB = %d\n", (ttyopt.c_cflag & PARENB)?1:0);
	RTL_TRDBG(verbose_lvl, "CellState tty flags CSTOPB = %d\n", (ttyopt.c_cflag & CSTOPB)?1:0);
	RTL_TRDBG(verbose_lvl, "CellState tty flags IGNPAR = %d\n", (ttyopt.c_iflag & IGNPAR)?1:0);
	RTL_TRDBG(verbose_lvl, "CellState tty flags ICRNL  = %d\n", (ttyopt.c_iflag & ICRNL)?1:0);
	RTL_TRDBG(verbose_lvl, "CellState tty flags IGNCR  = %d\n", (ttyopt.c_iflag & IGNCR)?1:0);
	RTL_TRDBG(verbose_lvl, "CellState tty flags IXON   = %d\n", (ttyopt.c_iflag & IXON)?1:0);
	RTL_TRDBG(verbose_lvl, "CellState tty flags IXOFF  = %d\n", (ttyopt.c_iflag & IXOFF)?1:0);
	RTL_TRDBG(verbose_lvl, "CellState tty flags ICANON = %d\n", (ttyopt.c_lflag & ICANON)?1:0);
	RTL_TRDBG(verbose_lvl, "CellState tty flags ISIG   = %d\n", (ttyopt.c_lflag & ISIG)?1:0);
	RTL_TRDBG(verbose_lvl, "CellState tty flags IEXTEN = %d\n", (ttyopt.c_lflag & IEXTEN)?1:0);
	RTL_TRDBG(verbose_lvl, "CellState tty flags ECHO   = %d\n", (ttyopt.c_lflag & ECHO)?1:0);
	RTL_TRDBG(verbose_lvl, "CellState tty flags ECHOE  = %d\n", (ttyopt.c_lflag & ECHOE)?1:0);
	RTL_TRDBG(verbose_lvl, "CellState tty flags ECHOK  = %d\n", (ttyopt.c_lflag & ECHOK)?1:0);
	RTL_TRDBG(verbose_lvl, "CellState tty flags cc[VMIN] = %d\n", ttyopt.c_cc[VMIN]);
	RTL_TRDBG(verbose_lvl, "CellState tty flags cc[VTIME] = %d\n", ttyopt.c_cc[VTIME]);
}

// set the attributes to read on the device
static int CSEnable(int *fd_cs)
{
	int				x;
	struct termios	ttyopt; /* serial port options */

	/* Save current TTY serial port parameters */
	RTL_TRDBG(4, "CS saving current terminal attributes\n");
	x = tcgetattr(*fd_cs, &cs_ttyopt_restore);
	if (x != 0) {
		RTL_TRDBG(0, "ERROR: failed to get CellState serial port parameters\n");
		return -1;
	}
	/* Get TTY serial port parameters */
	x = tcgetattr(*fd_cs, &ttyopt);
	if ( x != 0 )
	{
		RTL_TRDBG(0, "ERROR: failed to get CellState serial port parameters\n");
		return -1;
	}

	/* Update TTY terminal parameters */
	/* The following configuration should allow to:
		- Get ASCII NMEA messages
		- Get UBX binary messages
		- Send UBX binary commands
		Note: as binary data have to be read/written, we need to disable
		  various character processing to avoid loosing data */
	/* Control Modes */
	ttyopt.c_cflag |= CLOCAL;   /* local connection, no modem control */
	ttyopt.c_cflag |= CREAD;	/* enable receiving characters */
	ttyopt.c_cflag |= CS8;	  /* 8 bit frames */
	ttyopt.c_cflag &= ~PARENB;  /* no parity */
	ttyopt.c_cflag &= ~CSTOPB;  /* one stop bit */
	/* Input Modes */
	ttyopt.c_iflag |= IGNPAR;   /* ignore bytes with parity errors */
	ttyopt.c_iflag &= ~ICRNL;   /* do not map CR to NL on input*/
	ttyopt.c_iflag &= ~IGNCR;   /* do not ignore carriage return on input */
	ttyopt.c_iflag &= ~IXON;	/* disable Start/Stop output control */
	ttyopt.c_iflag &= ~IXOFF;   /* do not send Start/Stop characters */
	/* Output Modes */
	ttyopt.c_oflag = 0;	 /* disable everything on output as we only write binary */
	/* Local Modes */
	ttyopt.c_lflag &= ~ICANON;  /* disable canonical input - cannot use with binary input */
	ttyopt.c_lflag &= ~ISIG;	/* disable check for INTR, QUIT, SUSP special characters */
	ttyopt.c_lflag &= ~IEXTEN;  /* disable any special control character */
	ttyopt.c_lflag &= ~ECHO;	/* do not echo back every character typed */
	ttyopt.c_lflag &= ~ECHOE;   /* does not erase the last character in current line */
	ttyopt.c_lflag &= ~ECHOK;   /* do not echo NL after KILL character */
	/* Settings for non-canonical mode:
	   read will block for until the lesser of VMIN or requested chars have been received */
	ttyopt.c_cc[VMIN]  = 1;	 /* byte granularity at init time to ensure we get all UBX responses */
	ttyopt.c_cc[VTIME] = 0;

	CSDebugTermios(ttyopt, 4); /* Log termios flag configuration at verbose lvl 4 */

	/* Set new TTY serial ports parameters */
	x = tcsetattr( *fd_cs, TCSANOW, &ttyopt );
	if (x != 0) {
		RTL_TRDBG(0, "ERROR: failed to update CellState serial port parameters\n");
		return -1;
	}

	tcflush(*fd_cs, TCIOFLUSH);
	
	return 0;
}

// Open the device that report cellular state continuously and initialize structure to store the samples
static void CSDeviceStart()
{
	// device already open
	if (CSFd >= 0)
		return;

	CSFd = open(CellStateDev, O_RDWR | O_NOCTTY);
	if (CSFd <= 0) {
		RTL_TRDBG(0, "ERROR: opening TTY port for CellState failed - %s (%s)\n", strerror(errno), CellStateDev);
		return ;
	}

	CellStateSample = (t_cell_indic *) malloc(CellStateSampleSize * sizeof(t_cell_indic));
	if (!CellStateSample)
	{
			RTL_TRDBG(0, "ERROR: failed to allocate size for %d samples !\n", CellStateSampleSize);
			close(CSFd);
			CSFd = -1;
			return;
	}
	CellStateSampleCount = 0;
	CellStateSampleIdx = 0;

	CSEnable(&CSFd);

//	fcntl(CSFd, F_SETFL, 0);	// set blocking read
	_CSDeviceStatus = CS_DEVICE_STATUS_STARTED;
}

// Close the device that report cellular state continuously and initialize structure to store the samples
static void CSDeviceStop()
{
	if (CSFd >= 0)
		close(CSFd);
	CSFd = -1;
	_CSDeviceStatus = CS_DEVICE_STATUS_STOPPED;
}

int CSDeviceStatus()
{
	return _CSDeviceStatus;
}

int CellStateThreadStatus()
{
	return _CSThreadStatus;
}

static void cleanup_thread_cs(void * a)
{
	RTL_TRDBG(0, "stop lrr.x/cellstate th=%lx pid=%d count=%d\n", (long)pthread_self(), getpid(), --count_thread_cs);
	CSDeviceStop();
	_CSThreadStatus = 0;
}

static void sendCmd(int fd, char *send, char *res, int szres, int skipFirst)
{
	int		sz;
	char	buf[128];
	char	*pt, *pte;
	
	if (!send || !res)
		return;

	// send command
	write(fd, send, strlen(send));
	sleep(1);
	// read result
	sz = read(fd, buf, sizeof(buf)-1);
	if (sz >= 0)
		buf[sz] = '\0';

	// clean result
	pt = buf;
	// go to end of the line
	while (*pt && *pt != '\r' && *pt != '\n')
		pt++;
	// go to next line
	while (*pt && (*pt == '\r' || *pt == '\n'))
		pt++;
	// skip first word of the response ?
	if (skipFirst)
	{
		// go to first space
		while (*pt && !isspace(*pt))
			pt++;
		// skip space
		if (isspace(*pt))
			pt++;
	}
	pte = pt;
	// go to end of the line
	while (*pte && *pte != '\r' && *pte != '\n')
		pte++;
	sz = MIN(pte - pt, szres-1);
	strncpy(res, pt, sz);
	res[sz] = '\0';

	RTL_TRDBG(4, "sendCmd: send:'%s' raw response:'%s' after cleaning:%s\n", send, buf, res);
}

// Get IMEI, IMSI, ICCID
int CSGetIMEI(t_cell_infos *ci)
{
	char	buf[128];
	int	 fd;

	if (!ci)
		return -1;

	// open the device that gives the informations about the identity
	fd = open(CellStateDev, O_RDWR | O_NOCTTY);
	if (fd <= 0) {
		RTL_TRDBG(0, "ERROR: opening TTY port for CellStateDev failed - %s (%s)\n", strerror(errno), CellStateDev);
		return 0;
	}

	CSEnable(&fd);

	ci->imei[0] = '\0';
	ci->imsi[0] = '\0';
	ci->iccid[0] = '\0';

	// disabled unsollicited messages to get clean results to our commands
	sendCmd(fd, "AT^CURC=0\r", buf, sizeof(buf), 0);

	// PIN code READY ?
	sendCmd(fd, "AT+CPIN?\r", buf, sizeof(buf), 0);
	if (strstr("buf", "READY"))
	{
		RTL_TRDBG(1, "CSGetIMEI: PIN not ready, abort getting IMEI ... (%s)\n", buf);
		return -1;
	}

	// Get IMEI
	// response format:
	// AT+CGSN
	// 865035035677608
	//
	// OK
	sendCmd(fd, "AT+CGSN\r", buf, sizeof(buf), 0);
	strncpy(ci->imei, buf, sizeof(ci->imei) - 1);

	// Get IMSI
	// response format:
	// AT+CIMI
	// 208104289832620
	//
	// OK
	sendCmd(fd, "AT+CIMI\r", buf, sizeof(buf), 0);
	strncpy(ci->imsi, buf, sizeof(ci->imsi) - 1);

	// Get ICCID
	// response format:
	// AT^ICCID?
	// ^ICCID: 89331042140212492505
	//
	//OK
	sendCmd(fd, "AT^ICCID?\r", buf, sizeof(buf), 1);
	strncpy(ci->iccid, buf, sizeof(ci->iccid) - 1);

	// enable unsollicited messages to get periodically RSSI, HCSQ ...
	sendCmd(fd, "AT^CURC=1\r", buf, sizeof(buf), 0);

	close(fd);

	RTL_TRDBG(1, "Cellular: IMEI='%s' IMSI='%s' ICCID='%s'\n", ci->imei, ci->imsi, ci->iccid);
	// force a LRR_config to be sent
	fd = creat(ExtConfigRefresh, O_WRONLY);
	if (fd < 0)
	{
		RTL_TRDBG(0, "Failed to create file '%s' !\n", ExtConfigRefresh);
	}
	else
		close(fd);
	return 1;
}

// Copy ident information (IMEI ...) in t_lrr_cellid structure
void CellStateCopyIdent(t_lrr_cellid *cellid)
{
	if (!cellid)
		return;

	if (!CellState)
	{
		RTL_TRDBG(1,"No cellular information available now (IMEI, IMSI ...)\n");
		return;
	}
	strncpy	((char *)cellid->cf_ItfImei,CellState->imei,
			sizeof(cellid->cf_ItfImei)-1);
	strncpy	((char *)cellid->cf_ItfImsi,CellState->imsi,
			sizeof(cellid->cf_ItfImsi)-1);
	strncpy	((char *)cellid->cf_ItfIccid,CellState->iccid,
			sizeof(cellid->cf_ItfIccid)-1);
	RTL_TRDBG(4,"CellStateCopyIdent: imei='%s' imsi='%s' iccid='%s'\n",
		cellid->cf_ItfImei, cellid->cf_ItfImsi, cellid->cf_ItfIccid);
}

// thread starts
void * CellStateRun(void * param)
{
	pthread_cleanup_push(cleanup_thread_cs, NULL);
	RTL_TRDBG(0, "start lrr.x/cellstate th=%lx pid=%d count=%d\n", (long)pthread_self(), getpid(), ++count_thread_cs);
	_CSThreadStatus = 1;

	if (!CellState)
	{
		CellState = (t_cell_infos *) malloc(sizeof(t_cell_infos));
		if (!CellState)
		{
			RTL_TRDBG(0, "Failed to malloc CellState ! Abort.\n");
			return NULL;
		}
		CellState->indic.update_count = 0;
		CellState->ope[0] = '\0';
		CellState->imei[0] = '\0';
		CellState->imsi[0] = '\0';
		CellState->iccid[0] = '\0';
	}

	// device that gives IMEI ...
	while(!CSGetIMEI(CellState))
		sleep(60);

	// device that gives informations about the cellular connection
	CSDeviceStart();
	while (CS_DEVICE_STATUS_STARTED != CSDeviceStatus())
	{
		sleep(60);
		CSDeviceStart();
	}

	CSMainLoop();

	pthread_cleanup_pop(1);
	return NULL;
}

// add val in tot excepted if val==255, return 1 if added, else 0
int	AddIfNot255(float *tot, float val)
{
	if (val == 255)
		return 0;
	*tot += val;
	return 1;
}

// Calculate average values to be reported from the samples available
void CSUpdate()
{	
	t_cell_indic	*ci;
	time_t			now;
	int				idx, count, mode=-1;
	int				rssicnt=0, rsrpcnt=0, sinrcnt=0, rsrqcnt=0, rscpcnt=0, eciocnt=0;
	float			rssitot=0, rsrptot=0, sinrtot=0, rsrqtot=0, rscptot=0, eciotot=0;

	RTL_TRDBG(9, "CSUpdate ...\n");

	count = 0;
	now = time(NULL);
	while (count < CellStateSampleCount)
	{
		// get index of last value, then the previous one ...
		idx = (CellStateSampleIdx - 1 - count + CellStateSampleSize) % CellStateSampleSize;
		ci = &CellStateSample[idx];
		// the mode is defined by the first sample
		if (mode < 0)
		{
			mode = ci->mode;
			// check if outdated
			if (ci->tm < now - CellStateSampleTimeout)
			{
				RTL_TRDBG(1, "Informations about cellular are too old, do not report anything\n");
				break;
			}
		}
		else if (mode != ci->mode)
		{
			// the mode changed, we can't make an average with different fields, the other
			// samples won't be used
			RTL_TRDBG(4, "CSUpdate: mode changed mode=%d ci->mode=%d idx=%d\n", mode, ci->mode, idx);
			break;
		}
		RTL_TRDBG(4, "CSUpdate: treat sample idx=%d mode=%d...\n", idx, ci->mode);
		switch (ci->mode)
		{
			case CELL_MODE_LTE:
				rssicnt += AddIfNot255(&rssitot, ci->cont.lte.rssi);
				rsrpcnt += AddIfNot255(&rsrptot, ci->cont.lte.rsrp);
				sinrcnt += AddIfNot255(&sinrtot, ci->cont.lte.sinr);
				rsrqcnt += AddIfNot255(&rsrqtot, ci->cont.lte.rsrq);
				break;
			case CELL_MODE_WCDMA:
				rssicnt += AddIfNot255(&rssitot, ci->cont.wcdma.rssi);
				rscpcnt += AddIfNot255(&rscptot, ci->cont.wcdma.rscp);
				eciocnt += AddIfNot255(&eciotot, ci->cont.wcdma.ecio);
				break;
			case CELL_MODE_GSM:
				rssicnt += AddIfNot255(&rssitot, ci->cont.gsm.rssi);
				break;
			case CELL_MODE_NOSERV:
				rssicnt += 1;
				break;
		}
		count++;
	}

	RTL_TRDBG(4, "CSUpdate: rssi_count=%d ...\n", rssicnt);
	// no sample treated or report structure no more available
	if (rssicnt <= 0 || !CellState)
		return;

	// calculate average values
	switch (mode)
	{
		case CELL_MODE_LTE:
			CellState->indic.update_count += 1;
			CellState->mode = CELL_MODE_LTE;
			CellState->indic.cont.lte.rssi_count = rssicnt;
			CellState->indic.cont.lte.rssi = rssitot / rssicnt;
			CellState->indic.cont.lte.rsrp = rsrptot / rsrpcnt;
			CellState->indic.cont.lte.sinr = sinrtot / sinrcnt;
			CellState->indic.cont.lte.rsrq = rsrqtot / rsrqcnt;
			RTL_TRDBG(4, "CSUpdate: LTE rssi_count=%d rssi=%.2f rsrp=%.2f sinr=%.2f rsrq=%.2f\n",
					CellState->indic.cont.lte.rssi_count, CellState->indic.cont.lte.rssi, CellState->indic.cont.lte.rsrp,
					CellState->indic.cont.lte.sinr, CellState->indic.cont.lte.rsrq);
			break;
		case CELL_MODE_WCDMA:
			CellState->indic.update_count += 1;
			CellState->mode = CELL_MODE_WCDMA;
			CellState->indic.cont.wcdma.rssi_count = rssicnt;
			CellState->indic.cont.wcdma.rssi = rssitot / rssicnt;
			CellState->indic.cont.wcdma.rscp = rscptot / rscpcnt;
			CellState->indic.cont.wcdma.ecio = eciotot / eciocnt;
			RTL_TRDBG(4, "CSUpdate: WCDMA rssi_count=%d rssi=%.2f rscp=%.2f ecio=%.2f\n",
					CellState->indic.cont.wcdma.rssi_count, CellState->indic.cont.wcdma.rssi,
					CellState->indic.cont.wcdma.rscp, CellState->indic.cont.wcdma.ecio);
			break;
		case CELL_MODE_GSM:
			CellState->indic.update_count += 1;
			CellState->mode = CELL_MODE_GSM;
			CellState->indic.cont.gsm.rssi_count = rssicnt;
			CellState->indic.cont.gsm.rssi = rssitot / rssicnt;
			RTL_TRDBG(4, "CSUpdate: GSM rssi_count=%d rssi=%.2f\n",
				CellState->indic.cont.gsm.rssi_count, CellState->indic.cont.gsm.rssi);
			break;
		case CELL_MODE_NOSERV:
			CellState->mode = CELL_MODE_NOSERV;
			RTL_TRDBG(4, "CSUpdate: NOSERV, nothing to report\n");
			break;
	}
}

/*
	example of what is written on tty:

	^RSSI:20

	^HCSQ:"LTE",48,35,86,14

	^RSSI:20

	^HCSQ:"LTE",49,36,91,16
*/
// Analyze what is sent by the device
char *AnaLine(char *buf, int sz)
{
	char			*pt, *pte;
	int			 v1, v2, v3, v4, needUpdate;
	t_cell_indic	*ci;

	if (!buf)
		return NULL;

	RTL_TRDBG(4, "AnaLine(%s)\n", buf);
	if (TraceLevel >= 9)
		DebugFrameHexdump(buf, strlen(buf));
	pt = buf;
	ci = &CellStateSample[CellStateSampleIdx];
	while (*pt)
	{
		// skip \n \r at the beginning of the line
		while ((pt - buf < sz ) && (*pt == '\n' || *pt == '\r'))
			pt++;

		// search end of line
		pte = pt;
		while (pte - buf < sz && *pte && *pte != '\n' && *pte != '\r')
			pte++;

		// if \r or \n not found the line is not complete
		if (!*pte || pte - buf >= sz)
		{
			RTL_TRDBG(9, "AnaLine: incomplete line '%s', nothing done\n", pt);
			RTL_TRDBG(9, "AnaLine: return *pt=%c\n", *pt);
			return pt;
		}

		if (!strncmp(pt, "^HCSQ:", 6))
		{
				RTL_TRDBG(4, "AnaLine: HCSQ line identified\n");
				needUpdate = 0;

				// ^HCSQ:"LTE",49,36,91,16
				pt += 6;
				// skip non alpha char
				while (pt - buf < sz && *pt && !isalpha(*pt))
					pt++;
				RTL_TRDBG(4, "AnaLine: compare with LTE pt='%s'\n", pt);
				if (!strncmp(pt, "LTE", 3))
				{
					ci->mode = CELL_MODE_LTE;

					// set pt on first digit
					while (pt - buf < sz && *pt && !isdigit(*pt))
						pt++;
					if (sscanf(pt, "%d,%d,%d,%d", &v1, &v2, &v3, &v4) == 4)
					{
						RTL_TRDBG(3, "AnaLine: LTE line parsed successfully rssi=%d rsrp=%d sinr=%d rsrq=%d\n", v1, v2, v3, v4);

						// rssi = v1 - 120, v1 = [0,96], 255 means unknown or undetectable
						ci->cont.lte.rssi = (v1==255)?v1:v1-120;

						// rsrp = v2 - 140, v2 = [0,97], 255 means unknown or undetectable
						ci->cont.lte.rsrp = (v2==255)?v2:v2-140;

						// sinr = 0.2 x v3 - 20, v3 = [0,251], 255 means unknown or undetectable
						ci->cont.lte.sinr = (v3==255)?v3:0.2*v3-20;

						// rsrq = 0,5 x v4 - 19.5, v4 = [0,34], 255 means unknown or undetectable
						ci->cont.lte.rsrq = (v4==255)?v4:0.5*v4-19.5;

						RTL_TRDBG(1, "Cellular: LTE rssi=%.2f rsrp=%.2f sinr=%.2f rsrq=%.2f\n",
							ci->cont.lte.rssi, ci->cont.lte.rsrp, ci->cont.lte.sinr, ci->cont.lte.rsrq);

						ci->tm = time(NULL);
						needUpdate = 1;
					}
				}
				else if (!strncmp(pt, "WCDMA", 5))
				{
					ci->mode = CELL_MODE_WCDMA;

					// set pt on first digit
					while (pt - buf < sz && *pt && !isdigit(*pt))
						pt++;
					if (sscanf(pt, "%d,%d,%d", &v1, &v2, &v3) == 3)
					{
						RTL_TRDBG(3, "AnaLine: WCDMA line parsed successfully rssi=%d rscp=%d ecio=%d\n", v1, v2, v3);

						// rssi = v1 - 120, v1 = [0,96], 255 means unknown or undetectable
						ci->cont.wcdma.rssi = (v1==255)?v1:v1-120;

						// rscp = v2 - 120, v2 = [0,96], 255 means unknown or undetectable
						ci->cont.wcdma.rscp = (v2==255)?v2:v2-120;

						// ecio = 0.5 x v3 - 32, v3 = [0,65], 255 means unknown or undetectable
						ci->cont.wcdma.ecio = (v3==255)?v3:0.5*v3-32;
						RTL_TRDBG(1, "Cellular: WCDMA rssi=%.2f rscp=%.2f ecio=%.2f\n",
							ci->cont.wcdma.rssi, ci->cont.wcdma.rscp, ci->cont.wcdma.ecio);
						ci->tm = time(NULL);
						needUpdate = 1;
					}
				}
				else if (!strncmp(pt, "GSM", 3))
				{
					ci->mode = CELL_MODE_GSM;

					// set pt on first digit
					while (pt - buf < sz && *pt && !isdigit(*pt))
						pt++;
					if (sscanf(pt, "%d", &v1) == 3)
					{
						RTL_TRDBG(3, "AnaLine: GSM line parsed successfully rssi=%d\n", v1);

						// rssi = v1 - 120, v1 = [0,96], 255 means unknown or undetectable
						ci->cont.gsm.rssi = (v1==255)?v1:v1-120;
						RTL_TRDBG(1, "Cellular: GSM rssi=%.2f\n", ci->cont.gsm.rssi);

						ci->tm = time(NULL);
						needUpdate = 1;
					}
				}
				else if (!strncmp(pt, "NOSERVICE", 9))
				{
					RTL_TRDBG(3, "AnaLine: NOSERVICE line parsed successfully\n");
					ci->mode = CELL_MODE_NOSERV;
					ci->tm = time(NULL);
					needUpdate = 1;
				}

				// new sample added ?
				if (needUpdate)
				{
						// shift to next index to use
						CellStateSampleIdx = (CellStateSampleIdx+1) % CellStateSampleSize;
						// update count of samples
						CellStateSampleCount = MIN(CellStateSampleCount+1, CellStateSampleSize);
						// update report
						CSUpdate();
				}

				pt = pte+1; 
		}
		else	// unidentified line, skip it
		{
			RTL_TRDBG(4, "AnaLine: skip line '%s' (%d:%d)\n", pt, pt-buf, pte-buf);
			pt = pte+1; 
		}
	}
	RTL_TRDBG(4, "AnaLine: return *pt=%c\n", *pt);
	return pt;
}

// thread loop
static void CSMainLoop()
{
	char			buf[1024];
	char			*pt, *ptn;
	ssize_t		 nb_char, sz;

	ptn = buf;
	memset(buf, 0, sizeof(buf));
	/* Start thread main loop */
	while (!ServiceStopped && _CSDeviceStatus )
	{
		// size available in the buffer
		sz = sizeof(buf) - (ptn - buf) - 1;
		// read the device
		nb_char = read(CSFd, ptn, sz);

		if (nb_char <= 0) {
			RTL_TRDBG(0, "CellState device with fd=%d  read() returned %d\n", CSFd, nb_char);
			break;
		}

		// add \0 at the end
		sz = MIN(nb_char+ptn-buf, sizeof(buf)-1);
		buf[sz] = '\0';
		RTL_TRDBG(4, "CSMainLoop: %d chars read '%s' on device\n", nb_char, buf);
		// analyze the buffer
		pt = AnaLine(buf, sz);
		if (pt && !*pt)
		{
			// pt is on the end of the string => all was treated
			RTL_TRDBG(9, "CSMainLoop: buf completely treated\n");
			buf[0] = '\0';
			pt = buf;
			ptn = buf;
		}
		else
		{
			// pt is on the part not treated of buf => shift part not treated for next time
			RTL_TRDBG(4, "CSMainLoop: shift not treated part '%s' to the beginning of buf\n", pt);
			ptn = buf;
			while (pt && *pt && pt < buf+sizeof(buf))
				*ptn++ = *pt++;
			*ptn = '\0';
			RTL_TRDBG(4, "CSMainLoop: buf after shift: '%s'\n", buf);
		}

	} /* while main loop */

	// the read failed => key unplugged. Close everything
	if (CSFd)
	{
		close(CSFd);
		CSFd = -1;
	}
}


#endif /* CELLULAR_C */

/* vim: ft=c: set noet ts=4 sw=4 sts=4: */
