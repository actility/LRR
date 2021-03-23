
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

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <dirent.h>

#include "rtlbase.h"
#include "rtlimsg.h"
#include "rtllist.h"
#include "rtlhtbl.h"

char	*CfgStr(void *ht,char *sec,int index,char *var,char *def);
int	CfgInt(void *ht,char *sec,int index,char *var,int def);
int CfgCBIniLoad(void *user,const char *section,const char *name,const char *value);

char	*RootAct	= NULL;
char	*System		= NULL;

char	*SerialMode	= NULL;
char	*BoardType	= NULL;
char	*ServiceLrr	= NULL;

void	*HtVarLrr;

/* new configuration mechanism */
#if defined(SYSTEM_NAME)

void	InitSystem()
{
	System	= getenv("SYSTEM");
	RootAct	= getenv("ROOTACT");

/* TODO: to be replaced with getenv once present on gateway */
/* in the meantime, stringify macro value */
#define xstr(s) str(s)
#define str(s) #s
	SerialMode	= xstr(ARCH_COMM);
	BoardType	= xstr(BOARD_TYPE);
}

#else

#ifdef	NATRBPI_USB
#define	SYSTEM_DEFINED
void	InitSystem()
{
	System		= "natrbpi_usb_v1.0";
	SerialMode	= "usb";
	BoardType	= "x1";
}
#endif

#ifdef	RBPI_V1_0
#define	SYSTEM_DEFINED
void	InitSystem()
{
	System		= "rbpi_v1.0";
	SerialMode	= "spi";
	BoardType	= "x1";
}
#endif

#ifdef	SEMPICO
#define	SYSTEM_DEFINED
void	InitSystem()
{
	System		= "sempico";
	SerialMode	= "tty";
	BoardType	= "x1";
}
#endif

#ifdef	IR910
#define	SYSTEM_DEFINED
void	InitSystem()
{
	System	= "ir910";
	SerialMode	= "usb";
	BoardType	= "x1";
}
#endif

#ifdef	CISCOMS
#define	SYSTEM_DEFINED
void	InitSystem()
{
	System	= "ciscoms";
	SerialMode	= "spi";
	BoardType	= "x8";
}
#endif

#ifdef  FCMLB
#define	SYSTEM_DEFINED
void    InitSystem()
{
        System          = "fcmlb";
        SerialMode      = "spi";
        BoardType       = "x1";
        RootAct = getenv("ROOTACT");
        if      (!RootAct)
        {
                RootAct = "/home/actility";
                setenv  ("ROOTACT",strdup(RootAct),1);
        }
}
#endif

#ifdef  FCPICO
#define	SYSTEM_DEFINED
void    InitSystem()
{
        System          = "fcpico";
        SerialMode      = "spi";
        BoardType       = "x1";
        RootAct = getenv("ROOTACT");
        if      (!RootAct)
        {
                RootAct = "/home/actility";
                setenv  ("ROOTACT",strdup(RootAct),1);
        }
}
#endif

#ifdef  FCLAMP
#define	SYSTEM_DEFINED
void    InitSystem()
{
        System          = "fclamp";
        SerialMode      = "spi";
        BoardType       = "x1";
        RootAct = getenv("ROOTACT");
        if      (!RootAct)
        {
                RootAct = "/home/actility";
                setenv  ("ROOTACT",strdup(RootAct),1);
        }
}
#endif

#ifdef  FCLOC
#define	SYSTEM_DEFINED
void    InitSystem()
{
        System          = "fcloc";
        SerialMode      = "spi";
        BoardType       = "x8";
        RootAct = getenv("ROOTACT");
        if      (!RootAct)
        {
                RootAct = "/home/actility";
                setenv  ("ROOTACT",strdup(RootAct),1);
        }
}
#endif

#ifdef	RFILR
#define	SYSTEM_DEFINED
void	InitSystem()
{
	System	= "rfilr";
	SerialMode	= "usb";
	BoardType	= "x1";
}
#endif

#ifdef  OIELEC
#define	SYSTEM_DEFINED
void    InitSystem()
{
        System  = "oielec";
        SerialMode      = "spi";
        BoardType       = "x1";
        RootAct = getenv("ROOTACT");
        if      (!RootAct)
        {
                RootAct = "/home/actility";
                setenv  ("ROOTACT",strdup(RootAct),1);
        }
}
#endif

#ifdef  FLEXPICO
#define	SYSTEM_DEFINED
void    InitSystem()
{
        System     = "flexpico";
        SerialMode = "spi";
        BoardType  = "x1";
        RootAct    = getenv("ROOTACT");
        if (!RootAct) {
                RootAct = "/home/actility";
                setenv("ROOTACT", strdup(RootAct), 1);
        }
}
#endif

#ifdef TRACKNET
#define	SYSTEM_DEFINED
void   InitSystem()
{
        System  = "tracknet";
        SerialMode  = "spi";
        BoardType   = "x1";
        RootAct    = getenv("ROOTACT");
        if (!RootAct) {
                RootAct = "/mnt/data/actility";
                setenv("ROOTACT", strdup(RootAct), 1);
        }
}
#endif


#ifndef SYSTEM_DEFINED
void    InitSystem()
{
#warning "you are compiling the LRR for linux 32/64bits generic target system"
#warning "this implies the use of a Semtech Picocell connected with ttyACMx"
        System  = "linux";
        SerialMode      = "tty";
        BoardType       = "x1";
        RootAct = getenv("ROOTACT");
        if      (!RootAct)
        {
                RootAct = "/home/actility";
                setenv  ("ROOTACT",strdup(RootAct),1);
        }
}
#endif

#endif // SYSTEM_NAME

int     main(int argc,char *argv[])
{
	char	fdefaultexe[PATH_MAX];
	char	factiveexe[PATH_MAX];
	char	*fexe;
	char	fparams[PATH_MAX];
	FILE	*log;
	FILE	*f;

/*
	freopen	("/tmp/lrrlauncher.log","w",stdout);
	freopen	("/tmp/lrrlauncher.log","w+",stderr);
*/

	log	= fopen("/tmp/lrrlauncher.log","w");

	HtVarLrr	= rtl_htblCreateSpec(25,NULL,
						HTBL_KEY_STRING|HTBL_FREE_DATA);
	if	(!HtVarLrr)
	{
		fprintf(log,"cannot alloc internal resources (htables)\n");
		exit(1);
	}

	InitSystem();

#if defined(SYSTEM_NAME)

   if (!System)
    {
        fprintf(log,"SYSTEM not defined => definitive failure\n");
        exit(1);
    }
	if	(!RootAct)
	{
        fprintf(log,"ROOTACT not defined => definitive failure\n");
        exit(1);
	}

#else // SYSTEM_NAME


	if	(!System)
	{
		fprintf(log,"SYSTEM not defined => definitve failure\n");
		exit(1);
	}
	if	(!RootAct)
		RootAct	= getenv("ROOTACT");
	if	(!RootAct)
	{
		fprintf	(log,"$ROOTACT not set\n");
		setenv	("ROOTACT",strdup("/home/actility"),1);
		RootAct	= getenv("ROOTACT");
	}
#endif // SYSTEM_NAME

	sprintf	(fdefaultexe,"%s/lrr/com/exe_%s_%s/lrr.x",RootAct,SerialMode,BoardType);

	fprintf	(log,"SYSTEM=%s\n",System);
	fprintf	(log,"ROOTACT=%s\n",RootAct);

	fprintf	(log,"SERIALMODE=%s (default)\n",SerialMode);
	fprintf	(log,"BOARDTYPE=%s (default)\n",BoardType);
	fprintf	(log,"SERVICELRR=%s (default)\n",ServiceLrr);
	fprintf	(log,"file to launch '%s' (default)\n",fdefaultexe);

	if	(rtl_openDir(RootAct) == NULL)
	{
		fprintf	(log,"ROOTACT does not exist or can not be opened => definitve failure\n");
		exit(1);
	}

	sprintf	(fparams,"%s/usr/etc/lrr/_parameters.sh",RootAct);
	f	= fopen(fparams,"r");
	if	(f)
	{
		int	err;

		err	= rtl_iniParse(fparams,CfgCBIniLoad,HtVarLrr);
		if	(err < 0)
		{
			fprintf	(log,"'%s' parse error=%d => definitve failure \n",
				fparams,err);
			exit(1);
		}
		fclose(f);
	}
	else
	{
		fprintf	(log,"'%s' does not exist : use default values\n",fparams);
	}

	SerialMode	= CfgStr(HtVarLrr,"",-1,"SERIALMODE",SerialMode);
	BoardType	= CfgStr(HtVarLrr,"",-1,"BOARDTYPE",BoardType);
	ServiceLrr	= CfgStr(HtVarLrr,"",-1,"SERVICELRR",ServiceLrr);
#ifdef	WIRMAV2
	char	file[1024];
	char	cmd[1024];
	sprintf	(file,"%s/usr/etc/lrr/_system.sh",RootAct);
	if	(access(file,R_OK) != 0)
	{
		fprintf	(log,"file '%s' does not exist => try to create it\n",
							file);
		sprintf	(cmd,"/usr/local/bin/get_version > %s",file);
		system(cmd);
	}

#endif
#ifdef	CISCOMS
	char	file[1024];
	char	cmd[1024];
	sprintf	(file,"%s/usr/etc/lrr/_system.sh",RootAct);
	if	(access(file,R_OK) != 0)
	{
		fprintf	(log,"file '%s' does not exist => try to create it\n",
							file);
		sprintf	(cmd,"[ -z \"$(grep CISCOSN %s 2>/dev/null)\" ] && echo CISCOSN=$(getsn) >> %s", file, file);
		system(cmd);
	}

#endif

	sprintf	(factiveexe,"%s/lrr/com/exe_%s_%s/lrr.x",RootAct,SerialMode,BoardType);

	fprintf	(log,"-----------\n");
	fprintf	(log,"----------- load '%s'\n",fparams);
	fprintf	(log,"-----------\n");
	fprintf	(log,"SERIALMODE=%s (active)\n",SerialMode);
	fprintf	(log,"BOARDTYPE=%s (active)\n",BoardType);
	fprintf	(log,"SERVICELRR=%s (active)\n",ServiceLrr);
	fprintf	(log,"file to launch '%s' (active)\n",factiveexe);

	fexe	= factiveexe;
	if	(access(fexe,X_OK) != 0)
	{
		fprintf	(log,"file to launch '%s' does not exist or !X_OK\n",fexe);
		fexe	= fdefaultexe;
		if	(access(fexe,X_OK) != 0)
		{
			fprintf	(log,"file to launch '%s' does not exist or !X_OK\n"
						,fexe);
			exit(1);
		}
	}

	setenv	("SERIALMODE",strdup(SerialMode),1);
	setenv	("BOARDTYPE",strdup(BoardType),1);
	setenv	("SERVICELRR",strdup(ServiceLrr),1);

	fprintf	(log,"-----------\n");
	fprintf	(log,"----------- launch '%s'\n",fexe);
	fprintf	(log,"-----------\n");

	fflush	(log);
	fclose	(log);

	execv	(fexe,argv);

	printf	("launch error errno=%d\n",errno);
	
	exit(1);
}
