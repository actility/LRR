
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
#include <sys/vfs.h>

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

#include "semtech.h"

#include "xlap.h"
#include "infrastruct.h"
#include "struct.h"

#include "headerloramac.h"

typedef unsigned char u8;
typedef unsigned short u16;

#include "_whatstr.h"
#include "define.h"
#include "cproto.h"
#include "extern.h"

#define		CMD_OTHER			0
#define		CMD_BUILTIN			1
#define		CMD_BUILTIN_THREAD		2
#define		CMD_POPEN_THREAD		3
#define		CMD_SYSTEM_DETACH		4
#define		CMD_SYSTEM_THREAD		5
#define		CMD_SYSTEM_PENDING_THREAD	6

#define		CMD_MAX_THREAD			5

static	unsigned int	CmdThread	= 0;

static	unsigned int	CmdOpenSsh	= 0;

static	unsigned int	CmdRfScan	= 0;


unsigned int CmdCountOpenSsh()
{
	return	CmdOpenSsh;
}

unsigned int CmdCountRfScan()
{
	return	CmdRfScan;
}

// used in a child thread
static	void	ClearCmdPending()
{
	void	*dir;
	char	*fname;
	struct	stat	st;
	time_t	now;
	time_t	delta;

	time(&now);
	dir	= rtl_openDir(DirCommands);
	if	(!dir)
		return;
	if	(strstr(DirCommands,"usr/data/lrr/cmd_shells") == NULL)
	{
		RTL_TRDBG(0,"strange dir commands '%s'\n",dir);
		return;
	}
	while	((fname=rtl_readAbsDir(dir)))
	{
		if	(stat(fname,&st) != 0)	continue;
		if	(!S_ISREG(st.st_mode))	continue;	// "." & ".."
		delta	= ABS(now - st.st_mtime);
		if	(delta < 3600)		continue;
		unlink	(fname);
		RTL_TRDBG(1,"delete cmd file '%s'\n",fname);
	}
	rtl_closeDir(dir);
}

static	void	*LoopCmdThread(void *p)
{
	while	(1)
	{
		ClearCmdPending();
		sleep	(120);
	}
	return	NULL;
}

void	StartCmdThread()
{
	pthread_attr_t	threadAt;
	pthread_t	thread;

	if	(pthread_attr_init(&threadAt))
	{
		RTL_TRDBG(0,"cannot init thread clearcmd err=%s\n",STRERRNO);
		return;
	}

	if	(pthread_create(&thread,&threadAt,LoopCmdThread,NULL))
	{
		RTL_TRDBG(0,"cannot create thread clearcmd err=%s\n",STRERRNO);
		return;
	}
	RTL_TRDBG(0,"cmd thread is started\n");
}

// used in a child thread
void	CreateCmdPending(t_lrr_pkt *downpkt)
{
	char		file[PATH_MAX];
	t_lrr_shell_cmd	*shell;
	FILE		*f;


	shell	= &(downpkt->lp_u.lp_shell_cmd);
	sprintf	(file,"%s/%u_pending",DirCommands,shell->cm_cmd.cm_serial);
	f	= fopen(file,"w");
	if	(f)
		fclose(f);
}

// used in a child thread
void	TerminateCmdPending(t_lrr_pkt *downpkt,int status)
{
	char		file[PATH_MAX];
	char		filenew[PATH_MAX];
	t_lrr_shell_cmd	*shell;
	FILE		*f;

	shell	= &(downpkt->lp_u.lp_shell_cmd);
	sprintf	(file,"%s/%u_pending",DirCommands,shell->cm_cmd.cm_serial);
	sprintf	(filenew,"%s/%u_terminated",DirCommands,
						shell->cm_cmd.cm_serial);
	rename	(file,filenew);

	f	= fopen(filenew,"w");
	if	(f)
	{
		fprintf	(f,"%d\n",status);
		fclose	(f);
	}
}

// used in main thread to terminate built-in command
void	TerminateCmdBuiltIn(t_lrr_pkt *downpkt,int status,char *txt)
{
	char		file[PATH_MAX];
	t_lrr_shell_cmd	*shell;
	FILE		*f;


	shell	= &(downpkt->lp_u.lp_shell_cmd);
RTL_TRDBG(1,"terminate builtin serial=%u\n",shell->cm_cmd.cm_serial);

	sprintf	(file,"%s/%u_pending",DirCommands,shell->cm_cmd.cm_serial);
	unlink	(file);

	sprintf	(file,"%s/%u.log",DirCommands,shell->cm_cmd.cm_serial);
	unlink	(file);
	if	(txt && *txt)
	{
		f	= fopen(file,"w");
		if	(f)
		{
			fprintf	(f,"%s\n",txt);
			fclose	(f);
		}
	}

	sprintf	(file,"%s/%u_terminated",DirCommands,shell->cm_cmd.cm_serial);
	unlink	(file);
	f	= fopen(file,"w");
	if	(f)
	{
		fprintf	(f,"%d\n",status);
		fclose	(f);
	}
}



// used in main or child thread
static	char	*PreResponseCommand(t_lrr_pkt *downpkt,t_lrr_pkt *resppkt)
{
	memset	(resppkt,0,sizeof(t_lrr_pkt));
	resppkt->lp_vers	= LP_LRX_VERSION;
	resppkt->lp_flag	= LP_INFRA_PKT_INFRA;
	resppkt->lp_lrrid	= LrrID;
	resppkt->lp_type	= LP_TYPE_LRR_INF_CMD_RESPONSE;
	resppkt->lp_szh	= LP_PRE_HEADER_PKT_SIZE+sizeof(t_lrr_cmd_resp);

	resppkt->lp_u.lp_cmd_resp.rp_fmt	= 0;	// null term string
	resppkt->lp_u.lp_cmd_resp.rp_code	= 0;	// OK
	resppkt->lp_u.lp_cmd_resp.rp_codeemb	= 0;
	resppkt->lp_u.lp_cmd_resp.rp_more	= 0;	// no more data
	resppkt->lp_u.lp_cmd_resp.rp_seqnum	= 0;	// first data
	resppkt->lp_u.lp_cmd_resp.rp_continue	= 0;	// finale response
	resppkt->lp_u.lp_cmd_resp.rp_type	= LP_TYPE_LRR_INF_SHELL_CMD;

	resppkt->lp_u.lp_cmd_resp.rp_serial	= 
				downpkt->lp_u.lp_shell_cmd.cm_cmd.cm_serial;
	resppkt->lp_u.lp_cmd_resp.rp_cref	= 
				downpkt->lp_u.lp_shell_cmd.cm_cmd.cm_cref;
	resppkt->lp_u.lp_cmd_resp.rp_itf	= 
				downpkt->lp_u.lp_shell_cmd.cm_cmd.cm_itf;
	resppkt->lp_u.lp_cmd_resp.rp_tms	= 
				downpkt->lp_u.lp_shell_cmd.cm_cmd.cm_tms;

	resppkt->lp_lk	= downpkt->lp_lk;

	return	(char *)resppkt->lp_u.lp_cmd_resp.rp_cmd;
}

// used in main thread
static	char	*envstrdup(char *v)
{
#ifdef	__clang__
	// do not dup to avoid static analyze errors
	return	v;
#else
	// remember that some old versions of setenv do not dup value ???
	return	strdup(v);
#endif
}

static	void	DoEnvCmd()
{
	static	char	doenv	= 0;

	if	(doenv == 0)
	{
		char	tmp[1024];
		char	*pt;
		int	index = 0;

		doenv	= 1;

		//NFR684
		pt	= CfgStr(HtVarSys,"",-1,"LRROUI","");
		if	(pt && *pt)
			setenv	("LRROUI",envstrdup(pt),1);
		pt	= CfgStr(HtVarSys,"",-1,"LRRGID","");
		if	(pt && *pt)
			setenv	("LRRGID",envstrdup(pt),1);


		sprintf	(tmp,"%d",getpid());
		setenv	("LRRPID",envstrdup(tmp),1);

		sprintf	(tmp,"%08x",LrrID);
		setenv	("LRRID",envstrdup(tmp),1);

		sprintf	(tmp,"%s",System);
		setenv	("LRRSYSTEM",envstrdup(tmp),1);

		// rdtp-6639
		sprintf	(tmp,"%d",AutoRevSshPort);
		setenv	("LRRAUTOREVSSHPORT",envstrdup(tmp),1);

		pt	= CfgStr(HtVarLrr,tmp,-1,"spidevice", NULL);
		if	(pt && *pt)
			setenv	("SPIDEVICE",envstrdup(pt),1);

		pt	= (char *)CfgStr(HtVarLrr,"laplrc",0,"addr","");
		if	(pt && *pt)
			setenv	("LRCPRIMARY",envstrdup(pt),1);

		index	= 0;
		pt	= (char *)CfgStr(HtVarLrr,"support",index,"addr","");
		if	(pt && *pt)
			setenv	("SSHHOSTSUPPORT",envstrdup(pt),1);
		pt	= (char *)CfgStr(HtVarLrr,"support",index,"user","");
		if	(pt && *pt)
			setenv	("SSHUSERSUPPORT",envstrdup(pt),1);
		pt	= (char *)CfgStr(HtVarLrr,"support",index,"pass","");
		if	(pt /* && *pt */)
			setenv	("SSHPASSSUPPORT",envstrdup(pt),1);
		pt	= (char *)CfgStr(HtVarLrr,"support",index,"port","");
		if	(pt && *pt)
			setenv	("SSHPORTSUPPORT",envstrdup(pt),1);

		index	= 1;
		pt	= (char *)CfgStr(HtVarLrr,"support",index,"addr","");
		if	(pt && *pt)
			setenv	("BKPSSHHOSTSUPPORT",envstrdup(pt),1);
		pt	= (char *)CfgStr(HtVarLrr,"support",index,"user","");
		if	(pt && *pt)
			setenv	("BKPSSHUSERSUPPORT",envstrdup(pt),1);
		pt	= (char *)CfgStr(HtVarLrr,"support",index,"pass","");
		if	(pt /* && *pt */)
			setenv	("BKPSSHPASSSUPPORT",envstrdup(pt),1);
		pt	= (char *)CfgStr(HtVarLrr,"support",index,"port","");
		if	(pt && *pt)
			setenv	("BKPSSHPORTSUPPORT",envstrdup(pt),1);


		index	= 0;
		pt	= (char *)CfgStr(HtVarLrr,"support",index,"ftpaddr","");
		if	(pt && *pt)
			setenv	("FTPHOSTSUPPORT",envstrdup(pt),1);
		pt	= (char *)CfgStr(HtVarLrr,"support",index,"ftpuser","");
		if	(pt && *pt)
			setenv	("FTPUSERSUPPORT",envstrdup(pt),1);
		pt	= (char *)CfgStr(HtVarLrr,"support",index,"ftppass","");
		if	(pt /* && *pt */)
			setenv	("FTPPASSSUPPORT",envstrdup(pt),1);
		pt	= (char *)CfgStr(HtVarLrr,"support",index,"ftpport","");
		if	(pt && *pt)
			setenv	("FTPPORTSUPPORT",envstrdup(pt),1);
		pt = (char *)CfgStr(HtVarLrr,"support",index,"use_sftp","");
		if (pt && *pt)
			setenv ("USE_SFTP_SUPPORT", envstrdup(pt), 1);

		index	= 1;
		pt	= (char *)CfgStr(HtVarLrr,"support",index,"ftpaddr","");
		if	(pt && *pt)
			setenv	("BKPFTPHOSTSUPPORT",envstrdup(pt),1);
		pt	= (char *)CfgStr(HtVarLrr,"support",index,"ftpuser","");
		if	(pt && *pt)
			setenv	("BKPFTPUSERSUPPORT",envstrdup(pt),1);
		pt	= (char *)CfgStr(HtVarLrr,"support",index,"ftppass","");
		if	(pt /* && *pt */)
			setenv	("BKPFTPPASSSUPPORT",envstrdup(pt),1);
		pt	= (char *)CfgStr(HtVarLrr,"support",index,"ftpport","");
		if	(pt && *pt)
			setenv	("BKPFTPPORTSUPPORT",envstrdup(pt),1);
		pt = (char *)CfgStr(HtVarLrr,"support",index,"use_sftp","");
		if (pt && *pt)
			setenv ("BKP_USE_SFTP_SUPPORT", envstrdup(pt), 1);

		index   = 0;
		pt = (char *)CfgStr(HtVarLrr,"download",index,"ftpaddr","");
		/* If ftpaddr key from download section is either not set or empty,
		 * a default equal to env LRCPRIMARY is used */
		if (pt && *pt)
			setenv ("FTPHOSTDL", envstrdup(pt), 1);
		else
			setenv ("FTPHOSTDL", getenv("LRCPRIMARY"), 1);
		pt = (char *)CfgStr(HtVarLrr,"download",index,"ftpuser","");
		if (pt && *pt)
			setenv ("FTPUSERDL", envstrdup(pt), 1);
		pt = (char *)CfgStr(HtVarLrr,"download",index,"ftppass","");
		if (pt && *pt)
			setenv ("FTPPASSDL", envstrdup(pt), 1);
		pt = (char *)CfgStr(HtVarLrr,"download",index,"ftpport","");
		if (pt && *pt)
			setenv ("FTPPORTDL", envstrdup(pt), 1);
		pt = (char *)CfgStr(HtVarLrr,"download",index,"use_sftp","");
		if (pt && *pt)
			setenv ("USE_SFTP", envstrdup(pt), 1);
	}
}

// used in main thread
void	DoSystemCmdBackGround(char *cmd)
{
	int	lg;

	if	(!cmd || !*cmd)
		return;

	lg	= strlen(cmd);
	if	(cmd[lg-1] != '&')
		strcat	(cmd,"&");

	DoEnvCmd();

	system(cmd);
}

void	DoSystemCmdForeGround(char *cmd)
{
	int	lg;
	int	ret;

	if	(!cmd || !*cmd)
		return;

	lg	= strlen(cmd);
	if	(cmd[lg-1] == '&')
		cmd[lg-1] = '\0';

	DoEnvCmd();
RTL_TRDBG(3,"system('%s')\n",cmd);
	ret=system(cmd);
RTL_TRDBG(3,"\t\tsystem() => ret=%d\n",ret);
}

// used in main thread
void	DoSystemCmd(t_lrr_pkt *downpkt,char *cmd)
{
	DoEnvCmd();

	system(cmd);

	if	(downpkt == NULL)
		return;

	t_lrr_pkt	resppkt;
	t_xlap_link	*lk;
	char		*resp;

	lk	= (t_xlap_link *)downpkt->lp_lk;
	resp	= PreResponseCommand(downpkt,&resppkt);
	strcpy	(resp,"#asynchronous shell, exit code not controlled");
	LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
}

// used in a child thread
static	void	*_DoBuiltinCmdThread(void *p)
{
	t_lrr_pkt	*downpkt	= (t_lrr_pkt *)p;
	char		*name	= NULL;
	char		*cmd	= NULL;
	char		*params	= NULL;

	t_lrr_pkt	resppkt;
	t_xlap_link	*lk;
	char		*resp;

	t_imsg		*msg;
	int		sz	= sizeof(t_lrr_pkt);

	if	(!downpkt)
		goto	endcmd;

	name	= downpkt->lp_cmdname;
	if	(!name || !*name)
		goto	endcmd;
	cmd	= downpkt->lp_cmdfull;
	if	(!cmd || !*cmd)
		goto	endcmd;

	lk	= (t_xlap_link *)downpkt->lp_lk;
	resp	= PreResponseCommand(downpkt,&resppkt);
	resppkt.lp_lk	= lk;


	params	= cmd+strlen(name);
	while	(*params && (*params == ' ' || *params == '\t')) params++;

RTL_TRDBG(1,"start builtin thread cmd='%s' params='%s'\n",name,params);
	if	(strcmp(name,"ttest") == 0)
	{
		int	i,nb;

		if	(!params || !*params)	params	= "1";
		nb	= atoi(params);
		if	(nb <= 0 || nb > 100)
			nb	= 100;
		resppkt.lp_u.lp_cmd_resp.rp_seqnum	= 0;
		resppkt.lp_u.lp_cmd_resp.rp_more	= 1;
		for	(i = 0 ; i < nb ; i++)
		{
			sprintf	(resp,"test%d",i);

			if	(i == nb-1)
				resppkt.lp_u.lp_cmd_resp.rp_more	= 0;

			msg	= rtl_imsgAlloc(IM_DEF,IM_CMD_RECV_DATA,NULL,0);
			if	(msg && rtl_imsgDupData(msg,&resppkt,sz))
				rtl_imsgAdd(MainQ,msg);
			resppkt.lp_u.lp_cmd_resp.rp_seqnum++;
			sleep(1);
		}
		goto	endcmd;
	}

endcmd :

	if	(downpkt && downpkt->lp_cmdname)
		free	(downpkt->lp_cmdname);
	if	(downpkt && downpkt->lp_cmdfull)
		free	(downpkt->lp_cmdfull);
	if	(downpkt)
		free	(downpkt);

RTL_TRDBG(1,"end builtend thread\n");

	__sync_fetch_and_sub(&CmdThread,1);

	return	NULL;
}

// used in a child thread
static	void	*_DoPopenCmdThread(void *p)
{
	t_lrr_pkt	*downpkt	= (t_lrr_pkt *)p;
	char		*name	= NULL;
	char		*cmd	= NULL;

	t_lrr_pkt	resppkt;
	t_xlap_link	*lk;
	char		*resp;

	FILE		*pp	= NULL;
	int		status;

	t_imsg		*msg;
	int		sz	= sizeof(t_lrr_pkt);
	u_int		nice	= 10*1000;	// 10ms
	int		stopreport	= 0;

	if	(!downpkt)
		goto	endcmd;

	name	= downpkt->lp_cmdname;
	if	(!name || !*name)
		goto	endcmd;
	cmd	= downpkt->lp_cmdfull;
	if	(!cmd || !*cmd)
		goto	endcmd;

	lk	= (t_xlap_link *)downpkt->lp_lk;
	resp	= PreResponseCommand(downpkt,&resppkt);
	resppkt.lp_lk	= lk;

RTL_TRDBG(1,"start popen thread cmd='%s'\n",name);
	pp	= popen(cmd,"r");
	if	(!pp)
		goto	endcmd;

	if	(strcmp(name,"openssh") == 0)
		__sync_fetch_and_add(&CmdOpenSsh,1);

	if	(strcmp(name,"rfscanv0") == 0 || strcmp(name,"rfscanv1") == 0)
		__sync_fetch_and_add(&CmdRfScan,1);

	resppkt.lp_u.lp_cmd_resp.rp_seqnum	= 0;
	resppkt.lp_u.lp_cmd_resp.rp_more	= 1;
	while	(pp && fgets(resp,LP_DATA_CMD_LEN,pp) != NULL)
	{
		if	(!stopreport)
		{
			msg	= rtl_imsgAlloc(IM_DEF,IM_CMD_RECV_DATA,NULL,0);
			if	(msg && rtl_imsgDupData(msg,&resppkt,sz))
				rtl_imsgAdd(MainQ,msg);
			resppkt.lp_u.lp_cmd_resp.rp_seqnum++;
			if	(resppkt.lp_u.lp_cmd_resp.rp_seqnum >= 50)
			{
				usleep	(nice);
				nice	+= 100;
			}
			if	(resppkt.lp_u.lp_cmd_resp.rp_seqnum >= 10000)
				stopreport	= 1;
		}
		if	(resppkt.lp_u.lp_cmd_resp.rp_seqnum >= 60000)
		{
			pclose	(pp);
			pp	= NULL;
		}
	}

	if	(pp)
	{
		status	= pclose(pp);
		if	(status == -1)
		{
RTL_TRDBG(1,"end popen command cmd='%s' cannot get exitstatus err=%s\n",
				name,STRERRNO);
			status	= 127;
		}
		else
			status	= WEXITSTATUS(status);
		if	(stopreport)
			strcpy	(resp,"#synchronous shell, stop report");
		else
			strcpy	(resp,"");
	}
	else
	{
		strcpy	(resp,"#synchronous shell, stop control");
		status	= 126;
	}
	resppkt.lp_u.lp_cmd_resp.rp_more = 0;
	resppkt.lp_u.lp_cmd_resp.rp_code = status;
	resppkt.lp_u.lp_cmd_resp.rp_codeemb	= 0;
	msg	= rtl_imsgAlloc(IM_DEF,IM_CMD_RECV_DATA,NULL,0);
	if	(msg && rtl_imsgDupData(msg,&resppkt,sz))
		rtl_imsgAdd(MainQ,msg);
RTL_TRDBG(1,"end popen command cmd='%s' exit=%d\n",name,status);

	if	(strcmp(name,"openssh") == 0)
		__sync_fetch_and_sub(&CmdOpenSsh,1);

	if	(strcmp(name,"rfscanv0") == 0 || strcmp(name,"rfscanv1") == 0)
		__sync_fetch_and_sub(&CmdRfScan,1);

	if	(strcmp(name,"rfscanv0") == 0)
	{
		DownRadioStop		= 0;
		LgwThreadStopped	= 0;
	}

endcmd :

	if	(downpkt && downpkt->lp_cmdname)
		free	(downpkt->lp_cmdname);
	if	(downpkt && downpkt->lp_cmdfull)
		free	(downpkt->lp_cmdfull);
	if	(downpkt)
		free	(downpkt);

RTL_TRDBG(1,"end popen thread\n");

	__sync_fetch_and_sub(&CmdThread,1);

	return	NULL;
}

// used in a child thread
static	void	*_DoSystemCmdThread(void *p)
{
	t_lrr_pkt	*downpkt	= (t_lrr_pkt *)p;
	char		*name	= NULL;
	char		*cmd	= NULL;

	t_lrr_pkt	resppkt;
	t_xlap_link	*lk;
	char		*resp;

	int		status;

	t_imsg		*msg;
	int		sz	= sizeof(t_lrr_pkt);
	u_int		cserial;

	if	(!downpkt)
		goto	endcmd;

	name	= downpkt->lp_cmdname;
	if	(!name || !*name)
		goto	endcmd;
	cmd	= downpkt->lp_cmdfull;
	if	(!cmd || !*cmd)
		goto	endcmd;

	cserial	= downpkt->lp_u.lp_shell_cmd.cm_cmd.cm_serial;
	if	(strcmp(name,"openssh") == 0)
		__sync_fetch_and_add(&CmdOpenSsh,1);

	if	(strcmp(name,"rfscanv0") == 0 || strcmp(name,"rfscanv1") == 0)
		__sync_fetch_and_add(&CmdRfScan,1);

	lk	= (t_xlap_link *)downpkt->lp_lk;
	resp	= PreResponseCommand(downpkt,&resppkt);
	resppkt.lp_lk	= lk;

RTL_TRDBG(1,"start system thread cmd='%s'\n",name);
	if	(0 && downpkt->lp_cmdtype == CMD_SYSTEM_THREAD)
	{
		sprintf	(resp,"#command %s started ser=%u",name,cserial);
		resppkt.lp_u.lp_cmd_resp.rp_seqnum	= 0;
		resppkt.lp_u.lp_cmd_resp.rp_more 	= 0;
		resppkt.lp_u.lp_cmd_resp.rp_code	= 0;
		resppkt.lp_u.lp_cmd_resp.rp_codeemb	= 0;
		resppkt.lp_u.lp_cmd_resp.rp_continue	= 1;

		msg	= rtl_imsgAlloc(IM_DEF,IM_CMD_RECV_DATA,NULL,0);
		if	(msg && rtl_imsgDupData(msg,&resppkt,sz))
			rtl_imsgAdd(MainQ,msg);
	}
	if	(downpkt->lp_cmdtype == CMD_SYSTEM_PENDING_THREAD)
	{
		CreateCmdPending(downpkt);
		sprintf	(resp,"#command %s started ser=%u (end not signaled)",name,cserial);
		resppkt.lp_u.lp_cmd_resp.rp_seqnum	= 0;
		resppkt.lp_u.lp_cmd_resp.rp_more 	= 0;
		resppkt.lp_u.lp_cmd_resp.rp_code	= 0;
		resppkt.lp_u.lp_cmd_resp.rp_codeemb	= 0;
		resppkt.lp_u.lp_cmd_resp.rp_continue	= 0;

		msg	= rtl_imsgAlloc(IM_DEF,IM_CMD_RECV_DATA,NULL,0);
		if	(msg && rtl_imsgDupData(msg,&resppkt,sz))
			rtl_imsgAdd(MainQ,msg);
	}
	status	= system(cmd);
	status	= WEXITSTATUS(status);
	if	(downpkt->lp_cmdtype == CMD_SYSTEM_THREAD)
	{
		sprintf	(resp,"#command %s ended ser=%u",name,cserial);
		resppkt.lp_u.lp_cmd_resp.rp_seqnum	= 0;
		resppkt.lp_u.lp_cmd_resp.rp_more 	= 0;
		resppkt.lp_u.lp_cmd_resp.rp_code	= status;
		resppkt.lp_u.lp_cmd_resp.rp_codeemb	= 0;
		resppkt.lp_u.lp_cmd_resp.rp_continue	= 0;
		msg	= rtl_imsgAlloc(IM_DEF,IM_CMD_RECV_DATA,NULL,0);
		if	(msg && rtl_imsgDupData(msg,&resppkt,sz))
			rtl_imsgAdd(MainQ,msg);
	}
	if	(downpkt->lp_cmdtype == CMD_SYSTEM_PENDING_THREAD)
	{
		TerminateCmdPending(downpkt,status);
	}
RTL_TRDBG(1,"end system command cmd='%s' exit=%d\n",name,status);

	if	(strcmp(name,"openssh") == 0)
		__sync_fetch_and_sub(&CmdOpenSsh,1);

	if	(strcmp(name,"rfscanv0") == 0 || strcmp(name,"rfscanv1") == 0)
		__sync_fetch_and_sub(&CmdRfScan,1);

	if	(strcmp(name,"rfscanv0") == 0)
	{
		DownRadioStop		= 0;
		LgwThreadStopped	= 0;
	}

endcmd :

	if	(downpkt && downpkt->lp_cmdname)
		free	(downpkt->lp_cmdname);
	if	(downpkt && downpkt->lp_cmdfull)
		free	(downpkt->lp_cmdfull);
	if	(downpkt)
		free	(downpkt);

RTL_TRDBG(1,"end system thread\n");
	__sync_fetch_and_sub(&CmdThread,1);

	return	NULL;
}

// used in main thread
static	int	StarterCmdThread(t_lrr_pkt *downpkt,char *name,char *cmd)
{
	pthread_attr_t	cmdThreadAt;
	pthread_t	cmdThread;
	t_lrr_pkt	*dpkt;

	DoEnvCmd();

	if	(!downpkt || !name || !*name || !cmd || !*cmd)
		return	-1;
	name	= strdup(name);
	if	(!name)
		return	-1;
	cmd	= strdup(cmd);
	if	(!cmd)
	{
		free(name);
		return	-1;
	}
	dpkt	= (t_lrr_pkt *)malloc(sizeof(t_lrr_pkt));
	if	(!dpkt)
	{
		free(name);
		free(cmd);
		return	-1;
	}
	memcpy	(dpkt,downpkt,sizeof(t_lrr_pkt));
	dpkt->lp_cmdname	= name;
	dpkt->lp_cmdfull	= cmd;

	if	(pthread_attr_init(&cmdThreadAt))
	{
		RTL_TRDBG(0,"cannot init cmd thread attr err=%s\n",STRERRNO);
		free(dpkt);
		return	-1;
	}

	pthread_attr_setdetachstate(&cmdThreadAt,PTHREAD_CREATE_DETACHED);

	if	(dpkt->lp_cmdtype == CMD_SYSTEM_THREAD)
	{
		if	(pthread_create(&cmdThread,&cmdThreadAt,
						_DoSystemCmdThread,dpkt))
		{
			RTL_TRDBG(0,"cannot create system thread err=%s\n",
								STRERRNO);
			return	-1;
		}
		__sync_fetch_and_add(&CmdThread,1);
		return	0;
	}

	if	(dpkt->lp_cmdtype == CMD_SYSTEM_PENDING_THREAD)
	{
		if	(pthread_create(&cmdThread,&cmdThreadAt,
						_DoSystemCmdThread,dpkt))
		{
			RTL_TRDBG(0,"cannot create system thread err=%s\n",
								STRERRNO);
			return	-1;
		}
		__sync_fetch_and_add(&CmdThread,1);
		return	0;
	}


	if	(dpkt->lp_cmdtype == CMD_POPEN_THREAD)
	{
		if	(pthread_create(&cmdThread,&cmdThreadAt,
							_DoPopenCmdThread,dpkt))
		{
			RTL_TRDBG(0,"cannot create popen thread err=%s\n",
								STRERRNO);
			return	-1;
		}
		__sync_fetch_and_add(&CmdThread,1);
		return	0;
	}

	if	(dpkt->lp_cmdtype == CMD_BUILTIN_THREAD)
	{
		if	(pthread_create(&cmdThread,&cmdThreadAt,
						_DoBuiltinCmdThread,dpkt))
		{
			RTL_TRDBG(0,"cannot create cmd thread err=%s\n",
								STRERRNO);
			return	-1;
		}
		__sync_fetch_and_add(&CmdThread,1);
		return	0;
	}

	free(dpkt);

	return	-1;
}


// used in main thread
static	int	BuiltInCmd(t_lrr_pkt *downpkt,char *parcmd,char *params)
{
	t_lrr_pkt	resppkt;
	t_xlap_link	*lk;
	char		*resp;
	char		cmd[256];

	strcpy	(cmd,parcmd);
	lk	= (t_xlap_link *)downpkt->lp_lk;

	resp	= PreResponseCommand(downpkt,&resppkt);
	if	(strcmp(cmd,"test") == 0)
	{
		int	i,nb;

		if	(!params || !*params)	params	= "1";
		nb	= atoi(params);
		if	(nb <= 0 || nb > 100)
			nb	= 100;
		resppkt.lp_u.lp_cmd_resp.rp_seqnum	= 0;
		resppkt.lp_u.lp_cmd_resp.rp_more	= 1;
		for	(i = 0 ; i < nb ; i++)
		{
			sprintf	(resp,"test%d",i);

			if	(i == nb-1)
				resppkt.lp_u.lp_cmd_resp.rp_more	= 0;
			LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
			resppkt.lp_u.lp_cmd_resp.rp_seqnum++;
		}
		return	CMD_BUILTIN;
	}
	if	(strcmp(cmd,"ttest") == 0)
	{
		return	CMD_BUILTIN_THREAD;
	}
	if	(strcmp(cmd,"tail") == 0)
	{
		return	CMD_POPEN_THREAD;
	}
#ifdef	LP_TP31
	if	(strcmp(cmd,"dtc") == 0)
	{
		char	log[512];
		FILE	*f;

		if	(LgwThreadStopped)
		{
			TerminateCmdBuiltIn(downpkt,1,"radio stopped");
			LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
			return	CMD_BUILTIN;
		}

		sprintf	(log,"%s/var/log/lrr/DTC.log",RootAct);
		f	= fopen(log,"w");
		if	(!f)
		{
			TerminateCmdBuiltIn(downpkt,1,"can not save dtc file");
			LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
			return	CMD_BUILTIN;
		}
//		DcSaveCounters(f);
#ifndef		__clang__
		inline	void	fctprint(void *pf,int type,int ant,int idx,
			float up,float dn)
		{
			switch	(type)
			{
			case	'C':	// channel
				fprintf(f,"ant=%d chan=%03d up=%f dn=%f\n",
								ant,idx,up,dn);
				fflush(f);
			break;
			case	'S':	// subband
				fprintf(f,"ant=%d subb=%03d up=%f dn=%f\n",
								ant,idx,up,dn);
				fflush(f);
			break;
			}
		}
		DcWalkCounters(f,fctprint);
#else
		RTL_TRDBG(0,"LRR compiled with clang !!!\n");
#endif
		fclose(f);
		// cat of the result will be done by dtc.sh
		return	CMD_POPEN_THREAD;
	}
	if	(strcmp(cmd,"dtclt") == 0)
	{
		// generate dtclt.log
		DcDumpHisto();
		// use dtclt.sh to cat dtclt.log
		return	CMD_POPEN_THREAD;
	}
#endif
	if	(strcmp(cmd,"rfregion") == 0)
	{
		char	*reg;

		reg	= RfRegionId ? RfRegionId : "";
		sprintf	(resp,"ID=%s\nVERSION=%u\nISM=%s\nISMALTER=%s\n",
			reg,RfRegionIdVers,IsmBand,IsmBandAlter);
		TerminateCmdBuiltIn(downpkt,0,resp);
		LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
		return	CMD_BUILTIN;
	}
#ifdef	WITH_GPS
	if	(strcmp(cmd,"gpslocation") == 0)
	{
		u_char	locmeth;
		float	lat;
		float	lon;
		short	alt;
		u_int	cnt;
		GpsGetInfos(&locmeth,&lat,&lon,&alt,&cnt);
		sprintf	(resp,
			"MODE=%d\nLAT=%f\nLON=%f\nALT=%d\nCNT=%u\n"
			"LAT_av=%f\nLON_av=%f\nALT_av=%d\nLOC_rej=%u\n",
			locmeth,lat,lon,alt,cnt,LrrLat,LrrLon,LrrAlt,NbLocRjTot);
		TerminateCmdBuiltIn(downpkt,0,resp);
		LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
		return	CMD_BUILTIN;
	}
#else	// even if no GPS device we have manual coordinates
	if	(strcmp(cmd,"gpslocation") == 0)
	{
		u_char	locmeth;
		float	lat;
		float	lon;
		short	alt;
		u_int	cnt;
		GpsGetInfos(&locmeth,&lat,&lon,&alt,&cnt);
		sprintf	(resp,
			"MODE=%d\nLAT=%f\nLON=%f\nALT=%d\nCNT=%u\n",
			locmeth,lat,lon,alt,cnt);
		TerminateCmdBuiltIn(downpkt,0,resp);
		LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
		return	CMD_BUILTIN;
	}
#endif
	if	(strcmp(cmd,"beacon") == 0)
	{
		struct	timeval	tv;
		char	when[128];

		memset	(&tv,0,sizeof(tv));
		tv.tv_sec		= LgwBeaconUtcTime.tv_sec;
		rtl_gettimeofday_to_iso8601date(&tv,NULL,when);

		sprintf	(resp,"REQCNT=%u\nSNDCNT=%u\nDUPCNT=%u\nLATCNT=%u\nUTCSEC=%u\nUTC=%s\nLASTCAUSE=%02x\n",
		LgwBeaconRequestedCnt,LgwBeaconSentCnt,
		LgwBeaconRequestedDupCnt,LgwBeaconRequestedLateCnt,
		(u_int)tv.tv_sec,when,LgwBeaconLastDeliveryCause);
		TerminateCmdBuiltIn(downpkt,0,resp);
		LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
		return	CMD_BUILTIN;
	}
	if	(strcmp(cmd,"classcmc") == 0)
	{
		struct	timeval	tv;
		char	when[128];

		memset	(&tv,0,sizeof(tv));
		tv.tv_sec		= LgwClassCUtcTime.tv_sec;
		rtl_gettimeofday_to_iso8601date(&tv,NULL,when);

		sprintf	(resp,"REQCNT=%u\nSNDCNT=%u\nDUPCNT=%u\nLATCNT=%u\nUTCSEC=%u\nUTC=%s\nLASTCAUSE=%02x\n",
		LgwClassCRequestedCnt,LgwClassCSentCnt,
		LgwClassCRequestedDupCnt,LgwClassCRequestedLateCnt,
		(u_int)tv.tv_sec,when,LgwClassCLastDeliveryCause);
		TerminateCmdBuiltIn(downpkt,0,resp);
		LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
		return	CMD_BUILTIN;
	}
#ifdef	WITH_SX1301_X1
// RDTP-5475
	if	(strcmp(cmd,"synchrorefresh") == 0)
	{
		SynchroForced	= 1;
		strcpy	(resp,"");
		TerminateCmdBuiltIn(downpkt,0,resp);
		LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
		return	CMD_BUILTIN;
	}
	if	(strcmp(cmd,"synchroreset") == 0)
	{
		SynchroReset	= 1;
		strcpy	(resp,"");
		TerminateCmdBuiltIn(downpkt,0,resp);
		LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
		return	CMD_BUILTIN;
	}
#endif
	if	(strcmp(cmd,"filterrefresh") == 0)
	{
		extern	void SendNetworkFilterRequestCmd();
		SendNetworkFilterRequestCmd();
		strcpy	(resp,"");
		TerminateCmdBuiltIn(downpkt,0,resp);
		LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
		return	CMD_BUILTIN;
	}
	if	(strcmp(cmd,"configrefresh") == 0)
	{
		DoConfigRefresh(0);
		strcpy	(resp,"");
		TerminateCmdBuiltIn(downpkt,0,resp);
		LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
		return	CMD_BUILTIN;
	}
	if	(strcmp(cmd,"statrefresh") == 0)
	{
		DoStatRefresh(0);
		strcpy	(resp,"");
		TerminateCmdBuiltIn(downpkt,0,resp);
		LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
		return	CMD_BUILTIN;
	}
	if	(strcmp(cmd,"rfcellrefresh") == 0)
	{
		DoRfcellRefresh(0);
		strcpy	(resp,"");
		TerminateCmdBuiltIn(downpkt,0,resp);
		LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
		return	CMD_BUILTIN;
	}
	if	(strcmp(cmd,"version") == 0)
	{
		sprintf	(resp,"%d.%d.%d",VersionMaj,VersionMin,VersionRel);
		TerminateCmdBuiltIn(downpkt,0,resp);
		LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
		return	CMD_BUILTIN;
	}
#ifdef	SX1301AR_MAX_BOARD_NB
	if	(strcmp(cmd, "getkeys") == 0)
	{
		DoLocKeyRefresh(0, &resp);
		TerminateCmdBuiltIn(downpkt, 0, resp);
		LapPutOutQueue(lk, (u_char *)&resppkt, resppkt.lp_szh);
		return	CMD_BUILTIN;
	}
#else
	if	(strcmp(cmd, "getkeys") == 0)
	{
		strcpy(resp,"# not supported");
		resppkt.lp_u.lp_cmd_resp.rp_code	= 1;
		LapPutOutQueue(lk, (u_char *)&resppkt, resppkt.lp_szh);
		return	CMD_BUILTIN;
	}
#endif
	if	(strcmp(cmd, "getants") == 0)
	{
		DoAntsConfigRefresh(0, &resp);
		TerminateCmdBuiltIn(downpkt, 0, resp);
		LapPutOutQueue(lk, (u_char *)&resppkt, resppkt.lp_szh);
		return	CMD_BUILTIN;
	}
	if	(strcmp(cmd,"versions") == 0)
	{
		resppkt.lp_u.lp_cmd_resp.rp_more	= 1;
		strcpy	(resp,rtl_version());
		LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
		resppkt.lp_u.lp_cmd_resp.rp_seqnum++;
		resppkt.lp_u.lp_cmd_resp.rp_more	= 1;
		strcpy	(resp,lrr_whatStr);
		LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
		resppkt.lp_u.lp_cmd_resp.rp_seqnum++;
		resppkt.lp_u.lp_cmd_resp.rp_more	= 0;
		strcpy	(resp,LgwVersionInfo());
		TerminateCmdBuiltIn(downpkt,0,resp);
		LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
		return	CMD_BUILTIN;
	}
#ifdef _HALV_COMPAT
	if	(strcmp(cmd,"halversion") == 0)
	{
		resppkt.lp_u.lp_cmd_resp.rp_more	= 1;
		strcpy	(resp, hal_version);
		LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
		resppkt.lp_u.lp_cmd_resp.rp_seqnum++;
		resppkt.lp_u.lp_cmd_resp.rp_more	= 0;
		strcpy	(resp,LgwVersionInfo());
		TerminateCmdBuiltIn(downpkt,0,resp);
		LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
		return	CMD_BUILTIN;
	}
#endif /* _HALV_COMPAT */
	if	(strcmp(cmd,"stop") == 0)
	{
		MainWantStop	= 1;
		MainStopDelay	= 5;
		if	(params && *params && atoi(params) > 5)
			MainStopDelay	= atoi(params);
		sprintf	(resp,"#lrr will be restarted in %d sec",MainStopDelay);
		TerminateCmdBuiltIn(downpkt,0,resp);
		LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
		return	CMD_BUILTIN;
	}
	if	(strcmp(cmd,"uptimesyst") == 0)
	{
		strcpy	(resp,UptimeSystStr);
		TerminateCmdBuiltIn(downpkt,0,resp);
		LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
		return	CMD_BUILTIN;
	}
	if	(strcmp(cmd,"uptimeproc") == 0)
	{
		strcpy	(resp,UptimeProcStr);
		TerminateCmdBuiltIn(downpkt,0,resp);
		LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
		return	CMD_BUILTIN;
	}
	if	(strcmp(cmd,"rfscanv0") == 0)
	{
		t_imsg	*msg;
		strcpy	(resp,"");
		msg	= rtl_imsgAlloc(IM_DEF,IM_LGW_EXIT,NULL,0);
		if	(msg)
		{
			rtl_imsgAdd(LgwQ,msg);
			LgwThreadStopped	= 1;
		}
		return	CMD_SYSTEM_PENDING_THREAD;
//		return	CMD_POPEN_THREAD;
	}
	if	(strcmp(cmd,"radiorestart") == 0)
	{
#ifdef	WITH_TTY
		strcpy(resp,"# not supported");
		resppkt.lp_u.lp_cmd_resp.rp_code	= 1;
		TerminateCmdBuiltIn(downpkt,0,resp);
		LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
		return	CMD_BUILTIN;
#endif
		strcpy	(resp,"");
		DownRadioStop		= 0;
		CfgRadioDnStop		= 0;
		LgwThreadStopped	= 0;
		SaveConfigFileState();
		ReStartLgwThread();
		TerminateCmdBuiltIn(downpkt,0,resp);
		LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
		SendRfCellNow = 1;
		return	CMD_BUILTIN;
	}
	if	(strcmp(cmd,"radiostop") == 0)
	{
		if	(LgwThreadStopped)
			strcpy	(resp,"#already stopped");
		else
		{
			t_imsg	*msg;
			strcpy	(resp,"#stopped");
			msg	= rtl_imsgAlloc(IM_DEF,IM_LGW_EXIT,NULL,0);
			if	(msg)
			{
				rtl_imsgAdd(LgwQ,msg);
				LgwThreadStopped	= 1;
			}
		}
		TerminateCmdBuiltIn(downpkt,0,resp);
		LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
		CfgRadioStop	= 1;
		SaveConfigFileState();
		SendRfCellNow = 1;
		return	CMD_BUILTIN;
	}
	if	(strcmp(cmd,"radiostart") == 0)
	{
		if	(LgwThreadStopped == 0)
			strcpy	(resp,"#already started");
		else
			strcpy	(resp,"#started");
		DownRadioStop		= 0;
		LgwThreadStopped	= 0;
		TerminateCmdBuiltIn(downpkt,0,resp);
		LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
		CfgRadioStop	= 0;
		CfgRadioDnStop	= 0;
		SaveConfigFileState();
		SendRfCellNow = 1;
		return	CMD_BUILTIN;
	}
	if	(strcmp(cmd,"dnradiostop") == 0)
	{
		if	(DownRadioStop)
			strcpy	(resp,"#already downstopped");
		else
		{
			strcpy	(resp,"#downstopped");
			DownRadioStop	= 1;
		}
		TerminateCmdBuiltIn(downpkt,0,resp);
		LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
		CfgRadioDnStop	= 1;
		SaveConfigFileState();
		SendRfCellNow = 1;
		return	CMD_BUILTIN;
	}
	if	(strcmp(cmd,"dnradiostart") == 0)
	{
		if	(DownRadioStop == 0)
			strcpy	(resp,"#already downstarted");
		else
		{
			strcpy	(resp,"#downstarted");
			DownRadioStop	= 0;
		}
		TerminateCmdBuiltIn(downpkt,0,resp);
		LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
		CfgRadioDnStop	= 0;
		SaveConfigFileState();
		SendRfCellNow = 1;
		return	CMD_BUILTIN;
	}
	if	(strcmp(cmd,"radiostatus") == 0)
	{
		if	(LgwThreadStopped)
			strcpy	(resp,"stopped");
		else
			if	(DownRadioStop)
				strcpy	(resp,"downstopped");
			else
				strcpy	(resp,"started");
		TerminateCmdBuiltIn(downpkt,0,resp);
		LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
		return	CMD_BUILTIN;
	}
	if	(strcmp(cmd,"cmdstatus") == 0)
	{
		u_int	serial	= 0;
		int	ret;
		char	file[PATH_MAX];
		char	file2[PATH_MAX];
		FILE	*f;

		ret	= sscanf(params,"%u",&serial);
		if	(ret != 1 || serial == 0)
		{
			strcpy	(resp,"#cmdstatus <cmdserial>");
			resppkt.lp_u.lp_cmd_resp.rp_code	= 255;
			resppkt.lp_u.lp_cmd_resp.rp_codeemb	= 0;
			LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
			return	CMD_BUILTIN;
		}
		sprintf	(file,"%s/%u_pending",DirCommands,serial);
		if	(access(file,R_OK) == 0)
		{
			sprintf	(file2,"%s/%u.pid",DirCommands,serial);
			if	(access(file2,R_OK) == 0 && (f = fopen(file2,"r")))
			{
				ret	= 0;
				fscanf	(f,"%d",&ret);
				fclose	(f);
				if	(kill(ret,0) != 0)
				{
					strcpy	(resp,"#command status error");
					resppkt.lp_u.lp_cmd_resp.rp_code	= 254;
					resppkt.lp_u.lp_cmd_resp.rp_codeemb	= 0;
					LapPutOutQueue(lk,(u_char *)&resppkt,
								resppkt.lp_szh);
					return	CMD_BUILTIN;
				}
			}
			strcpy	(resp,"#command still in progress");
			resppkt.lp_u.lp_cmd_resp.rp_code	= 1;
			resppkt.lp_u.lp_cmd_resp.rp_codeemb	= 0;
			LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
			return	CMD_BUILTIN;
		}
		sprintf	(file,"%s/%u_terminated",DirCommands,serial);
		f	= fopen(file,"r");
		if	(f)
		{
			ret	= 0;
			fscanf	(f,"%d",&ret);
			fclose	(f);
			sprintf	(resp,"#command completed\n%d",ret);
			resppkt.lp_u.lp_cmd_resp.rp_code	= 0;
			resppkt.lp_u.lp_cmd_resp.rp_codeemb	= ret;
			LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
			return	CMD_BUILTIN;
		}

		strcpy	(resp,"#command not found");
		resppkt.lp_u.lp_cmd_resp.rp_code	= 2;
		resppkt.lp_u.lp_cmd_resp.rp_codeemb	= 0;
		LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
		return	CMD_BUILTIN;
	}
	if	(strcmp(cmd,"cmdresult") == 0)
	{
		u_int	serial	= 0;
		int	ret;
		char	file[PATH_MAX];
		char	file2[PATH_MAX];
		FILE	*f;

		ret	= sscanf(params,"%u",&serial);
		if	(ret != 1 || serial == 0)
		{
			strcpy	(resp,"#cmdresult <cmdserial>");
			resppkt.lp_u.lp_cmd_resp.rp_code	= 255;
			resppkt.lp_u.lp_cmd_resp.rp_codeemb	= 0;
			LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
			return	CMD_BUILTIN;
		}
		sprintf	(file,"%s/%u_pending",DirCommands,serial);
		if	(access(file,R_OK) == 0)
		{
			sprintf	(file2,"%s/%u.pid",DirCommands,serial);
			if	(access(file2,R_OK) == 0 && (f = fopen(file2,"r")))
			{
				ret	= 0;
				fscanf	(f,"%d",&ret);
				fclose	(f);
				if	(kill(ret,0) != 0)
				{
					strcpy	(resp,"#command status error");
					resppkt.lp_u.lp_cmd_resp.rp_code	= 254;
					resppkt.lp_u.lp_cmd_resp.rp_codeemb	= 0;
					LapPutOutQueue(lk,(u_char *)&resppkt,
								resppkt.lp_szh);
					return	CMD_BUILTIN;
				}
			}
			strcpy	(resp,"#command still in progress");
			resppkt.lp_u.lp_cmd_resp.rp_code	= 1;
			resppkt.lp_u.lp_cmd_resp.rp_codeemb	= 0;
			LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
			return	CMD_BUILTIN;
		}
		sprintf	(file,"%s/%u_terminated",DirCommands,serial);
		if	(access(file,R_OK) != 0)
		{
			strcpy	(resp,"#command not found");
			resppkt.lp_u.lp_cmd_resp.rp_code	= 2;
			resppkt.lp_u.lp_cmd_resp.rp_codeemb	= 0;
			LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
			return	CMD_BUILTIN;
		}
		sprintf	(file,"%s/%u.log",DirCommands,serial);
		if	(access(file,R_OK) != 0)
		{
			strcpy	(resp,"#command no result");
			resppkt.lp_u.lp_cmd_resp.rp_code	= 3;
			resppkt.lp_u.lp_cmd_resp.rp_codeemb	= 0;
			LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
			return	CMD_BUILTIN;
		}
		// the "cat" of command traces is done by cmdresult.sh
		return	CMD_POPEN_THREAD;
	}
	if	(strcmp(cmd,"countopenssh") == 0)
	{
		sprintf	(resp,"%u",CmdCountOpenSsh());
		TerminateCmdBuiltIn(downpkt,0,resp);
		LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
		return	CMD_BUILTIN;
	}
	if	(strcmp(cmd,"countrfscan") == 0)
	{
		sprintf	(resp,"%u",CmdCountRfScan());
		TerminateCmdBuiltIn(downpkt,0,resp);
		LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
		return	CMD_BUILTIN;
	}
	if	(!strcmp(cmd,"countcmdthread") || !strcmp(cmd,"countcmd"))
	{
		sprintf	(resp,"%u",CmdThread);
		TerminateCmdBuiltIn(downpkt,0,resp);
		LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
		return	CMD_BUILTIN;
	}
#ifdef	WITH_USB_PROTECTION
	if	(!strcmp(cmd,"usbprotecton"))
	{
		strcpy	(resp,"");
		resppkt.lp_u.lp_cmd_resp.rp_code	= UsbProtectOn();
		TerminateCmdBuiltIn(downpkt,0,resp);
		LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
		return	CMD_BUILTIN;
	}
	if	(!strcmp(cmd,"usbprotectoff"))
	{
		strcpy	(resp,"");
		resppkt.lp_u.lp_cmd_resp.rp_code	= UsbProtectOff();
		TerminateCmdBuiltIn(downpkt,0,resp);
		LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
		return	CMD_BUILTIN;
	}
#endif
	if	(strcmp(cmd,"trace") == 0 || strcmp(cmd,"tracep") == 0)
	{
		int	tr	= 0;
		int	ret;
		char	tmp[32];
		extern	int TraceLevelP;

		ret	= sscanf(params,"%d",&tr);
		if	(ret != 1)
		{
		}
		else
		{
			if	(tr >= -1 && tr <= 3)
			{
				TraceLevelP	= tr;
				if (tr >= 0)
					TraceLevel	= tr;
				else
					TraceLevel	= CfgInt(HtVarLrr,"trace",-1,"level",TraceLevel);
				// TRDBG level 0 to display it whatever is the tracelevel
				RTL_TRDBG(0,"Trace level set to %d\n", TraceLevel);
				rtl_tracelevel(TraceLevel);
				if	(strcmp(cmd,"tracep") == 0)
					SaveConfigFileState();
			}
			else
				resppkt.lp_u.lp_cmd_resp.rp_code	= 1;
		}
		if	(TraceLevelP < 0)
			strcpy(tmp,"<unset>");
		else
			sprintf	(tmp,"%d",TraceLevelP);
		sprintf	(resp,"LEVEL=%d\nRAMDIR=%d\nLEVELP=%s\nFILESIZE=%d\n",
				TraceLevel,LogUseRamDir,tmp,TraceSize);
		LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
		return	CMD_BUILTIN;
	}
	// Just for testing NFR997 until lrc send partition id
	if	(strcmp(cmd,"masterlrc") == 0)
	{
		int	idx	= 0;
		int	ret;

		ret	= sscanf(params,"%d",&idx);
		if	(ret == 1)
		{
			MasterLrc = idx;
			RTL_TRDBG(1,"Set masterlrc=%d\n\n", MasterLrc);
		}
		strcpy	(resp,"");
		TerminateCmdBuiltIn(downpkt,0,resp);
		LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
		return	CMD_BUILTIN;
	}
	if	(strcmp(cmd,"tmoa") == 0)
	{
		int	sf;
		int	bw;
		int	sz	= 0;
		int	ret;
		double	tmoa	= 0;
		t_lrr_pkt	tpkt;
#if		defined(WIRMAV2) || defined(MTAC_REFRESH_V1_5)
		struct lgw_pkt_tx_s	hpkt;
		memset	(&hpkt,0,sizeof(hpkt));
		hpkt.modulation		= MOD_LORA;
#endif

		memset	(&tpkt,0,sizeof(tpkt));
		ret	= sscanf(params,"%d",&sz);
		if	(ret == 1)
		{
			if	(sz > 256)	sz	= 256;
			tpkt.lp_size	= sz;
		}
		tpkt.lp_correct	= 5;
		resppkt.lp_u.lp_cmd_resp.rp_more        = 1;
		resppkt.lp_u.lp_cmd_resp.rp_seqnum	= 0;
		sprintf	(resp,"sz=%d bw=%d/%fKHz bw=%d/%fKHz bw=%d/%fKHz",sz,
			0,FreqBandWidth(0)/1000,
			1,FreqBandWidth(1)/1000,
			2,FreqBandWidth(2)/1000);
		LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
		resppkt.lp_u.lp_cmd_resp.rp_seqnum++;
		for	(bw = 0 ; bw <= 2 ; bw++)	// 125,250,500
		{
			for	(sf = 7 ; sf <= 12 ; sf++)
			{
				tpkt.lp_bandwidth	= bw;
				tpkt.lp_spfact		= sf;
				TmoaLrrPacket(&tpkt);
#if		defined(WIRMAV2) || defined(MTAC_REFRESH_V1_5)
				hpkt.no_header	= LgwNoHeader;
				hpkt.size	= tpkt.lp_size;
				hpkt.bandwidth	= DeCodeBandWidth(bw);
				hpkt.datarate	= DecodeSpreadingFactor(sf);
				hpkt.coderate	= DecodeCorrectingCode(5);
				tmoa	= (double)lgw_time_on_air(&hpkt)/1000.0;
#endif
				sprintf	(resp,"bw=%d sf=%02d t=%f hal=%f",bw,sf,
				(double)tpkt.lp_tmoa/1000.0/1000.0,tmoa);
				resppkt.lp_u.lp_cmd_resp.rp_seqnum++;
				LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
			}
		}
		resppkt.lp_u.lp_cmd_resp.rp_seqnum++;
		resppkt.lp_u.lp_cmd_resp.rp_more        = 0;
		strcpy	(resp,"");
		TerminateCmdBuiltIn(downpkt,0,resp);
		LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
		return	CMD_BUILTIN;
	}
	if	(strcmp(cmd,"malloc") == 0)
	{
		u_int	szmalloc;
		struct	mallinfo info;

		info	= mallinfo();
		szmalloc= info.uordblks;

		sprintf	(resp,"SZMALLOC=%d\n",szmalloc);
		LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
		return	CMD_BUILTIN;
	}
	return	CMD_OTHER;
}

// used in main thread
void	DoShellCommand(t_lrr_pkt *downpkt)
{
	t_lrr_shell_cmd	*shell;
	char		cmd[1024];
	char		syscmd[1024];
	char		cmdserial[1024];
	char		trace[1024];
	int		ret;
	char		*params;
	char		*pt;
	u_int		cserial;

	t_lrr_pkt	resppkt;
	t_xlap_link	*lk;
	char		*resp;

	// in case of immediate response
	lk	= (t_xlap_link *)downpkt->lp_lk;
	resp	= PreResponseCommand(downpkt,&resppkt);


	downpkt->lp_cmdtype	= CMD_SYSTEM_DETACH;	// default
	shell	= &(downpkt->lp_u.lp_shell_cmd);
	cserial	= shell->cm_cmd.cm_serial;
	cmd[0]	= '\0';
	ret	= sscanf((char *)shell->sh_cmd,"%s",cmd);
	if	(ret != 1 || strlen(cmd) == 0)
		return;
	if	(strchr(cmd,'.') || strchr(cmd,'/') || strchr(cmd,'$'))
	{
		if	(strcmp(cmd,"../sysconfiglrr"))
		{
			sprintf	(resp,"#unauthorized char in command name");
			resppkt.lp_u.lp_cmd_resp.rp_code = 255;
			LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
			return;
		}
	}

	// set pt on the last char of the command
	pt = (char *) shell->sh_cmd + strlen((char *)shell->sh_cmd) - 1;
	while (pt > (char *) shell->sh_cmd && isspace(*pt))
        pt--;
	
	if      (*pt == '&')
	{
        	*pt     = ' ';
	        downpkt->lp_cmdtype     = CMD_SYSTEM_DETACH;
	}
	if      (*pt == '?')
	{
        	*pt     = ' ';
	        downpkt->lp_cmdtype     = CMD_POPEN_THREAD;
	}
	if      (*pt == '!')
	{
        	*pt     = ' ';
	        downpkt->lp_cmdtype     = CMD_SYSTEM_THREAD;
	}
	if      (*pt == ';')
	{
        	*pt     = ' ';
	        downpkt->lp_cmdtype     = CMD_SYSTEM_PENDING_THREAD;
	}

	params	= (char *)shell->sh_cmd+strlen(cmd);
	while	(*params && (*params == ' ' || *params == '\t')) params++;

	ret	= BuiltInCmd(downpkt,cmd,params);

	RTL_TRDBG(1,"shell ret=%d type=%d cmd='%s' params='%s'\n",
				ret,downpkt->lp_cmdtype,cmd,params);

	if	(ret == CMD_BUILTIN)
		return;

	if	(ret == CMD_BUILTIN_THREAD || 
			downpkt->lp_cmdtype == CMD_BUILTIN_THREAD)
	{
		downpkt->lp_cmdtype	= CMD_BUILTIN_THREAD;
		if	(CmdThread >= CMD_MAX_THREAD)
		{
			sprintf	(resp,"#too much threads (%d)",CmdThread);
			resppkt.lp_u.lp_cmd_resp.rp_code = 255;
			LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
			return;
		}
		sprintf	(syscmd,"%s %s",cmd,params);
		ret	= StarterCmdThread(downpkt,cmd,syscmd);
		if	(ret < 0)
		{
			strcpy	(resp,"#can not start thread+builtin");
			resppkt.lp_u.lp_cmd_resp.rp_code = 255;
			LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
			return;
		}
		return;
	}

	// add -Z cmdserial to final command
	sprintf	(cmdserial,"%s -Z %u %s",cmd,cserial,params);

	if	(strcmp(cmd,"upgrade") == 0)
		sprintf	(trace,"%s/var/log/lrr/UPGRADE.log",RootAct);
	else
		sprintf	(trace,"%s/var/log/lrr/SHELL.log",RootAct);


	if	(ret == CMD_POPEN_THREAD ||
			downpkt->lp_cmdtype == CMD_POPEN_THREAD)// ended with ?
	{	// thread + popen(3C)
		downpkt->lp_cmdtype	= CMD_POPEN_THREAD;
		if	(CmdThread >= CMD_MAX_THREAD)
		{
			sprintf	(resp,"#too much threads (%d)",CmdThread);
			resppkt.lp_u.lp_cmd_resp.rp_code = 255;
			LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
			return;
		}
		sprintf	(syscmd,"./shelllrr.sh %s 2>&1",cmdserial);
		RTL_TRDBG(1,"popen(%s) cmd='%s'\n",syscmd,cmd);

#if	1	// start a "popen thread"
		ret	= StarterCmdThread(downpkt,cmd,syscmd);
		if	(ret < 0)
		{
			strcpy	(resp,"#can not start thread+popen(3C)");
			resppkt.lp_u.lp_cmd_resp.rp_code = 255;
			LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
			return;
		}
#else		// run popen in main thread for tests only
		t_lrr_pkt	*pkt;
		pkt	= (t_lrr_pkt *)malloc(sizeof(t_lrr_pkt));
		memcpy	(pkt,downpkt,sizeof(t_lrr_pkt));
		pkt->lp_cmdname	= strdup(name);
		pkt->lp_cmdfull	= strdup(syscmd);
		_DoPopenCmd(pkt);
#endif
		return;
	}

	if	(ret == CMD_SYSTEM_THREAD ||
			downpkt->lp_cmdtype == CMD_SYSTEM_THREAD)// ended with !
	{	// thread + system(3C)
		downpkt->lp_cmdtype	= CMD_SYSTEM_THREAD;
		if	(CmdThread >= CMD_MAX_THREAD)
		{
			sprintf	(resp,"#too much threads (%d)",CmdThread);
			resppkt.lp_u.lp_cmd_resp.rp_code = 255;
			LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
			return;
		}
		sprintf	(syscmd,"./shelllrr.sh %s > %s 2>&1",
							cmdserial,trace);
		RTL_TRDBG(1,"system(%s) cmd='%s'\n",syscmd,cmd);
		ret	= StarterCmdThread(downpkt,cmd,syscmd);
		if	(ret < 0)
		{
			strcpy	(resp,"#can not start thread+system(3C)");
			resppkt.lp_u.lp_cmd_resp.rp_code = 255;
			LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
			return;
		}
		return;
	}

	if	(ret == CMD_SYSTEM_PENDING_THREAD ||
		downpkt->lp_cmdtype == CMD_SYSTEM_PENDING_THREAD)//ended with ;
	{	// thread + system(3C)
		downpkt->lp_cmdtype	= CMD_SYSTEM_PENDING_THREAD;
		if	(CmdThread >= CMD_MAX_THREAD)
		{
			sprintf	(resp,"#too much threads (%d)",CmdThread);
			resppkt.lp_u.lp_cmd_resp.rp_code = 255;
			LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
			return;
		}
		sprintf	(trace,"%s/usr/data/lrr/cmd_shells/%u.log",RootAct,
						shell->cm_cmd.cm_serial);
		sprintf	(syscmd,"./shelllrr.sh %s > %s 2>&1",
						cmdserial,trace);
		RTL_TRDBG(1,"system(%s) cmd='%s'\n",syscmd,cmd);
		ret	= StarterCmdThread(downpkt,cmd,syscmd);
		if	(ret < 0)
		{
			strcpy	(resp,"#can not start thread+system(3C)");
			resppkt.lp_u.lp_cmd_resp.rp_code = 255;
			LapPutOutQueue(lk,(u_char *)&resppkt,resppkt.lp_szh);
			return;
		}
		return;
	}

	if	(ret == CMD_SYSTEM_DETACH || 
			downpkt->lp_cmdtype == CMD_SYSTEM_DETACH)// ended with &
	{	// no thread + system(3C) + background
		downpkt->lp_cmdtype	= CMD_SYSTEM_DETACH;
		sprintf	(syscmd,"./shelllrr.sh %s > %s 2>&1",
							cmdserial,trace);
		strcat	(syscmd," &");
		RTL_TRDBG(1,"system(%s) cmd='%s'\n",syscmd,cmd);
		DoSystemCmd(downpkt,syscmd);
		return;
	}

}

