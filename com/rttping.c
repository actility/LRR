
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

/*! @file netitf.c
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
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <net/if.h>
#include <net/if_arp.h>
#include <ifaddrs.h>
#include <dirent.h>

#include <resolv.h>

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
u16 crc_ccitt(u16 crc, const u8 *buffer, int len);

// Not needed here #include "_whatstr.h"
#include "define.h"
#include "cproto.h"
#include "extern.h"

static	int	FindItfSourceAddr(char *itfname,int family,char *retaddr)
{
	struct ifaddrs *ifaddr, *ifa;
	int	found	= 0;

	if	(!itfname || !*itfname || !retaddr)
		return	-1;

	if	(getifaddrs(&ifaddr) == -1)
	{
		RTL_TRDBG(0,"getifaddrs() err=%s\n",STRERRNO);
		return	-1;
	}

	for	(ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next)
	{
//		int	ifam	= ifa->ifa_addr->sa_family;
		int	ifam;
		char	*inam	= ifa->ifa_name;
		int	lgaddr;
		int	ret;
		char	*pt;

		if	(ifa->ifa_addr == NULL)	// RDTP-6963
		{	// AF_PACKET slip interfaces have ifa_addr NULL
			continue;
		}

		ifam	= ifa->ifa_addr->sa_family;
		if	(family != ifam)
			continue;
		if	(strcmp(itfname,inam) != 0)
			continue;
		switch	(ifam)
		{
		case	AF_INET:
			lgaddr	= sizeof(struct sockaddr_in);
		break;
		case	AF_INET6:
			lgaddr	= sizeof(struct sockaddr_in6);
		break;
		default:
			continue;
		break;
		}
		ret	= getnameinfo(ifa->ifa_addr,lgaddr,retaddr,
					NI_MAXHOST,NULL,0,NI_NUMERICHOST);
		if	(ret != 0)
		{
			continue;
		}
		pt	= strchr(retaddr,'%');
		if	(pt)
			*pt	= '\0';
		found	= 1;
		break;
	}
	freeifaddrs(ifaddr);

	if	(found)
	{
		RTL_TRDBG(4,"-I%s => -I%s\n",itfname,retaddr);
	}
	else
	{
		RTL_TRDBG(0,"no source address found for %s fam=%d\n",
					itfname,family);
	}

	return	found;
}

static	int	DoPing(t_wan_itf *itf,char *targetaddr)
{
	char	cmd[1024];
	char	resp[1024];
	FILE	*pp;
	int	status;
	int	lg;

	int	ok = 0;
	int	rtt = 0;
	int	line = 0;
	int	ipv6 = 0;

	struct addrinfo hints;
	struct addrinfo *result, *rp;
	char	host[NI_MAXHOST];
	char	source[NI_MAXHOST];
	char	*sourceaddr;

	if	(!targetaddr || !*targetaddr)
		return	-1;

	host[0]	= '\0';
	memset(&hints,0,sizeof(struct addrinfo));
	hints.ai_family		= AF_UNSPEC;
	if	((status=getaddrinfo(targetaddr,NULL,&hints,&result)) != 0)
	{
		//let's force the app to re-read resolv.conf on failure and try to resolve one more time
		res_init();
		if  ((status=getaddrinfo(targetaddr,NULL,&hints,&result)) != 0)
		{
			RTL_TRDBG(3,"error getaddrinfo(%s) -> %s(%d)\n",targetaddr, gai_strerror(status),status);
			return  -1;
		}
	}
	for	(rp = result ; rp != NULL ; rp = rp->ai_next)
	{
		status	= getnameinfo(rp->ai_addr,rp->ai_addrlen,
					host,NI_MAXHOST,NULL,0,NI_NUMERICHOST);
		if	(status == 0)
		{
			if	(rp->ai_family == AF_INET6)
				ipv6	= 1;
			if	(strcmp(targetaddr,host))
			{
				RTL_TRDBG(3,"'%s' -> '%s' ipv6=%d\n",
							targetaddr,host,ipv6);
			}
			break;	// use first address
		}
	}
	freeaddrinfo(result);
	if	(strlen(host) == 0)
	{
		RTL_TRDBG(3,"error can resolve '%s'\n",targetaddr);
		return	-1;
	}
	targetaddr	= host;

	// by default we use -Iitfname option of ping
	sourceaddr	= itf->it_name;
	if	(itf->it_pingaddrsrc && strlen(itf->it_pingaddrsrc))
	{
		if	(strcmp(itf->it_pingaddrsrc,"@itf") == 0)
		{
			int	ifam	= ipv6 ? AF_INET6:AF_INET;

			source[0]	= '\0';
			status	= FindItfSourceAddr(itf->it_name,ifam,source);
			if	(status > 0 && strlen(source) > 0)
			{
				sourceaddr	= source;
			}
			else
			{	// keep -Iitfname
			}
		}
		else
		{	// other value than @itf => ip address ?
			sourceaddr	= itf->it_pingaddrsrc;
		}
	}



	cmd[0]	= '\0';

	if	(ipv6)
		strcat	(cmd,"ping6 ");
	else
		strcat	(cmd,"ping ");
#if	1
	// when using -sX where X < 4 we can not get RTT infos ...
	if	(itf->it_type == 1)
	{
		strcat	(cmd,"-s8 ");
	}
	lg	= strlen(cmd);
#else
	lg	= 0;
#endif

#if	defined(WIRMAV2) || defined(IR910) || defined(MLINUX) || defined(NATRBPI)
	sprintf	(cmd+lg," -w%d -W%d -c1 -I%s %s 2>&1",
					10,10,sourceaddr,targetaddr);
#else
	sprintf	(cmd+lg," -c1 -I%s %s 2>&1",sourceaddr,targetaddr);
#endif
	RTL_TRDBG(3,"%s\n",cmd);

	pp	= popen(cmd,"r");
	if	(!pp)
		return	-1;

	while	(fgets(resp,sizeof(resp)-10,pp) != NULL && line < 10)
	{
		char	*pt;

		line++;
		pt	= strstr(resp,"time=");
		if	(!pt)		continue;
		pt	+= 5;
		if	(!pt || !*pt)	continue;
		ok	= 1;
		rtt	= atoi(pt);
	}

	status	= pclose(pp);
	if	(status == -1)
	{
RTL_TRDBG(1,"end popen command cmd='%s' cannot get exitstatus err=%s\n",
				cmd,STRERRNO);
		return	-1;
	}
	status	= WEXITSTATUS(status);

	sprintf	(cmd,"ping on %s for %s => status=%d ok=%d tms=%d",
			itf->it_name,targetaddr,status,ok,rtt);
	RTL_TRDBG(3,"%s\n",cmd);

	if	(status != 0)
	{
		// this appends if itf is down or DNS resolution failure
		return	0;
	}

	if	(ok == 0)
	{
		// this appends if the dest addr does not answer or packet lost
		return	0;
	}
	if	(rtt == 0)
		rtt	= 1;	//not less than 1 ms
	return	rtt;
}

static	int	ComputePingRtt(t_wan_itf *itf,char *targetaddr)
{
	int	reportPeriod	= WanRefresh;
	int	rtt;

	rtt	= DoPing(itf,targetaddr);
	if	(rtt < 0)
	{
		return	-1;
	}
	itf->it_sentprtt++;
	if	(rtt == 0)
	{
		itf->it_lostprtt++;
		return	0;
	}
	if	(rtt > 0)
	{
		// separate thread and low frequency we have time to
		// compute avdv after each new value, so the main thread
		// has always fresh data
		itf->it_okayprtt++;
		AvDvUiAdd(&itf->it_avdvprtt,rtt,Currtime.tv_sec);
		itf->it_nbpkprtt =
		AvDvUiCompute(&itf->it_avdvprtt,reportPeriod,Currtime.tv_sec);
		return	rtt;
	}
	return	rtt;
}

static	void	*LoopItfThread(void *pitf)
{
	t_wan_itf	*itf		= (t_wan_itf *)pitf;
	char		*targetaddr	= CfgStr(HtVarLrr,"laplrc",0,"addr","");
	//int		rtt;

	RTL_TRDBG(0,"thread itf idx=%d name='%s' is looping\n",
			itf->it_idx,itf->it_name);

	itf->it_lastprtt	= Currtime.tv_sec;
	itf->it_lastavdv	= Currtime.tv_sec;

	// RDTP-4176
	if	(itf->it_pingaddr && *(itf->it_pingaddr))
		targetaddr	= itf->it_pingaddr;

	while	(1)
	{
		sleep	(1);
		if	(PingRttPeriod &&
			ABS(Currtime.tv_sec - itf->it_lastprtt) > PingRttPeriod)
		{
			itf->it_lastprtt	= Currtime.tv_sec;
			// warning: variable 'rtt' set but not used [-Wunused-but-set-variable]
			// rtt = ComputePingRtt(...
			ComputePingRtt(itf,targetaddr);
		}
		if	(WanRefresh &&
			ABS(Currtime.tv_sec - itf->it_lastavdv) > WanRefresh)
		{
			itf->it_lastavdv	= Currtime.tv_sec;
RTL_TRDBG(1,"ping thd %s st=%u ok=%u ls=%u nb=%d av=%u dv=%u mx=%u addr='%s'\n",
			itf->it_name,itf->it_sentprtt,itf->it_okayprtt,
			itf->it_lostprtt,
			itf->it_nbpkprtt,
			itf->it_avdvprtt.ad_aver,
			itf->it_avdvprtt.ad_sdev,
			itf->it_avdvprtt.ad_vmax,
			targetaddr);
		}
	}
	return	NULL;
}

void	StartItfThread()
{
	int	i;
	pthread_attr_t	threadAt;
	pthread_t	*thread;
	t_wan_itf	*itf;

	if	(pthread_attr_init(&threadAt))
	{
		RTL_TRDBG(0,"cannot init thread itf err=%s\n",STRERRNO);
		return;
	}

	for	(i = 0; i < NB_ITF_PER_LRR ; i++)
	{
		if	(!TbItf[i].it_enable)	continue;
		if	(!TbItf[i].it_name)	continue;

		itf		= &TbItf[i];
		itf->it_idx	= i;
		thread		= &(itf->it_thread);
		if(pthread_create(thread,&threadAt,LoopItfThread,(void *)itf))
		{
			RTL_TRDBG(0,"cannot create thread itf err=%s\n",STRERRNO);
			continue;
		}
		RTL_TRDBG(0,"thread itf idx=%d name='%s' is started\n",
			i,TbItf[i].it_name);
	}
}
