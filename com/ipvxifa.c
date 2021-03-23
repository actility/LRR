
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

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

void	Usage()
{
printf	("-4 : only ipv4 addresses\n");
printf	("-6 : only ipv6 addresses\n");
printf	("-I itfname : get addresses only for this itf\n");
printf	("-e filter : exclude addresses and interfaces starting by filter (ex fe80)\n");
printf	("-u : first address only per one interface (-I mandatory)\n");
printf	("-t : do not print addresses types 4|6\n");
printf	("-i : do not print interfaces names\n");
printf	("\n");
printf	("ex : ipvxifa.x -Ieth0 -u -6 -e fe80 =>\n\t fc00::219:99ff:fe62:8a90\n");
printf	("ex : ipvxifa.x -Ilo -u -e 127 -t =>\n\t ::1\n");
printf	("ex : ipvxifa.x -e 127 -e fe80 -e ::1 -e vmnet =>\n");
printf	("	eth0            192.168.1.11                   4\n");
printf	("	eth0            fc00::219:99ff:fe62:8a90       6\n");
}

char	*TbExclude[128];

int	main(int argc,char *argv[])
{
	int	family		= -1;	// all
	char	*itfname	= NULL;	// all
	int	exclude		= 0;	// exclude addresses started by ...

	struct ifaddrs *ifaddr, *ifa;
	char	host[NI_MAXHOST];
	int	ret;
	int	opt;
	int	uniq		= 0;
	int	ptype		= 1;
	int	pinterface	= 1;
	int	found		= 0;
	int	i;
	int	ok;

	while	((opt=getopt(argc,argv,"h46I:ue:ti")) != -1)
	{
		switch	(opt)
		{
		case	'4':
			family	= AF_INET;
		break;
		case	'6':
			family	= AF_INET6;
		break;
		case	'I':
			itfname	= strdup(optarg);
		break;
		case	'u':
			uniq	= 1;
		break;
		case	'e':
			TbExclude[exclude]	= strdup(optarg);
			exclude++;
		break;
		case	't':
			ptype	= 0;
		break;
		case	'i':
			pinterface	= 0;
		break;
		case	'h':
			Usage();
			exit(EXIT_SUCCESS);
		break;
		default :
			Usage();
			exit(EXIT_FAILURE);
		break;
		}
	}
#if	0
	for	(i = 0 ; i < exclude ; i++)
	{
		char	*filter	= TbExclude[i];
		printf	("filter '%s'\n",filter);
	}
#endif

	if	(uniq && itfname == NULL)
	{
		printf	("-u option require -I itfname\n");	
		exit(EXIT_FAILURE);
	}

	if	(getifaddrs(&ifaddr) == -1) 
	{
		perror("getifaddrs");
		exit(EXIT_FAILURE);
	}

	for	(ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next)
	{
//		int	ifam	= ifa->ifa_addr->sa_family;
		int	ifam;
		char	*inam	= ifa->ifa_name;
		int	lgaddr;
		char	*txt	= "";
		char	*pt;

		if	(ifa->ifa_addr == NULL)	// RDTP-6963
		{	// AF_PACKET slip interfaces have ifa_addr NULL
			continue;
		}

		ifam	= ifa->ifa_addr->sa_family;
		if	(family != -1 && family != ifam)
			continue;

		if	(itfname && strcmp(itfname,inam) != 0)
			continue;

		if	(uniq && found)
			break;

		switch	(ifam)
		{
		case	AF_INET:
			lgaddr	= sizeof(struct sockaddr_in);
			txt	= "4";
		break;
		case	AF_INET6:
			lgaddr	= sizeof(struct sockaddr_in6);
			txt	= "6";
		break;
		default:
			continue;
		break;
		}
		ret	= getnameinfo(ifa->ifa_addr,lgaddr,host,
					NI_MAXHOST,NULL,0,NI_NUMERICHOST);
		if	(ret != 0)
		{
			continue;
		}
		pt	= strchr(host,'%');
		if	(pt)
			*pt	= '\0';

		if	(!ptype)
		{
			txt	= "";
		}
		ok	= 1;
		for	(i = 0 ; i < exclude ; i++)
		{
			char	*filter	= TbExclude[i];
			if	(strncmp(filter,host,strlen(filter)) == 0)
			{
				ok	= 0;
				break;
			}
		}
		if	(!ok)
		{
			continue;
		}
		for	(i = 0 ; i < exclude ; i++)
		{
			char	*filter	= TbExclude[i];
			if	(strncmp(filter,inam,strlen(filter)) == 0)
			{
				ok	= 0;
				break;
			}
		}
		if	(!ok)
		{
			continue;
		}

		if	(family == -1 && itfname == NULL)
		{
			found++;
			if	(pinterface)
			printf	("%-15.15s %-30.30s %s\n",inam,host,txt);
			else
			printf	("%-30.30s %s\n",host,txt);
			continue;
		}
		if	(family == -1 && itfname != NULL)
		{
			found++;
			printf	("%-30.30s %s\n",host,txt);
			continue;
		}

		if	(family != -1 && itfname == NULL)
		{
			found++;
			if	(pinterface)
			printf	("%-15.15s %-30.30s\n",inam,host);
			else
			printf	("%-30.30s\n",host);
			continue;
		}
		if	(family != -1 && itfname != NULL)
		{
			found++;
			printf	("%s\n",host);
			continue;
		}
	}
	freeifaddrs(ifaddr);
	exit(EXIT_SUCCESS);
}
