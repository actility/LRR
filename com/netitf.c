
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
u16 crc_ccitt(u16 crc, const u8 *buffer, int len);

// Not needed here #include "_whatstr.h"
#include "define.h"
#include "cproto.h"
#include "extern.h"

char	*FindItfDefautRoute()
{
	static	char	iface[128];
	unsigned int	dest, gate, flags, refcnt, use, metric, mask;
	char		buff[1024];
	int		line = 0;
	FILE		*f;

	f	= fopen("/proc/net/route","r");
	if	(!f)
		return	"";
	while	(fgets(buff,sizeof(buff)-1,f))
	{
		if	(line++ <= 0)
			continue;
		if	(sscanf(buff,"%s%x%x%x%x%x%x%x",
			iface,&dest,&gate,&flags,&refcnt,&use,&metric,&mask)==8)
		{
			if	(dest == 0 && gate != 0)
			{	// this is a default route
				fclose	(f);
				return	iface;
			}
			if	(dest == 0 && mask == 0)
			{	// this is a default route
				fclose	(f);
				return	iface;
			}
		}
	}
	fclose	(f);
	return	"";
}

void	FindIpInterfaces(t_lrr_config *config)
{
	char	*ldir[] = { "/sys/class/net" , NULL };

	int	d;
	DIR	*dir;
	struct	dirent	*entry;
	int	i;
	t_wan_itf	*itf;

	for	(d = 0 ; ldir[d] ; d++ )
	{
		dir	= opendir(ldir[d]);
		if	(!dir)
			continue;

		while	((entry = readdir(dir)))
		{
			if	(strcmp(entry->d_name,".") == 0)
				continue;
			if	(strcmp(entry->d_name,"..") == 0)
				continue;
			if	(strcmp(entry->d_name,"all") == 0)
				continue;
			if	(strcmp(entry->d_name,"default") == 0)
				continue;
			entry->d_name[10]	= '\0';
			if	((itf=FindItfByName(entry->d_name)) == NULL)
				continue;
			itf->it_exists	= 1;
		}
		closedir(dir);
	}

	for	(i = 0 ; i < NB_ITF_PER_LRR ; i++)
	{
		int	defroute;
		itf	= &TbItf[i];
		strcpy	((char *)config->cf_IpInt[i],"");
		if	(itf->it_enable /*&& itf->it_exists*/)
		{
			defroute = !strcmp(itf->it_name,FindItfDefautRoute());
RTL_TRDBG(1,"find ip interface %d '%s' defaultroute=%d type=%d\n",
				i,itf->it_name,defroute,itf->it_type);
			strncpy	((char *)config->cf_IpInt[i],itf->it_name,
					sizeof(config->cf_IpInt[i])-1);
			config->cf_ItfType[i]	= itf->it_type;
			// [2420][12898]
			switch	(itf->it_type)
			{
			case	ITF_TYPE_ETHE:
			break;
			case	ITF_TYPE_CELL:
			{	
				// TODO: useless ? At startup we don't have th einformations and this function
				// is only called from LoadLrrConfig, function used only at startup ?
				CellStateCopyIdent(&config->cf_ItfIdentif_u[i].cf_ItfCellId);
			}
			break;
			case	ITF_TYPE_WIFI:
			{
				t_lrr_wifiid	*wifiid;
				wifiid = &config->cf_ItfIdentif_u[i].cf_ItfWifiId;
				strncpy	((char *)wifiid->cf_ItfSsid, WifiStateSsid,
						sizeof(wifiid->cf_ItfSsid)-1);
			}
			break;
			default:
			break;
			}
		}
	}
}



t_wan_itf	*FindItfByName(char *name)
{
	int	i;

	for	(i = 0; i < NB_ITF_PER_LRR ; i++)
	{
		if	(!TbItf[i].it_enable)	continue;
		if	(!TbItf[i].it_name)	continue;
		if	(strcmp(TbItf[i].it_name,name) == 0)
			return	&TbItf[i];
	}
	return	NULL;
}

uint32_t	GetIpv4Addr(char *name)
{
 if (!name || !*name){
  return 0;
 }

 #if defined(CISCOMS)
  FILE *f; 
  f = fopen("/var/run/hosts_ip_status","r");
  char ip[16];
  if (f){
   fscanf(f, "%s", ip);
   fclose (f);
   if (strlen(ip) < 3) return 0;
   struct sockaddr_in sa;

   // store this IP address in sa:
   inet_pton(AF_INET, ip, &(sa.sin_addr));
   return (uint32_t)sa.sin_addr.s_addr;
  } else {
   return 0;
  }
 #else

  int fd;
  int ret;
  struct ifreq ifr;

  fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0){
   return 0;
  }
  memset(&ifr,0,sizeof(ifr));
  /* I want to get an IPv4 IP address */
  ifr.ifr_addr.sa_family = AF_INET;

  /* I want IP address attached to "eth0" */
  strncpy(ifr.ifr_name,name,IFNAMSIZ-1);
  ret = ioctl(fd,SIOCGIFADDR,&ifr);
  close(fd);
  if (ret < 0)
   return 0;

  /* display result */
        return (uint32_t)((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr.s_addr;
 #endif
}

uint32_t	GetMacAddr32(char *name,u_short *ext)
{
	int fd;
	int ret;
	struct ifreq ifr;

	if	(!name || !*name)
		return	0;
	fd = socket(AF_INET, SOCK_DGRAM, 0);
	if	(fd < 0)
		return	0;

	memset(&ifr,0,sizeof(ifr));
	/* I want MAC address attached to "eth0" */
	strncpy(ifr.ifr_name,name,IFNAMSIZ-1);
	ret	= ioctl(fd,SIOCGIFHWADDR,&ifr);
        close(fd);
	if	(ret < 0)
		return	0;

	if	(ifr.ifr_hwaddr.sa_family != ARPHRD_ETHER)
		return	0;

	unsigned char* mac=(unsigned char*)ifr.ifr_hwaddr.sa_data;

	*ext	= mac[0]<<8 | mac[1];

	return	(uint32_t)(mac[2]<<24 | mac[3]<<16 | mac[4]<<8 | mac[5]);

}

// used to set a LRRID based on MAC@ of ethernet board
uint32_t	FindEthMac32(u_short *ext)
{
	int	i;

	*ext	= 0;
	for	(i = 0; i < NB_ITF_PER_LRR ; i++)
	{
		if	(!TbItf[i].it_enable)	continue;
		if	(!TbItf[i].it_name)	continue;
//		if	(!TbItf[i].it_exists)	continue;
		if	(TbItf[i].it_type == ITF_TYPE_ETHE || 
				TbItf[i].it_type == ITF_TYPE_WIFI)
		{
			return	GetMacAddr32(TbItf[i].it_name,ext);
		}
	}
	return	0;
}

// used to set a LRRID when first interface is cellular
uint32_t	GetLrrIDIfCellular()
{
	int	i;

	for	(i = 0; i < NB_ITF_PER_LRR ; i++)
	{
		if	(!TbItf[i].it_enable)	continue;
		if	(!TbItf[i].it_name)	continue;
//		if	(!TbItf[i].it_exists)	continue;
		if	(TbItf[i].it_type == ITF_TYPE_CELL)
		{
			return	0xffffffff;
		}
	}
	return	0;
}

static	void	ReadItfInfos(t_wan_itf *itf)
{
	char	path[256];
	char	buff[256];
	FILE	*f;

	itf->it_readcnt++;
	itf->it_rxbytes_c	= 0.0;
	itf->it_txbytes_c	= 0.0;
	itf->it_l2		= 0;
	itf->it_up		= 0;
	itf->it_ipv4		= 0;
	itf->it_def		= 0;
	sprintf	(path,"/sys/class/net/%s/carrier",itf->it_name);
	f	= fopen(path,"r");
	if	(f)
	{
		fscanf	(f,"%d",&itf->it_l2);
		fclose	(f);
		if	(itf->it_l2)
			itf->it_l2cnt++;
	}
	if	(itf->it_l2 == 0)
		return;
	sprintf	(path,"/sys/class/net/%s/statistics/rx_bytes",itf->it_name);
	f	= fopen(path,"r");
	if	(f)
	{
		fscanf	(f,"%lf",&itf->it_rxbytes_c);
		fclose	(f);
	}
	sprintf	(path,"/sys/class/net/%s/statistics/tx_bytes",itf->it_name);
	f	= fopen(path,"r");
	if	(f)
	{
		fscanf	(f,"%lf",&itf->it_txbytes_c);
		fclose	(f);
	}
	sprintf	(path,"/sys/class/net/%s/operstate",itf->it_name);
	f	= fopen(path,"r");
	if	(f)
	{
		fscanf	(f,"%s",buff);
		fclose	(f);

		itf->it_ipv4	= GetIpv4Addr(itf->it_name);
		if	(strcmp(buff,"up") == 0)
		{
			itf->it_up = 1;
			itf->it_upcnt++;
		}
		if	(itf->it_up == 0 && itf->it_ipv4 
						&& strcmp(buff,"unknown") == 0)
		{
			itf->it_up = 1;
			itf->it_upcnt++;
		}
		if	(itf->it_ipv4)
		{
			if	(strcmp(itf->it_name,FindItfDefautRoute()) == 0)
			{
				itf->it_def = 1;
				itf->it_defcnt++;
			}
		}
	}
}

static	void	CompItfInfos(t_wan_itf *itf,t_wan_stat *st,time_t when)
{
	double	delta	= 1;

	delta	= (ABS (when - st->it_tmms)) / 1000;
	st->it_tmms	= when;
	if	(!delta)
		delta	= 1;

	if	(st->it_rxbytes_p == 0)
	{
		st->it_rxbytes_d	= 0.0;
		st->it_rxbytes_br	= 0.0;
	}
	else
	{
		st->it_rxbytes_d	= 
				ABS(itf->it_rxbytes_c - st->it_rxbytes_p);
		st->it_rxbytes_br	= st->it_rxbytes_d / delta;
	}
	if	(st->it_rxbytes_br > st->it_rxbytes_mbr)
	{
		st->it_rxbytes_mbr	= st->it_rxbytes_br;
		st->it_rxbytes_mbrt	= time(NULL);
	}

	if	(st->it_txbytes_p == 0)
	{
		st->it_txbytes_d	= 0.0;
		st->it_txbytes_br	= 0.0;
	}
	else
	{
		st->it_txbytes_d	= 
				ABS(itf->it_txbytes_c - st->it_txbytes_p);
		st->it_txbytes_br	= st->it_txbytes_d / delta;
	}
	if	(st->it_txbytes_br > st->it_txbytes_mbr)
	{
		st->it_txbytes_mbr	= st->it_txbytes_br;
		st->it_txbytes_mbrt	= time(NULL);
	}

	st->it_rxbytes_p	= itf->it_rxbytes_c;
	st->it_txbytes_p	= itf->it_txbytes_c;
}

void	ResetItfInfos(t_wan_itf *itf)
{
	itf->it_lgt.it_rxbytes_mbr	= itf->it_sht.it_rxbytes_mbr;
	itf->it_lgt.it_txbytes_mbr	= itf->it_sht.it_txbytes_mbr;
	itf->it_lgt.it_rxbytes_mbrt	= itf->it_sht.it_rxbytes_mbrt;
	itf->it_lgt.it_txbytes_mbrt	= itf->it_sht.it_txbytes_mbrt;

	itf->it_sht.it_rxbytes_mbr	= 0.0;
	itf->it_sht.it_txbytes_mbr	= 0.0;
	itf->it_sht.it_rxbytes_mbrt	= 0;
	itf->it_sht.it_txbytes_mbrt	= 0;

	itf->it_readcnt	= 0;
	itf->it_l2cnt	= 0;
	itf->it_upcnt	= 0;
	itf->it_defcnt	= 0;
}

void	InitItfInfos(t_wan_itf *itf)
{
	if	(itf->it_lgt.it_tmms == 0 && itf->it_sht.it_tmms)
	{
		itf->it_lgt.it_tmms	= itf->it_sht.it_tmms;
		itf->it_lgt.it_rxbytes_p= itf->it_sht.it_rxbytes_p;
		itf->it_lgt.it_txbytes_p= itf->it_sht.it_txbytes_p;
	}
}

uint8_t	CompItfState(t_wan_itf *itf)
{
	uint8_t	l2;
	uint8_t	up;
	uint8_t	ip;
	uint8_t	df;

	l2	= itf->it_l2;
	up	= itf->it_up;
	ip	= itf->it_ipv4 ? 1 : 0;
	df	= itf->it_def;

	return	l2 + (up << 1) + (ip << 2) + (df << 3);
}

uint8_t	StateItf(t_wan_itf *itf)
{
	int	okay;

	if	(!itf->it_enable /*|| !itf->it_exists*/)
		return	5;
	if	(!itf->it_name || !strlen(itf->it_name))
		return	5;
	if	(!itf->it_up)
		return	5;

	if	(!itf->it_l2)
		return	4;

	if	(!itf->it_ipv4)
		return	3;

	// no ping packet acked everything is OK but network seems down
	okay	= ABS(itf->it_okayprtt - itf->it_okayprtt_p);
	if	(okay == 0)
		return	2;

	if	(!itf->it_def)
		return	1;

	return	0;
}

void	CompAllItfInfos(char shortlongtime)
{
	t_wan_itf	*itf;
	time_t		t0;
	time_t		delta;
	int		i;

	t0	= rtl_tmmsmono();
	for	(i = 0; i < NB_ITF_PER_LRR ; i++)
	{
		itf	= &TbItf[i];
		if	(!itf->it_enable)	continue;
		if	(!itf->it_name)		continue;
//		if	(!itf->it_exists)	continue;
		switch	(shortlongtime)
		{
		case	'S' :
			ReadItfInfos(itf);
			CompItfInfos(itf,&itf->it_sht,t0);
RTL_TRDBG(5,"short time iface %d '%s' %08x l2=%d up=%d def=%d drx=%lf dtx=%lf brx=%lf btx=%lf\n",
			i,itf->it_name,itf->it_ipv4,
			itf->it_l2,itf->it_up,itf->it_def,
			itf->it_sht.it_rxbytes_d,itf->it_sht.it_txbytes_d,
			itf->it_sht.it_rxbytes_br,itf->it_sht.it_txbytes_br);
			InitItfInfos(itf);

		break;
		case	'L' :
			delta	= ABS(itf->it_lgt.it_tmms - t0)/1000;
			if	(delta == 0)
				delta	= 1;
			if	(itf->it_readcnt == 0)
				itf->it_readcnt	= 1;

			itf->it_l2time	= delta *
				(double)itf->it_l2cnt/(double)itf->it_readcnt;

			itf->it_uptime	= delta *
				(double)itf->it_upcnt/(double)itf->it_readcnt;

			itf->it_deftime	= delta *
				(double)itf->it_defcnt/(double)itf->it_readcnt;

			itf->it_ipv4	= GetIpv4Addr(itf->it_name);
			CompItfInfos(itf,&itf->it_lgt,t0);
			ResetItfInfos(itf);
RTL_TRDBG(1,"long time iface %d '%s' %08x l2=%d up=%d def=%d drx=%lf dtx=%lf brx=%lf btx=%lf \n",
			i,itf->it_name,itf->it_ipv4,
			itf->it_l2,itf->it_up,itf->it_def,
			itf->it_lgt.it_rxbytes_d,itf->it_lgt.it_txbytes_d,
			itf->it_lgt.it_rxbytes_br,itf->it_lgt.it_txbytes_br);

RTL_TRDBG(1,"\t\t\tmbrx=%lf mbtx=%lf l2time=%lf uptime=%lf deftime=%lf\n",
			itf->it_lgt.it_rxbytes_mbr,itf->it_lgt.it_txbytes_mbr,
			itf->it_l2time,itf->it_uptime,itf->it_deftime);

		break;
		}
	}
}

t_wan_itf	*FindItfByType(int type)
{
	int	i;

	for	(i = 0; i < NB_ITF_PER_LRR ; i++)
	{
		if	(!TbItf[i].it_enable)	continue;
		if	(!TbItf[i].it_name)	continue;
		if	(TbItf[i].it_type == type)
			return	&TbItf[i];
	}
	return	NULL;
}

// add a global scope link based on the local link on the first netif of type 0
void	AddGlobalScopeIpv6()
{
	int	doipv6;
	char	*pref;
	char	cmd[1024];
	t_wan_itf	*itf;

	doipv6	= CfgInt(HtVarLrr,"ipv6",-1,"addglobalscope",0);
	pref	= CfgStr(HtVarLrr,"ipv6",-1,"prefix","aaaa");
	itf	= FindItfByType(0);
	if	(doipv6 <= 0 || !pref || !*pref || !itf)
		return;


RTL_TRDBG(1,"ipv6 .addglobalscope=%d .prefix='%s' netitf='%s'\n",
			doipv6,pref,itf->it_name);


	sprintf	(cmd,"%s/lrr/com/shells/shelllrr.sh %s -p %s -i %s",
		RootAct,"addipv6addr.sh",pref,itf->it_name);
	strcat	(cmd," 2>&1 > /dev/null");
	DoSystemCmdForeGround(cmd);
}
