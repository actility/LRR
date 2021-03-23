
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
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>
#include <poll.h>
#include <ctype.h>
#ifndef MACOSX
#include <malloc.h>
#endif

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>

#include "rtlbase.h"

#include "networkfilter.h"

#ifdef	NWF_MAIN
int	TraceLevel = 3;
int	TraceDebug = 0;
char	*RootAct = ".";
#else
#include "semtech.h"
#include "define.h"
#include "infrastruct.h"
#include "struct.h"
#include "extern.h"
#endif


u_int	NfVersion	= 0;
u_int	NfVersionP	= 0;
u_int	NfNetwork	= 0;
u_int	NfNetworkP	= 0;
char	*NfFilter	= NULL;

u_int	NfFilterCount	= 0;
u_int	NfFilterEx	= 0;
u_int	NfFilterIn	= 0;
u_int	NfSortExIn	= 1;
u_int	NfSortBits	= 1;	

t_nf_mask	TbNfFilter[NF_MAX_MASK];

static	void	NfReset()
{
	NfFilterCount	= 0;
	NfFilterIn	= 0;
	NfFilterEx	= 0;
	NfSortExIn	= 1;
	NfSortBits	= 1;
}

static	char	*strtobin(u_int n,char *dest)
{
	int	i;
	char	*pt;

	pt	= dest;
	*pt	= '\0';
	for	(i = 31; i >= 0; i--)
	{
		u_int	k;
		k	= n >> i;
		if	((k&1) == 1)
			*pt	= '1';
		else
			*pt	= '0';
		pt++;
	}
	*pt	= '\0';
	return	dest;
}

static	char	*DoConfigFile(char *file)
{
	static	char	path[1024];

	sprintf	(path,"%s/%s",RootAct,file);
	RTL_TRDBG(1,"search '%s'\n",path);
	if	(access(path,R_OK) == 0)
		return	path;
	RTL_TRDBG(1,"no custom configuration file '%s'\n",path);
	return	NULL;
}

static	int CfgCBIniLoad(void *user,const char *section,const char *name,const char *value)
{
	RTL_TRDBG(9,"NWF %s.%s='%s'\n",section,name,value);
	if	(strcmp(name,"Network") == 0)
	{
		NfNetwork	= strtoul(value,NULL,16);
	}	else
	if	(strcmp(name,"NetworkFilter") == 0)
	{
		if	(NfFilter)
			free(NfFilter);
		NfFilter	= strdup(value);
	}	else
	if	(strcmp(name,"NetworkFilterVersion") == 0)
	{
		NfVersion	= atoi(value);
	}
	return	1;
}

static	void	LoadConfig(int hot,int dumponly)
{
	int	err;
	char	*file;

	file	= DoConfigFile(NF_LOCAL_FILE);
	if	(file)
	{
		RTL_TRDBG(1,"load custom '%s'\n",file);
	}
	if	(file && (err=rtl_iniParse(file,CfgCBIniLoad,NULL)) < 0)
	{
		RTL_TRDBG(0,"parse '%s' error=%d\n",file,err);
		exit(1);
	}
}

static	int	ParseOneFilter(char *filter,u_char *ex,u_int *mask,u_char *nbb)
{
	int	base	= 16;
	int	padding	= 8;
	char	tmp[256];
	char	*pt;
	char	*pt2;
	int	ret;
	*ex	= 0;
	*mask	= 0;
	*nbb	= 0;

	if	(!filter || !*filter)
		return	0;

	if	((pt=strstr(filter,"sortexin:")) != NULL)
	{
		pt		+= strlen("sortexin:");
		NfSortExIn	= atoi(pt);
		RTL_TRDBG(1,"NWF filter sortexin:%d\n",NfSortExIn);
		return	0;
	}
	if	((pt=strstr(filter,"sortbits:")) != NULL)
	{
		pt		+= strlen("sortbits:");
		NfSortBits	= atoi(pt);
		RTL_TRDBG(1,"NWF filter sortbits:%d\n",NfSortBits);
		return	0;
	}

	pt	= filter;
	switch	(*pt)
	{
	case	'+':
		pt++;
		*ex	= 0;
	break;
	case	'-':
		pt++;
		*ex	= 1;
	break;
	default :
		*ex	= 0;
	break;
	}
	pt2	= strchr(pt,'/');
	if	(pt2)
	{
		*pt2	= '\0';
		*nbb	= atoi(pt2+1);
	}
	else
	{
		pt2	= strchr(pt,'.');
		if	(pt2)
		{
			*pt2	= '\0';
			*nbb	= strlen(pt);
			base	= 2;
			padding	= 32;
		}
	}
	strncpy	(tmp,pt,64);
	if	((ret=strlen(tmp)) < padding)
	{
		int	i;
		for	(i = 0 ; i < (padding - ret) ; i++)
			strcat(tmp,"0");
	}
	*mask	= (unsigned int)strtoul(tmp,NULL,base);
	RTL_TRDBG(3,"NWF filter='%s' padded='%s' om=%08x\n",pt,tmp,*mask);
	if	(*nbb == 0 || *nbb > 32)
	{
		*nbb	= 32;
	}
	else
	{
		*mask	= ((*mask) & (0xFFFFFFFF << (32 - *nbb)));
	}
	strtobin(*mask,tmp);
	tmp[*nbb]	= '\0';

	RTL_TRDBG(3,"NWF filter='%s' => ex=%u m=%08x b=%u '%s'\n",
			filter,*ex,*mask,*nbb,tmp);
	return	1;
}

void	NfDump()
{
	int	i;
	t_nf_mask	*f;
	char	tmp[256];

	for	(i = 0 ; i < NfFilterCount ; i++)
	{
		f	= &TbNfFilter[i];
		strtobin(f->nf_mask,tmp);
		tmp[f->nf_nbb]	= '\0';
		RTL_TRDBG(0,"NWF filter[%03d] %cm=%08x b=%02u '%s'\n",
			f->nf_index,f->nf_ex?'-':'+',f->nf_mask,f->nf_nbb,tmp);
	}
}

static	int	CmpNfMask(const void *m1,const void *m2)
{
	t_nf_mask	*e1	= (t_nf_mask *)m1;
	t_nf_mask	*e2	= (t_nf_mask *)m2;

	return	e2->nf_ex - e1->nf_ex;
}

static	int	CmpNbBits(const void *m1,const void *m2)
{
	t_nf_mask	*e1	= (t_nf_mask *)m1;
	t_nf_mask	*e2	= (t_nf_mask *)m2;

	return	e1->nf_nbb - e2->nf_nbb;
}

static	void	NfSort()
{
	int	last_ex		= -1;
	int	i;
	t_nf_mask	*f;

	if	(NfFilterCount <= 0)
		return;
	if	(NfSortExIn <= 0 && NfSortBits <= 0)
		return;


	if	(NfSortExIn >= 1)
	{
		RTL_TRDBG(1,"NWF filter sort ex/in started %d=%d+%d\n",
			NfFilterCount,NfFilterEx,NfFilterIn);
		qsort	(TbNfFilter,NfFilterCount,sizeof(t_nf_mask),CmpNfMask);
		if	(NfSortBits <= 0)
		{
			RTL_TRDBG(1,"NWF filter sort ended\n");
			return;
		}

		for     (i = 0 , f =  &TbNfFilter[0] ; i < NfFilterCount ; i++ , f++)
		{
			if	(f->nf_ex == 0)
				break;
			last_ex	= i;
		}
		if	(NfSortBits >= 1 && NfFilterEx > 0 && last_ex > 0)
		{
			RTL_TRDBG(1,"NWF filter/ex sort by #bits started\n");
			f	= &TbNfFilter[0];
			qsort	(f,NfFilterEx,sizeof(t_nf_mask),CmpNbBits);
		}
		if	(NfSortBits >= 1 && NfFilterIn > 0 && last_ex > 0)
		{
			RTL_TRDBG(1,"NWF filter/in sort by #bits started\n");
			f	= &TbNfFilter[last_ex+1];
			qsort	(f,NfFilterIn,sizeof(t_nf_mask),CmpNbBits);
		}
		RTL_TRDBG(1,"NWF filter sort ended\n");
		return;
	}
	if	(NfSortBits >= 1)
	{
		RTL_TRDBG(1,"NWF filter sort by #bits started\n");
		qsort	(TbNfFilter,NfFilterCount,sizeof(t_nf_mask),CmpNbBits);
		RTL_TRDBG(1,"NWF filter sort ended\n");
	}

}

static	u_int	GetBits(u_int d,int p,int n)
{
	if	(n >= 32)
		return	d;
	return (d >> (p+1-n)) & ~(~0 << n);
}

static	int	NfParse()
{
	int	ret;
	char	*r;
	char	*tok;
	char	*end;

	u_char	ex;
	u_char	nbb;
	u_int	mask;

	NfReset();
	r	= NfFilter;
	if	(!r || !*r)
	{
		RTL_TRDBG(1,"NWF filter is <empty>\n");
		return	0;
	}
	r	= strdup(r);
	if	(!r)
	{
		RTL_TRDBG(0,"NWF filter cannot strdup\n");
		return	0;
	}

	tok	= r;
	end	= r;

	while	(tok != NULL)
	{
		strsep(&end,",");
		if	(!tok || !*tok)
			break;
		ret	= ParseOneFilter(tok,&ex,&mask,&nbb);
		if	(ret < 0)
			break;
		if	(ret > 0)
		{
			u_int	masknbb	= GetBits(mask,31,nbb);

			TbNfFilter[NfFilterCount].nf_ex		= ex;
			TbNfFilter[NfFilterCount].nf_nbb	= nbb;
			TbNfFilter[NfFilterCount].nf_mask	= mask;
			TbNfFilter[NfFilterCount].nf_mask_nbb	= masknbb;
			TbNfFilter[NfFilterCount].nf_index	= NfFilterCount;
			NfFilterCount++;
			if	(ex)
				NfFilterEx++;
			else
				NfFilterIn++;
		}
		tok	= end;
		if	(NfFilterCount >= NF_MAX_MASK)
		{
			RTL_TRDBG(0,"NWF max filters reached %d\n",NF_MAX_MASK);
			break;
		}
	}
	free(r);
	NfSort();
	NfDump();
	return	NfFilterCount;
}

int	NfCheck(u_int devaddr)
{
	int	i;
	t_nf_mask	*f;
	char	tmp[256];

	strtobin(devaddr,tmp);

	RTL_TRDBG(1,"NWF addr=%08x %s check\n",devaddr,tmp);

	if	(NfFilterCount <= 0)
	{
		RTL_TRDBG(1,"NWF addr=%08x no mask -+ defined\n",devaddr);
		return	1;
	}

	// check all exclusions first
	for	(i = 0 , f =  &TbNfFilter[0] ; i < NfFilterCount ; i++ , f++)
	{
		if	(f->nf_ex == 0)
			break;
//		if	(GetBits(devaddr,31,f->nf_nbb) == GetBits(f->nf_mask,31,f->nf_nbb))
		if	(GetBits(devaddr,31,f->nf_nbb) == f->nf_mask_nbb)
		{
			if	(TraceLevel >= 3)
			{
				strtobin(f->nf_mask,tmp);
				tmp[f->nf_nbb]	= '\0';
				RTL_TRDBG(3,"NWF mask=%08x %-32.32s /%02d\n",
					f->nf_mask,tmp,f->nf_nbb);
			}

		RTL_TRDBG(1,"NWF addr=%08x excluded by %08x/%02d [%03d]\n",
			devaddr,f->nf_mask,f->nf_nbb,f->nf_index);

			return	0;
		}
	}

	if	(NfFilterIn <= 0)
	{
		RTL_TRDBG(1,"NWF addr=%08x no +mask defined\n",devaddr);
		return	1;
	}

	// one inclusion ok
	for	( ; i < NfFilterCount ; i++ , f++)
	{
		if	(f->nf_ex == 1)
			break;
//		if	(GetBits(devaddr,31,f->nf_nbb) == GetBits(f->nf_mask,31,f->nf_nbb))
		if	(GetBits(devaddr,31,f->nf_nbb) == f->nf_mask_nbb)
		{
			if	(TraceLevel >= 3)
			{
				strtobin(f->nf_mask,tmp);
				tmp[f->nf_nbb]	= '\0';
				RTL_TRDBG(3,"NWF mask=%08x %-32.32s /%02d\n",
					f->nf_mask,tmp,f->nf_nbb);
			}

		RTL_TRDBG(1,"NWF addr=%08x included by %08x/%02d [%03d]\n",
			devaddr,f->nf_mask,f->nf_nbb,f->nf_index);

			return	1;
		}
	}

	RTL_TRDBG(1,"NWF addr=%08x no +mask found\n",devaddr);
	return	0;
}

void	NfSave(u_int version,u_int netid,char *filter)
{
	char	path[1024];
	FILE	*f;

	if	(!filter)
		filter	= "";

	sprintf	(path,"%s/%s",RootAct,NF_LOCAL_FILE);
	RTL_TRDBG(1,"NWF save '%s'\n",path);
	f	= fopen(path,"w");
	if	(f)
	{
		fprintf	(f,"### from LRC\n");
		fprintf	(f,"Network=%06x\n",netid);
		fprintf	(f,"NetworkFilterVersion=%u\n",version);
		fprintf	(f,"NetworkFilter=%s\n",filter);
		fclose(f);
	}
}

void	NfInit()
{
	NfVersion	= 0;
	NfNetwork	= 0;
	NfFilter	= strdup("");

	LoadConfig(0,0);
	NfParse();
}

void	NfReload()
{
	int	reload	= 0;

	NfVersionP	= NfVersion;
	NfNetworkP	= NfNetwork;
	LoadConfig(1,0);
	if	(NfVersion != NfVersionP)
	{
		RTL_TRDBG(0,"NWF from version=%u to version=%u => reload\n",
					NfVersionP,NfVersion);
		reload	= 1;
	}
	if	(NfNetwork != NfNetworkP)
	{
		RTL_TRDBG(0,"NWF from network=%06x to network=%06x => reload\n",
					NfNetworkP,NfNetwork);
		reload	= 1;
	}
	if	(reload)
	{
		NfParse();
	}
	else
	{
		RTL_TRDBG(0,"NWF same version=%u and same network=%06x => not reload\n",
					NfVersion,NfNetwork);
	}
}

#ifdef	NWF_MAIN
#if	0
static	void	Test()
{
	int	i;
	u_int tb[] = { 
	0x6b8b4567 , 0x327b23c6 , 0x643c9869 , 0x66334873 , 0x74b0dc51 ,
	0x19495cff , 0x2ae8944a , 0x625558ec , 0x238e1f29 , 0x46e87ccd ,
	0x00000000 , 0x00000001 , 0xffffffff , 0xfffffff0 , 0x030303ff ,
	0x02020200 , 0x0000aaaa , 0x000000bb , 0x12345678
	};

	for	(i = 0 ; i < sizeof(tb)/sizeof(u_int) ; i++)
	{
		NfCheck(tb[i]);
	}
}
#endif
#endif

#ifdef	NWF_MAIN
#if	0
static	void	LoopTest()
{
	while	(1)
	{
		Test();
		sleep(30);
		NfReload();
	}
}
#endif
#endif

#ifdef	NWF_MAIN
int	main(int argc,char *argv[])
{
	char	buff[256];
	u_int	devaddr;
	int	ret;

	rtl_tracelevel(TraceLevel);
	NfInit();
	while	(1)
	{
		memset	(buff,0,sizeof(buff));
		printf	("enter 'r' to reload ./nwkfilter.ini file\n");
		printf	("enter devaddr in hexa format [0-9a-f]x8:\n");
		ret	= scanf("%[^\n]",buff);
		if	(ret == EOF)	exit(0);
		getchar();      // get \n
		if	(buff[0] == 'r')
		{
			NfReload();
			continue;
		}
		if	(buff[0] == 'e')
		{
			ret	= system("nano ./nwkfilter.ini");
			NfReload();
			continue;
		}
		if	(buff[0] == 'v')
		{
			ret	= system("vi ./nwkfilter.ini");
			NfReload();
			continue;
		}
		buff[8]	= '\0';
		devaddr	= (unsigned int)strtoul(buff,NULL,16);
		printf	("%08x\n",devaddr);
		NfCheck(devaddr);
	}
	return	0;
}
#endif
