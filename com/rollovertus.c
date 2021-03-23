
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
#include <stdbool.h>
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

//
//	        now  pastmax              futuremax
//	---------|---|--------------------|------------> tus
//	-------------|++++++++++++++++++++|------------> tus
//	             |  target tus is ok  | 
//
//		now + past < tus < now + future
//	=>	tus - now > past && tus - now < future	=> OK
//	=>	tus - now < past || tus - now > future	=> KO
//

#define		NBTAB	100
//	UINT_MAX	4294967295
uint32_t	Tab[NBTAB][3] = 
{
	//	tus target		tus now		result 'O' 'P' 'F'
	//	xxxxxxxxxx		xxxxxxxxxx
	{	1028864916,		1428810567,	'P',	}	,
	{	       100,		         1,	'P'	}	,
	{	     20000,		         1,	'O'	}	,
	{	 829797331,		 829732905,	'O'	}	,
	{	1428864916,		1428810567,	'O',	}	,
	{	     12000,		  UINT_MAX,	'O',	}	,
	{	 400000000,		         1,	'F',	}	,
	{	 300000000,	        4000000000U,	'F',	}	,

};

int		Nb	= 8;

void	Usage()
{
printf	(" -p xxxxxx : us in the \"past\" (default 10000)\n");
printf	(" -f xxxxxx : us in the future (default 3000000)\n");
printf	(" -c target:now:[P|O|F] : add a test, POF is the expected result:\n");
printf	("	P:in the past O:ok F:in the future\n");
}

void	AddControl(char *c)
{
	int		ret;
	uint32_t	tus;
	uint32_t	now;
	char		exp;

	if	(!c || !*c)
		return;
	if	(Nb >= NBTAB)
		return;

	ret	= sscanf(c,"%u:%u:%c",&tus,&now,&exp);
	if	(ret != 3)
		return;

	Tab[Nb][0]	= tus;
	Tab[Nb][1]	= now;
	Tab[Nb][2]	= exp;
	Nb++;
}

int	main(int argc,char *argv[])
{
	int	past	= 10000;	// us
	int	future	= 3000000;	// us

	uint32_t	tus;
	uint32_t	now;
	int32_t		delta;

	int		i;
	int		error;
	int		opt;
	char		exp;
	char		res;
	char		*test;

	while	((opt=getopt(argc,argv,"hp:f:c:")) != -1)
	{
		switch	(opt)
		{
		case	'p' : past	= atoi(optarg);	break;
		case	'f' : future	= atoi(optarg);	break;
		case	'c' : AddControl(optarg); break;
		case	'h' : Usage(); exit(0); break;
		default	: Usage(); exit(1); break;
		}
	}

	printf	("UINT_MAX=%u\n",UINT_MAX);
	printf	("past=%d future=%d\n",past,future);
	error	= 0;
	for	(i = 0 ; i < Nb && i < NBTAB ; i++)
	{
		tus	= Tab[i][0];
		now	= Tab[i][1];
		exp	= Tab[i][2];

		res	= 'O';
		test	= "ok";
		delta	= tus - now;
		if	(delta < past)
		{
			res	= 'P';
		}
		else
		{
			if	(delta > future)
			{
				res	= 'F';
			}
		}
		if	(exp != res)
		{
			test	= "failure";
			error++;
		}
		printf	("target=%010u now=%010u expect=%c result=%c %s (%d)\n",
			tus,now,exp,res,test,delta);
	}
	if	(error)
	{
		printf	("%d tests fail\n",error);
		exit(1);
	}
	printf	("all tests are OK\n");
	exit(0);
}
