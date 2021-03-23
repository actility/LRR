
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

int	checkOne(int v)
{
	if	(v < 0 || v > 0xFF)
		return	1;
	return	0;
}

int	main(int argc,char *argv[])
{
	int	ret;
	char	*mac;
	int	A,B,C,D,E,F;
	unsigned char	Aa;

	if	(argc <= 1)
	{
		fprintf	(stderr,"%s mac@ (02x:02x:02x:02x:02x:02x)\n",argv[0]);
		exit	(1);
	}
	mac	= argv[1];
	if	(!mac || strlen(mac) == 0)
	{
		fprintf	(stderr,"%s mac@ (02x:02x:02x:02x:02x:02x)\n",argv[0]);
		exit	(1);
	}
	ret	= sscanf(mac,"%x:%x:%x:%x:%x:%x",&A,&B,&C,&D,&E,&F);
	if	(ret != 6)
	{
		fprintf	(stderr,"%s mac@ (02x:02x:02x:02x:02x:02x)\n",argv[0]);
		exit	(1);
	}
	ret	= checkOne(A)|checkOne(B)|checkOne(C)
				|checkOne(D)|checkOne(E)|checkOne(F);
	if	(ret != 0)
	{
		fprintf	(stderr,"%s mac@ (02x:02x:02x:02x:02x:02x)\n",argv[0]);
		exit	(1);
	}
	Aa	= (unsigned char)A;
	if	((Aa & 0x02) == 0x02)
	{
//		fprintf	(stderr,"%02x bit1 set => unset\n",Aa);
		Aa      = Aa & (~0x02);
	}
	else
	{
//		fprintf	(stderr,"%02x bit1 unset => set\n",Aa);
		Aa	= Aa | 0x02;
	}

	if	(Aa)
		printf	("%x%02x:",Aa,B);
	else
		printf	("%x:",B);
	if	(C)
		printf	("%xff:",C);
	else
		printf	("ff:");
	printf	("fe%02x:",D);
	if	(E)
		printf	("%x%02x",E,F);
	else
		printf	("%x",F);

	printf	("\n");
	exit	(0);
}
