
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

#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int	main(int argc,char *argv[])
{
	unsigned char buf[sizeof(struct in6_addr)];
	int domain,s;
	char addrin[INET6_ADDRSTRLEN+10];
	char addrout[INET6_ADDRSTRLEN+10];
	char *input;

	input = addrin;
	if	(argc != 3 || strlen(argv[1]) == 0 || strlen(argv[2]) == 0) 
	{
		fprintf(stderr,"Usage: %s {-4|-6} string\n",argv[0]);
		exit(EXIT_FAILURE);
	}

	domain = (strcmp(argv[1],"-4") == 0) ? AF_INET :
		(strcmp(argv[1],"-6") == 0) ? AF_INET6 : -1;

	if	(domain == -1)
	{
		fprintf(stderr,"Wrong domain\n");
		exit(EXIT_FAILURE);
	}

//	fprintf(stderr,"%s\n",argv[2]);

	memset	(addrin,0,sizeof(addrin));
	strncpy(addrin,argv[2],INET6_ADDRSTRLEN+2);	// +2 for [...]
	if	(domain == AF_INET6 || domain == AF_INET)
	{
		if	(*input == '[')
			input++;
		s	= strlen(input);
		if	(s >= 1 && input[s-1] == ']')
			input[s-1]	= '\0';
	}

//	fprintf(stderr,"%s\n",input);

	s = inet_pton(domain,input,buf);
	if	(s <= 0) 
	{
		if	(s == 0)
			fprintf(stderr,"Not in presentation format\n");
		else
			perror("inet_pton");
		exit(EXIT_FAILURE);
	}

	if	(inet_ntop(domain,buf,addrout,INET6_ADDRSTRLEN) == NULL) 
	{
		perror("inet_ntop");
		exit(EXIT_FAILURE);
	}

	printf("%s\n",addrout);
	exit(EXIT_SUCCESS);
}
