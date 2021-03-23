
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

/*! @file get_fpga_version.c
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include "semtech.h"
#include "infrastruct.h"
#include "struct.h"
#include "headerloramac.h"
#include "define.h"
#include "extern.h"
#include <getopt.h>
#if defined (REF_DESIGN_V1)
#include "loragw_spi.h"
#endif

#if defined (REF_DESIGN_V2)
#include "spi_linuxdev.h"
/* Used to read FPGA version for Semtech V2 hardware */
int spi1_context = -1; /* context for linuxdev SPI driver (file descriptor) */
/* Wrappers around linuxdev SPI functions with explicit (global variables) SPI targets */
static int spi1_read( uint8_t header, uint16_t address, uint8_t * data, uint32_t size, uint8_t * status );
static int spi1_write( uint8_t header, uint16_t address, const uint8_t * data, uint32_t size, uint8_t * status );
#endif

int get_fpga_version(char* device) {
//This is a copy of part of lgw_connect() function that will read the FPGA version
#if defined (REF_DESIGN_V1) && (defined(WITH_USB) || defined(NATRBPI_USB))
	fprintf(stdout,"NO FPGA\n");
#elif defined (REF_DESIGN_V1)
	extern void* lgw_spi_target;
	extern const struct lgw_reg_s loregs[LGW_TOTALREGS];
	int spi_stat = LGW_SPI_SUCCESS;
	uint8_t u;
    /* check SPI link status */
    if (lgw_spi_target != NULL) {
        lgw_spi_close(lgw_spi_target);
    }

    /* open the SPI link */
#if defined(TRACKNET) || defined(HAL_HAVE_LGW_SPI_OPEN_WITH_DEVICE) // TRACKNET: to be removed
    spi_stat = lgw_spi_open(&lgw_spi_target, device);
#else
    spi_stat = lgw_spi_open(&lgw_spi_target);
#endif
    if (spi_stat != LGW_SPI_SUCCESS) {
		fprintf(stdout,"%u \n",0);
        return 0;
    }
    /* Detect if the gateway has an FPGA with SPI mux header support */
    /* First, we assume there is an FPGA, and try to read its version */
    spi_stat = lgw_spi_r(lgw_spi_target, LGW_SPI_MUX_MODE1, LGW_SPI_MUX_TARGET_FPGA, loregs[LGW_VERSION].addr, &u);
    if (spi_stat != LGW_SPI_SUCCESS) {
		fprintf(stdout,"%u \n",-1);
        return 0;
    }
	fprintf(stdout,"FPGA Version:%d\n",u);
#elif defined (REF_DESIGN_V2)
	int i,val,x;
   /* Open SPI link */
    i = spi_linuxdev_open( device, -1, &spi1_context );
    if( i != 0 )
    {
        fprintf(stdout,"ERROR: opening SPI failed, spi_linuxdev_open returned %i\n", i );
        return EXIT_FAILURE;
    }
    /* Register SPI callbacks */
    x = sx1301ar_reg_setup( 0, BRD_MASTER, spi1_read, spi1_write );
    if( x != 0 )
    {
        fprintf(stdout,"ERROR: sx1301ar_reg_setup failed: %s\n", sx1301ar_err_message( sx1301ar_errno ) );
        return EXIT_FAILURE;
    }
    /* Check FPGA version register */
    sx1301ar_reg_brd_r( 0, BREG_VERSION, &val );
    fprintf(stdout,"FPGA Version:%d\n", val);

#endif
	return 0;
}


int	main(int argc,char *argv[]) {
#if defined (REF_DESIGN_V1) && !defined(WITH_MULTI_BOARD)
	get_fpga_version(NULL);
#else
	int opt;
	while ((opt = getopt (argc, argv, "d:")) != -1)
	{
		switch (opt)
		{
			case 'd':
				if(optarg != NULL) {
					get_fpga_version(optarg);
				} else {
					fprintf(stdout, "No SPI device path was given, cannot fetch FPGA version. exemple: ./get_fpga_version -d /dev/spidev0.0 \n");
				}
                break;
		}
	}
#endif
	return 0;
}

#if defined (REF_DESIGN_V2)
/* Used to read FPGA version for Semtech V2 hardware */
static int spi1_read( uint8_t header, uint16_t address, uint8_t * data, uint32_t size, uint8_t * status )
{
    return spi_linuxdev_read( header, spi1_context, address, data, size, status );
}
static int spi1_write( uint8_t header, uint16_t address, const uint8_t * data, uint32_t size, uint8_t * status )
{
    return spi_linuxdev_write( header, spi1_context, address, data, size, status );
}
#endif


