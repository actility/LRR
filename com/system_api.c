
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

/*!
* @file

* System API handling gateway-specific scripts
*
* @ingroup system
*/

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

//#define TEST    1
#ifdef TEST
#define DEBUG(...)  { printf(__VA_ARGS__); }
#else
#define DEBUG(...)
#endif

#include "system_api.h"

/* file organization
    <root>/file
    <root>/<system>/<file>
    <root>/<family>/<file>
    <root>/<firmware>/<file>
    <root>/<manufacturer>/<file>
*/

#define SYSTEM_MAX_NAME_SIZE    128
#define SYSTEM_MAX_PATH         256

#define SYSTEM_OK               0
#define SYSTEM_FILE_NOT_FOUND   -1
#define SYSTEM_NOT_DEFINED      -2

static int  system_initialized = 0;
static char sys_system[SYSTEM_MAX_NAME_SIZE];
static char sys_family[SYSTEM_MAX_NAME_SIZE];
static char sys_firmware[SYSTEM_MAX_NAME_SIZE];
static char sys_manufacturer[SYSTEM_MAX_NAME_SIZE];

static int checkFile(
                const char *root,
                const char *dir,
                const char *filename,
                char *fullpath,
                int  maxpath
                ) {

    /* build the file name */
    memset(fullpath, 0, maxpath);
    if (dir == NULL) {
        snprintf(fullpath, maxpath, "%s/%s", root, filename);
    } else {
        snprintf(fullpath, maxpath, "%s/%s/%s", root, dir, filename);
    }
    DEBUG("checking fullpath=%s\n", fullpath); 
    if (access(fullpath, R_OK) == 0) {
        return SYSTEM_OK;
    }
    memset(fullpath, 0, maxpath);
    return SYSTEM_FILE_NOT_FOUND;
}

/*!
* @fn int SystemInit()
* @brief init API by translating various envvar related to the current base station
* @return SYSTEM_OK or SYSTEM_NOT_DEFINED
*/
int SystemInit() {

    char    *pt;

    DEBUG("SystemInit\n");

    memset(sys_system, 0, SYSTEM_MAX_NAME_SIZE);
    pt = getenv("SYSTEM");
    if (pt == NULL) {
        return SYSTEM_NOT_DEFINED;
    }
    strncpy(sys_system, pt, SYSTEM_MAX_NAME_SIZE);
    DEBUG("SYSTEM=%s\n", sys_system);

    memset(sys_family, 0, SYSTEM_MAX_NAME_SIZE);
    pt = getenv("FAMILY");
    if (pt != NULL) {
        strncpy(sys_family, pt, SYSTEM_MAX_NAME_SIZE);
        DEBUG("FAMILY=%s\n", sys_family);
    }
    
    memset(sys_firmware, 0, SYSTEM_MAX_NAME_SIZE);
    pt = getenv("FIRMWARE");
    if (pt != NULL) {
        strncpy(sys_firmware, pt, SYSTEM_MAX_NAME_SIZE);
        DEBUG("FIRMWARE=%s\n", sys_firmware);
    }
    
    memset(sys_manufacturer, 0, SYSTEM_MAX_NAME_SIZE);
    pt = getenv("MANUFACTURER");
    if (pt != NULL) {
        strncpy(sys_manufacturer, pt, SYSTEM_MAX_NAME_SIZE);
        DEBUG("MANUFACTURER=%s\n", sys_manufacturer);
    }

    system_initialized = 1;
    return SYSTEM_OK;
}

/*!
* @fn int SystemGetName(char *system, int max)
* @brief returns the current SYSTEM name
* @param system     pointer to returned string
* @param max        max of the returned string
* @return SYSTEM_OK or SYSTEM_NOT_DEFINED
*/
int SystemGetName(char *system, int max) {

    int     cr;
   
    DEBUG("internal system=%s\n", sys_system);
    memset(system, 0, max); 
    if (! system_initialized) {
        DEBUG("init system\n");
        cr = SystemInit();
        if (cr != SYSTEM_OK) {
            DEBUG("init failed!!!\n");
            return cr;
        }
    }
    DEBUG("internal system=%s\n", sys_system);
    strncpy(system, sys_system, max);
    return SYSTEM_OK;
}


/*!
* @fn int SystemGetFilePath(const char *root, const char *filename, char *fullpath, int maxpath)
* @param root       root directory of file searching
* @param filemane   file name to look for
* @param fullpath   pointer to full path 
* @param maxpath    max of fullpath
* @return SYSTEM_OK or SYSTEM_FILE_NOT_FOUND
*/
int SystemGetFilePath(
            const char *root,
            const char *filename,
            char *fullpath,
            int maxpath
            )
{
    int     cr;

    if (! system_initialized) {
        cr = SystemInit();
        if (cr != SYSTEM_OK) {
            return cr;
        }
    }

    /* does SYSTEM-specific file exist? */
    if (checkFile(root, sys_system , filename, fullpath, maxpath) == SYSTEM_OK) {
        return SYSTEM_OK;
    }

    /* does FAMILY-specific file exists? */
    if (strlen(sys_family) > 0) {
        if (checkFile(root, sys_family, filename, fullpath, maxpath) == SYSTEM_OK) {
            return SYSTEM_OK;
        }
    }

    /* does FIRMARE-specific file exists? */
    if (strlen(sys_firmware) > 0) {
        if (checkFile(root, sys_firmware, filename, fullpath, maxpath) == SYSTEM_OK) {
            return SYSTEM_OK;
        }
    }

    /* does MANUFACTURER-specific file exists? */
    if (strlen(sys_manufacturer) > 0) {
        if (checkFile(root, sys_manufacturer, filename, fullpath, maxpath) == SYSTEM_OK) {
            return SYSTEM_OK;
        }
    }

    /* generic */
   return (checkFile(root, NULL, filename, fullpath, maxpath));

}

/*!
* @fn char *SystemGetEnv(const char *env_var, const char *def)
* @brief used by SUPLOG to get the value of a configuration variable
* @param env_var    variable to translate
* @param def        default value to return if not defined
* @return value of the environment variable or the default value
* @note SAVE the returned value. Use a internal variable that will be superseded by next calls
*/
char *SystemGetEnv(const char *env_var, const char *def) {

    FILE    *fd;
    int     ret;
    char    cmd[256];
    char    tmpfile[32];

    static char    val[256];

    memset(val, 0, sizeof(val));

    /* translate the env variable and store the result into a tmp file */
    snprintf(tmpfile, sizeof(tmpfile), "/tmp/suplog.%d", getpid());

    /* using pipe hides any output from scripts */
    // snprintf(cmd, sizeof(cmd), ". /var/run/lrrsystem ; . $ROOTACT/lrr/com/system_setting.sh | echo $%s > %s", env_var, tmpfile);
    snprintf(cmd, sizeof(cmd), ". /var/run/lrrsystem ; . $ROOTACT/lrr/com/system_setting.sh ; echo $%s > %s", env_var, tmpfile);
    ret = system(cmd);
    DEBUG("cmd system status: %d\n", ret);
    if (ret == 0)
    {
        /* read the resulting string */
        fd = fopen(tmpfile, "r");
        if (fd != NULL)
        {
            fgets(val, sizeof(val), fd);
            DEBUG("val before update: %s\n", val);
            fclose(fd);
        }
    }
    remove(tmpfile);

    int pos = 0;
    int i;
    for (i = 0 ; i < strlen(val) && val[i] != '\0'; i++)
    {
        pos = i;
    }
    val[pos] = '\0';

    if (strlen(val) == 0 && def != NULL) {
        strcat(val, def);
    }

    DEBUG("val before returning: %s\n", val);

    return val;
}

/* ------------------------------------- */

#ifdef TEST

int main(int argc, char *argv[0]) {

#define MAX_STR     100
    int     cr  = 0;
    char    string[MAX_STR];
    char    *rootact;
    char    dir[MAX_STR];
    char    fullpath[MAX_STR];

    cr = SystemGetName(string, MAX_STR);
    printf("cr=%d  SYSTEM=%s\n", cr, string);

    cr = SystemGetFilePath(argv[1], argv[2], fullpath, MAX_STR);
    printf("cr=%d path=%s\n", cr, fullpath);

    printf("NETSTAT=%s\n", SystemGetEnv("NETSTAT", "default_tst"));
    
}
#endif
