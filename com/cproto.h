
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



void	FindIpInterfaces(t_lrr_config *config);
t_wan_itf	*FindItfByName(char *name);
void	CompAllItfInfos(char shortlongtime);
char	*FindItfDefautRoute();
uint8_t	StateItf(t_wan_itf *itf);
uint32_t	GetMacAddr32(char *name,u_short *ext);
uint32_t	GetIpv4Addr(char *name);
uint32_t	FindEthMac32(u_short *ext);
uint32_t	GetLrrIDIfCellular();
void	AddGlobalScopeIpv6();

void	AvDvUiClear(t_avdv_ui *ad);
void	AvDvUiAdd(t_avdv_ui *ad,uint32_t val,time_t when);
int	AvDvUiCompute(t_avdv_ui *ad,time_t tmax,time_t when);

char	*CfgStr(void *ht,char *sec,int index,char *var,char *def);
int	CfgInt(void *ht,char *sec,int index,char *var,int def);
int CfgCBIniLoad(void *user,const char *section,const char *name,const char *value);

int	LgwGenConfigure(int hot,int config);
void	LgwDumpGenConf(FILE *f);
int LgwTxPowerComp(char *ismband, int freq);
int	LgwTempPowerGain();

int	AdjustFreqForCalibration(char * ism);
void	AdjustDefaultIsmValues();

int	LgwStart();
int	LgwStarted();
void	LgwStop();
#if defined(KONA) && defined(WITH_GPS)
int	LgwDlReady();
#endif
void	*LgwRun(void *param);
int	LgwSendPacket(t_lrr_pkt *downpkt,int seqnum,int m802,int ack);
int	LgwPacketDelayMsFromUtc(t_lrr_pkt *downpkt,struct timespec *utc);
int	LgwPacketDelayMsFromUtc2(t_lrr_pkt *downpkt,struct timespec *utc);
int	LgwDiffMsUtc(struct timespec *utc1,struct timespec *utc0);
void	LgwEstimUtc(struct timespec *utc);
void	LgwInitPingSlot(t_lrr_pkt *pkt);
void	LgwResetPingSlot(t_lrr_pkt *pkt);
int	LgwNextPingSlot(t_lrr_pkt *pkt,struct timespec *eutc,int maxdelay,int *retdelay);

int	BinSearchFirstChannel(uint32_t freq);
int	BinSearchFirstChannelDn(uint32_t freq);
t_channel	*FindChannelUp(uint32_t freq);
t_channel	*FindChannelDn(uint32_t freq);

void	ChangeChannelFreq(t_lrr_pkt *downpkt,T_lgw_pkt_tx_t *txpkt);
int	CmpChannel(const void *m1, const void *m2);
int	ChannelConfigure(int hot,int config);
void	TmoaLrrPacket(t_lrr_pkt *downpkt);
float	TmoaLrrPacketUp(T_lgw_pkt_rx_t *pkt);
void	DoPcap(char *data,int sz);
void	AutoRx2Settings(t_lrr_pkt *downpkt);
void	AdjustRx2Settings(t_lrr_pkt *downpkt);


void	LgwForceDefine(void *htbl);
char	*LgwVersionInfo();
int	LgwConfigure(int hot,int config);
int	LgwLINKUP();
int	LgwDoSendPacket(time_t now);
int	LgwDoRecvPacket(time_t tms);
void	LgwGpsTimeUpdated(struct timespec *utc_from_gps, struct timespec * ubxtime_from_gps);
int LgwGetTrigCnt(uint8_t board, uint32_t *ptrig_tstamp);
int	LgwTxFree(uint8_t board,uint8_t *txstatus);
int	LgwGetTrigNow(uint8_t board,uint32_t *tus);
int	LgwTrigControl(uint8_t board, uint32_t tus, int32_t *diff);

void	DoSystemCmdForeGround(char *cmd);
void	DoSystemCmdBackGround(char *cmd);
void	DoSystemCmd(t_lrr_pkt *downpkt,char *cmd);
void	DoShellCommand(t_lrr_pkt *downpkt);
void	StartCmdThread();
unsigned int CmdCountOpenSsh();
unsigned int CmdCountRfScan();

int8_t GetTxCalibratedEIRP(int8_t tx_requested, float antenna_gain, float cable_loss, uint8_t board, uint8_t rfc);

void	DoLocKeyRefresh(int delay, char ** resp);
void	DoConfigRefresh(int delay);
void	DoAntsConfigRefresh(int delay, char ** resp);
void	DoCustomVersionRefresh(int delay);
void	DoStatRefresh(int delay);
void	DoRfcellRefresh(int delay);


int	OkDevAddr(char *dev);


unsigned short
crc_contiki(unsigned short acc,const unsigned char *data, int len);


void ReStartLgwThread();
void	StartItfThread();
int	UsbProtectOff();
int	UsbProtectOn();


void	lrr_keyInit();
int lrr_keyEnc(unsigned char *plaintext, int plaintext_len, unsigned char *key,
unsigned char *iv, unsigned char *ciphertext,int maxlen,int aschex);
int lrr_keyDec(unsigned char *ciphertext,int ciphertext_len,unsigned char *key,
  unsigned char *iv, unsigned char *plaintext,int maxlen,int aschex);
unsigned char        *BuildHex(int version);


void	SaveConfigFileState();
void	GpsGetInfos(u_char *mode,float *lat,float *lon,short *alt,u_int *cnt);
#ifdef WITH_GPS
void	GpsPositionUpdated(LGW_COORD_T *loc_from_gps);
int	GpsDeviceStatus();
int	GpsThreadStatus();
void	* GpsRun(void * param);
#endif /* WITH_GPS */

int	CellStateThreadStatus();
void	*CellStateRun(void * param);
void	CellStateCopyIdent(t_lrr_cellid *cellid);

void DcCheckLists();
int DcTreatDownlink(t_lrr_pkt *pkt);
int DcTreatUplink(t_lrr_pkt *pkt);
void DcSaveCounters(FILE *f);
void DcWalkCounters(void *f,void (*fct)(void *pf,int type,int ant,int idx,
                        float up,float dn));
void DcWalkChanCounters(void *f,void (*fct)(void *pf,int ant,int c,int s,
                        float up,float dn,float upsub,float dnsub));
void DcDumpHisto();

uint8_t	CodeSpreadingFactor(int sf);
uint8_t	CodeCorrectingCode(int coderate);
uint8_t	CodeBandWidth(int bandwidth);
uint8_t	DeCodeBandWidth(int bandwidth);
float	FreqBandWidth(int bandwidth);

int	IsIndicSet(t_lrr_pkt *pkt);
void	SetIndic(t_lrr_pkt *pkt,int delivered,int c1,int c2,int c3);
void	SendIndicToLrc(t_lrr_pkt *pkt);
void	SendCapabToLrc(t_xlap_link *lktarget);

void	DecryptHtbl(void *htbl);


#if defined(KEROS) && defined(WITH_LED_MGT)
int LedConfigure();
int LedShotTxLora();
int LedShotRxLora();
int LedBackhaul(int val);
#endif


void	NfInit();
void	NfReload();
int	NfCheck(u_int devaddr);
void	NfSave(u_int version,u_int netid,char *filter);

int	LgwSynchroUp();
int	LgwSynchroUtc2Cnt(u_int gss,u_int gns,u_int *trig_tstamp);
void	LgwSynchroTimeUpdated(t_lrr_sync_dn *sync);
void	LgwSynchroPseudoGps();
void	LgwSynchroDoClockSc(time_t now);

void	DoGpsRefresh(int delay, u_short lptype, u_char state, float srate_wma);
