// ST7735 library example
// SD Card Info, Tester and CID Analyzer
// Requires SdFat, Arduino_ST7735_STM and RRE font libraries
// (C)2017-20 Pawel A. Hernik
// YouTube video: https://youtu.be/s2bYx58kJ_U 

/*
 ST7735 128x160 1.8" LCD pinout (header at the top, from left):
 #1 LED   -> 3.3V
 #2 SCK   -> SCL/D13/PA5
 #3 SDA   -> MOSI/D11/PA7
 #4 A0/DC -> D8/PA1  or any digital
 #5 RESET -> D9/PA0  or any digital
 #6 CS    -> D10/PA2 or any digital
 #7 GND   -> GND
 #8 VCC   -> 3.3V

           SPI2/SPI1
 SD_SCK  - PB13/PA5
 SD_MISO - PB14/PA6
 SD_MOSI - PB15/PA7
 SD_CS   - PB12/PA4
*/

/*
 STM32 SPI1/SPI2 pins:
 
 SPI1 MOSI PA7
 SPI1 MISO PA6
 SPI1 SCK  PA5
 SPI1 CS   PA4

 SPI2 MOSI PB15
 SPI2 MISO PB14
 SPI2 SCK  PB13
 SPI2 CS   PB12
*/

#include <SPI.h>
#include <Adafruit_GFX.h>

#if (__STM32F1__) // bluepill
#define TFT_CS  PA2
#define TFT_DC  PA1
#define TFT_RST PA0
#include <Arduino_ST7735_STM.h>
#else
#define TFT_CS 10
#define TFT_DC  8
#define TFT_RST 9
//#include <Arduino_ST7735_Fast.h>
#endif

#define SCR_WD 128
#define SCR_HT 160
Arduino_ST7735 lcd = Arduino_ST7735(TFT_DC, TFT_RST, TFT_CS);

#include "RREFont.h"
#include "rre_5x8.h"

RREFont font;

// -------------------------
// needed for RREFont library initialization, define your fillRect
void customRect(int x, int y, int w, int h, int c) { lcd.fillRect(x, y, w, h, c); }
// -------------------------

#include "SdFat.h"

#define USE_SDIO 0
const int8_t DISABLE_CHIP_SELECT = -1;
//const uint8_t SD_CS = PB12;
//SdFat sd(2);
const uint8_t SD_CS = PA4;
SdFat sd(1);

uint32_t cardSize;
uint32_t eraseSize;

#define GREY    RGBto565(160,160,160)
#define LGREY   RGBto565(200,200,200)
#define DGREY   RGBto565(100,100,100)
#define ORANGE  RGBto565(255,105,0)
#define DGREEN  RGBto565(0,160,0)
#define PINK    RGBto565(225,180,175)
#define MAGENTA RGBto565(255,100,255)

// use 10 and 5 for more optimal text density
#define LINE_HT 9
#define LINE_GAP 3

/*
uint16_t bgCol = BLACK;
uint16_t errCol = RED;
uint16_t okCol = GREEN;

uint16_t tabActCol    = RGBto565(0,150,150);
uint16_t tabActTxtCol = WHITE;
uint16_t tabActLinCol = RGBto565(0,255,255);
uint16_t tabIdlCol    = RGBto565(120,120,120);
uint16_t tabIdlTxtCol = RGBto565(90,90,90);
uint16_t tabIdlLinCol = RGBto565(200,200,200);

uint16_t cid1Col = GREEN;
uint16_t cid2Col = ORANGE;
uint16_t cid3Col = CYAN;
uint16_t cid4Col = MAGENTA;
uint16_t cid5Col = YELLOW;
uint16_t cid6Col = GREY;
uint16_t cid7Col = RED;
uint16_t vendCol = WHITE;
uint16_t hdCol    = WHITE;
uint16_t part1Col = DGREEN;
uint16_t part2Col = GREEN;
uint16_t partLCol = GREY;
uint16_t fsCol    = CYAN;
uint16_t statCol  = LGREY;
uint16_t csdCol   = RGBto565(125,255,175);
uint16_t cardCol  = GREEN;
uint16_t ocrCol   = CYAN;
*/
uint16_t bgCol  = RGBto565(140,140,140);
uint16_t errCol = RGBto565(250,0,0);
uint16_t okCol  = RGBto565(80,250,80);

uint16_t tabActCol    = bgCol;
uint16_t tabActTxtCol = WHITE;
uint16_t tabActLinCol = RGBto565(220,220,220);
uint16_t tabIdlCol    = RGBto565(100,100,100);
uint16_t tabIdlTxtCol = RGBto565(60,60,60);
uint16_t tabIdlLinCol = RGBto565(160,160,160);

uint16_t cid1Col = RGBto565(100,250,100); // MID
uint16_t cid2Col = RGBto565(255,200,120); // OID
uint16_t cid3Col = RGBto565(100,250,250); // PNM
uint16_t cid4Col = RGBto565(255,170,255); // rev
uint16_t cid5Col = YELLOW; // PSN
uint16_t cid6Col = RGBto565(220,220,220); // MDT
uint16_t cid7Col = RGBto565(255,40,40); // CRC

uint16_t vendCol  = WHITE;
uint16_t hdCol    = RGBto565(0,0,180);
uint16_t part1Col = RGBto565(160,80,0);
uint16_t part2Col = RGBto565(140,40,0);
uint16_t partLCol = RGBto565(210,210,90);
uint16_t fsCol    = RGBto565(0,60,60);
uint16_t statCol  = RGBto565(0,70,70);
uint16_t csdCol   = RGBto565(20,90,20);
uint16_t cardCol  = RGBto565(110,110,0);
uint16_t ocrCol   = RGBto565(0,100,100);

void lcdSPI()
{
  SPI.beginTransaction(SPISettings(36000000, MSBFIRST, SPI_MODE3, DATA_SIZE_16BIT));
}

#define SD_SPEED 18
void sdSPI()
{
  SPI.beginTransaction(SD_SCK_MHZ(SD_SPEED));
}

int xp=0, yp=0;
char txt[40];
int xpSt = 1;
void incX(int wd)
{
  xp+=wd;
  if(xp>=SCR_WD) {  xp = xpSt; incY(); }
}

void incY() { yp+=LINE_HT; }

int printString(const char *str)
{
  lcdSPI();
  while(*str) {
    if(*str=='\n') {
      xp=xpSt; incY();
    } else {
      int wd = font.drawChar(xp, yp, *str);
      incX(wd);
    }
    str++;
  }
  sdSPI();
  return xp;
}

int printStringCol(const char *str, uint16_t col)
{
  font.setColor(col);
  printString(str);
}

// ------------------------------------------------

char str[30];
const char *val2dec(int val, char *end=NULL)
{
  sprintf(str,"%d%s",val,end?end:"");
  return str;
}
const char *val2hex(int val, char *end=NULL)
{
  sprintf(str,"0x%x%s",val,end?end:"");
  return str;
}

// ------------------------------------------------

const char *toManuf(int val)
{
  switch(val){
    case 1: return "Panasonic";break;
    case 2: return "Toshiba";break;
    case 3: return "SanDisk";break;
    case 6: return "Ritek";break;
    case 9: return "ATP";break;
    case 19: return "Kingmax";break;
    case 25: return "Dynacard";break;
    case 26: return "PQI";break;
    case 27: return "Samsung";break;
    case 29: return "ADATA";break;
    case 39: return "Phison";break;
    case 40: return "Barun";break;
    case 81: return "STEC";break;
    case 93: return "SwissBit";break;
    case 97: return "Netlist";break;
    case 99: return "Cactus";break;
    case 116: return "Jiaelec";break;
    case 130: return "JangTay";break;
    case 131: return "Netcom";break;
    case 132: return "Strontium";break;
    default: return "unknown";break;
  }
}

uint8_t showCID() 
{
  cid_t cid;
  if(!sd.card()->readCID(&cid)) {
    font.setBold(1);
    printStringCol("\nReadCID failed!\n",errCol);
    font.setBold(0);
    return 0;
  }

  font.setColor(hdCol);
  yp+=LINE_GAP;
  font.setBold(1);
  printString("CID Decoder\n");
  font.setBold(0);
  yp+=1;
  const char *cid8 = (const char *)&cid;
  font.setFont(&rre_5x8); font.setFontMinWd(5); font.setBold(1);
  uint16_t cidCol;
  xp=xpSt;
  int yc = yp;
  for(int i=0;i<16;i++) {
    if(i==8) { xp=1; yp+=10; }
    if(i==0) cidCol=cid1Col; else // MID
    if(i>=1 && i<=2) cidCol=cid2Col; else // OID
    if(i>=3 && i<=7) cidCol=cid3Col; else // PNM
    if(i>=8 && i<=8) cidCol=cid4Col; else // PRV
    if(i>=9 && i<=12) cidCol=cid5Col; else //PSN
    if(i>=13 && i<=14) cidCol=cid6Col; else cidCol=cid7Col; // MDT/CRC
    sprintf(txt,"%02X",cid8[i]); printStringCol(txt,cidCol);
  }
  font.setBold(0);
  yp=yc+21;
  xp=xpSt;
  sprintf(txt,"MnfID:  %02X %s\n",int(cid.mid),toManuf(cid.mid)); font.setColor(cid1Col); printString(txt);
  sprintf(txt,"OEMID:  %c%c\n",cid.oid[0],cid.oid[1]); font.setColor(cid2Col); printString(txt);
  sprintf(txt,"PrdID:  "); font.setColor(cid3Col); printString(txt);
  for (uint8_t i = 0; i < 5; i++) { sprintf(txt,"%c",cid.pnm[i]); printString(txt); }
  sprintf(txt,"\nRev:    %d.%d\n", int(cid.prv_n), int(cid.prv_m)); font.setColor(cid4Col); printString(txt);
  sprintf(txt,"Serial: %02X%02X%02X%02X\n",cid.psn&0xff,(cid.psn>>8)&0xff,(cid.psn>>16)&0xff,(cid.psn>>24)&0xff); font.setColor(cid5Col); printString(txt);
  sprintf(txt,"MnfDat: %d/%d\n",int(cid.mdt_month),(2000 + cid.mdt_year_low + 10 * cid.mdt_year_high)); font.setColor(cid6Col); printString(txt);
  return 1;
}
//------------------------------------------------------------------------------

const char *toTAAC3_6(int val)
{
  switch(val>>3){
    case 0:  return "reserved" ;break;
    case 1:  return "1.0" ;break;
    case 2:  return "1.2" ;break;
    case 3:  return "1.3" ;break;
    case 4:  return "1.5" ;break;
    case 5:  return "2.0" ;break;
    case 6:  return "2.5" ;break;
    case 7:  return "3.0" ;break;
    case 8:  return "3.5" ;break;
    case 9:  return "4.0" ;break;
    case 10:  return "4.5" ;break;
    case 11:  return "5.0" ;break;
    case 12:  return "5.5" ;break;
    case 13:  return "6.0" ;break;
    case 14:  return "7.0" ;break;
    case 15:  return "8.0" ;break;
  }
}

const char *toTAAC0_2(int val)
{
  switch(val&0x07){
    case 0:  return "1ns" ; break;
    case 1:  return "10ns" ; break;
    case 2:  return "100ns" ; break;
    case 3:  return "1us" ; break;
    case 4:  return "10us" ; break;
    case 5:  return "100us" ; break;
    case 6:  return "1ms" ; break;
    case 7:  return "10ms" ; break;
  }
}

const char *toMaxCurVDDMin(int val)
{
  switch(val){
    case 0:  return "0.5" ;break;
    case 1:  return "1" ;break;
    case 2:  return "5" ;break;
    case 3:  return "10" ;break;
    case 4:  return "25" ;break;
    case 5:  return "35" ;break;
    case 6:  return "60" ;break;
    case 7:  return "100" ;break;
    default:  return "??" ;break;
  }
}

const char *toMaxCurVDDMax(int val)
{
  switch(val){
    case 0:  return "1" ;break;
    case 1:  return "5" ;break;
    case 2:  return "10" ;break;
    case 3:  return "25" ;break;
    case 4:  return "35" ;break;
    case 5:  return "45" ;break;
    case 6:  return "80" ;break;
    case 7:  return "200" ;break;
    default:  return "???" ;break;
  }
}

const char *toCSDVer(int val)
{
  switch(val){
    case 0:  return "V1.0" ;break;
    case 1:  return "V2.0" ;break;
    default:  return "????" ;break;
  }
}

const char *toTransSpeedV1(int val)
{
  switch(val&7){
    case 0:  return "100kbps" ; break;
    case 1:  return "1Mbps" ; break;
    case 2:  return "10Mbps" ; break;
    case 3:  return "100Mbps" ; break;
    default: return "wrong" ; break;
  }
}

const char *toTransSpeedV2(int val)
{
  switch(val){
    case 0x0b:  return "100Mbps" ; break;
    case 0x2b:  return "200Mbps" ; break;
    case 0x32:  return "25MHz" ; break;
    case 0x5a:  return "50MHz" ; break;
    default: return "wrong" ; break;
  }
}

const char *toFileFormat(int val)
{
  switch(val&3){
    //case 0:  return "HardDisk/part"; break;
    case 0:  return "HDD/part"; break;
    //case 1:  return "Floppy/boot"; break;
    case 1:  return "FDD/boot"; break;
    case 2:  return "UFS"; break;
    default: return "Other" ; break;
  }
}

uint8_t showCSD() 
{
  csd_t csd;
  uint8_t eraseSingleBlock;
  sdSPI();
  if(!sd.card()->readCSD(&csd)) {
    printStringCol("readCSD failed!\n",errCol);
    return 0;
  }
  int ver = csd.v1.csd_ver;
  font.setColor(hdCol);
  yp+=LINE_GAP;
  sprintf(txt,"CSD         %s\n", toCSDVer(ver)); printString(txt);
  font.setColor(csdCol);
  if(ver==0) {
    sprintf(txt,"TAAC:       %s %s\n", toTAAC0_2(csd.v1.taac), toTAAC3_6(csd.v1.taac)); printString(txt);
    sprintf(txt,"TransSpeed: %02x %s\n", csd.v1.tran_speed, toTransSpeedV1(csd.v1.tran_speed)); printString(txt);
    sprintf(txt,"Read/Write: %d\n", 1<<csd.v1.r2w_factor); printString(txt);
    sprintf(txt,"FileFormat: %s\n", toFileFormat(csd.v1.file_format)); printString(txt);
    sprintf(txt,"VDDRdCur:   %s-%smA\n", toMaxCurVDDMin(csd.v1.vdd_r_curr_min),toMaxCurVDDMax(csd.v1.vdd_r_curr_max)); printString(txt);
    sprintf(txt,"VDDWrCurMn: %s-%smA\n", toMaxCurVDDMin(csd.v1.vdd_w_curr_min),toMaxCurVDDMax(csd.v1.vdd_w_cur_max)); printString(txt);
    sprintf(txt,"Tmp/PermWP: %s/%s\n", csd.v1.tmp_write_protect?"YES":"NO", csd.v1.perm_write_protect?"YES":"NO"); printString(txt);
    sprintf(txt,"Copy:       %s\n", csd.v1.copy?"YES":"NO"); printString(txt);
    eraseSingleBlock = csd.v1.erase_blk_en;
    eraseSize = (csd.v1.sector_size_high << 1) | csd.v1.sector_size_low;
  } else if(ver==1) {
    sprintf(txt,"TAAC:       %02d\n", csd.v2.taac); printString(txt);
    sprintf(txt,"TransSpeed: %02x %s\n", csd.v2.tran_speed, toTransSpeedV2(csd.v2.tran_speed)); printString(txt);
    sprintf(txt,"Read/Write: %d\n", 1<<csd.v2.r2w_factor); printString(txt);
    sprintf(txt,"FileFormat: %s\n", toFileFormat(csd.v2.file_format)); printString(txt);
    sprintf(txt,"Tmp/PermWP: %s/%s\n", csd.v1.tmp_write_protect?"YES":"NO", csd.v1.perm_write_protect?"YES":"NO"); printString(txt);
    sprintf(txt,"Copy:       %s\n", csd.v2.copy?"YES":"NO"); printString(txt);
    eraseSingleBlock = csd.v2.erase_blk_en;
    eraseSize = (csd.v2.sector_size_high << 1) | csd.v2.sector_size_low;
  } else {
    return false;
  }
  eraseSize++;
  font.setColor(cardCol);
  printString("Card Size:  "); dtostrf(0.000512*cardSize,4,0,txt); printString(txt); printString("MB\n"); 
  sprintf(txt,"EraseSize:  %d blcks\n", int(eraseSize)); printString(txt);
  sprintf(txt,"EraseSingl: %s\n", eraseSingleBlock ? "YES":"NO"); printString(txt);
  return 1;
}
//------------------------------------------------------------------------------

uint8_t showPartTab() 
{
  sdSPI();
  mbr_t mbr;
  if(!sd.card()->readBlock(0, (uint8_t*)&mbr)) {
    font.setBold(1);
    printStringCol("\nRead MBR failed\n",errCol);
    font.setBold(0);
    return 0;
  }
  for(uint8_t ip = 1; ip < 5; ip++) {
    part_t *pt = &mbr.part[ip - 1];
    if((pt->boot & 0X7F) != 0 || pt->firstSector > cardSize) {
      printStringCol("\nNo MBR.\nSuper Floppy format?\n",errCol);
      return 1;
    }
  }
  yp+=LINE_GAP;
  font.setBold(1);
  printStringCol("SD Partition Table\n",hdCol);
  font.setBold(0);
  yp+=LINE_GAP*2;
  font.setColor(part1Col);
  xp=1;
  printString("# Bt Tp Start  Length\n");
  yp+=LINE_GAP;
  font.setColor(part2Col);
  for(uint8_t ip = 1; ip < 5; ip++) {
    part_t *pt = &mbr.part[ip - 1];
    if(!pt->type) continue;
    sprintf(txt,"%d %02x %02x", int(ip),int(pt->boot),int(pt->type));
    xp=1;
    printString(txt);  
    sprintf(txt,"%dk", pt->firstSector/1024);
    xp=1+6*8; printString(txt);  
    sprintf(txt,"%dk\n", pt->totalSectors/1024);
    xp=1+6*15; printString(txt);  
  }
  lcdSPI();
  int yf = 17+LINE_GAP*3+5, hf=yp-yf+1;
  uint16_t c = partLCol;
  lcd.drawFastHLine(0,yf,SCR_WD,c);
  lcd.drawFastHLine(0,yf+12,SCR_WD,c);
  lcd.drawFastHLine(0,yp+1,SCR_WD,c);
  lcd.drawFastVLine(6*0+0,yf,hf,c);
  lcd.drawFastVLine(6*1+3,yf,hf,c);
  lcd.drawFastVLine(6*4+3,yf,hf,c);
  lcd.drawFastVLine(6*7+3,yf,hf,c);
  lcd.drawFastVLine(SCR_WD-6*6-4,yf,hf,c);
  lcd.drawFastVLine(SCR_WD-1,yf,hf,c);
  sdSPI();
  return 1;
}

//------------------------------------------------------------------------------

void showFileSystem() 
{
  if(!sd.fsBegin()) {
    font.setBold(1);
    printStringCol("\nFileSystem init\nfailed\n",errCol);
    font.setBold(0);
    return;
  }
  font.setColor(hdCol);
  yp+=LINE_GAP;
  sprintf(txt,"FileSystem:   FAT%d\n", int(sd.vol()->fatType())); printString(txt);
  yp+=LINE_GAP;
  font.setColor(fsCol);
  sprintf(txt,"Blocks/Clust: %d\n", int(sd.vol()->blocksPerCluster())); printString(txt);
  sprintf(txt,"ClusterCount: %d\n",  sd.vol()->clusterCount() ); printString(txt);
  printStringCol("Reading...",hdCol); font.setColor(fsCol);
  uint32_t volFree = sd.vol()->freeClusterCount();
  xp=0;
  lcdSPI(); lcd.fillRect(xp,yp,SCR_WD,8, bgCol);
  sprintf(txt,"FreeClusters: %ld\n", volFree); printString(txt);
  float fs = 0.000512*volFree*sd.vol()->blocksPerCluster();
  printString("FreeSpace:    "); dtostrf(fs,2,0,txt); printString(txt); printString("MB\n"); 
  sprintf(txt,"FATStartBlk:  %d\n",  sd.vol()->fatStartBlock()); printString(txt);
  sprintf(txt,"FATCount:     %d\n",  int(sd.vol()->fatCount())); printString(txt);
  sprintf(txt,"BlocksPerFAT: %d\n",  sd.vol()->blocksPerFat()); printString(txt);
  sprintf(txt,"RootDirStart: %d\n",  sd.vol()->rootDirStart()); printString(txt);
  sprintf(txt,"DataStartBlk: %d\n",    sd.vol()->dataStartBlock()); printString(txt);
  sprintf(txt,"Align:        "); printString(txt);
  sdSPI();
  if(sd.vol()->dataStartBlock() % eraseSize) printStringCol("BAD",errCol); else printStringCol("OK",okCol);
}

// ------------------------------------------------

int showOCR()
{
  uint32_t ocr;
  if(!sd.card()->readOCR(&ocr)) {
    printStringCol("ReadOCR failed!\n",errCol);
    return 0;
  }
  yp+=LINE_GAP;
  sprintf(txt,"OCR:        %08X\n",ocr); printStringCol(txt,ocrCol);
  printString("VDD 2.7-3.6V:");
  if((ocr&0xff8000)==0xff8000) printString(" ALL"); else {
    int b = (1<<15);
    for(int i=0;i<9;i++) {
      printString((ocr & b)?(char*)"1":(char*)"0");
      b<<=1;
    }
  }
  printString("\n");
  if(ocr & (1<<24)) printString("Switching to 1.8V: YES\n");
  if(ocr & (1<<29)) printString("UHS-II Interface:  YES\n");
  return 1;
}

// ------------------------------------------------

const char *toClass(int val)
{
  switch(val){
    case 0: return "0"; break;
    case 1: return "2"; break;
    case 2: return "4"; break;
    case 3: return "6"; break;
    case 4: return "10"; break;
    default: return val2hex(val) ; break;
  }
}

const char *toPerf(int val)
{
  switch(val){
    case 0:   return "SeqWrite"; break;
    case 255: return "Infinity"; break;
    default: return val2dec(val," MB/s") ; break;
  }
}

const char *toUHS(int val)
{
  switch(val){
    case 0x00: return "<10 MB/s"; break;
    case 0x10: return "10+ MB/s"; break;
    case 0x30: return "30+ MB/s"; break;
    default: return val2hex(val) ; break;
  }
}

int showStatus()
{
  uint8_t stat[64];
  if(!sd.card()->readStatus(stat)) {
    printStringCol("\nReadStatus failed!",errCol);
    return 0;
  }
  font.setBold(1);
  printStringCol("Card: ",hdCol);
  switch (sd.card()->type()) {
    case SD_CARD_TYPE_SD1: printString("SD1"); break;
    case SD_CARD_TYPE_SD2: printString("SD2"); break;
    case SD_CARD_TYPE_SDHC: printString(cardSize < 70000000 ? "SDHC":"SDXC"); break;
    default: printStringCol("???",errCol);
  }
  sprintf(txt," Class%s\n",toClass(stat[8])); printString(txt);
  font.setBold(0);
  font.setColor(statCol);
  sprintf(txt,"PerfMove: %s\n",toPerf(stat[9])); printString(txt);
  //sprintf(txt,"Bus:  %x\n",stat[0]&0xc0); printString(txt);
  sprintf(txt,"UHS Spd:  %s\n",toUHS(stat[13]&0xf0)); printString(txt);
  //sprintf(txt,"VideoSpeed:  %d\n",stat[15]); printString(txt);
  return 1;
}

// ------------------------------------------------

#define PROGRAM_CID_OPCODE    26
#define SAMSUNG_VENDOR_OPCODE 62
#define ENTER_VENDOR          0xEFAC62EC
#define EXIT_VENDOR           0x00DECCEE
#define UNLOCK_CID_WRITE      0xEF50

int checkCIDhack()
{
  int retEn = sd.card()->cardCommand(SAMSUNG_VENDOR_OPCODE, ENTER_VENDOR);
  int retCW = sd.card()->cardCommand(SAMSUNG_VENDOR_OPCODE, UNLOCK_CID_WRITE);
  int retEx = sd.card()->cardCommand(SAMSUNG_VENDOR_OPCODE, EXIT_VENDOR);
  yp+=2;
  sprintf(txt,"Enter Vendor: "); printStringCol(txt,vendCol);
  sprintf(txt,"%s (%X)\n",retEn?"NO":"YES",retEn); printStringCol(txt,retEn?errCol:okCol);
  if(retEn==0) {
    sprintf(txt,"CID Write:    "); printStringCol(txt,vendCol);
    sprintf(txt,"%s (%X)\n",retCW?"NO":"YES",retCW); printStringCol(txt,retCW?errCol:okCol);
    sprintf(txt,"Exit Vendor:  "); printStringCol(txt,vendCol);
    sprintf(txt,"%s (%X)\n",retEx?"NO":"YES",retEx); printStringCol(txt,retEx?errCol:okCol);
    //if(ret==0 && ret2==0) printStringCol("CID UNLOCKED\n",okCol); else printStringCol("CID LOCKED\n",errCol); 
  } else printStringCol("CID CHANGE:   LOCKED\n",errCol); 
  return 1;
}

// --------------------------------------------------------------------------
#define BUTTON PB9
int stateOld = HIGH;
long btDebounce    = 30;
long btDoubleClick = 600;
long btLongClick   = 500;
long btLongerClick = 2000;
long btTime = 0, btTime2 = 0;
int clickCnt = 1;

// 0=idle, 1,2,3=click, -1,-2=longclick
int checkButton()
{
  int state = digitalRead(BUTTON);
  if( state == LOW && stateOld == HIGH ) { btTime = millis(); stateOld = state; return 0; } // button just pressed
  if( state == HIGH && stateOld == LOW ) { // button just released
    stateOld = state;
    if( millis()-btTime >= btDebounce && millis()-btTime < btLongClick ) { 
      if( millis()-btTime2<btDoubleClick ) clickCnt++; else clickCnt=1;
      btTime2 = millis();
      return clickCnt; 
    } 
  }
  if( state == LOW && millis()-btTime >= btLongerClick ) { stateOld = state; return -2; }
  if( state == LOW && millis()-btTime >= btLongClick ) { stateOld = state; return -1; }
  return 0;
}
// --------------------------------------------------------------------------
void setColorSet(int set)
{
  if(set==0) {
    bgCol = BLACK;
    errCol = RED;
    okCol = GREEN;
    
    tabActCol    = RGBto565(0,150,150);
    tabActTxtCol = WHITE;
    tabActLinCol = RGBto565(0,255,255);
    tabIdlCol    = RGBto565(120,120,120);
    tabIdlTxtCol = RGBto565(90,90,90);
    tabIdlLinCol = RGBto565(200,200,200);
    
    cid1Col = GREEN;
    cid2Col = ORANGE;
    cid3Col = CYAN;
    cid4Col = MAGENTA;
    cid5Col = YELLOW;
    cid6Col = GREY;
    cid7Col = RED;
    vendCol = WHITE;
    hdCol    = WHITE;
    part1Col = DGREEN;
    part2Col = GREEN;
    partLCol = GREY;
    fsCol    = CYAN;
    statCol  = LGREY;
    csdCol   = RGBto565(125,255,175);
    cardCol  = GREEN;
    ocrCol   = CYAN;
  } else {
    bgCol  = RGBto565(140,140,140);
    errCol = RGBto565(250,0,0);
    okCol  = RGBto565(80,250,80);
    
    tabActCol    = bgCol;
    tabActTxtCol = WHITE;
    tabActLinCol = RGBto565(220,220,220);
    tabIdlCol    = RGBto565(100,100,100);
    tabIdlTxtCol = RGBto565(60,60,60);
    tabIdlLinCol = RGBto565(160,160,160);
    
    cid1Col = RGBto565(100,250,100); // MID
    cid2Col = RGBto565(255,200,120); // OID
    cid3Col = RGBto565(100,250,250); // PNM
    cid4Col = RGBto565(255,170,255); // rev
    cid5Col = YELLOW; // PSN
    cid6Col = RGBto565(220,220,220); // MDT
    cid7Col = RGBto565(255,40,40); // CRC
    
    vendCol  = WHITE;
    hdCol    = RGBto565(0,0,180);
    part1Col = RGBto565(160,80,0);
    part2Col = RGBto565(140,40,0);
    partLCol = RGBto565(210,210,90);
    fsCol    = RGBto565(0,60,60);
    statCol  = RGBto565(0,70,70);
    csdCol   = RGBto565(20,90,20);
    cardCol  = RGBto565(110,110,0);
    ocrCol   = RGBto565(0,100,100);
  }
}
// --------------------------------------------------------------------------

void drawTabs(int mode)
{
  lcdSPI();
  lcd.fillScreen(bgCol); xp=xpSt; yp=0;
  uint16_t c,i;
  lcd.fillRect(0,0,SCR_WD,2,BLACK);
  for(i=0;i<4;i++) {
    lcd.fillRect(1+32*i,1,i==3?30:31,i==mode?12:11,(mode==i) ? tabActCol : tabIdlCol);
    c = tabIdlLinCol;
    lcd.drawFastVLine(32*(i+0),2,12-2,c);
    lcd.drawFastVLine(i==3 ? 32*(i+1)-1 : 32*(i+1),2,12-2,c);
    lcd.drawFastHLine(32*i+2,0,i==3 ? 32-4 : 32-3,c);
    lcd.drawPixel(32*i+1,1,c);
    lcd.drawPixel(i==3 ? 32*(i+1)-2 : 32*(i+1)-1,1,c);
  }
  i = mode;
  c = tabActLinCol;
  lcd.drawFastVLine(32*(i+0),2,12-2,c);
  lcd.drawFastVLine(i==3 ? 32*(i+1)-1 : 32*(i+1),2,12-2,c);
  lcd.drawFastHLine(32*i+2,0,i==3 ? 32-4 : 32-3,c);
  lcd.drawPixel(32*i+1,1,c);
  lcd.drawPixel(i==3 ? 32*(i+1)-2 : 32*(i+1)-1,1,c);
  lcd.drawFastHLine(0,12,32*i+1,c);
  if(i<3) lcd.drawFastHLine(32*(i+1),12,32*(3-i),c); else lcd.drawFastHLine(32*(i+1)-1,12,1,c);
  
  font.setColor(mode==0 ? tabActTxtCol : tabIdlTxtCol); font.printStr(7,3,"CID");
  font.setColor(mode==1 ? tabActTxtCol : tabIdlTxtCol); font.printStr(40,3,"CSD");
  font.setColor(mode==2 ? tabActTxtCol : tabIdlTxtCol); font.printStr(70,3,"Part");
  font.setColor(mode==3 ? tabActTxtCol : tabIdlTxtCol); font.printStr(107,3,"FS");
  yp=16;
}
// --------------------------------------------------------------------------

unsigned long ms;

void setup(void)
{
  Serial.begin(115200);
  pinMode(BUTTON, INPUT_PULLUP);
  lcd.init();
  font.init(customRect, SCR_WD, SCR_HT); // custom fillRect function and screen width and height values
  font.setScale(1,1);
  font.setFont(&rre_5x8); font.setFontMinWd(5);
  xp=0; yp=0;
  setColorSet(0);
  ms=millis();
}

// --------------------------------------------------------------------------

int screen=0;
int colSet=0;

void loop(void)
{
  lcdSPI();
  font.setScale(1,1);
  lcd.fillScreen(bgCol); xp=xpSt; yp=0;

  //font.setBold(1); printStringCol("SD Card\ninitializing ...",okCol); font.setBold(0);
  if(!sd.cardBegin(SD_CS, SD_SCK_MHZ(SD_SPEED))) {
    lcdSPI(); lcd.fillScreen(bgCol); xp=yp=0;
    font.setBold(1);
    printStringCol("Cannot initialize\nSD Card!",errCol);
    font.setBold(0);
    screen=-1;
    goto wait;
  }
  drawTabs(screen);
 
  sdSPI();
  cardSize = sd.card()->cardSize();
  if(cardSize <= 0) {
    font.setBold(1);
    printStringCol("cardSize failed!\n",errCol);
    font.setBold(0);
    screen=-1;
    goto wait;
  }
  switch(screen) {
    case 0: showStatus(); showCID(); checkCIDhack(); break;
    case 1: showCSD(); showOCR(); break;
    case 2: showPartTab(); break;
    case 3: showFileSystem(); break;
  }

wait:
  int bt;
  if(screen>=0) while((bt = checkButton())==0 && millis()-ms<15000);
  else          while((bt = checkButton())==0);

  ms = millis();
  if(screen<0) { screen=0; return; }
  if(bt<0) {
    while((bt = checkButton())<0);
    screen=0;
    if(++colSet>1) colSet=0;
    setColorSet(colSet);
  }
  if(++screen>=4) screen=0;
}

// ------------------------------------------------

