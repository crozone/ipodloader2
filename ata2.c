/*
 * Basic ATA2 driver for the ipodlinux bootloader
 * 
 * Supports:
 *  PIO (Polling)
 *  Multiple block reads
 *
 * Author: James Jacobsson ( slowcoder@mac.com )
 * 
 * ATA2 code modified to support double sector reads as a single block for the
 * 5.5G 80GB iPod - by Vincent Huisman ( dataghost at dataghost dot com ) 
 * at 2007-01-23
 *
 */
#include "bootloader.h"
#include "console.h"
#include "ipodhw.h"
#include "minilibc.h"
#include "ata2.h"

/*
 * LBA28 registers
 */
#define REG_DATA       0x0
#define REG_ERROR      0x1
#define REG_FEATURES   0x1
#define REG_SECT_COUNT 0x2
#define REG_SECT       0x3
#define REG_CYL_LOW    0x4
#define REG_CYL_HIGH   0x5
#define REG_DEVICEHEAD 0x6
#define REG_STATUS     0x7
#define REG_COMMAND    0x7
#define REG_CONTROL    0x8
#define REG_ALTSTATUS  0x8

/*
 * LBA48 registers.
 */

// aka REG_SECT_COUNT. High byte is REG_SECCOUNT1
#define REG_SECCOUNT0  0x2
// aka REG_SECT. High byte is REG_LBA3
#define REG_LBA0       0x3
// aka REG_CYL_LOW. High byte is REG_LBA4
#define REG_LBA1       0x4
// aka REG_CYL_HIGH. High byte is REG_LBA5
#define REG_LBA2       0x5
#define REG_SECCOUNT1  0x0A
#define REG_LBA3       0x0B
#define REG_LBA4       0x0C
#define REG_LBA5       0x0D

#define REG_DA         0x9

#define CONTROL_NIEN   0x2
#define CONTROL_SRST   0x4

// all commands: see include/linux/hdreg.h

// IDENTIFY DEVICE (Identify)
#define COMMAND_IDENTIFY_DEVICE       0xEC
// READ MULTIPLE (RdMul). LBA28.
#define COMMAND_READ_MULTIPLE         0xC4
// READ SECTOR(S) (RdSec) - 20h, PIO Data-In. LBA28.
#define COMMAND_READ_SECTORS          0x20
// READ SECTORS WITHOUT RETRY (RdSecN). LBA28
#define COMMAND_READ_SECTORS_NORETRY  0x21
// READ SECTOR(S) EXT (RdSecEx) - 24h, PIO Data-In. LBA48.
#define COMMAND_READ_SECTORS_EXT      0x24
// STANDBY IMMEDIATE (StandbyIm)
#define COMMAND_STANDBY               0xE0

#define DEVICE_0       0xA0
#define DEVICE_1       0xB0

#define STATUS_BSY     0x80
#define STATUS_DRDY    0x40
#define STATUS_DF      0x20
#define STATUS_DSC     0x10
#define STATUS_DRQ     0x08
#define STATUS_CORR    0x04
#define STATUS_IDX     0x02
#define STATUS_ERR     0x01

unsigned int pio_base_addr1,pio_base_addr2;
unsigned int pio_reg_addrs[14];

/*
 * To keep memory usage at the same level, 8 blocks of 1024 instead of 16 of 512
 * Blocksize _NECESSARY_ for 1024b-sector-devices, unless uncached reads are used
 * Maybe this needs to be worked around in some way
 */
#define CACHE_NUMBLOCKS 8
#define CACHE_BLOCKSIZE 1024
static uint8  *cachedata;
static uint32 *cacheaddr;
static uint32 *cachetick;
static uint32  cacheticks;

/*
 * drivetype only has two valid values:
 *
 * 0: Drive does 512b sector reads with COMMAND_READ_SECTORS
 * 1: Drive needs 2x 512b sector reads with COMMAND_READ_MULTIPLE when unable to read odd
 *    sectors (5.5g iPod 80gb)
 */
static uint8 drivetype = 0;
static uint8 readcommand = COMMAND_READ_SECTORS; //COMMAND_READ_SECTORS_NORETRY;
static uint8 sectorcount = 1;

static struct {
  uint16 chs[3];
  uint32 sectors;
} ATAdev;

void pio_outbyte(unsigned int addr, unsigned char data) {
  outb( data, pio_reg_addrs[ addr ] );
}

void pio_outword(unsigned int addr, unsigned int data) {
  outl( data, pio_reg_addrs[ addr ] );
}

volatile unsigned char pio_inbyte( unsigned int addr ) {
  return( inl( pio_reg_addrs[ addr ] ) );
}
volatile unsigned short pio_inword( unsigned int addr ) {
  return( inl( pio_reg_addrs[ addr ] ) );
}
volatile unsigned int pio_indword( unsigned int addr ) {
  return( inl( pio_reg_addrs[ addr ] ) );
}

#define DELAY400NS { \
 pio_inbyte(REG_ALTSTATUS); pio_inbyte(REG_ALTSTATUS); \
 pio_inbyte(REG_ALTSTATUS); pio_inbyte(REG_ALTSTATUS); \
 pio_inbyte(REG_ALTSTATUS); pio_inbyte(REG_ALTSTATUS); \
 pio_inbyte(REG_ALTSTATUS); pio_inbyte(REG_ALTSTATUS); \
 pio_inbyte(REG_ALTSTATUS); pio_inbyte(REG_ALTSTATUS); \
 pio_inbyte(REG_ALTSTATUS); pio_inbyte(REG_ALTSTATUS); \
 pio_inbyte(REG_ALTSTATUS); pio_inbyte(REG_ALTSTATUS); \
 pio_inbyte(REG_ALTSTATUS); pio_inbyte(REG_ALTSTATUS); \
}

uint32 ata_init(void) {
  uint8   tmp[2],i;
  ipod_t *ipod;

  ipod = ipod_get_hwinfo();

  pio_base_addr1 = ipod->ide_base;
  pio_base_addr2 = pio_base_addr1 + 0x200;

  /*
   * Sets up a number of "shortcuts" for us to use via the pio_ macros
   * Note: The PP chips have their IO regs 4 byte aligned
   */
  pio_reg_addrs[ REG_DATA       ] = pio_base_addr1 + 0 * 4;
  pio_reg_addrs[ REG_FEATURES   ] = pio_base_addr1 + 1 * 4;
  pio_reg_addrs[ REG_SECT_COUNT ] = pio_base_addr1 + 2 * 4; // aka REG_SECCOUNT0
  pio_reg_addrs[ REG_SECT       ] = pio_base_addr1 + 3 * 4; // aka REG_LBA0
  pio_reg_addrs[ REG_CYL_LOW    ] = pio_base_addr1 + 4 * 4; // aka REG_LBA1
  pio_reg_addrs[ REG_CYL_HIGH   ] = pio_base_addr1 + 5 * 4; // aka REG_LBA2
  pio_reg_addrs[ REG_DEVICEHEAD ] = pio_base_addr1 + 6 * 4;
  pio_reg_addrs[ REG_COMMAND    ] = pio_base_addr1 + 7 * 4;
  pio_reg_addrs[ REG_CONTROL    ] = pio_base_addr2 + 6 * 4;
  pio_reg_addrs[ REG_DA         ] = pio_base_addr2 + 7 * 4;

  /*
   * "Shortcuts" for LBA48.
   * These are one byte above their LBA28 counterparts.
   */
  pio_reg_addrs[ REG_SECCOUNT1  ] = pio_reg_addrs[ REG_SECCOUNT0 ] + 1;
  pio_reg_addrs[ REG_LBA3       ] = pio_reg_addrs[ REG_LBA0      ] + 1;
  pio_reg_addrs[ REG_LBA4       ] = pio_reg_addrs[ REG_LBA1      ] + 1;
  pio_reg_addrs[ REG_LBA5       ] = pio_reg_addrs[ REG_LBA2      ] + 1;

  /*
   * Black magic
   */
  if( ipod->hw_ver > 3 ) {
    /* PP502x */
    outl(inl(0xc3000028) | 0x20, 0xc3000028);  // clear intr
    outl(inl(0xc3000028) & ~0x10000000, 0xc3000028); // reset?
    
    outl(0x10, 0xc3000000);
    outl(0x80002150, 0xc3000004);
  } else {
    /* PP5002 */
    outl(inl(0xc0003024) | 0x80, 0xc0003024);
    outl(inl(0xc0003024) & ~(1<<2), 0xc0003024);
    
    outl(0x10, 0xc0003000);
    outl(0x80002150, 0xc0003004);
  }

  /* 1st things first, check if there is an ATA controller here
   * We do this by writing values to two GP registers, and expect
   * to be able to read them back
   */
  pio_outbyte( REG_DEVICEHEAD, DEVICE_0 ); /* Device 0 */
  DELAY400NS;
  pio_outbyte( REG_SECT_COUNT, 0x55 );
  pio_outbyte( REG_SECT      , 0xAA );
  pio_outbyte( REG_SECT_COUNT, 0xAA );
  pio_outbyte( REG_SECT      , 0x55 );
  pio_outbyte( REG_SECT_COUNT, 0x55 );
  pio_outbyte( REG_SECT      , 0xAA );
  tmp[0] = pio_inbyte( REG_SECT_COUNT );
  tmp[1] = pio_inbyte( REG_SECT );
  if( (tmp[0] != 0x55) || (tmp[1] != 0xAA) ) return(1);

  /*
   * Okay, we're sure there's an ATA2 controller and device, so
   * lets set up the caching
   */
  cachedata  = (uint8 *)mlc_malloc(CACHE_NUMBLOCKS * CACHE_BLOCKSIZE);
  cacheaddr  = (uint32*)mlc_malloc(CACHE_NUMBLOCKS * sizeof(uint32));
  cachetick  = (uint32*)mlc_malloc(CACHE_NUMBLOCKS * sizeof(uint32));
  cacheticks = 0;
  
  for(i=0;i<CACHE_NUMBLOCKS;i++) {
    cachetick[i] =  0;  /* Time is zero */
    cacheaddr[i] = -1;  /* Invalid sector number */
  }

  return(0);
}

static void ata_clear_intr ()
{
  if( ipod_get_hwinfo()->hw_ver > 3 ) {
    outl(inl(0xc3000028) | 0x30, 0xc3000028); // this hopefully clears all pending intrs
  } else {
    outl(inl(0xc0003024) | 0x80, 0xc0003024);
  }
}

void ata_exit(void)
{
  ata_clear_intr ();
}


/*
 * Stops (spins down) the drive
 */
void ata_standby (int cmd_variation)
{
  uint8  status, cmd = COMMAND_STANDBY;
  // this is just a wild guess from "tempel" - I have no idea if this is the correct way to spin a disk down
  if (cmd_variation == 1) cmd = 0x94;
  if (cmd_variation == 2) cmd = 0x96;
  if (cmd_variation == 3) cmd = 0xE0;
  if (cmd_variation == 4) cmd = 0xE2;
  pio_outbyte( REG_COMMAND, cmd );
  DELAY400NS;
  while( pio_inbyte( REG_ALTSTATUS) & STATUS_BSY ); /* wait until drive is not busy */
  status = pio_inbyte( REG_STATUS );

  // The linux kernel notes mention that some drives might cause an interrupt when put to standby mode.
  // This interrupt is then to be ignored.
  ata_clear_intr ();
}


/*
 * Copies one block of data (512 or 1024 bytes) from the device
 * to host memory
 */
static void ata_transfer_block(void *ptr) {
  uint32  words;
  uint16 *dst;

  dst = (uint16*)ptr;

  if(drivetype == 1) { // 1024b sector reads
    words = 512;
  } else { // Default: 0 or other
    words = 256;
  }
  while(words--) {
    *dst++ = inw( pio_reg_addrs[REG_DATA] );
  }
}

/*
 * Detect what type of drive we are dealing with (512b-sectors default drive or
 * 1024b-sectors for 5.5G 80GB (unable to read odd sectors).
 * The variable drivetype is set to:
 * 0: 512b sector reads with COMMAND_READ_SECTORS_VRFY (default assumption)
 * 1: 2x 512b sector reads with COMMAND_READ_MULTIPLE when unable to read odd 
 *    sectors
 */
void ata_find_transfermode(void) {
  uint32 sector = 1; /* We need to read an odd sector */
  uint8 status;

  pio_outbyte( REG_DEVICEHEAD, (1<<6) | DEVICE_0 | ((sector & 0xF000000) >> 24) );
  DELAY400NS;
  pio_outbyte( REG_FEATURES  , 0 );
  pio_outbyte( REG_CONTROL   , CONTROL_NIEN | 0x08); /* 8 = HD15 */
  pio_outbyte( REG_SECT_COUNT, 1 );
  pio_outbyte( REG_SECT      ,  sector & 0xFF );
  pio_outbyte( REG_CYL_LOW   , (sector & 0xFF00) >> 8 );
  pio_outbyte( REG_CYL_HIGH  , (sector & 0xFF0000) >> 16 );

  pio_outbyte( REG_COMMAND, COMMAND_READ_SECTORS );
  DELAY400NS;  DELAY400NS;

  while( pio_inbyte( REG_ALTSTATUS) & STATUS_BSY ); /* Spin until drive is not busy */
  DELAY400NS;  DELAY400NS;

  status = pio_inbyte( REG_STATUS );
  if ((status & (STATUS_ERR)) == STATUS_ERR) {
    drivetype = 1;
    readcommand = COMMAND_READ_MULTIPLE;
    sectorcount = 2;
  } else {
    drivetype = 0;
    readcommand = COMMAND_READ_SECTORS;
    sectorcount = 1;
  }

#ifdef DEBUG
  mlc_printf("find_trans: dt=%d\n", drivetype);
#endif

}

/*
 * Does some extended identification of the ATA device
 */
void ata_identify(void) {
  uint8  status,c;
  uint16 *buff = (uint16*)mlc_malloc(512);

  pio_outbyte( REG_DEVICEHEAD, DEVICE_0 );
  pio_outbyte( REG_FEATURES  , 0 );
  pio_outbyte( REG_CONTROL   , CONTROL_NIEN );
  pio_outbyte( REG_SECT_COUNT, 0 );
  pio_outbyte( REG_SECT      , 0 );
  pio_outbyte( REG_CYL_LOW   , 0 );
  pio_outbyte( REG_CYL_HIGH  , 0 );

  pio_outbyte( REG_COMMAND, COMMAND_IDENTIFY_DEVICE );
  DELAY400NS;

  while( pio_inbyte( REG_ALTSTATUS) & STATUS_BSY ); /* Spin until drive is not busy */

  status = pio_inbyte( REG_STATUS );
  if( status & STATUS_DRQ ) {
    ata_transfer_block( buff );

    ATAdev.sectors = (buff[61] << 16) + buff[60];
    ATAdev.chs[0]  = buff[1];
    ATAdev.chs[1]  = buff[3];
    ATAdev.chs[2]  = buff[6];
    
    mlc_printf("ATA Device\n");
    mlc_printf("Size: %uMB (%u/%u/%u)\n",ATAdev.sectors/2048,ATAdev.chs[0],ATAdev.chs[1],ATAdev.chs[2]);

    mlc_printf("HDDid: ");
    for(c=27;c<47;c++) {
      if( buff[c] != ((' ' << 8) + ' ') ) {
        mlc_printf("%c%c", buff[c]>>8, buff[c]&0xFF);
      }
    }
    mlc_printf("\n");
  } else {
    mlc_printf("DRQ not set..\n");
  }

  /*
   * Now also detect the transfermode. It's done afterwards since ata_identify
   * expects to get 512 bytes instead of (possibly) 1024.
   */
  ata_find_transfermode();
}

/*
 * Sets up the transfer of one block of data
 */
static int ata_readblock2(void *dst, uint32 sector, int storeInCache) {
  uint8   status, i, cacheindex, secteven;

  /*
   * Static 1024 byte buffer.
   * Only use if drivetype == 1, otherwise buff will be NULL
   */
  static uint16 *buff = NULL;

  /*
   * If drivetype == 1, we need to do 1024 byte reads and then do work on it
   * before copying it to the dst buffer.
   * Allocate buff to help us do that.
   *
   * If drivetype == 0, buff is never used, so it's safe to leave it as NULL.
   */
  if ((buff == NULL) && (drivetype == 1)) buff = (uint16*)mlc_malloc(1024);

  if(drivetype == 0) {
    /* The sector is always even for drivetype 0 */
    secteven = 1;
  }
  else if (drivetype == 1) {
    if ((sector % 2) == 0) {
      secteven = 1;
    } else {
      secteven = 0;
      sector--;
    }
  }
  else {
    mlc_printf("Invalid drivetype %u\n", drivetype);
    mlc_show_fatal_error ();
    return(0);
  }

  /*
   * Check if we have this block in cache first
   */
  if (sector != 0) { /* Never EVER try to read sector 0 from cache, it won't be there or needed anyway */
    for(i=0;i<CACHE_NUMBLOCKS;i++) {
      if( cacheaddr[i] == sector ) {
        if (secteven) {
          mlc_memcpy(dst,cachedata + CACHE_BLOCKSIZE*i,512);  /* We did.. No need to bother the ATA controller */
        }
        else {
          mlc_memcpy(dst,cachedata + CACHE_BLOCKSIZE*i+512,512);
        }
        cacheticks++;
        cachetick[i] = cacheticks;
        return(0);
      }
    }
  }

  /*
   * Okay, it wasnt in cache.. We need to figure out which block
   * to replace in the cache.  Lets use a simple LRU
   */
  cacheindex = 0;
  if (storeInCache) {
    for(i=0;i<CACHE_NUMBLOCKS;i++) {
      if( cachetick[i] < cachetick[cacheindex] ) cacheindex = i;
    }
    cachetick[cacheindex] = cacheticks;
  }

  /* TODO: Set this based on number of LBAs discovered on device */
  uint8 useLBA48 = 1;

  if(!useLBA48 && (sector & 0xF0000000)) {
    /* The sector is too large for the current addressing scheme */
    mlc_printf("Sector %u is too large for LBA28 addressing.\n", sector);
    mlc_show_fatal_error ();
    return(0);
  }

  /*
  * REG_DEVICEHEAD bits are:
  *
  * | 1 |  2  | 3 |  4  | 5678 |
  * | 1 | LBA | 1 | DRV | HEAD |
  *
  * LBA = 0 for CHS addressing
  * LBA = 1 for logical block addressing
  *
  * DRV = 0 for master
  * DRV = 1 for slave
  *
  * Head = 0 for LBA 48
  * Head = lower nibble of top byte of sector, for LBA28
  */
  uint8 head = useLBA48 ? 0 : ((sector & 0x0F000000) >> 24);
  pio_outbyte( REG_DEVICEHEAD  , (1<<6) | DEVICE_0 | head );
  DELAY400NS;
  pio_outbyte( REG_FEATURES    , 0 );
  pio_outbyte( REG_CONTROL     , CONTROL_NIEN | 0x08); /* 8 = HD15 */

  if(!useLBA48) {
    pio_outbyte( REG_SECT_COUNT, (sectorcount & 0x000000FF) >> 0  );

    pio_outbyte( REG_SECT      , (sector      & 0x000000FF) >> 0  );
    pio_outbyte( REG_CYL_LOW   , (sector      & 0x0000FF00) >> 8  );
    pio_outbyte( REG_CYL_HIGH  , (sector      & 0x00FF0000) >> 16 );

    pio_outbyte( REG_COMMAND, readcommand );
  }
  else {
    pio_outbyte( REG_SECCOUNT1 , (sectorcount & 0x0000FF00) >> 8  );

    pio_outbyte( REG_LBA3      , (sector      & 0xFF000000) >> 24 );
    pio_outbyte( REG_LBA4      , 0 );
    pio_outbyte( REG_LBA5      , 0 );

    pio_outbyte( REG_SECCOUNT0 , (sectorcount & 0x000000FF) >> 0  );

    pio_outbyte( REG_LBA0      , (sector      & 0x000000FF) >> 0  );
    pio_outbyte( REG_LBA1      , (sector      & 0x0000FF00) >> 8  );
    pio_outbyte( REG_LBA2      , (sector      & 0x00FF0000) >> 16 );

    pio_outbyte( REG_COMMAND, COMMAND_READ_SECTORS_EXT );
  }

  DELAY400NS;  DELAY400NS;

  while( pio_inbyte( REG_ALTSTATUS) & STATUS_BSY ); /* Spin until drive is not busy */
  DELAY400NS;  DELAY400NS;

  status = pio_inbyte( REG_STATUS );
  if( (status & (STATUS_BSY | STATUS_DRQ)) == STATUS_DRQ) {
    if (storeInCache) {
      cacheaddr[cacheindex] = sector;
      ata_transfer_block(cachedata + cacheindex * CACHE_BLOCKSIZE);
      if (secteven) {
        mlc_memcpy(dst,cachedata + cacheindex*CACHE_BLOCKSIZE,512);
      }
      else {
        mlc_memcpy(dst,cachedata + cacheindex*CACHE_BLOCKSIZE+512, 512);
      }
      cacheticks++;
    }
    else {
      if (drivetype == 0) {
        ata_transfer_block(dst);
      }
      else {
        /* drivetype == 1 */
        ata_transfer_block(buff);

        if (secteven) {
          mlc_memcpy(dst,buff,512);
        } else {
          mlc_memcpy(dst,buff+256,512);
        }
      }
    }
  }
  else {
    mlc_printf("\nATA2 IO Error\n");
    status = pio_inbyte( REG_ERROR );
    mlc_printf("Error reg: %u\n",status);
    mlc_printf("dst: %lx, blk: %ld\n", dst, sector);
    mlc_show_fatal_error ();
  }

  return(0);
}

int ata_readblock(void *dst, uint32 sector) {
  return ata_readblock2(dst, sector, 1);
}

int ata_readblocks(void *dst,uint32 sector,uint32 count) {
  /* Replace this with COMMAND_READ_MULTIPLE for FAT32 speedups: */
  int err;
  while (count-- > 0) {
    err = ata_readblock2 (dst, sector++, 1);
    if (err) return err;
    dst = (char*)dst + 512;
  }
  return 0;
}

int ata_readblocks_uncached (void *dst, uint32 sector, uint32 count) {
  /* Replace this with COMMAND_READ_MULTIPLE for FAT32 speedups: */
  int err;
  while (count-- > 0) {
    err = ata_readblock2 (dst, sector++, 0);
    if (err) return err;
    dst = (char*)dst + 512;
  }
  return 0;
}

uint8 ata_get_drivetype (void) {
  return drivetype;
}
