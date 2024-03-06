/*
 * Basic ATA driver for the ipodlinux bootloader
 * 
 * Supports:
 *  PIO (Polling)
 *  Multiple block reads
 *  LBA48 reads
 *  Block caching
 * 
 *  See ATA-ATAPI-6 specification for operational details of how to talk to an ATA drive.
 *
 * Authors: James Jacobsson ( slowcoder@mac.com )
 *          Vincent Huisman ( dataghost@dataghost.com ) - 5.5 support (double sector reads) - 2007-01-23
 *          Ryan Crosby     ( ryan.crosby@live.com ) - LBA48 support and significant rewrites, documentation and comments - 2020-08-12 -> 2024-XX-XX
 *
 * 
 * In this code, "blocks" (blks) refers to fixed 512 byte units of data. The calling code requests data in units of block count.
 * Regardless of drive sector size, this code must return the expected number of 512 byte blocks, given the requested block count.
 * Luckily, all drives usually emulate 512 byte "logical" sectors, regardless of their physical sector size.
 * This means we don't have to do any translation of sector sizes internally, and for all intents and purposes, block size is equal to sector size.
 * 
 * However, some drives with > 512 byte physical sector sizes cannot read LBA numbers that aren't aligned to physical sector boundaries.
 * A notable example of this is the 80GB iPod 5.5G HDD, which has 1024kb physical sectors, and will error if you attempt to read odd sector sizes.
 * To overcome this, reads are always aligned and expanded to align and match the length of physical sectors. Any additional data read will be cached,
 * to reduce read amplification.
 * 
 */
#include "bootloader.h"
#include "console.h"
#include "ipodhw.h"
#include "minilibc.h"
#include "ata2.h"

/*
 * ATA controller registers
 */

/* Data register
 * The data register is 16-bits wide, read/write.
 *
 * This register shall be accessed for host PIO data transfer only when DRQ is set to one and DMACK- is not
 * asserted. The contents of this register are not valid while a device is in the Sleep mode.
 *
 * PIO out data transfers are processed by a series of reads to this register, each read transferring the data that
 * follows the previous read. PIO in data transfers are processed by a series of writes to this register, each write
 * transferring the data that follows the previous write. The results of a read during a PIO in or a write during a PIO
 * out are indeterminate
*/
#define REG_DATA       0x0
/* Error register
 * Read-only.
 * 
 * The contents of this register shall be valid when BSY and DRQ equal zero and ERR equals one. The contents
 * of this register shall be valid upon completion of power-on, or after a hardware or software reset, or after
 * command completion of an EXECUTE DEVICE DIAGNOSTICS or DEVICE RESET command. The contents of
 * this register are not valid while a device is in the Sleep mode.
 *
 * This register contains status for the current command.
 * Following a power-on, a hardware or software reset (see 9.1), or command completion of an EXECUTE DEVICE
 * DIAGNOSTIC (see 8.12) or DEVICE RESET command (see 8.10), this register contains a diagnostic code .
 * At command completion of any command except EXECUTE DEVICE DIAGNOSTIC, the contents of this
 * register are valid when the ERR bit is set to one in the Status register.
 */
#define REG_ERROR      0x1
/* Features register
 * Write-only.
 *
 * This register shall be written only when BSY and DRQ equal zero and DMACK- is not asserted. If this register
 * is written when BSY or DRQ is set to one, the result is indeterminate.
 * 
 * The content of this register becomes a command parameter when the Command register is written.
*/
#define REG_FEATURES   0x1
#define REG_SECT_COUNT 0x2 // LBA28
#define REG_SECT       0x3 // LBA28
#define REG_CYL_LOW    0x4 // LBA28
#define REG_CYL_HIGH   0x5 // LBA28
/*
 * Device register (DEV).
 * Read/write.
 * Used for device select, and LBA select + HEAD during read/write commands.
 */
#define REG_DEVICEHEAD 0x6 // LBA28
/* Status register
 * Read-only.
 *
 * The contents of this register, except for BSY, shall be ignored when BSY is set to one. BSY is valid at all
 * times. The contents of this register are not valid while a device is in the Sleep mode.
 *
 * This register contains the device status. The contents of this register are updated to reflect the current state of
 * the device and the progress of any command being executed by the device.
 * 
 * Reading this register when an interrupt is pending causes the interrupt pending to be cleared.
 * The host should not read the Status register when an interrupt is expected as this may clear the interrupt pending
 * before the INTRQ can be recognized by the host.
*/
#define REG_STATUS     0x7
#define REG_COMMAND    0x7
/* Device Control register
 * Write-only.
 *
 * This register shall only be written when DMACK- is not asserted.
 * 
 * This register allows a host to software reset attached devices and to enable or disable the assertion of the
 * INTRQ signal by a selected device. When the Device Control register is written, both devices respond to the
 * write regardless of which device is selected. When the SRST bit is set to one, both devices shall perform the
 * software reset protocol. The device shall respond to the SRST bit when in the SLEEP mode.
*/
#define REG_CONTROL    0x8
/*
 * Read-only register.
 * This register contains the same information as the Status Register in the
 * command block.  The only difference being that reading this register does not
 * imply interrupt acknowledge or clear a pending interrupt.
 */
#define REG_ALTSTATUS  0x8

/*
 * LBA48 specific registers.
 */
#define REG_SECCOUNT_LOW  0x2 // Same as LBA28 REG_SECT_COUNT.
#define REG_LBA0          0x3 // Same as LBA28 REG_SECT.
#define REG_LBA1          0x4 // Same as LBA28 REG_CYL_LOW.
#define REG_LBA2          0x5 // Same as LBA28 REG_CYL_HIGH.
#define REG_SECCOUNT_HIGH 0xA
#define REG_LBA3          0xB
#define REG_LBA4          0xC
#define REG_LBA5          0xD

#define REG_DA          0x9

/* nIEN: Negated Interrupt Enable bit in Device Control register.
 * Sets device Assertion of INTRQ to the host.
 * When the nIEN bit is cleared to zero, and the device is selected, INTRQ shall be enabled
 * When the nIEN bit is set to one, or the device is not selected, the INTRQ signal is disabled
 */
#define CONTROL_NIEN    0x2
/* SRST: Software Reset bit in Device Control register */
#define CONTROL_SRST    0x4

// all commands: see include/linux/hdreg.h

/* IDENTIFY DEVICE (Identify)
 *
 */
#define COMMAND_IDENTIFY_DEVICE       0xEC

/* READ SECTOR(S) (RdSec) - 20h, PIO Data-In. LBA28.
 *
 * This command reads from 1 to 256 sectors as specified in the Sector Count register. A sector count of 0
 * requests 256 sectors. The transfer shall begin at the sector specified in the LBA Low, LBA Mid, LBA High, and
 * Device registers.
 * The DRQ bit is always set to one prior to data transfer regardless of the presence or absence of an error
 * condition.
 */
#define COMMAND_READ_SECTORS          0x20

/* READ SECTOR(S) EXT (RdSecEx) - 24h, PIO Data-In. LBA48.
 *
 * This command reads from 1 to 65,536 sectors as specified in the Sector Count register. A sector count of
 * 0000h requests 65,536 sectors. The transfer shall begin at the sector specified in the LBA Low, LBA Mid, and
 * LBA High registers.
 * The DRQ bit is always set to one prior to data transfer regardless of the presence or absence of an error
 * condition.
 */
#define COMMAND_READ_SECTORS_EXT      0x24

/* STANDBY IMMEDIATE (StandbyIm)
 *
 * This command causes the device to immediately enter the Standby mode.
*/
#define COMMAND_STANDBY               0xE0

/* SLEEP E6h
 *
 * This command is the only way to cause the device to enter Sleep mode.
 * 
 * This command causes the device to set the BSY bit to one, prepare to enter Sleep mode, clear the BSY bit to
 * zero and assert INTRQ. The host shall read the Status register in order to clear the interrupt pending and allow
 * the device to enter Sleep mode. In Sleep mode, the device only responds to the assertion of the RESET- signal
 * and the writing of the SRST bit in the Device Control register and releases the device driven signal lines. The
 * host shall not attempt to access the Command Block registers while the device is in Sleep mode.
 * 
 * Because some host systems may not read the Status register and clear the interrupt pending, a device may
 * automatically release INTRQ and enter Sleep mode after a vendor specific time period of not less than 2 s.
 * 
 * The only way to recover from Sleep mode is with a software reset, a hardware reset, or a DEVICE RESET
 * command.
 * 
 * A device shall not power-on in Sleep mode nor remain in Sleep mode following a reset sequence
*/
#define COMMAND_SLEEP                 0xE6

#define DEVICE_0       0x00
#define DEVICE_1       0x10

#define CHS_ADDRESSING 0x00
#define LBA_ADDRESSING 0x40

/* BSY (Busy)
 *
 * BSY indicates that the device is handling a command.
 * 
 * In practice, what this means is that the device still has control
 * over the Command Block registers.
 * The host should not write to the Command Block registers while BSY is asserted,
 * with the exception of sending a DEVICE RESET command.
 * 
 * While BSY is not asserted, the device will never set DRQ, or change ERR,
 * or change any other Command Block register.
 * The device will always assert BSY first, then set DRQ, ERR, or other Command Block registers,
 * and then deassert BSY again.
*/
#define STATUS_BSY     0x80
#define STATUS_DRDY    0x40
#define STATUS_DF      0x20
#define STATUS_DSC     0x10
/* DRQ (Data request)
 *
 * DRQ indicates that the device is ready to transfer a word of data between the host and the device. After the
 * host has written the Command register the device shall either set the BSY bit to one or the DRQ bit to one,
 * until command completion or the device has performed a bus release for an overlapped command.
*/
#define STATUS_DRQ     0x08
#define STATUS_CORR    0x04
#define STATUS_IDX     0x02
#define STATUS_ERR     0x01

/* ABRT (command aborted) is set to one to indicate the requested command has been command
 * aborted because the command code or a command parameter is invalid, the command is not
 * supported, a prerequisite for the command has not been met, or some other error has occurred
 */
#define ERROR_ABRT 0x04
/* There are 8 total error bits that can be set, but besides ABRT, they are all command dependant.*/

unsigned int pio_base_addr1,pio_base_addr2;
unsigned int pio_reg_addrs[14];

/* 
 * 8K of cache divided into 16 x 512 byte blocks.
 * When doing >512 byte reads, the drive will overwrite multiple blocks of cache,
 * and then the cache lookup table will be updated to reflect this.
*/
#define CACHE_NUMBLOCKS 16
static uint8  *cachedata;
static uint32 *cacheaddr;
static uint32 *cachetick;
static uint32  cacheticks;

/*
 * Drive configuration
*/
/* Logical blocks are always 512 bytes */
#define BLOCK_SIZE 512
/* Sectors are 512 bytes. Therefore there are 2 sectors per KB, and 2048 sectors per MB.*/
#define BLOCKS_PER_MB 2048
/*
 * The number of 512 byte logical blocks that fit within a physical
 * on-disk sector of the device.
 * Reads must be aligned to physical sectors, so this is critical.
 */
static uint8 blks_per_phys_sector = 1;
/* Drive supports LBA 48? */
static uint8 drive_lba48 = 0;

static struct {
  uint16 chs[3];
  uint32 sectors;
} ATAdev;

/* Forward declaration of static functions (not exported via header file) */
static inline void spinwait_drive_busy(void);
static inline void bug_on_ata_error(void);
static void ata_clear_intr(void);
static void ata_find_transfermode(void);
static inline void clear_cache(void);
static inline int create_cache_entry(uint32 sector);
static inline int find_cache_entry(uint32 sector);
static inline inline void *get_cache_entry_buffer(int cacheindex);
static void ata_send_read_command(uint32 lba, uint32 count);
static uint32 ata_transfer_block(void *ptr, uint32 count);
static uint32 ata_receive_read_data(void *dst, uint32 count);
static int ata_readblock2(void *dst, uint32 sector, int useCache);


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
  uint8   tmp[2];
  ipod_t *ipod;

  ipod = ipod_get_hwinfo();

  pio_base_addr1 = ipod->ide_base;
  pio_base_addr2 = pio_base_addr1 + 0x200;

  /*
   * Set up lookup table of ATA register addresses, for use with the pio_ macros
   * Note: The PP chips have their IO registers 4 byte aligned
   */
  pio_reg_addrs[ REG_DATA       ] = pio_base_addr1 + 0 * 4;
  pio_reg_addrs[ REG_FEATURES   ] = pio_base_addr1 + 1 * 4;
  pio_reg_addrs[ REG_SECT_COUNT ] = pio_base_addr1 + 2 * 4; // REG_SECT_COUNT = REG_SECCOUNT_LOW
  pio_reg_addrs[ REG_SECT       ] = pio_base_addr1 + 3 * 4; // REG_SECT       = REG_LBA0
  pio_reg_addrs[ REG_CYL_LOW    ] = pio_base_addr1 + 4 * 4; // REG_CYL_LOW    = REG_LBA1
  pio_reg_addrs[ REG_CYL_HIGH   ] = pio_base_addr1 + 5 * 4; // REG_CYL_HIGH   = REG_LBA2
  pio_reg_addrs[ REG_DEVICEHEAD ] = pio_base_addr1 + 6 * 4;
  pio_reg_addrs[ REG_COMMAND    ] = pio_base_addr1 + 7 * 4;
  pio_reg_addrs[ REG_CONTROL    ] = pio_base_addr2 + 6 * 4;
  pio_reg_addrs[ REG_DA         ] = pio_base_addr2 + 7 * 4;

  /*
   * Registers for LBA48.
   * These are one byte address above their LBA28 counterparts.
   */
  pio_reg_addrs[ REG_SECCOUNT_HIGH  ] = pio_reg_addrs[ REG_SECCOUNT_LOW ] + 1;
  pio_reg_addrs[ REG_LBA3           ] = pio_reg_addrs[ REG_LBA0         ] + 1;
  pio_reg_addrs[ REG_LBA4           ] = pio_reg_addrs[ REG_LBA1         ] + 1;
  pio_reg_addrs[ REG_LBA5           ] = pio_reg_addrs[ REG_LBA2         ] + 1;

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
  pio_outbyte( REG_DEVICEHEAD, 0xA0 | DEVICE_0 ); /* Device 0 */
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

  // cachedata holds the actual data read from the device, in CACHE_BLOCKSIZE byte blocks.
  cachedata  = (uint8 *)mlc_malloc(CACHE_NUMBLOCKS * BLOCK_SIZE);
  // cacheaddr maps each index of the cachedata array to its sector number
  cacheaddr  = (uint32*)mlc_malloc(CACHE_NUMBLOCKS * sizeof(uint32));
  // cachetick maps each index of the cachedata array to its age, for finding LRU
  cachetick  = (uint32*)mlc_malloc(CACHE_NUMBLOCKS * sizeof(uint32));
  
  clear_cache();

  return(0);
}

static void ata_clear_intr(void)
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
 * Spinwait until the drive is not busy
*/
static inline void spinwait_drive_busy(void) {
  /* Force this busyloop to not be optimised away */
   while( pio_inbyte( REG_ALTSTATUS) & STATUS_BSY ) __asm__ __volatile__("");
}

/*
 * Checks for ATA error, and upon error prints the error and fatal bugchecks
*/
static inline void bug_on_ata_error(void) {
  uint8 status = pio_inbyte( REG_STATUS );
  if(status & STATUS_ERR) {
    uint8 error = pio_inbyte( REG_ERROR );
    mlc_printf("\nATA2 IO Error\n");
    mlc_printf("STATUS: %02hhX\n", status);
    mlc_printf("ERROR: %02hhX\n", error);
    // mlc_printf("dst: %lx, blk: %ld\n", dst, sector);
    mlc_show_fatal_error ();
    return;
  }
}


/*
 * Stops (spins down) the drive
 */
void ata_standby(int cmd_variation)
{
  uint8 cmd = COMMAND_STANDBY;
  // this is just a wild guess from "tempel" - I have no idea if this is the correct way to spin a disk down
  if (cmd_variation == 1) cmd = 0x94;
  if (cmd_variation == 2) cmd = 0x96;
  if (cmd_variation == 3) cmd = 0xE0;
  if (cmd_variation == 4) cmd = 0xE2;
  pio_outbyte( REG_COMMAND, cmd );
  DELAY400NS;

  /* Wait until drive is not busy */
  spinwait_drive_busy(); 

  /* Read the status register to clear any pending interrupts */
  pio_inbyte( REG_STATUS );

  // The linux kernel notes mention that some drives might cause an interrupt when put to standby mode.
  // This interrupt is then to be ignored.
  ata_clear_intr();
}

/*
 * Detect what type of drive we are dealing with and set blks_per_phys_sector appropriately.
 * The handled cases are:
 * - 512 byte physical sectors. This is the majority of drives (usually they're 512 physical, or 4K physical with 512e emulation)
 * - 2048 byte physical sectors, 512 byte logical sectors. The only known drive is the iPod 5.5G 80GB hard drive.
 *
 * The 80GB HDD still returns 512 byte logical sectors for each LBA, but
 * disallows reading from unaligned LBAs, so reading odd numbered LBAs will fail.
 * In this case, two 512 byte sectors must be read at once, from a 2048 byte physical sector aligned boundary.
 * Basically, round the LBA down to the nearest even number and then read two blocks, always.
 */
static void ata_find_transfermode(void) {
  uint32 bytesread;

  /* 
   * Read even sector, this should always work.
   * The count of bytes returned will reveal the logical sector size.
   */
  ata_send_read_command(0, 1);
  bytesread = ata_transfer_block(NULL, 64);
  spinwait_drive_busy();
  bug_on_ata_error();
  mlc_printf("Logical sector size: %lu\n", bytesread);

  if(bytesread != BLOCK_SIZE) {
    mlc_printf("Unexpected logical sector size: %lu\n", bytesread);
    mlc_show_fatal_error ();
  }

  /* 
   * Read odd sector, this is expected to fail on the 80GB iPod 5.5G HDD.
   */
  ata_send_read_command(1, 1);
  bytesread = ata_transfer_block(NULL, 1);
  spinwait_drive_busy();
  uint8 status = pio_inbyte( REG_STATUS );
  if(status & STATUS_ERR) {
    uint8 error = pio_inbyte( REG_ERROR );
#ifdef DEBUG
    mlc_printf("Error on odd sector read! 80GB 5.5G?");
    mlc_printf("STATUS: %02hhX\n", status);
    mlc_printf("ERROR: %02hhX\n", error);
#endif
    blks_per_phys_sector = 4;
  }
  else {
    blks_per_phys_sector = 1;
  }
  
  mlc_printf("Physical sector size: %u\n", blks_per_phys_sector * BLOCK_SIZE);
}

/*
 * Does some extended identification of the ATA device
 */
void ata_identify(void) {
  uint8  status,c;
  uint16 *buff = (uint16*)mlc_malloc(512); // TODO: Remove unnecessary allocation

  pio_outbyte( REG_DEVICEHEAD, 0xA0 | DEVICE_0 );
  pio_outbyte( REG_FEATURES  , 0 );
  pio_outbyte( REG_CONTROL   , CONTROL_NIEN );
  pio_outbyte( REG_SECT_COUNT, 0 );
  pio_outbyte( REG_SECT      , 0 );
  pio_outbyte( REG_CYL_LOW   , 0 );
  pio_outbyte( REG_CYL_HIGH  , 0 );

  pio_outbyte( REG_COMMAND, COMMAND_IDENTIFY_DEVICE );
  DELAY400NS;

  /* Spin until drive is not busy */
  spinwait_drive_busy();

  status = pio_inbyte( REG_STATUS );
  if( status & STATUS_DRQ ) {
    ata_transfer_block( buff, 1 );

    /*
     * Words 60..61 contain a value that is one greater than the maximum user addressable LBA.
     * The maximum value that shall be placed in this field is 0FFF_FFFFh.
     * If words 60..61 contain 0FFF_FFFFh and the device has user addressable LBAs greater than or equal to 0FFF_FFFFh,
     * then the ACCESSIBLECAPACITY field (see 9.11.4.2) contains the total number of user addressable LBAs (see 4.1).
     */
    ATAdev.sectors = (buff[61] << 16) + buff[60];

    if(ATAdev.sectors == 0x0FFFFFFF) {
        /* Enable LBA48 mode, since the drive has more than the max LBAs of LBA28 */
         drive_lba48 = 1;

        /*
         * TODO: Implement the log code described below.
         *
         * TODO: Maybe we should always try to read these logs (instead of just on large LBA drives)
         * because they also tell us LOGICAL SECTOR SIZE. Doing this would let us skip the IDENTIFY DEVICE
         * and only use it as a fallback.
         */

        /*
         * To get the full details of the device, including the sector count, physical sector size,
         * and other capabilities, we need to use the General Purpose Logging (GPL) feature set.
         *
         * Reference: http://www.t13.org/Documents/UploadedDocuments/docs2016/di529r14-ATAATAPI_Command_Set_-_4.pdf
         *
         * To read a log, use 7.22 READ LOG EXT - 2Fh, PIO Data-In
         * This takes a LOG ADDRESS, PAGE NUMBER, and PAGE COUNT.
         *
         * 1. Read Page 00h of the 9.11 IDENTIFY DEVICE data log (Log Address 30h)
         *
         * 2. Check page 00h 9.11.2 List of Supported IDENTIFY DEVICE data log pages (Page 00h)
         *    to see if Page 02h,  9.11.4 Capacity (page 02) is supported.
         *
         * 3. If supported, read 9.11.4 Capacity (page 02) page
         *
         * 4. Extract: ACCESSIBLE CAPACITY field (see 9.11.4.2)
         *             LOGICAL SECTOR SIZE SUPPORTED bit, and if set,
         *             LOGICAL SECTOR SIZE
         *
         * Note: I think LOGICAL SECTOR SIZE will remove the need to do the ata_find_transfermode(),
         *       since it tells us the physical sector alignment we need to adhere to.
         *       We can also just assume the COMMAND_READ_SECTORS_EXT command and use a sector count > 1.
         *       This will allow us to generalize the 5.5g 80GB read solution to work for all drives.
         */
    }

    ATAdev.chs[0]  = buff[1];
    ATAdev.chs[1]  = buff[3];
    ATAdev.chs[2]  = buff[6];

    mlc_printf("ATA Device\n");

    if(drive_lba48) {
      /* Until we implement reading ACCESSIBLE CAPACITY, the best we can do is say this drive is >128GB */
      mlc_printf("Size: >%uMB (%u/%u/%u)\n", ATAdev.sectors/BLOCKS_PER_MB, ATAdev.chs[0], ATAdev.chs[1], ATAdev.chs[2]);
      mlc_printf("LBA48 addressing\n");
    }
    else {
      mlc_printf("Size: %uMB (%u/%u/%u)\n", ATAdev.sectors/BLOCKS_PER_MB, ATAdev.chs[0], ATAdev.chs[1], ATAdev.chs[2]);
      mlc_printf("LBA28 addressing\n");
    }

    mlc_printf("HDDid: ");
    for(c=27;c<47;c++) {
      if( buff[c] != ((' ' << 8) + ' ') ) {
        mlc_printf("%c%c", buff[c]>>8, buff[c]&0xFF);
      }
    }
    mlc_printf("\n");
  }
  else {
    mlc_printf("DRQ not set..\n");
  }

  /*
   * Find drive parameters (physical and logical sector sizes).
   * TODO( ryan.crosby@live.com ): Replace this entirely with the ATA log code as described above.
   * We should be able to find both physical sector size and logical sector size without kludges.
   */
  ata_find_transfermode();
}

/*
 * lba:       The Logical Block Adddress to begin reading blocks from.
 * count:     The number of logical blocks to read.
*/
// TODO: Propagate the errors up as return codes using constants,
//       instead of throwing fatal error internally.
static void ata_send_read_command(uint32 lba, uint32 count) {
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
  uint8 head = drive_lba48 ? 0 : ((lba & 0x0F000000) >> 24);
  pio_outbyte( REG_DEVICEHEAD  , 0xA0 | LBA_ADDRESSING | DEVICE_0 | head );
  DELAY400NS;
  pio_outbyte( REG_FEATURES    , 0 );
  pio_outbyte( REG_CONTROL     , CONTROL_NIEN | 0x08); /* 8 = HD15 */

  if(drive_lba48) {
    /*
    * IMPORTANT: The ATA controller is sensitive to the order in which registers are written.
    *            For LBA48, we MUST write the high registers first, before we write the low registers.
    *            It doesn't work if the lower registers are written first.
    */

    /* Write the high bytes of the registers */
    pio_outbyte( REG_SECCOUNT_HIGH , (count & 0x0000FF00) >> 8  );
    pio_outbyte( REG_LBA3          , (lba   & 0xFF000000) >> 24 );
    pio_outbyte( REG_LBA4          , 0 );
    pio_outbyte( REG_LBA5          , 0 );
  }

  /* Write the low bytes of the registers */
  pio_outbyte( REG_SECCOUNT_LOW , (count & 0x000000FF) >> 0  );
  pio_outbyte( REG_LBA0         , (lba   & 0x000000FF) >> 0  );
  pio_outbyte( REG_LBA1         , (lba   & 0x0000FF00) >> 8  );
  pio_outbyte( REG_LBA2         , (lba   & 0x00FF0000) >> 16 );

  uint8 readcommand;
  if (drive_lba48) {
    readcommand = COMMAND_READ_SECTORS_EXT;
  }
  else {
    readcommand = COMMAND_READ_SECTORS;
  }

  /* Send read command */
  pio_outbyte( REG_COMMAND, readcommand );

  DELAY400NS;  DELAY400NS;
  /* Spinwait until drive is not busy */
  spinwait_drive_busy();
  DELAY400NS;  DELAY400NS;
}

/*
 * Copies blocks of data (512 bytes each) from the device
 * to host memory.
 * 
 * *ptr: Destination buffer. If NULL, data will be read from the device and discarded.
 * count: The number of 512 byte blocks to read from the device into the buffer
 * return: The number of bytes actually read from the device
 */
static uint32 ata_transfer_block(void *ptr, uint32 count) {
  // Data is read in as 16 bit words, so 2 bytes at a time.
  uint32 words = (BLOCK_SIZE / 2) * count;
  uint32 words_received = 0;

  if(ptr != NULL) {
    uint16 *dst = (uint16*)ptr;
    while(words--) {
      /* Wait until drive is not busy */
      spinwait_drive_busy();

      /* Check DRQ to see if there's more data to read, or if an error has occured */
      if((pio_inbyte(REG_STATUS) & (STATUS_ERR | STATUS_DRQ)) != STATUS_DRQ) {
        break;
      }

      /* Read another 16 bits of data into buffer */
      *dst++ = inw( pio_reg_addrs[REG_DATA] );
      ++words_received;
    }
  }
  else {
    while(words--) {
      /* Wait until drive is not busy */
      spinwait_drive_busy();
      
      /* Check DRQ to see if there's more data to read, or if an error has occured */
      if((pio_inbyte(REG_STATUS) & (STATUS_ERR | STATUS_DRQ)) != STATUS_DRQ) {
        break;
      }

      /* Read another 16 bits of data and discard it */
      inw( pio_reg_addrs[REG_DATA] );
      ++words_received;
    }
  }

  return words_received * 2;
}

/*
 * Receive data back from the device after a read out command has been issued.
 *
 * *dst: Destination buffer. If NULL, data will be read from the device and discarded.
 * count: The number of 512 byte blocks to read from the device into the buffer
*/
static uint32 ata_receive_read_data(void *dst, uint32 count) {
  uint32 bytesread;
  bytesread = ata_transfer_block(dst, count);

  /* Wait for any final busy state to clear */
  spinwait_drive_busy();

  /* Check if reading ended on an error */
  bug_on_ata_error();

  /* Verify we read the expected number of bytes */
  if(bytesread != count * BLOCK_SIZE) {
    /* We read an unexpected number of bytes from the device */
    mlc_printf("\nATA2 IO Error\n");
    mlc_printf("\nUnexpected number of bytes received.\n");
  
    mlc_printf("Expected: %lu, Actual: %lu\n", count * BLOCK_SIZE, bytesread);
    mlc_show_fatal_error();
    return bytesread;
  }

  return bytesread;
}

static inline void clear_cache(void) {
  int i;

  cacheticks = 0;

  for(i = 0; i < CACHE_NUMBLOCKS; i++) {
    cachetick[i] =  0;  /* Time is zero */
    cacheaddr[i] = ~0;  /* Invalid sector number */
  }
}

/* 
 * Creates a cache entry for a given sector, and returns the index of the cache buffer
 * that was created.
*/
static inline int create_cache_entry(uint32 sector) {
  int cacheindex;
  int i;

  cacheindex = find_cache_entry(sector);

  if(cacheindex < 0) {
    cacheindex = 0;
    for(i = 0; i < CACHE_NUMBLOCKS; i++) {
      if( cachetick[i] < cachetick[cacheindex] ) {
        cacheindex = i;
      }
    }
  }

  cacheaddr[cacheindex] = sector;
  cachetick[cacheindex] = cacheticks;

  return(cacheindex);
}

static inline int find_cache_entry(uint32 sector) {
  if(sector == ~0) {
    return(-1);
  }

  for(int i = 0; i < CACHE_NUMBLOCKS; i++) {
    if( cacheaddr[i] == sector ) {
      /* cacheticks is incremented every time the cache is hit */
      cachetick[i] = ++cacheticks;
      return(i);
    }
  }

  return(-1);
}

static inline void *get_cache_entry_buffer(int cacheindex) {
    if(cacheindex >= 0 && cacheindex < CACHE_NUMBLOCKS) {
      return(cachedata + (BLOCK_SIZE * cacheindex));
    }
    else {
      mlc_printf(
      "Invalid cache index!\n"
      "Index %d is out of bounds.\n"
      , cacheindex);

      mlc_show_fatal_error();
      return NULL;
    }
}

/*
 * Sets up the transfer of one block of data
 */
static int ata_readblock2(void *dst, uint32 sector, int useCache) {
  /*
   * Check if we have this block in cache first
   */
  if (useCache) {
    int cacheindex = find_cache_entry(sector);
    if( cacheindex >= 0 ) {
        /* In cache! No need to bother the ATA controller */
      void *cachedsrc = get_cache_entry_buffer(cacheindex);
      mlc_memcpy(dst, cachedsrc, BLOCK_SIZE);
      return(0);
    }
  }

  if(!drive_lba48 && (sector > 0x0FFFFFFF)) {
    /* The sector is too large for the current addressing scheme */
    mlc_printf(
      "Out of bounds read!\n"
      "Sector %u is too large for LBA28 addressing.\n"
      , sector);
    mlc_show_fatal_error ();
    return(0);
  }

  /* Calculate an aligned LBA for the specified sector */
  uint32 read_size;
  uint32 sector_to_read;
  if(useCache && (blks_per_phys_sector < 4)) {
      /* Optimization: If we're caching, bump read size up to 2K for speedups */
      read_size = 4;
      sector_to_read = sector & ~0x03;
  }
  else {
    read_size = blks_per_phys_sector;
    sector_to_read = sector - (sector % read_size);
  }

  /* Send the read command to the device*/
  ata_send_read_command(sector_to_read, read_size);

  if (useCache) {
    /*
      * In cached mode, store every 512 byte block we read into the cache,
      * and then copy the requested sector out to dst
    */
    for(int i = sector_to_read; i < (sector_to_read + read_size); i++) {
      int cacheindex = create_cache_entry(i);
      void *cachedst = get_cache_entry_buffer(cacheindex);

      /* Read data directly into the cache*/
      int bytesread = ata_receive_read_data(cachedst, 1);

      if(i == sector) {
        /* This is the sector that was actually requested, copy it out of the cache block into the destination */
        mlc_memcpy(dst, cachedst, bytesread);
      }
    }
    cacheticks++;
  }
  else {
    /*
      * In non-cached mode, discard the sectors we read unless
      * they were the requested sector.
    */
    for(int i = sector_to_read; i < (sector_to_read + read_size); i++) {
      if(i == sector) {
        /* This is the sector that was actually requested, read data directly into the destination */
        ata_receive_read_data(dst, 1);
      }
      else {
        /* Discard data we can't use */
        ata_receive_read_data(NULL, 1);
      }
    }
  }

  return(0);
}

int ata_readblock(void *dst, uint32 sector) {
  return ata_readblock2(dst, sector, 1);
}

int ata_readblocks(void *dst, uint32 sector, uint32 count) {
  int err;
  while (count-- > 0) {
    err = ata_readblock2 (dst, sector++, 1);
    if (err) return err;
    dst = (char*)dst + BLOCK_SIZE;
  }
  return 0;
}

int ata_readblocks_uncached (void *dst, uint32 sector, uint32 count) {
  int err;
  while (count-- > 0) {
    err = ata_readblock2 (dst, sector++, 0);
    if (err) return err;
    dst = (char*)dst + 512;
  }
  return 0;
}

uint8 ata_get_blks_per_phys_sector(void) {
  return blks_per_phys_sector;
}
