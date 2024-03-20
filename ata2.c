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
#include "ata2_definitions.h"

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

/* These track the last command sent, so that if an error occurs the details can be printed. */
static uint8 last_command = 0;
static uint32 last_sector = 0;
static uint16 last_sector_count = 0;

/*
 * Drive configuration
 */
static struct {
  /* Drive CHS geometry */
  uint16 chs[3];

  /* Non-zero if LBA48 is supported */
  uint8 lba48;

 /*
 * The log2 of the number of 512 byte logical blocks that fit within a physical
 * on-disk sector of the device.
 */
  uint8 alignment_log2;

  /* The total number of sectors the device has */
  uint64 sectors;
} ATAdev;

/* Forward declaration of static functions (not exported via header file) */
static inline void spinwait_drive_busy(void);
static inline void bug_on_ata_error(void);
static void ata_clear_intr(void);
static inline void clear_cache(void);
static inline int create_cache_entry(uint32 sector);
static inline int find_cache_entry(uint32 sector);
static inline inline void *get_cache_entry_buffer(int cacheindex);
static void ata_send_read_command(uint32 lba, uint16 count);
static uint32 ata_transfer_block(void *ptr, uint32 count);
static uint32 ata_receive_read_data(void *dst, uint32 count);
static int ata_readblock2(void *dst, uint32 sector, int useCache);


inline static void pio_outbyte(unsigned int addr, unsigned char data) {
  outb( data, pio_reg_addrs[ addr ] );
}

inline static void pio_outword(unsigned int addr, unsigned int data) {
  outl( data, pio_reg_addrs[ addr ] );
}

inline static volatile unsigned char pio_inbyte( unsigned int addr ) {
  return( inl( pio_reg_addrs[ addr ] ) );
}

inline static volatile unsigned short pio_inword( unsigned int addr ) {
  return( inl( pio_reg_addrs[ addr ] ) );
}

inline static volatile unsigned int pio_indword( unsigned int addr ) {
  return( inl( pio_reg_addrs[ addr ] ) );
}

inline static void ata_command(uint8 cmd) {
  last_command = cmd;
  pio_outbyte( REG_COMMAND, cmd );
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

  if( (pio_inbyte( REG_SECT_COUNT ) != 0x55)
    || (pio_inbyte( REG_SECT ) != 0xAA) )
    {
      return(1);
    }

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
  
  /* Initialize cache */
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
    mlc_printf("STATUS: %02hhX, ", status);
    mlc_printf("ERROR: %02hhX\n", error);
    mlc_printf("LAST COMMAND: %02hhX\n", last_command);
    if(last_command == COMMAND_READ_SECTORS
      || last_command == COMMAND_READ_SECTORS_EXT) {
      mlc_printf("SECTOR: %d, ", last_sector);
      mlc_printf("COUNT: %d\n", last_sector_count);
    }

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
  ata_command(cmd);
  DELAY400NS;

  /* Wait until drive is not busy */
  spinwait_drive_busy(); 

  /* Read the status register to clear any pending interrupts */
  pio_inbyte( REG_STATUS );

  /*
   * The linux kernel notes mention that some drives might cause an interrupt when put to standby mode.
   * This interrupt is then to be ignored.
   */
  ata_clear_intr();
}

void ata_sleep() {
  ata_command( COMMAND_SLEEP );
  DELAY400NS; DELAY400NS;
  spinwait_drive_busy();
  DELAY400NS; DELAY400NS;
  /*
   * When the device is ready to ender sleep mode, it will set an interrupt and wait.
   * It will then wait until we clear that interrupt by reading the STATUS register
   * to actually enter sleep mode.
   */
  pio_inbyte( REG_STATUS );

  /* The device should now be asleep, and will not respond until a DEVICE_RESET command is sent. */
}

/*
 * Print fixed size uint16 big-endian ASCII string.
*/
static void print_str_be16(uint16 *buff, size_t length) {
  /* Walk backwards from end of string to trim whitespace */
  while((length > 0) && (buff[--length] == ((' ' << 8) + ' ')));

  /* Print each word as big endian (this is why we can't just reinterpret buff as a uint8*) */
  for(int i = 0; i < length; i++) {
      mlc_printf("%c%c", buff[i] >> 8, buff[i] & 0xFF);
  }
}

/*
 * Compares a standard C string to a uint16 big-endian ASCII string.
 *
 * str1: First string as a standard C string.
 * str1_start: The offset in characters into which str1 should start to be compared.
 * str2: Second string as a uint16 big-endian ASCII string.
 * str2_start: The offset in characters into which str2 should start to be compared.
 * length: The length in characters which to compare str1 and str2.
 */
static int strncmp_be16(char* str1, size_t str1_start, uint16* str2, size_t str2_start, size_t length) {
  int result = 0;

  while(length--) {
    char lc = str1[str1_start];
    char rc = (str2_start & 1)
        ? str2[str2_start / 2] & 0xFF /* Right character of uint16 */
        : str2[str2_start / 2] >> 8   /* Left character of uint16 */
        ;
    
    result = lc - rc;
    if(result != 0 || lc == '\0' || rc == '\0') break;

    ++str1_start;
    ++str2_start;
  }

  return result;
}

/*
 * Does some extended identification of the ATA device
 */
void ata_identify(void) {
  uint16 *buff = (uint16*)mlc_malloc(sizeof(uint16) * 256); // TODO: Remove unnecessary allocation

  pio_outbyte( REG_DEVICEHEAD, 0xA0 | DEVICE_0 );
  pio_outbyte( REG_FEATURES  , 0 );
  pio_outbyte( REG_CONTROL   , CONTROL_NIEN );
  pio_outbyte( REG_SECT_COUNT, 0 );
  pio_outbyte( REG_SECT      , 0 );
  pio_outbyte( REG_CYL_LOW   , 0 );
  pio_outbyte( REG_CYL_HIGH  , 0 );

  ata_command( COMMAND_IDENTIFY_DEVICE );
  DELAY400NS;

  ata_receive_read_data(buff, 1);

  /*
  * Verify the IDENTIFY DEVICE response integrity
  *
  * The use of this word is optional. If bits 7:0 of this word contain the signature A5h, bits 15:8 contain the data
  * structure checksum. The data structure checksum is the two’s complement of the sum of all bytes in words 0
  * through 254 and the byte consisting of bits 7:0 in word 255. Each byte shall be added with unsigned arithmetic,
  * and overflow shall be ignored.
  * The sum of all 512 bytes is zero when the checksum is correct.
  */

  if((buff[255] & 0x00FF) == 0xA5) {
    uint8 calculated_sum = 0;
    for(int i = 0; i < 256; i++) {
      calculated_sum += buff[i] & 0x00FF;
      calculated_sum += buff[i] >> 8;
    }

    if(calculated_sum != 0) {
      /* Checksum error */
      mlc_printf("HDD identify FAIL (checksum mismatch)\n");
      mlc_printf("Integrity word: %04hhX\n", buff[255]);
      mlc_printf("Sum: %d\n", calculated_sum);
      mlc_show_fatal_error();
      return;
    }
    else {
      mlc_printf("HDD identify OK (checksum pass)\n");
    }
  }
  else {
    mlc_printf("HDD identify OK (no checksum)\n");
  }

  /* Major version number */
  if(buff[80] == 0x0000 || buff[80] == 0xFFFF) {
    /* device does not report version */
  }
  else {
    for(int i = 14; i >= 2; i--) {
      if(buff[80] & (1 << i)) {
        if(i > 3) {
          mlc_printf("ATA/ATAPI-%d\n", i);
        }
        else {
          mlc_printf("ATA-%d\n", i);
        }
        break;
      }
    }
  }

  /*
   * This field contains the model number of the device. The contents of this field is an ASCII character string of forty
   * bytes. The device shall pad the character string with spaces (20h), if necessary, to ensure that the string is the
   * proper length. The combination of Serial number (words 10-19) and Model number (words 27-46) shall be unique
   * for a given manufacturer.
   */
  uint16 *hdd_model = &buff[27];
  mlc_printf("HDD Model: ");
  print_str_be16(hdd_model, 20);
  mlc_printf("\n");

  /*
   * This field contains the serial number of the device. The contents of this field is an ASCII character string of
   * twenty bytes. The device shall pad the character string with spaces (20h), if necessary, to ensure that the
   * string is the proper length. The combination of Serial number (words 10-19) and Model number (words 27-46)
   * shall be unique for a given manufacturer.
   */
  uint16 *hdd_serial = &buff[10];
  mlc_printf("HDD Serial: ");
  print_str_be16(hdd_serial, 10);
  mlc_printf("\n");

  /*
   * This field contains the firmware revision number of the device. The contents of this field is an ASCII character
   * string of eight bytes. The device shall pad the character string with spaces (20h), if necessary, to ensure that
   * the string is the proper length.
   */
  uint16 *hdd_fw_rev = &buff[23];
  mlc_printf("HDD FW Rev: ");
  print_str_be16(hdd_fw_rev, 4);
  mlc_printf("\n");

  /* Get CHS geometry info */
  ATAdev.chs[0] = buff[1];
  ATAdev.chs[1] = buff[3];
  ATAdev.chs[2] = buff[6];

  mlc_printf("CHS: %u/%u/%u\n", ATAdev.chs[0], ATAdev.chs[1], ATAdev.chs[2]);

  /*
   * Word 83 is the command set supported flags.
   * Bit 10 = LBA48 supported
   * 
   * Note form the ATA-ATAPI-6 spec:
   * The contents of words (61:60) and (103:100) shall not be used to determine if 48-bit addressing is
   * supported. IDENTIFY DEVICE bit 10 word 83 indicates support for 48-bit addressing. 
   * */
  ATAdev.lba48 = (buff[83] & (1 << 10)) ? 1 : 0;

  if(ATAdev.lba48) {
    mlc_printf("LBA48, ");
   /*
    * Words 100-103 contain a value that is one greater than the maximum LBA address in used addressable space
    * when the 48-bit Addressing feature set is supported. The maximum value that shall be placed in this field is
    * 0000FFFFFFFFFFFFh. Support of these words is mandatory if the 48-bit Address feature set is supported.
    */
    ATAdev.sectors = (
          ((uint64)buff[103] << 48)
        | ((uint64)buff[102] << 32)
        | ((uint64)buff[101] << 16)
        | ((uint64)buff[100] << 0)
        );
  }
  else {
    mlc_printf("LBA28, ");

   /*
    * Words (61:60) shall contain the value one greater than the largest user-addressable
    * sector in 28-bit addressing and shall not exceed 0FFFFFFFh. The content of words (61:60) shall
    * be greater than or equal to one and less than or equal to 268,435,455.
    */
    ATAdev.sectors = (
        ((uint64)buff[61] << 16)
      | ((uint64)buff[60] << 0)
      );
  }

  uint64 size_mb = ATAdev.sectors/BLOCKS_PER_MB;
  mlc_printf("Size: %lu.%luGB\n", (uint32)(size_mb / 1024), (uint32)((size_mb % 1024) / 10));

  /*
   * HDD quirks:
   *
   * The iPod 5.5G 80GB uses the "TOSHIBA MK8010GAH" ZIF hard drive.
   * MK = Prefix
   * 80 = 80GB
   * 10 = DSMR (Dual Stripe MR Head)
   * G = >10GB capacity
   * A = PATA
   * H = 1.8", 8mm thick, 4200RPM.
   * 
   * The Toshiba 10GAH and 31GAL drive families have a quirk where they only support reading whole physical sectors.
   * The logical sector (block) size is still 512 bytes, however reads will only succeed if:
   * - The start LBA is aligned to the physical sector boundary; and
   * - The number of blocks read is an exact multiple of the physical sector size.
   * 
   * The 10GAH family has 1024 byte (2 block) physical sectors. The 31GAL family has 4096 byte (8 block) physical sectors.
   * 
   * This means that for the iPod 5.5G MK8010GAH, we can only read even numbered LBAs, and the count must be a multiple of 2.
   * To read an odd LBA, read the even numbered LBA below it, and read 2 blocks to cover the LBA actually requested.
  */

  /* "TOSHIBA ????10GAH" */
  if(strncmp_be16("TOSHIBA ", 0, hdd_model, 0, sizeof("TOSHIBA ") - 1) == 0
    && strncmp_be16("10GAH", 0, hdd_model, 12, sizeof("10GAH") - 1) == 0)
  {
    mlc_printf("Enabling TOSHIBA 10GAH quirks\n");
    /* 1024 byte physical sectors, 2 blocks per physical sector */
    ATAdev.alignment_log2 = 1;
  }
  else {
    if(size_mb > (127 * 1024)) {
      /* HDD is larger than 127GB, it's probably a 4K sector HDD, or a flash modded iPod */
      /* Do 4K, 8 block sector reads */
      mlc_printf("Large drive, enabling 4K reads\n");
      ATAdev.alignment_log2 = 3;
    }
    else {
      /* 512 byte physical sectors, 1 blocks per physical sector */
      ATAdev.alignment_log2 = 0;
    }
  }

  #if DEBUG
    mlc_show_critical_error();
  #endif
}

/*
 * lba:       The Logical Block Adddress to begin reading blocks from.
 * count:     The number of logical blocks to read.
*/
static void ata_send_read_command(uint32 lba, uint16 count) {
  last_sector = lba;
  last_sector_count = count;

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
  uint8 head = ATAdev.lba48 ? 0 : ((lba & 0x0F000000) >> 24);
  pio_outbyte( REG_DEVICEHEAD  , 0xA0 | LBA_ADDRESSING | DEVICE_0 | head );
  DELAY400NS;
  pio_outbyte( REG_FEATURES    , 0 );
  pio_outbyte( REG_CONTROL     , CONTROL_NIEN | 0x08); /* 8 = HD15 */

  if(ATAdev.lba48) {
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

  /* Send read command */
  if (ATAdev.lba48) {
    ata_command( COMMAND_READ_SECTORS_EXT );
  }
  else {
    ata_command( COMMAND_READ_SECTORS );
  }

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

  if(!ATAdev.lba48 && (sector > 0x0FFFFFFF)) {
    /* The sector is too large for the current addressing scheme */
    mlc_printf(
      "Out of bounds read!\n"
      "Sector %lu is too large for LBA28 addressing.\n"
      , sector);
    mlc_show_fatal_error ();
    return(0);
  }

  /* Calculate an aligned LBA for the specified sector */
  uint16 read_size = (1u << ATAdev.alignment_log2);
  uint32 sector_mask = ~((1u << (ATAdev.alignment_log2)) - 1u);
  uint32 sector_to_read = sector & sector_mask;

  /* Send the read command to the device*/
  ata_send_read_command(sector_to_read, read_size);

  if (useCache) {
    /*
      * In cached mode, store every 512 byte block we read into the cache,
      * and then copy the requested sector out to dst
    */
    for(uint32 i = sector_to_read; i < (sector_to_read + read_size); i++) {
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
    for(uint32 i = sector_to_read; i < (sector_to_read + read_size); i++) {
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
