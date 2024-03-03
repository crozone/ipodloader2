#include "bootloader.h"
#include "ata2.h"
#include "fat32.h"
#include "ext2.h"
#include "fwfs.h"
#include "vfs.h"
#include "minilibc.h"
#include "macpartitions.h"

#define MAX_FILES 10
#define MAX_FS    4

static filesystem *fs[MAX_FS]; /* Hardlimit of 4 registered filesystems */
static uint32 fsCnt;

typedef struct {
  uint32 fsIdx;
  int    fd;
} vfs_handle_t;

vfs_handle_t vfs_handle[MAX_FILES]; /* Hardlimit of 10 open files */
uint32 maxHandle;

int vfs_find_part(vfs_type type) {
  int i;

  for(i = 0; i < MAX_FS; i++){
    if(fs[i] && fs[i]->type == type){
      return (i);
    }
  }

  return (-1);
}

int vfs_open(char *fname) {
  int8 part;
  int  i;

  part = -1;

  /* FAT32: [dos], [fat], [win], [vfat], [fat32]
     EXT2:  [ext], [ext2], [linux]
     HFS+:  [hfs], [hfs+], [linux] */
  if( fname[0] == '[' ){
      if( !mlc_strncmp(fname,"[dos]",5) || !mlc_strncmp(fname,"[fat]",5) ||
          !mlc_strncmp(fname,"[win]",5) || !mlc_strncmp(fname,"[vfat]",6) ||
          !mlc_strncmp(fname,"[fat32]",7) ){
          part = vfs_find_part(FAT32);
      }
      if( !mlc_strncmp(fname,"[ext]",5) || !mlc_strncmp(fname,"[ext2]",6) ||
          !mlc_strncmp(fname,"[linux]",7) ){
          part = vfs_find_part(EXT2);
      }
      if( !mlc_strncmp(fname,"[hfs]",5) || !mlc_strncmp(fname,"[hfs+]",6) ||
          (part == -1 && !mlc_strncmp(fname,"[linux]",7)) ){
          part = vfs_find_part(HFSPLUS);
      }
      /* [xxx]/ == position of ] + 2 chars */
      fname = mlc_strchr(fname, ']') + 2;
  }
  else if( !mlc_strncmp(fname,"(hd0,",5) ){
      part = fname[5] - '0'; /* atoi, the old-fashioned way */
      /* (hd0,0)/ == 8 chars */
      fname = fname + 8;
  }

  if( part == -1 ) return(-1);

  if (fs[part]) {
    i = 0;
    while( (vfs_handle[i].fd != -1) && (i<MAX_FILES) ) i++;
  
    vfs_handle[i].fsIdx = part;
    vfs_handle[i].fd = fs[part]->open(fs[part]->fsdata,fname);

    if( vfs_handle[i].fd != -1 )
      return(i);
  }
  
  return(-1);
}

void vfs_close(int fd) {
  if(vfs_handle[fd].fd != -1) {
    uint32 part = vfs_handle[fd].fsIdx;
    if (fs[part]->close) {
      fs[part]->close( fs[part]->fsdata, vfs_handle[fd].fd );
    }
    vfs_handle[fd].fd = -1;
  }
}

int vfs_seek(int fd,long offset,int whence) {
  uint32 part;
  
  if(vfs_handle[fd].fd == -1) return(-1);

  part = vfs_handle[fd].fsIdx;

  return( fs[part]->seek( fs[part]->fsdata,vfs_handle[fd].fd,offset,whence) );
}

long vfs_tell(int fd) {
  uint32 part;
  
  if(vfs_handle[fd].fd == -1) return(-1);

  part = vfs_handle[fd].fsIdx;

  return( fs[part]->tell( fs[part]->fsdata,vfs_handle[fd].fd) );
}

int vfs_getinfo(int fd, long *out_chksum) {
  uint32 part;
  if(vfs_handle[fd].fd == -1) return(-1);
  part = vfs_handle[fd].fsIdx;
  if (!fs[part]->getinfo) return -1;
  return fs[part]->getinfo( fs[part]->fsdata, vfs_handle[fd].fd, out_chksum );
}

size_t vfs_read(void *ptr,size_t size, size_t nmemb,int fd) {
  uint32 part;
  
  if(vfs_handle[fd].fd == -1) return(-1);

  part = vfs_handle[fd].fsIdx;

  return( fs[part]->read( fs[part]->fsdata,ptr,size,nmemb,vfs_handle[fd].fd) );
}

void vfs_registerfs( filesystem *newfs ) {
  fs[newfs->partnum] = newfs;
}

void vfs_init( void) {
  uint32 i;
  uint8 sectormultiplier;

  fsCnt = 0;
  
  mbr_t *iPodMBR;
  iPodMBR = mlc_malloc( sizeof(mbr_t));
  
  fs_header_t *fs_header;
  fs_header = mlc_malloc( sizeof(fs_header_t));

  ata_readblocks_uncached(iPodMBR, 0, 1);

  /* Sector multiplier is for 5.5G 80GB, which has 2048b sectors */
  // TODO: Is it 2048 or 1024kb sectors???
  sectormultiplier = ata_get_blks_per_phys_sector();
  // if(ata_get_drivetype() == 1) {
  //   sectormultiplier = 4;
  // } else {
  //   sectormultiplier = 1;
  // }

  for(i=0; i < MAX_FILES; i++) vfs_handle[i].fd = -1;

  /* The number of valid partitions that were found */
  uint8  foundpartcount = 0;

  if( iPodMBR->MBR_signature == 0xAA55 ) {
    /* this is a WinPod with a DOS/ext2 partition scheme */
    mlc_printf("Detected WinPod MBR\n");

    uint32 logBlkMultiplier = (iPodMBR->code[12] | iPodMBR->code[11]) / 2; // we usually find 02 00, 00 02 or 00 08 here
    if((logBlkMultiplier < 1) | (logBlkMultiplier > 4)) logBlkMultiplier = 1;
	
    /* Check each primary partition for a supported FS */
    for(i=0; i < MAX_FS; i++) {
      uint32 offset;
      uint8  type;
      uint8  validoffset;

      type   = iPodMBR->partition_table[i].type;
      offset = iPodMBR->partition_table[i].lba_offset;
      validoffset = 0;

      /*
       * Now find a valid partition table. This is actually a PITA since the 5.5G
       * does not necessarily have identical MBRs on every machine. On some the
       * logBlkMultiplier is 4, on some it is 1, while it should be 4. There's no
       * other way than just peeking for valid partitions.
       */
      switch(type) {
        case 0x00:
          /* Technically this is an "Empty partition entry", but Apple uses it for the proprietary firmware partition */
          ata_readblocks_uncached(fs_header, offset*sectormultiplier,1);
          if( mlc_strncmp((void*)(fs_header->fwfsmagic),"]ih[", 4) ) {
            offset = offset * sectormultiplier;
            validoffset = 1;
          }
          else if((logBlkMultiplier != 1) && (logBlkMultiplier != sectormultiplier)) {
            ata_readblocks_uncached(fs_header, offset*logBlkMultiplier,1);
            if( mlc_strncmp((void*)(fs_header->fwfsmagic),"]ih[", 4) )
            {
              offset = offset * logBlkMultiplier;
              validoffset = 1;
            }
          }

          if(validoffset) {
            foundpartcount++;
            mlc_printf("[%d]: iPod FW\n", i);
            fwfs_newfs(i, offset);
          }
          else {
            mlc_printf("[%d]: Empty\n", i);
          }
          
          break;
        case 0x83:
          /* EXT2 partition */
          ata_readblocks_uncached(fs_header, offset*sectormultiplier + 2, 1);
          if(fs_header->ext2magic == 0xEF53) {
            offset = offset*sectormultiplier;
            validoffset = 1;
          }
          else if((logBlkMultiplier != 1) && (logBlkMultiplier != sectormultiplier)) {
            ata_readblocks_uncached(fs_header, offset*logBlkMultiplier + 2, 1);
            if(fs_header->ext2magic == 0xEF53) {
              offset = offset*logBlkMultiplier;
              validoffset = 1;
            }
          }

          if(validoffset) {
            foundpartcount++;
            mlc_printf("[%d]: EXT2\n", i);
            ext2_newfs(i, offset);
          }
          else {
            mlc_printf("[%d]: Bad EXT2 entry\n", i);
            mlc_delay_ms(1000);
          }

          break;
        case 0xB:
          /* FAT partition */
          ata_readblocks_uncached(fs_header, offset*sectormultiplier,1);
          if(fs_header->fat32magic == 0xAA55) {
            offset = offset*sectormultiplier;
          }
          else if((logBlkMultiplier != 1) && (logBlkMultiplier != sectormultiplier)) {
            ata_readblocks_uncached(fs_header, offset*logBlkMultiplier,1);
            if(fs_header->fat32magic == 0xAA55) {
              offset = offset*logBlkMultiplier;
            }
          }

          if(validoffset) {
            foundpartcount++;
            mlc_printf("[%d]: FAT\n", i);
            fat32_newfs(i, offset);
          }
          else {
            mlc_printf("[%d]: Bad FAT entry\n", i);
            mlc_delay_ms(1000);
          }

          break;
        default:
          mlc_printf("[%d]: Unknown 0x%X2\n", i, type);
          break;
      }
    }

    if(foundpartcount > 0) {
      mlc_printf("Found %d valid partitions\n", i);
    }
    else {
      /* No valid partition was found */
      mlc_printf("No valid paritions found!\n");
      mlc_show_critical_error();
      return;
    }
  }
  /* TODO: Why is this ER and not PM (for Apple Parition Map)? What is this? */
  else if( iPodMBR->code[0] == 'E' && iPodMBR->code[1] == 'R') {
    /* this is a MacPod with a HFS partition scheme */
    mlc_printf("Detected MacPod partition\n");
    check_mac_partitions ((uint8 *)iPodMBR);
  }
  else {
    mlc_printf("Invalid MBR\n");
	  mlc_hexdump (iPodMBR, 16);
	  mlc_hexdump (((uint8*) iPodMBR)+512-16, 16);
    mlc_show_critical_error();
    return;
  }

}
