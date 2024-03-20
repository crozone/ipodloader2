 /*
 * Basic read-only FAT filesystem driver for the ipodlinux bootloader
 *
 * Supports:
 *  Read-only access
 *  FAT16 and FAT32 with automatic type detection
 *  Long Filename support (LFN)
 * 
 * Authors: Original Authors: Unknown
 *          Ryan Crosby     ( ryan.crosby@live.com ) - LFN fixes, 8.3 fixes, UCS-2 fixes, FAT type detection rewrite, comments. 
 * 
 *     Short cuts make long delays.
 *
 *      -"The Fellowship of the Ring", J.R.R Tolkien
 * 
 */

#include "bootloader.h"
#include "ata2.h"
#include "vfs.h"
#include "fat32.h"
#include "minilibc.h"

#define MAX_HANDLES 10

static filesystem myfs;

typedef struct {
  /*
   * The 512 byte block count offset from the start of the drive to the start of the FAT partition
   */
  uint32 offset;

  uint32 sectors_per_fat;
  uint32 root_dir_first_cluster;
  uint32 data_area_offset;
  uint32 bytes_per_cluster;

  uint16 bytes_per_sector;
  /* The number of 512 byte blocks per sector */
  uint16 blks_per_sector;
  /* The number of 512 byte blocks per cluster */
  uint16 blks_per_cluster;
  uint16 number_of_reserved_sectors;
  uint16 sectors_per_cluster;
  uint16 entries_in_rootdir;
  uint16 entries_per_sector;
  uint8  number_of_fats;
  uint8  bits_per_fat_entry;

  fat32_file *filehandles[MAX_HANDLES];
  uint32      numHandles;
} fat_t;

static fat_t fat;

static uint8 *clusterBuffer = NULL;

/*
 * This caches a single FAT sector and is at least the length of the FAT sector size.
 * It is initialized in fat32_newfs and used mainly in fat32_findnextcluster.
 */
static uint8 *gFATSectorBuf = NULL;
static uint32 gSecNumInFATBuf = -1;

static void readToSectorBuf (uint32 sector)
{
  if (gSecNumInFATBuf != sector) {
    ata_readblocks (gFATSectorBuf, sector * fat.blks_per_sector, fat.blks_per_sector);
    gSecNumInFATBuf = sector;
  }
}

/*
 * Get a 32 bit unsigned integer from the given array,
 * treating byte order as little-endian.
 */
inline static uint32 getLE32 (uint8* p) {
  return p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24);
}

/*
 * Get a 16 bit unsigned integer from the given array,
 * treating byte order as little-endian.
 */
inline static uint16 getLE16 (uint8* p) {
  return p[0] | (p[1] << 8);
}

/*
 * Finds the next cluster in the FAT chain, given a current cluster index.
 */
static uint32 fat32_findnextcluster(uint32 prev_cluster)
{
  uint32 sector, offset, ret = 0;

  /* Calculate the byte offset for the FAT entry for prev_cluster */
  uint32 fatOffset;
  if (fat.bits_per_fat_entry == 16) {
    fatOffset = prev_cluster * 2;
  }
  else if (fat.bits_per_fat_entry == 32) {
    fatOffset = prev_cluster * 4;
  }
  else {
    /* Unknown FAT type */
    mlc_printf("Invalid bits_per_fat_entry\nValue: %u\n", fat.bits_per_fat_entry);
    mlc_show_fatal_error();
    return 0;
  }

  /* Calculate the sector that the FAT entry is located within */
  sector = (fat.offset / fat.blks_per_sector) + // offset sectors
           (fat.number_of_reserved_sectors + (fatOffset / fat.bytes_per_sector)); // sector within FAT

  /* Calculate the byte offset within this sector for the entry */
  offset = fatOffset % fat.bytes_per_sector;

  readToSectorBuf (sector);

  if (fat.bits_per_fat_entry == 16) {
    ret = getLE16 (gFATSectorBuf+offset);
    if (ret < 2 || ret >= 0xFFF0) ret = 0;
  }
  else if (fat.bits_per_fat_entry == 32) {
    /*
     * A FAT32 FAT entry is actually only a 28-bit entry.
     * The high 4 bits of a FAT32 FAT entry are reserved.
     */
    ret = getLE32 (gFATSectorBuf+offset) & 0x0FFFFFFF;
    if (ret < 2 || ret >= 0x0FFFFFF0) ret = 0;
  }

  return ret;
}

static uint32 calc_lba (uint32 start, int isRootDir)
{
  uint32 lba;
  lba  = fat.offset +
         (fat.number_of_reserved_sectors + (fat.number_of_fats * fat.sectors_per_fat) +
         (start - 2) * fat.sectors_per_cluster + (isRootDir?0:fat.data_area_offset))
         * fat.blks_per_sector;
  //mlc_printf("LBA %ld - %ld\n", start, lba);
  return lba;
}

static uint8 lfn_checksum (const unsigned char *entryName)
// entryName must be the name filled with spaces and without the "."
// example: "FAT32   C  "
{
  uint8 sum = 0;
  for (int i = 11; i > 0; --i) {
    sum = ((sum & 1) ? 0x80 : 0) + (sum >> 1) + *entryName++;
  }
  return sum;
}

typedef struct {
  uint16 isRoot;
  uint16 entryIdx;
  uint32 cluster;
  uint8* buffer;
} dir_state;

static void* getNextRawEntry (dir_state *state)
{
  if (!state->buffer) {
    state->buffer = clusterBuffer;
  }
  uint16 idx = (state->entryIdx)++;
  if (idx % fat.entries_per_sector != 0) {
    return &state->buffer[(idx % fat.entries_per_sector) << 5];
  }
  else {
    // we're starting a new sector
    uint32 cluster_lba;
    uint16 sectorIdx = idx / fat.entries_per_sector; // there are 16 entries in a 512-byte sector
    if (state->isRoot && fat.entries_in_rootdir > 0) {
      // it's a FAT16 root dir - all its sectors are in succession
      if (idx >= fat.entries_in_rootdir) {
        // end of root dir
        return 0;
      }
    }
    else {
      sectorIdx = sectorIdx % fat.sectors_per_cluster;
      if (sectorIdx == 0 && idx > 0) {
        // next cluster
        state->cluster = fat32_findnextcluster (state->cluster);
        if (state->cluster <= 0) {
          // no more clusters -> end of dir
          return 0;
        }
      }
      else {
        // next sector in same cluster
      }
    }
    cluster_lba = calc_lba (state->cluster, state->isRoot);
    ata_readblocks( state->buffer, cluster_lba + sectorIdx * fat.blks_per_sector, fat.blks_per_sector );
    return &state->buffer[0];
  }
}

static void trimr (char *s) {
  int pos = mlc_strlen(s);
  while (pos > 0 && s[pos-1] == ' ') --pos;
  s[pos] = 0;
}

/*
 * Copies a USC-2 string into an ASCII string destination buffer.
 * If any of the UCS-2 characters fall outside of the ASCII character range,
 * the character will be replaced with a '_' character, and the function will return
 * the count of unsupported characters.
 */
static int ucs2cpy (char *dest, uint8 *ucs2src, int chars) {
  int unknown_chars = 0;
  while (chars--) {
    uint16 current_ucs2 = ucs2src[0] | (ucs2src[1] << 8);

    if(current_ucs2 == 0x0000) {
      /* NULL terminator */
      *dest = '\0';
    }
    else if(current_ucs2 == 0xFFFF) {
      /* Unused entry after the 0x0000 terminator character */
      *dest = '\0';
    }
    else if ((current_ucs2 >= 0x0020) && (current_ucs2 <= 0x007E)) {
      /* The character is in the valid ASCII range */
      *dest = current_ucs2 & 0x00FF;
    }
    else {
      /* Unmappable character */
      *dest = '_';
      unknown_chars++;
    }

    dest++;
    ucs2src += 2;
  }

  return unknown_chars;
}

static int getNextCompleteEntry (dir_state *dstate, char* *shortnameOut, char* *longnameOut, uint32 *cluster, uint32 *flength, uint8 *ftype)
{
  typedef struct {
    uint8    seq;    /* sequence number for slot, ored with 0x40 for last slot (= first in dir) */
    uint8    name0_4[10]; /* first 5 characters in name */
    uint8    attr;    /* attribute byte, = 0x0F */
    uint8    reserved;  /* always 0 */
    uint8    alias_checksum;  /* checksum for 8.3 alias, see lfn_checksum() */
    uint8    name5_10[12];  /* 6 more characters in name */
    uint16   start;   /* always 0 */
    uint8    name11_12[4];  /* last 2 characters in name */
  } long_dir_slot;

  /*
   * Buffer for the 8.3 filename.
   *
   * 8 characters for the name
   * 1 character for the '.'
   * 3 characters for the extension
   * 1 character for the null terminator
   */
  static char shortname[13];
  /*
   * Buffer for the LFN filename.
   * At most this can be 255 characters.
   * An extra character is added at the end to guarantee that a null terminator is always present.
   */
  static char longname[256];
  uint8 *entry;
  uint8 chksum = 0, namegood = 0;

  while ( (entry = getNextRawEntry (dstate)) != 0 ) {
    if (entry[0] == 0) {
      return 0; // end of dir
    }
    else if (entry[0] == 0xE5) {
        // deleted entry - continue with loop
    }
    else if (entry[0x0B] == 0x0F) {
      /*
       * A Long File Name (LFN) entry.
       * Attributes = Volume Label, System, Hidden, Read Only. (0x0F)
       */
      long_dir_slot *slot = (long_dir_slot*)entry;

      /*
       * LFN Sequence Number (slot->seq)
       *
       * bit 6 (& 0x40):    last logical, first physical LFN entry
       * bit 5 (& 0x20):    0
       * bits 4-0 (& 0x1F): index number 0x01..0x14 (20 max)
       *
       * A deleted entry is still 0xE5.
       */

      /*
       * Since each LFN entry always contains 13 characters of the LFN,
       * we can use the index to calculate the offset of the current entry
       * in the output file name string.
       */
      int offset = 13 * ((slot->seq & 0x1F) - 1);
      if (offset >= 0 && offset < ((sizeof(longname) - 1) - 13) && !(slot->seq & 0x80)) {
        if (slot->seq & 0x40) {
          /*
           * 0x40 bit set indicates we have discovered the first physical
           * and last logical LFN entry, which has the highest sequence number.
           */
          /* Zero out the existing LFN buffer */
          mlc_memset (&longname, 0, sizeof(longname));
          /* This entry contains the alias checksum. */
          chksum = slot->alias_checksum;
          /* Set namegood = 1 to indicate we have a potentally valid LFN. */
          namegood = 1;
        }

        if(namegood) {
          char *ln = &longname[offset];
          int invalid_chars =
              ucs2cpy (&ln[0], slot->name0_4, 5)
            + ucs2cpy (&ln[5], slot->name5_10, 6)
            + ucs2cpy (&ln[11], slot->name11_12, 2);

          if(invalid_chars > 0) {
            /*
            * The name might be valid UCS-2, but contains non-ASCII mappable characters that we cannot deal with.
            * We are forced to treat the name as invalid and only use the short name.
            */

            namegood = 0;
          }
        }
      }
      else {
        namegood = 0;
      }
    }
    else {
      /*
       * TODO: Make a struct for normal directory entries,
       * there's no reason we should be using all of these magic offset values
       */
      *ftype = entry[0x0B];
      if (!namegood || chksum != lfn_checksum (&entry[0])) {
        // previously collected name does not belong to this entry
        longname[0] = 0;
        namegood = 0;
      }
      uint32 cl = getLE16(entry+0x1A);
      if (fat.bits_per_fat_entry == 32) {
        cl |= getLE16(entry+0x14) << 16;
      }
      *cluster = cl;
      *flength = getLE32(entry+0x1C);
      if (*ftype & 8) {
        // volume label - no "." in name
        mlc_strlcpy (shortname, (char*)&entry[0], 11 + 1);
      } else {
        mlc_strlcpy (shortname, (char*)&entry[0], 8 + 1);
        trimr (shortname);
        char ext[4];
        mlc_strlcpy (ext, (char*)&entry[8], sizeof(ext));
        trimr (ext);
        if (ext[0]) {
          mlc_strlcat (shortname, ".", sizeof(shortname));
          mlc_strlcat (shortname, ext, sizeof(shortname));
        }
      }
      trimr (shortname);
      *shortnameOut = shortname;
      *longnameOut = longname;
      return 1;
    }
  }
  return 0; // end of dir
}

static fat32_file *fat32_findfile(uint32 startCluster, int isRoot, char *fname)
{
  uint32 flength, cluster;
  uint8  ftype;
  char *shortname, *longname;
  dir_state dstate = {isRoot, 0, startCluster, 0};
  char *next = mlc_strchr( fname, '/' );

  while ( getNextCompleteEntry (&dstate, &shortname, &longname, &cluster, &flength, &ftype) ) {
    if (*shortname == 0) {
      // deleted entry
    } else if ( (ftype & 0x1F) == 0 ) {
      // A file
      if ( mlc_strcasecmp( shortname, fname ) == 0 || mlc_strcasecmp( longname, fname ) == 0 ) {
        fat32_file *fileptr;
        fileptr = (fat32_file*)mlc_malloc( sizeof(fat32_file) );
        fileptr->cluster  = cluster;
        fileptr->opened   = 1;
        fileptr->position = 0;
        fileptr->length   = flength;
        return fileptr;
      }
    } else if ( ftype & 0x10 ) {
      // A directory
      int len = next-fname;
      if( next && ((mlc_strncasecmp( shortname, fname, len ) == 0 && shortname[len] == '\0')
                   || (mlc_strncasecmp( longname, fname, len ) == 0 && longname[len] == '\0')) ) {
        return fat32_findfile( cluster, 0, next+1 );
      }
    }
  }
  return 0; // end of dir
}

static int fat32_open(void *fsdata,char *fname) {
  fat_t      *fs;
  fat32_file *file;

  fs = (fat_t*)fsdata;

  file = fat32_findfile(fs->root_dir_first_cluster,1,fname);

  if(file != NULL) {
    if( fs->numHandles < MAX_HANDLES ) {
      fs->filehandles[fs->numHandles++] = file;
    } else return(-1);
  }
  else {
    mlc_printf("%s not found\n", fname);
    return(-1);
  }

  return(fs->numHandles-1);
}

static void fat32_close (void *fsdata, int fd)
{
  fat_t *fs = (fat_t*)fsdata;
  if (fd == fs->numHandles-1) {
    --fs->numHandles;
  }
  /* If mlc_free existed, we would mlc_free(fs->filehandles[fd]) here */
  /* For now, we just leak memory for every file opened. */
}

static size_t fat32_read(void *fsdata,void *ptr,size_t size,size_t nmemb,int fd) {
  uint32 read,toRead,lba,clusterNum,cluster,i;
  uint32 offsetInCluster, toReadInCluster;
  fat_t *fs;

  fs = (fat_t*)fsdata;

  read   = 0;
  toRead = size*nmemb;
  if( toRead > (fs->filehandles[fd]->length + fs->filehandles[fd]->position) ) {
    toRead = fs->filehandles[fd]->length + fs->filehandles[fd]->position;
  }

  /*
   * FFWD to the cluster we're positioned at
   * Could get a huge speedup if we cache this for each file
   * (Hmm.. With the addition of the sector-cache, this isn't as big of an issue, but it's still an issue though)
   */
  clusterNum = fs->filehandles[fd]->position / fs->bytes_per_cluster;
  cluster = fs->filehandles[fd]->cluster;

  for(i=0;i<clusterNum;i++) {
    cluster = fat32_findnextcluster( cluster );
  }
  
  offsetInCluster = fs->filehandles[fd]->position % fs->bytes_per_cluster;

  /* Calculate LBA for the cluster */
  lba = calc_lba (cluster, 0);
  toReadInCluster = fs->bytes_per_cluster - offsetInCluster;
  ata_readblocks( clusterBuffer, lba, ((toReadInCluster+fs->bytes_per_sector-1) / fs->bytes_per_sector) * fs->blks_per_sector );

  if( toReadInCluster > toRead ) toReadInCluster = toRead; 

  mlc_memcpy( (uint8*)ptr + read, clusterBuffer + offsetInCluster, toReadInCluster );

  read += toReadInCluster;

  /* Loops through all complete clusters */
  while(read < ((toRead / fs->bytes_per_cluster)*fs->bytes_per_cluster) ) {
    cluster = fat32_findnextcluster( cluster );
    lba = calc_lba (cluster, 0);
    ata_readblocks( clusterBuffer, lba, fs->blks_per_cluster );

    mlc_memcpy( (uint8*)ptr + read, clusterBuffer, fs->bytes_per_cluster );

    read += fs->bytes_per_cluster;
  }

  /* And the final bytes in the last cluster of the file */
  if( read < toRead ) {
    cluster = fat32_findnextcluster( cluster );
    lba = calc_lba (cluster, 0);
    ata_readblocks( clusterBuffer, lba, fs->blks_per_cluster );
    
    mlc_memcpy( (uint8*)ptr + read, clusterBuffer,toRead - read );

    read = toRead;
  }

  fs->filehandles[fd]->position += toRead;

  return(read / size);
}

static long fat32_tell(void *fsdata,int fd) {
  fat_t *fs;

  fs = (fat_t*)fsdata;

  return( fs->filehandles[fd]->position );
}

static int fat32_seek(void *fsdata,int fd,long offset,int whence) {
  fat_t *fs;

  fs = (fat_t*)fsdata;
  
  switch(whence) {
  case VFS_SEEK_CUR:
    offset += fs->filehandles[fd]->position;
    break;
  case VFS_SEEK_SET:
    break;
  case VFS_SEEK_END:
    offset += fs->filehandles[fd]->length;
    break;
  default:
    return -2;
  }

  if( offset < 0 || offset > fs->filehandles[fd]->length ) {
    return -1;
  }

  fs->filehandles[fd]->position = offset;
  return 0;
}


void fat32_newfs(uint8 part,uint32 offset) {
  // Reset fat info structure
  mlc_memset (&fat, 0, sizeof(fat));
  fat.offset = offset;

  /* Create a buffer for the BPB (BIOS Parameter Block),
   * aka boot sector, reserve sector, 0th sector.
   * The BPB is 512 bytes.
   *
   * Note:
   * We will repurpose this buffer as the gFATSectorBuf
   * after we are done with the BPB. The gFATSectorBuf
   * has a worse case of being 4096 bytes long, so we will allocate
   * 4096 bytes even though the BPB itself only uses 512 bytes
   * so this buffer can be repurposed as a sector buffer.
   */
  uint8* bpb = (uint8*)mlc_malloc(4096);

  /* Read in the BPB */
  ata_readblocks (bpb, offset, 1);

  /* Verify that this is a FAT partition */
  if( getLE16(bpb+510) != 0xAA55 ) {
    mlc_printf("Not valid FAT superblock\n");
    mlc_show_critical_error();
    return;
  }

  uint16 BPB_BytsPerSec = getLE16(bpb+11);
  /* Validate BPB_BytsPerSec is a legal value.*/
  /* TODO: This should pretty much ALWAYS be 512 bytes
   *       since even Advanced Format HDDs emulate 512 byte sectors.
   *       Should we just always validate that it's 512?
   */
  switch(BPB_BytsPerSec) {
    case 512:
    case 1024:
    case 2048:
    case 4096:
    break;
    default:
      /* Invalid bytes per sector count. */
      mlc_printf("Invalid FAT BPB_BytsPerSec\nValue: %u\n", BPB_BytsPerSec);
      mlc_show_critical_error();
      return;
  }

  fat.bytes_per_sector = BPB_BytsPerSec;

  uint8 BPB_SecPerClus = bpb[13];
  /* Validate BPB_SecPerClus is a legal value */
  switch(BPB_SecPerClus) {
    case 1:
    case 2:
    case 4:
    case 8:
    case 16:
    case 32:
    case 64:
    case 128:
      break;
    default:
      /* Invalid bytes per sector count. */
      mlc_printf("Invalid FAT BPB_SecPerClus\nValue: %u\n", BPB_SecPerClus);
      mlc_show_critical_error();
      return;
  }

  fat.sectors_per_cluster = BPB_SecPerClus;

  uint16 BPB_RootEntCnt = getLE16(bpb+17);

  // Calculate the root directory size. On FAT32, this is always 0.
  uint32 rootDirSectors = ((BPB_RootEntCnt * 32) + (BPB_BytsPerSec - 1)) / BPB_BytsPerSec; // root directory size

  uint32 fatSz;
  uint16 BPB_FATSz16 = getLE16(bpb+22);
  if(BPB_FATSz16 != 0) {
    fatSz = BPB_FATSz16;
  }
  else {
    uint32 BPB_FATSz32 = getLE32(bpb+36);
    fatSz = BPB_FATSz32;
  }

  uint32 totSec;
  uint16 BPB_TotSec16 = getLE16(bpb+19);
  if(BPB_TotSec16 != 0) {
    totSec = BPB_TotSec16;
  }
  else {
    uint32 BPB_TotSec32 = getLE32(bpb+32);
    totSec = BPB_TotSec32;
  }

  uint16 BPB_ResvdSecCnt = getLE16(bpb+14);
  uint16 BPB_NumFATs = bpb[16];
  uint32 firstDataSector = BPB_ResvdSecCnt + (BPB_NumFATs * fatSz) + rootDirSectors;
  uint32 dataSec = totSec - firstDataSector;
  uint32 countofClusters = dataSec / BPB_SecPerClus;
  
  fat.data_area_offset           = rootDirSectors;
  fat.entries_in_rootdir         = BPB_RootEntCnt;
  fat.number_of_reserved_sectors = BPB_ResvdSecCnt;
  fat.number_of_fats             = BPB_NumFATs;
  fat.sectors_per_fat            = fatSz;
  fat.bytes_per_cluster          = BPB_BytsPerSec * BPB_SecPerClus;
  fat.entries_per_sector         = BPB_BytsPerSec / 32;
  fat.blks_per_sector            = BPB_BytsPerSec / 512;
  fat.blks_per_cluster           = fat.bytes_per_cluster / 512;

  /* Determine FAT type */
  if(countofClusters < 4085) {
    /* Volume is FAT12 */
    mlc_printf("FAT12 detected.\nClusters = %u\n", countofClusters);
    mlc_printf("FAT12 is not supported by this driver\n");
    mlc_show_critical_error();
    return;
  }
  else if(countofClusters < 65525) {
    /* Volume is FAT16 */
    fat.bits_per_fat_entry = 16;

    /* Calculate the sector for the root directory */
    uint16 firstRootDirSecNum = BPB_ResvdSecCnt + (BPB_NumFATs * BPB_FATSz16);
    fat.root_dir_first_cluster = firstRootDirSecNum; // used to be harcoded to 2? Check.

    mlc_printf("FAT16 detected.\nClusters = %u\n", countofClusters);
  }
  else {
    /* Volume is FAT32 */
    fat.bits_per_fat_entry = 32;

    /* Find the sector for the root directory in BPB_RootClus */
    uint32 BPB_RootClus = getLE32(bpb+44);
    fat.root_dir_first_cluster = BPB_RootClus;

    mlc_printf("FAT32 detected.\nClusters = %u\n", countofClusters);
  }

  /*
   * We are now done with the BPB.
   * Reuse the bpb buffer as the sector buffer.
   */
  gFATSectorBuf = bpb;

  if( clusterBuffer == NULL ) {
    clusterBuffer = (uint8*)mlc_malloc( fat.bytes_per_cluster );
  }

  /* TODO: MyFS Should be malloc'd every time! Otherwise it gets overwritten by any other FAT parititon */

  myfs.open    = fat32_open;
  myfs.close   = fat32_close;
  myfs.tell    = fat32_tell;
  myfs.seek    = fat32_seek;
  myfs.read    = fat32_read;
  myfs.getinfo = 0;
  myfs.fsdata  = (void*)&fat;
  myfs.partnum = part;
  myfs.type    = FAT32;

  vfs_registerfs( &myfs);
}
