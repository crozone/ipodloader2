# iPod Linux install

By default, a Windows iPod has two partitions: An Apple firmware partition, and a FAT32 music partition. The FAT32 partition is what gets mounted when the iPod is in disk mode.

In an ideal world, Linux could be installed on the FAT32 partition, but FAT32 doesn't support the file permissions and symbolic links required for Linux to work. Linux at least requires an EXT2 partition.

There are two options for installing iPod Linux:

1. The original, hard way. This requires partitioning the iPod to add a Linux EXT2/EXT3 partition to the drive, and then installing iPod Linux onto that.
2. Use [ProjectZeroSlackr](https://github.com/ProjectZeroSlackr/ProjectZeroSlackr-SVN).
   This is a version of iPod Linux which does not require an additional partition to be created on the hard drive. Instead, an EXT2 image file is saved in the root of the standard FAT32 partition, containing the Linux userland. The kernel and an initrd images are also saved to the FAT32 partition. During boot, the initrd is extracted into RAM and mounted as a read-only rootfs. Then, `init` boots, which in turn loopback mounts the EXT2 image file and `pivot_root`'s it as the new root.

### TODO: Zeroslackr



## The hard way

### Partition the iPod

In order for iPod linux to work, you need the iPod partition layout to look like this:

[Apple Firmware Partition] | [Music Partition (FAT32)] | [Linux partition, EXT2 or EXT3]

[Apple Firmware Partition] is ~31MB
[Music Partition (FAT32)] as large as possible
[Linux partition (EXT2 or EXT3)] is small, usually 64MB-1GB depending on preference.

### Editing iPod partitions with GParted

By default, Windows formatted iPods have two partitions: a proprietary Apple firmware partition, followed by a FAT32 data partition. In the MBR of iPods formatted for Windows, the firmware partition has type "0". GParted  considers this to be unallocated space. Because of that, it can easily  overwrite the firmware partition or its partition table entry. This renders the device unbootable and requires a full restore to fix.

A workaround is to use fdisk to change the firmware partition type to a non-zero value. Then, the partitions can be edited with GParted without breaking the firmware partition. Finally, fdisk can be used to set the partition type back to zero. Fdisk warns that you probably shouldn't set the type to zero, but do it anyway.

### TODO: Install iPod Linux