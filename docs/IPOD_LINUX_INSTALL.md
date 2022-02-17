# iPod Linux installation

There are two options for installing iPod Linux:

1. The original, hard way. This requires partitioning the iPod to add a Linux ext2/ext3 partition to the drive, and then installing iPod Linux onto that.

**Note:** If you have seen other guides for installing iPod Linux that involve using the official iPod Linux installer, this is the old method. The installer partitions the iPod for you. However, the installer requires raw disk access on the iPod and does not work correctly with modern versions of Windows.


2. Use [ProjectZeroSlackr](https://github.com/ProjectZeroSlackr/ProjectZeroSlackr-SVN).
   

This document will cover the installation of Project ZeroSlackr, which is almost entirely "drag and drop". I do not recommend the use of the old official iPod Linux installer.

**NOTE:** HFS+ formatted MacPods are untested and not recommended. For a smooth experience, format the iPod as a WinPod using iTunes on Windows, so that it has a FAT32 data partition. WinPods can still be used on macOS, but MacPods cannot be used on Windows.

## Zeroslackr

![Project ZeroSlackr's ZeroLauncher interface](pics/zerolauncher.jpg)

### Overview

This is a version of iPod Linux which does not require an additional partition to be created on the hard drive. Instead, an ext2 image file is saved in the standard FAT32 (WinPod) or HFS+ *journaling disabled* (MacPod) partition, which is the same partition that is mounted for the iPod "Disk Mode". The ext2 image contains the Linux userland. During boot, the kernel uses this as the root partition, and then symlinks several directories in from the FAT32/HFS+ partition over the top so that it's still easy to drag and drop files into the iPod using Disk Mode and have them accessible from iPod Linux.

### Prepare the iPod

In iTunes, enable Disk Mode on the iPod. The iPod data partition should be mounted and accessible.

Ensure that the iPod is formatted with either FAT32 (WinPod, highly recommended), or HFS+ *journaling disabled* (MacPod, untested).

If you are using a MacPod, copy all the files off the iPod, reformat it to HFS+ with journaling disabled, and then copy the contents back onto the iPod. 

### Sourcing the installation files

Download the latest version of [Project ZeroSlackr from the SourceForge repo here](https://sourceforge.net/projects/zeroslackr/). At the time of writing, this is `ZeroSlackr-SVN-snapshot-2008-08-11.7z`. This distribution includes many pre-installed applications, such as iBoy (Gameboy emulator) and igpSP (GBA emulator). More lightweight installs with less pre-installed applications are available from [here](https://sourceforge.net/projects/zeroslackr/files/Pre-Releases/ZeroSlackr%20SVN%20snapshot%202008-08-11/).

Extract the 7zip archive to your computer.

### Copy files to the iPod

Copy the following directories from the archive to the root of the iPod:

* /etc/
* /ZeroSlackr/
* /bin/
* /boot/
* /dev/

### Setup `loader.cfg`

iPodLoader2 needs a bootloader entry to tell it how to boot Project ZeroSlackr. Create or edit `loader.cfg` in the root of the iPod, and add the line:

`ZeroSlackr @ (hd0,1)/boot/vmlinux root=/dev/hda2 rootfstype=vfat rw quiet`

Additional entries can be created to boot directly into a target application, by using the `autoexec` kernel argument.

Refer to the reference copy of `loader2.cfg` included in the archive.

#### loader2.cfg example:

```
Apple OS @ ramimg
Rockbox @ (hd0,1)/.rockbox/rockbox.ipod
ZeroSlackr @ (hd0,1)/boot/vmlinux root=/dev/hda2 rootfstype=vfat rw quiet
igpSP @ (hd0,1)/boot/vmlinux root=/dev/hda2 rootfstype=vfat rw quiet autoexec=/opt/Emulators/igpSP/Launch/Launch-no-sound.sh
Podzilla2-SVN @ (hd0,1)/boot/vmlinux root=/dev/hda2 rootfstype=vfat rw quiet autoexec=/opt/Zillae/Podzilla2-SVN/Launch/Launch.sh
Disk Mode @ diskmode
Sleep @ standby
```
### Test

Eject the iPod and reboot it.

iPodLoader2 should boot and present the boot menu with a ZeroSlackr entry.

Select ZeroSlackr. The Linux kernel should boot, followed by the ZeroLauncher interface.

All done. Go have fun!

## FAQ

### Q: Why can't Linux be directly installed onto a FAT32 (WinPod) partition?

You may wonder if iPod Linux can be installed by simply by extracting the root filesystem onto the iPod's data partition, and then booting the kernel with the right `root` argument. Unfortunately, FAT32 doesn't support the file permissions and symbolic link features required for Linux to work. I actually attempted to get this workign as a proof of concept with lots and lots of hacks, but I couldn't even get it to finish booting up.

This is why Linux needs to be booted off an ext2 partition. Luckily, this partition doesn't need to be a physical partion on the iPod hard drive. The ZeroSlackr modified linux kernel can boot from an ext2 image file, which can be stored within the FAT32 partition.

### Q: Why can't Linux be direclty installed onto a HFS+ (MacPod) partition?

HFS+ theoretically supports all the features required for Linux to boot and run (after all, macOS is a Unix-like operating system, and can boot from HFS+). I haven't tested it, but it's probably possible to get this working.

Journal support must be disabled on the iPod HFS+ partion, or iPod Linux will mount it as read-only. This is true whether the ext2 loopmount image is used, or the partition is used directly. By default, macOS+iTunes format the iPod to HFS+ *with journaling enabled*, so it must be manually reformatted to *journaling disabled* to use HFS+ with any flavour of iPod Linux.
