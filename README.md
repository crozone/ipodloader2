# iPodLoader 2

Multi-OS bootloader for PortalPlayer based classic iPods.

## Compatibility

iPodLoader 2 functions with the following iPod generations:

* 1st generation (mechanical spinning wheel, buttons surround wheel, firewire port)
* 2nd generation (touch wheel, buttons surround wheel, firewire port, appears very similar to 1st generation)
* 3rd generation (touch wheel, four touch buttons under display, first iPod with 30 pin dock connector)
* 4th generation monochrome (click wheel)
* 4th generation colour ("iPod Photo" a.k.a "iPod with color display")
* 5th / 5.5th generation (Video, "iPod with Video")
* mini 1st generation (dated 2004, all 4GB capacity, no HDD capacity engraving on the rear)
* mini 2nd generation (dated 2005, 4GB and 6GB models, HDD capacity engraved on the rear)
* nano 1st generation (while/black gloss)
* nano 2nd generation (colourful with rounded edges, looks like a tiny iPod mini)

iPodLoader 2 can boot the following operating systems:

* iPod OS (stock Apple firmware)
* Rockbox
* iPod Linux

## Building

### Installation of toolchain (arm cross-compiler)

Debian Linux x86_64 9 (Stretch) and 10 (Buster) are used as the reference distros, although the install commands will work on Ubuntu and are easily adapted for other distros.

#### Install `make`:

`sudo apt install make`

Or optionally, you can instead install the full `build-essential` which includes other useful tools like gcc:

`sudo apt install build-essential`

#### Install the `arm-none-eabi` cross compiler:

`sudo apt install gcc-arm-none-eabi`

### Building the source

In the root directory of the repository, run

`make`

If the build went smoothly, an output file called `loader.bin` should have been left in the root.

If you want to rebuild the solution, simply run `make clean` and then run `make` again.

## Overview

Loader2 & iPodPatcher / SansaPatcher

- This is the main bootloader for iPodLinux. It is capable
  of loading both the iPodLinux kernel and the original
  Apple OS as well as Rockbox. It is installed via Rockbox's
  ipodpatcher tool.
- Loader2 can be configured and customized. See the
  "loader.cfg" file.
- Rockbox can also be installed by extracting the latest
  Rockbox build directly to the iPod and editing the
  "loader.cfg" file.
- SansaLinux version uses pre-built sansapatcher files
  that are patched to load SansaLinux. The files currently
  used are from the original SansaLinux release. Much of
  the below information does not apply.
For more information, see:
- http://ipodlinux.org/wiki/Loader2
- http://www.rockbox.org/twiki/bin/view/Main/IpodPatcher
- http://en.wikipedia.org/wiki/Bootloader

## Usage

- Since Loader2 is a bootloader written from scratch,
  it has its own special syntax.
- For proper usage of Loader2 and its various options/syntax,
  see the provided "loader.cfg" file as well as
  the iPodLinux wiki page: http://ipodlinux.org/wiki/Loader2
- The iPodLinux kernel accepts most standard kernel arguments;
  see "/docs/kernel/Original/kernel-parameters.txt"
- Loader2 has also been patched to play a boot tune on startup.
  The default boot tune should be familiar to Final Fantasy
  gamers ; )

## Attribution


Loader 2:
- Authors: iPL Core Devs
- Source: SVN
- Link:
  https://ipodlinux.svn.sourceforge.net/svnroot/ipodlinux/apps/ipod/ipodloader2
- Date: Oct 31, 2008
- Version: Revision 2445
- See commit authors for modifications performed since the SVN revision was imported.

ZS Version:
- Modder: Keripo
- Type: Mod and Recompile
- Date: Nov 11, 2008
- Version: B X.X

iPodPatcher:
- Authors: Rockbox Devs
- Source: SVN
- Link:
  svn://svn.rockbox.org/rockbox/trunk/rbutil/ipodpatcher
- Date: Oct 10, 2008
- Version: Revision 18763

SansaPatcher:
- Authors: Rockbox Devs, Sebastian Duell
- Source: SansaLinux website
- Link:
  http://www.sansalinux.org/index.php?option=com_content&id=46
- Date: Mar 28, 2008
- Version: ???

Modifications:
- added patch to fix ipodpatcher error levels Rockbox's Flyspray
  task #8827: http://www.rockbox.org/tracker/task/8827
- added easy "patch" scripts for Windows and Linux
  (untested on Macs and 64-bit Linux)
- added pre-compiled ipodpatcher binaries
  for Windows (compiled by Keripo), 32-bit Linux
  (compiled by Keripo), 64-bit Linux (Rockbox wiki) and
  Mac OS X (compiled by Matthew Croop)

To do:
- find someone to help recompile ipodpatcher on 64-bit Linux
- FastLaunch.sh searcher
- sub-menu support (look at iBoot Advanced / iLoadz)

### License

Unless otherwise stated, all source code is licensed under
GNU GPL - see License.txt. For the licensing of the
software/application, refer to the license that comes
with the original/ported software.

All code/scripts written by Keripo are licensed under
GNU GPL - see License.txt. For the licensing of the
software/application, refer to the license that comes
with the original/ported software.