Last updated: Nov 11, 2008
~Keripo

All code/scripts written by Keripo are licensed under
GNU GPL - see License.txt. For the licensing of the
software/application, refer to the license that comes
with the original/ported software.

Overview:
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

Usage:
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


Loader 2:
- Authors: iPL Core Devs
- Source: SVN
- Link:
  https://ipodlinux.svn.sourceforge.net/svnroot/ipodlinux/apps/ipod/ipodloader2
- Date: Oct 31, 2008
- Version: Revision 2445

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

Changelog:
[keep untouched till beta release]