# Install Loader 2

See `README.md`  for how to build `loader.bin`, which will be used in the installation process below.

### Prerequisites

You will need `ipodpatcher.exe` and `ipod_fw.exe`. These are available from the Rockbox website and ipodlinux.org on the wayback machine.

TODO: Building linux versions of these install tools

### Install steps

### THE EASY WAY with ipodpatcher:

#### Find the drive index of the ipod.
Run `ipodpatcher -l`
If ipodpatcher found an iPod, it will print something like "Ipod found - 2nd Generation Mini ("winpod") - disk device 2"
Remember the disk device number.

#### Backup the original boot partition
Run `ipodpatcher <device> -r original_boot_partition.bin`

where `<device>` is the iPod device number (eg. 2)

This backs up the firmware partition to `original_boot_partition.bin`

#### Easy installation

Patch in the bootloader directly onto the iPod:

Run `ipodpatcher <device> -ab loader.bin`

#### If anything goes wrong

Put the iPod into forced disk mode by first resetting it (Menu + Select for 6 seconds) and then immediately holding down the disk mode key combo (Play/Pause + Select) until disk mode appears.

Revert to firmware backup:

Run `ipodpatcher <device> -w original_boot_partition.bin`

### THE HARD MANUAL WAY with ipod_fw:

#### Extract the Apple Firmware blob from the boot  partition
Run `ipod_fw -o apple_os.bin -e 0 original_boot_partition.bin`

For 5g only on fw > 1.2: `ipod_fw -o apple_sw_5g_rcsc.bin -e 1 original_boot_partition.bin`

#### Create a new boot partition with both the Apple Firmware and Loader2
Run `ipod_fw -g <ipod model> -o new_boot_partition.bin -i apple_os.bin loader.bin`

`loader.bin` is the Loader2 binary.

`<ipod model>` is one of 1g, 2g, 3g, 4g, 5g, scroll, touch, dock, mini, photo, color, nano, video

#### Write the new boot partition back to the iPod

Run `ipodpatcher.exe <device> -w new_boot_partition.bin`

where `<device>` is the iPod device number (eg. 2)

#### Create ipodloader.conf

Create a file called `ipodloader.conf` in the root of the FAT32 partition.

Loader2 will load this configuration at boot. It contains the boot menu items.

Here is an example config:

```
# iPod Loader 2 configuration file
backlight = 1
timeout = 5
default = 1

# Menu choices:
Apple OS @ ramimg
Rockbox @ (hd0,1)/.rockbox/rockbox.ipod
iPod Linux @ (hd0,1)/linux.bin
iBoy @ (hd0,1)/iboy_kernel.bin /bin/iboy
Disk Mode @ diskmode
Sleep @ standby

```

#### Test

Eject the iPod and reboot it by holding down Menu + Select for 6 seconds.

The Loader 2 boot menu should show at boot and allow you to select an operating system.