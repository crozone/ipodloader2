# Rockbox installation

![Rockbox home menu](pics/rockbox.jpg)

## Overview

[Rockbox](https://www.rockbox.org/) is open source jukebox firmware. It is available for many PortalPlayer based devices, including classic iPods.

This guide assumes that iPodLoader2 is already installed.

## Installation

### Get Rockbox

Download the correct Rockbox Firmware archive from the [Rockbox website](https://www.rockbox.org/download/byhand.cgi).

Extract the archive.

### Install files

Copy the extracted `.rockbox` directory to the root of the iPod.

### Add `loader.cfg` entry

Ensure that the Rockbox boot menu entry is included in loader.cfg:

`Rockbox @ (hd0,1)/.rockbox/rockbox.ipod`

### Done

Eject the iPod and reboot.

Done!
