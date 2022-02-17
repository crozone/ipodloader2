# Building iPodLoader2

Instructions for building iPodLoader2 from source code.

## Installation of toolchain (arm cross-compiler)

Debian Linux x86_64 11 (Bullseye) is used as the reference distro, although the install commands will work on Ubuntu and are easily adapted for other distributions.

### Install `make`:

`sudo apt install make`

Or optionally, you can instead install the full `build-essential` metapackage which also includes other useful tools like gcc:

`sudo apt install build-essential`

### Install the `arm-none-eabi` cross compiler:

`sudo apt install gcc-arm-none-eabi`

## Building the source

In the root directory of the repository, run

`make`

If the build went smoothly, an output file called `loader.bin` should have been output in the root of the project.

If you want to rebuild the solution, simply run `make clean` and then run `make` again.

### Example build output

```
$ make
VERSION is 2.8.1
Compiling startup.s
Compiling loader.c
Compiling fb.c
Compiling ipodhw.c
Compiling console.c
Compiling minilibc.c
Compiling ata2.c
Compiling vfs.c
Compiling fat32.c
Compiling ext2.c
Compiling fwfs.c
Compiling keypad.c
Compiling menu.c
Compiling config.c
Compiling macpartitions.cc
Compiling interrupts.c
Compiling interrupt-entry.s
Linking loader.elf
Converting loader.elf to binary
```
