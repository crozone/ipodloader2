# makefile for iPodLoader2
#
# Note by TT: the option "-mstructure-size-boundary=8" is necessary when compiling macpartitions.cc for the structs to get the correct sizes!

# Get the git hash, if the working directory is clean
GIT_SHELL_EXIT := $(shell git status --porcelain 2> /dev/null >&2 ; echo $$?)
# Return code can be non-zero when not in git repository or git is not installed.
# It can happen when downloaded using github's "Download ZIP" option.
ifeq ($(GIT_SHELL_EXIT),0)
# Check if working dir is clean.
GIT_STATUS := $(shell git status --porcelain)
ifndef GIT_STATUS
GIT_COMMIT_HASH := $(shell git rev-parse --short HEAD)
endif
endif

# Suffix with "d" for development version, "b" for beta version
VERSION   = 2.8

ifdef GIT_COMMIT_HASH
	VERSION   := "$(VERSION) $(GIT_COMMIT_HASH)"
endif

$(info    VERSION is $(VERSION))

CROSS    ?= arm-uclinux-elf-
CC        = $(CROSS)gcc
LD        = $(CROSS)ld
MYCFLAGS  = -Os -Wall -std=gnu99 -ffreestanding -nostdinc -fomit-frame-pointer -DVERSION=\"$(VERSION)\"
# -DDEBUG
MYCPPFLAGS= -Os -Wall -nostdinc -fomit-frame-pointer -mstructure-size-boundary=8
MYLDFLAGS = -Tarm_elf_40.x `$(CC) -print-libgcc-file-name`
OBJCOPY   = $(CROSS)objcopy

OBJFILES = startup.o loader.o fb.o ipodhw.o console.o minilibc.o ata2.o vfs.o fat32.o ext2.o fwfs.o keypad.o menu.o config.o macpartitions.o interrupts.o interrupt-entry.o

all: loader.bin $(OBJFILES) Makefile
#	@echo "Building firmware image"
#	@./make_fw -g 4g -o my_sw.bin -i apple_os.bin $<

clean:
	@echo "Cleaning up"
	@rm -f *.o *~ loader.bin loader.elf nohup.out my_sw.bin $(OBJFILES)

loader.bin: loader.elf
	@echo "Converting $< to binary"
	@$(OBJCOPY) -O binary $< $@

loader.elf: $(OBJFILES)
	@echo "Linking $@"
	@$(LD) -o $@ $^ $(MYLDFLAGS)

%.o:%.cc
	@echo "Compiling $<"
	@$(CC) $(MYCPPFLAGS) $(CFLAGS) -c $< -o $@

%.o:%.c
	@echo "Compiling $<"
	@$(CC) $(MYCFLAGS) $(CFLAGS) -c $< -o $@

%.o:%.s
	@echo "Compiling $<"
	@$(CC) $(MYCFLAGS) $(CFLAGS) -c $< -o $@
