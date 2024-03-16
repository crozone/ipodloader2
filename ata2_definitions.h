#ifndef _ATA2_DEFINITIONS_H_
#define _ATA2_DEFINITIONS_H_

/* Logical blocks are always 512 bytes */
#define BLOCK_SIZE 512
/* Sectors are 512 bytes. Therefore there are 2 sectors per KB, and 2048 sectors per MB.*/
#define BLOCKS_PER_MB 2048

/*
 * ATA controller registers
 */

/* Data register
 * The data register is 16-bits wide, read/write.
 *
 * This register shall be accessed for host PIO data transfer only when DRQ is set to one and DMACK- is not
 * asserted. The contents of this register are not valid while a device is in the Sleep mode.
 *
 * PIO out data transfers are processed by a series of reads to this register, each read transferring the data that
 * follows the previous read. PIO in data transfers are processed by a series of writes to this register, each write
 * transferring the data that follows the previous write. The results of a read during a PIO in or a write during a PIO
 * out are indeterminate
*/
#define REG_DATA       0x0

/* Error register
 * Read-only.
 * 
 * The contents of this register shall be valid when BSY and DRQ equal zero and ERR equals one. The contents
 * of this register shall be valid upon completion of power-on, or after a hardware or software reset, or after
 * command completion of an EXECUTE DEVICE DIAGNOSTICS or DEVICE RESET command. The contents of
 * this register are not valid while a device is in the Sleep mode.
 *
 * This register contains status for the current command.
 * Following a power-on, a hardware or software reset (see 9.1), or command completion of an EXECUTE DEVICE
 * DIAGNOSTIC (see 8.12) or DEVICE RESET command (see 8.10), this register contains a diagnostic code .
 * At command completion of any command except EXECUTE DEVICE DIAGNOSTIC, the contents of this
 * register are valid when the ERR bit is set to one in the Status register.
 */
#define REG_ERROR      0x1

/* Features register
 * Write-only.
 *
 * This register shall be written only when BSY and DRQ equal zero and DMACK- is not asserted. If this register
 * is written when BSY or DRQ is set to one, the result is indeterminate.
 * 
 * The content of this register becomes a command parameter when the Command register is written.
*/
#define REG_FEATURES   0x1
#define REG_SECT_COUNT 0x2 // LBA28
#define REG_SECT       0x3 // LBA28
#define REG_CYL_LOW    0x4 // LBA28
#define REG_CYL_HIGH   0x5 // LBA28

/*
 * Device register (DEV).
 * Read/write.
 * Used for device select, and LBA select + HEAD during read/write commands.
 */
#define REG_DEVICEHEAD 0x6 // LBA28

/* Status register
 * Read-only.
 *
 * The contents of this register, except for BSY, shall be ignored when BSY is set to one. BSY is valid at all
 * times. The contents of this register are not valid while a device is in the Sleep mode.
 *
 * This register contains the device status. The contents of this register are updated to reflect the current state of
 * the device and the progress of any command being executed by the device.
 * 
 * Reading this register when an interrupt is pending causes the interrupt pending to be cleared.
 * The host should not read the Status register when an interrupt is expected as this may clear the interrupt pending
 * before the INTRQ can be recognized by the host.
*/
#define REG_STATUS     0x7

#define REG_COMMAND    0x7

/* Device Control register
 * Write-only.
 *
 * This register shall only be written when DMACK- is not asserted.
 * 
 * This register allows a host to software reset attached devices and to enable or disable the assertion of the
 * INTRQ signal by a selected device. When the Device Control register is written, both devices respond to the
 * write regardless of which device is selected. When the SRST bit is set to one, both devices shall perform the
 * software reset protocol. The device shall respond to the SRST bit when in the SLEEP mode.
 * 
 * - HOB (high order byte) is defined by the 48-bit Address feature set (see 6.20). A write to any Command
 *     Block register shall clear the HOB bit to zero.
 * - Bits 6 through 3 are reserved.
 * - SRST is the host software reset bit (see 9.2).
 * - nIEN is the enable bit for the device Assertion of INTRQ to the host. When the nIEN bit is cleared to
 *     zero, and the device is selected, INTRQ shall be enabled through a tri-state buffer and shall be
 *     asserted or negated by the device as appropriate. When the nIEN bit is set to one, or the device is not
 *     selected, the INTRQ signal shall be in a high impedance state.
 * - Bit 0 shall be cleared to zero.
*/
#define REG_CONTROL    0x8

/*
 * Read-only register.
 * This register contains the same information as the Status Register in the
 * command block.  The only difference being that reading this register does not
 * imply interrupt acknowledge or clear a pending interrupt.
 */
#define REG_ALTSTATUS  0x8

/*
 * LBA48 specific registers.
 */
#define REG_SECCOUNT_LOW  0x2 // Same as LBA28 REG_SECT_COUNT.
#define REG_LBA0          0x3 // Same as LBA28 REG_SECT.
#define REG_LBA1          0x4 // Same as LBA28 REG_CYL_LOW.
#define REG_LBA2          0x5 // Same as LBA28 REG_CYL_HIGH.
#define REG_SECCOUNT_HIGH 0xA
#define REG_LBA3          0xB
#define REG_LBA4          0xC
#define REG_LBA5          0xD

#define REG_DA          0x9


/*
 * Device Control register flags
 */

/* nIEN: Negated Interrupt Enable bit in Device Control register. (bit 1)
 * Sets device Assertion of INTRQ to the host.
 * When the nIEN bit is cleared to zero, and the device is selected, INTRQ shall be enabled
 * When the nIEN bit is set to one, or the device is not selected, the INTRQ signal is disabled
 */
#define CONTROL_NIEN    0x02

/* SRST: Software Reset bit in Device Control register */
#define CONTROL_SRST    0x04

/* HOB: High Order Byte (bit 7). Defined by the 48-bit Address feature set (see 6.20).
 * A write to any Command Block register shall clear the HOB bit to zero.*/
#define CONTROL_HOB     0x80

// all commands: see include/linux/hdreg.h

/* IDENTIFY DEVICE (Identify)
 *
 */
#define COMMAND_IDENTIFY_DEVICE       0xEC

/* READ SECTOR(S) (RdSec) - 20h, PIO Data-In. LBA28.
 *
 * This command reads from 1 to 256 sectors as specified in the Sector Count register. A sector count of 0
 * requests 256 sectors. The transfer shall begin at the sector specified in the LBA Low, LBA Mid, LBA High, and
 * Device registers.
 * The DRQ bit is always set to one prior to data transfer regardless of the presence or absence of an error
 * condition.
 */
#define COMMAND_READ_SECTORS          0x20

/* READ SECTOR(S) EXT (RdSecEx) - 24h, PIO Data-In. LBA48.
 *
 * This command reads from 1 to 65,536 sectors as specified in the Sector Count register. A sector count of
 * 0000h requests 65,536 sectors. The transfer shall begin at the sector specified in the LBA Low, LBA Mid, and
 * LBA High registers.
 * The DRQ bit is always set to one prior to data transfer regardless of the presence or absence of an error
 * condition.
 */
#define COMMAND_READ_SECTORS_EXT      0x24

/* STANDBY IMMEDIATE (StandbyIm)
 *
 * This command causes the device to immediately enter the Standby mode.
*/
#define COMMAND_STANDBY               0xE0

/* SLEEP E6h
 *
 * This command is the only way to cause the device to enter Sleep mode.
 * 
 * This command causes the device to set the BSY bit to one, prepare to enter Sleep mode, clear the BSY bit to
 * zero and assert INTRQ. The host shall read the Status register in order to clear the interrupt pending and allow
 * the device to enter Sleep mode. In Sleep mode, the device only responds to the assertion of the RESET- signal
 * and the writing of the SRST bit in the Device Control register and releases the device driven signal lines. The
 * host shall not attempt to access the Command Block registers while the device is in Sleep mode.
 * 
 * Because some host systems may not read the Status register and clear the interrupt pending, a device may
 * automatically release INTRQ and enter Sleep mode after a vendor specific time period of not less than 2 s.
 * 
 * The only way to recover from Sleep mode is with a software reset, a hardware reset, or a DEVICE RESET
 * command.
 * 
 * A device shall not power-on in Sleep mode nor remain in Sleep mode following a reset sequence
*/
#define COMMAND_SLEEP                 0xE6

#define DEVICE_0       0x00
#define DEVICE_1       0x10

#define CHS_ADDRESSING 0x00
#define LBA_ADDRESSING 0x40

/* BSY (Busy)
 *
 * BSY indicates that the device is handling a command.
 * 
 * In practice, what this means is that the device still has control
 * over the Command Block registers.
 * The host should not write to the Command Block registers while BSY is asserted,
 * with the exception of sending a DEVICE RESET command.
 * 
 * While BSY is not asserted, the device will never set DRQ, or change ERR,
 * or change any other Command Block register.
 * The device will always assert BSY first, then set DRQ, ERR, or other Command Block registers,
 * and then deassert BSY again.
*/
#define STATUS_BSY     0x80
#define STATUS_DRDY    0x40
#define STATUS_DF      0x20
#define STATUS_DSC     0x10

/* DRQ (Data request)
 *
 * DRQ indicates that the device is ready to transfer a word of data between the host and the device. After the
 * host has written the Command register the device shall either set the BSY bit to one or the DRQ bit to one,
 * until command completion or the device has performed a bus release for an overlapped command.
*/
#define STATUS_DRQ     0x08
#define STATUS_CORR    0x04
#define STATUS_IDX     0x02
#define STATUS_ERR     0x01

/* ABRT (command aborted) is set to one to indicate the requested command has been command
 * aborted because the command code or a command parameter is invalid, the command is not
 * supported, a prerequisite for the command has not been met, or some other error has occurred
 */
#define ERROR_ABRT 0x04

/* There are 8 total error bits that can be set, but besides ABRT, they are all command dependant.*/

#endif
