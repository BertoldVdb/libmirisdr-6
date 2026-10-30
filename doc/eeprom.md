Firmware in the EEPROM
======================

Most receivers built on the MSi2500 keep a small EEPROM next to it. The chip reads it when it starts. It can hold the USB ids the chip shows, or a whole firmware image to run instead of the one in its ROM.

You do not need to change anything there to use the library. A receiver that starts from its ROM gets the library's firmware loaded into it each time it is opened. This page is for when you want the firmware in the EEPROM, so that the receiver runs it from the moment it is plugged in.


<h3>Why put the firmware there</h3>

The ROM does not report a serial. Until a program has opened the receiver once, it is listed with only its place on the USB, such as `usb_port=1:3` (see "Receivers without firmware in their EEPROM" in the [SoapySDR guide](soapysdr.md)). With the firmware in the EEPROM the receiver has its serial from the start, so programs find the same receiver again in any port.

The receiver needs an EEPROM for this. The tool to use is `miri_eeprom`, and its manual page (`man miri_eeprom`) lists all its commands.


<h3>Make a backup first</h3>

Writing the firmware replaces what the maker put in the EEPROM. Without a copy of the old contents it may not be easy to go back to the vendor software. Unplug the receiver and plug it in again, so it runs from its ROM, and read the EEPROM before any program opens it:

    miri_eeprom saferead backup.bin

`saferead` only reads the EEPROM. It does not load the library's firmware and does nothing else with the receiver. Keep `backup.bin` somewhere safe.

Every command that writes also saves the whole EEPROM first, to `miri_eeprom-backup-<serial or ids>-<time>.bin` in the current directory, and writes nothing if that fails. Keep the `saferead` backup all the same.

With more than one receiver plugged in, pick one with `-d` and its index, for example `miri_eeprom -d 1 saferead backup.bin`.


<h3>Writing the firmware</h3>

There are two ways, and they differ in the USB ids the receiver shows afterwards.

**Keep the receiver's own ids.** With no options, the firmware takes the USB ids and the serial the receiver has now. On an SDRplay board that is its SDRplay VID:PID and the serial from its EEPROM:

    miri_eeprom writefw default

Software that knows the board by its ids still takes it for that board, the vendor's included, but the board now runs other firmware.

**Use libmirisdr-6's own ids, 16d0:158c.** Give a serial with `-s` and the firmware shows up under these ids instead, which other software does not claim. They do not say which board it is, so on an SDRplay board the library tells it from the first two characters of the serial. The serial must be the board's letter and a dash, followed by its own 10 character SDRplay serial:

| Board | Serial starts with |
|---|---|
| RSP1B | `B-` |
| RSPdx-R2 | `C-` |
| RSPduo | `D-` |
| RSP2 | `E-` |
| nRSP-ST | `F-` |
| RSPdx | `G-` |

For example, on an RSP1B:

    miri_eeprom writefw default -s B-1234567890

Other boards take any serial of up to 12 characters. `random` makes one up:

    miri_eeprom writefw default -s random

`-v` and `-p` set other ids. `miri_eeprom` refuses ids the library would not find again, unless you add `--force`.

On a self powered board, the nRSP-ST, the firmware also says it is self powered, either way.


<h3>After writing</h3>

Unplug the receiver and plug it in again, so it starts from the new firmware. If its USB ids changed, udev may need to run its rules again before a user can open it. The rules that come with the library (`mirisdr.rules`) cover both the SDRplay ids and 16d0:158c.


<h3>Going back</h3>

To put the original contents back:

    miri_eeprom write backup.bin

`miri_eeprom disable` only sets the first byte back to 0xFF, so the chip starts from its ROM. It then shows the chip's default ids, 1df7:2500, and not the board's, so the library takes an SDRplay board for a plain one. Write the backup to have the board back as it was.
