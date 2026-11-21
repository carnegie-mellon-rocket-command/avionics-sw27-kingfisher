# NOR flash files and ST-LINK export

The W25Q128 stores numbered `F000001.CSV` files using littlefs. Each successful
append includes a synchronization, with bounded SPI operations and program/erase
verification. CSV columns identify integer milli-units: divide those values by
1000 for the named base unit. `time_ms` and `flags` are not scaled.

The littlefs sources came from the Kingfisher dev branch (`6c7dc87`). Its hardware
wrapper was replaced to use PA3 chip select, propagate errors, and avoid formatting
automatically on mount failure. The final 4 KB NOR sector holds a separate flight
latch and is excluded from the filesystem.

## Normal operation

Keep `ERASE_ON_BOOT=0`. `DEBUG=1` enables SWO diagnostics and prints existing files
at startup, then flight initialization continues. `DEBUG=0` disables SWO transport.
Large startup dumps delay initialization. The independent ST-LINK exporter works
with either DEBUG setting. There is no DUMP_MODE or USB_MSC_MODE.

USB device communication has been removed. CPU startup uses 16 MHz HSI and does
not depend on the USB clock, external crystal, or board USB-C data connections.
Logging and ST-LINK file requests run serially in the main loop. File export adds
work to that loop; perform large downloads on the bench or after landing.

## Copy files to a Mac

1. Build and flash the updated firmware using CubeIDE/ST-LINK, then resume it.
2. Terminate the CubeIDE debug session so the script can use ST-LINK. Keep the
   board powered and running. A halted CPU cannot service file requests.
3. From the repository root, run:

   ```sh
   python3 tools/read_logs.py flight-logs
   ```

The script finds CubeProgrammer and `arm-none-eabi-nm` inside the installed
STM32CubeIDE. Optional `--programmer`, `--nm`, and `--elf` arguments override paths.
The ELF must match the flashed firmware: the script uses its `stlink_mailbox`
symbol to locate the RAM interface and checks its magic/version before writing.

Files appear in the destination folder as ordinary CSV files. This is not a Finder
volume and does not work directly from iPhone. No USB-C data connection is needed;
only ST-LINK's SWD connection, target power, and a running CPU are required.

The script connects in HOTPLUG mode. It writes only a checked SRAM mailbox to
request file reads; it does not reset, flash, erase, or request file changes. Each
file's size is captured before copying, so a currently growing file exports the
committed prefix present when copying began. Existing destination files are never
overwritten. Interrupted exports can leave partial copies; use a new destination
directory for a retry. Do not run multiple exporters/debuggers against one board.

## Initialization and reset recovery

Raw logs from the earlier logger are not a littlefs volume. Mount failure faults
without erasing data. To intentionally initialize storage, use `ERASE_ON_BOOT=1`
for one bench boot, then restore 0 and reflash. Formatting destroys existing files.

After launch, a reset/power loss before successful completion causes the next boot
to command retraction and latch fault before the servo self-test. Rebooting does
not clear that latch. The ST-LINK exporter remains serviced in the fault loop and
can mount/read saved files after retraction, provided flash remains readable.
After physical recovery and disarming, call `flashlog_clear_flight_latch()` through
the debugger, check its true return value, then reset. A fault-free landing marks
completion; the next boot clears that marker while preserving files.

## Tests and limitations

```sh
cc -std=c11 -Wall -Wextra -include tests/hal_stub.h -ICore/Inc \
  Core/Src/flashlog.c Core/Src/lfs.c Core/Src/lfs_util.c Core/Src/stlink_export.c \
  tests/flashlog_test.c -o /tmp/kingfisher-flashlog-test
/tmp/kingfisher-flashlog-test
python3 -m unittest discover -s tests -p 'test_*.py'
```

Tests cover simulated NOR behavior, formatting, reset-latch persistence, CSV
recovery, corruption detection, SRAM mailbox requests during logging, chunked host
export, and overwrite refusal. The initial corrupt-directory message is expected
when the simulated blank chip is mounted before explicit formatting.

Per-row synchronization, flash erases, and export requests can exceed the configured
loop period. Hardware power-cut tests and flight-loop latency measurements remain
necessary. Acceleration and controller behavior were left unchanged as requested.
