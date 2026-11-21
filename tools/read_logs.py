#!/usr/bin/env python3
"""Copy littlefs CSV files through ST-LINK without resetting or erasing the MCU."""
import argparse
import glob
from pathlib import Path
import re
import shutil
import struct
import subprocess
import tempfile
import time

MAGIC = 0x4B464C47
CHUNK = 256  # 4096-byte firmware reads fail on the board; 256 works

def locate(name, pattern):
    result = shutil.which(name)
    matches = sorted(glob.glob(pattern))
    if result or matches:
        return result or matches[-1]
    raise RuntimeError(f"Cannot find {name}; install STM32CubeIDE/CubeProgrammer or supply its path")

class Link:
    def __init__(self, cli, address):
        self.cli, self.address = cli, address
        self.temp = tempfile.TemporaryDirectory(prefix="kingfisher-stlink-")

    def run(self, *args):
        result = subprocess.run([self.cli, "-c", "port=SWD", "mode=HOTPLUG", "freq=1000", *args],
                                text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=30)
        if "exclusive access" in result.stdout:
            raise RuntimeError("ST-LINK is in use. Terminate the CubeIDE debug session, keep the board powered/running, and retry")
        if result.returncode or "Error:" in result.stdout:
            raise RuntimeError(result.stdout[-2500:])

    def read(self, address, length):
        path = Path(self.temp.name) / "ram.bin"
        if path.exists():
            path.unlink()
        self.run("-u", hex(address), str(length), str(path))
        data = path.read_bytes()
        if len(data) != length:
            raise RuntimeError("ST-LINK returned an incomplete RAM read")
        return data

    def write(self, address, *values):
        self.run("-w32", hex(address), *(hex(v) for v in values))

    def request(self, command, index, offset=0, length=0):
        # Request fields first; command is the doorbell and must be written last.
        self.write(self.address + 16, index, offset, length)
        self.write(self.address + 12, 0)
        self.write(self.address + 8, command)
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            header = self.read(self.address, 48)
            fields = struct.unpack("<8I16s", header)
            if fields[0:2] != (MAGIC, 1):
                raise RuntimeError("Firmware reset or mailbox changed during export")
            if fields[2] == 0 and fields[3]:
                if fields[3] == 3:
                    raise RuntimeError("Firmware could not read the requested file; check flash/mount status")
                return fields
            time.sleep(0.05)
        raise RuntimeError("Firmware did not service the request. Resume the CPU and stop other ST-LINK sessions")

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", nargs="?", default="flight-logs", type=Path)
    parser.add_argument("--elf", type=Path, default=Path(__file__).resolve().parents[1] / "Debug/kingfisher_final.elf")
    parser.add_argument("--programmer")
    parser.add_argument("--nm")
    args = parser.parse_args()
    root = "/Applications/STMicroelectronics/STM32CubeIDE.app/Contents/Eclipse/plugins/"
    cli = args.programmer or locate("STM32_Programmer_CLI", root + "*/tools/bin/STM32_Programmer_CLI")
    nm = args.nm or locate("arm-none-eabi-nm", root + "*/tools/bin/arm-none-eabi-nm")
    symbols = subprocess.check_output([nm, "-n", str(args.elf)], text=True)
    match = re.search(r"^([0-9a-fA-F]+)\s+\w\s+stlink_mailbox$", symbols, re.MULTILINE)
    if not match:
        raise RuntimeError("ELF has no ST-LINK exporter. Build and flash the updated firmware first")
    address = int(match[1], 16)
    if not 0x20000000 <= address <= 0x20050000 - 48 - CHUNK:
        raise RuntimeError("Mailbox address is outside STM32F413 SRAM")
    link = Link(cli, address)
    try:
        if struct.unpack("<2I", link.read(address, 8)) != (MAGIC, 1):
            raise RuntimeError("Running firmware does not match this ELF, or startup has not reached ats_init")
        args.output.mkdir(parents=True, exist_ok=True)
        index = 0
        while True:
            info = link.request(1, index)
            if info[3] == 2:
                break
            name = info[8].split(b"\0", 1)[0].decode("ascii")
            if not re.fullmatch(r"F\d{6}\.CSV", name):
                raise RuntimeError(f"Unexpected firmware filename: {name!r}")
            size = info[7]
            path = args.output / name
            # Never overwrite an earlier recovery copy; a failed copy leaves only a .part file.
            if path.exists():
                raise RuntimeError(f"{path} already exists; choose a new output directory")
            part = path.with_name(name + ".part")
            part.unlink(missing_ok=True)
            with part.open("xb") as stream:
                for offset in range(0, size, CHUNK):
                    count = min(CHUNK, size-offset)
                    chunk = link.request(2, index, offset, count)
                    if chunk[3] != 1 or chunk[8] != info[8] or chunk[7] < size:
                        raise RuntimeError("File list changed during export; discard the partial copy and retry")
                    stream.write(link.read(address + 48, count))
            part.rename(path)
            print(f"Saved {path} ({size} bytes)")
            index += 1
        print(f"Copied {index} file(s). Target flash was not erased or modified by the exporter.")
    finally:
        link.temp.cleanup()

if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, OSError, subprocess.SubprocessError) as error:
        raise SystemExit(str(error))
