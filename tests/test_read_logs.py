"""Exercise the host protocol and file output without an ST-LINK device."""
import importlib.util
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location("read_logs", Path(__file__).parents[1] / "tools/read_logs.py")
reader = importlib.util.module_from_spec(spec)
spec.loader.exec_module(reader)
PAYLOAD = b"time_ms,value\r\n" + b"42,1234\r\n" * 1200

class FakeLink(reader.Link):
    def __init__(self, cli, address):
        self.address = address
        self.ram = bytearray(48 + reader.CHUNK)
        struct.pack_into("<2I", self.ram, 0, reader.MAGIC, 1)
        self.temp = type("Temporary", (), {"cleanup": lambda self: None})()

    def read(self, address, length):
        start = address - self.address
        return bytes(self.ram[start:start+length])

    def write(self, address, *values):
        start = address - self.address
        struct.pack_into("<" + "I" * len(values), self.ram, start, *values)
        if start != 8:
            return
        command, status, index, offset, length = struct.unpack_from("<5I", self.ram, 8)
        assert status == 0, "host must clear old response before publishing command"
        status = 2 if index else 1
        if not index:
            self.ram[32:48] = b"F000001.CSV".ljust(16, b"\0")
            struct.pack_into("<I", self.ram, 28, len(PAYLOAD))
            if command == 2:
                assert 0 <= offset <= len(PAYLOAD) and length <= reader.CHUNK
                self.ram[48:48+length] = PAYLOAD[offset:offset+length]
        struct.pack_into("<2I", self.ram, 8, 0, status)

class ExportTest(unittest.TestCase):
    def test_export_multiple_chunks_and_refuse_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            with patch.object(reader, "Link", FakeLink), patch.object(reader, "locate", return_value="unused"), \
                 patch.object(reader.subprocess, "check_output", return_value="20001000 B stlink_mailbox\n"), \
                 patch.object(sys, "argv", ["read_logs.py", directory]):
                reader.main()
                self.assertEqual((Path(directory) / "F000001.CSV").read_bytes(), PAYLOAD)
                with self.assertRaises(FileExistsError):
                    reader.main()

if __name__ == "__main__":
    unittest.main()
