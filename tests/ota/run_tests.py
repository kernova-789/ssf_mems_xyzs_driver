#!/usr/bin/env python3
"""Exercise the production OTA transfer and loader against a flash/serial model."""
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
TEST = Path(__file__).resolve().parent


def main():
    with tempfile.TemporaryDirectory(prefix="ssf-mems-ota-") as temp:
        executable = Path(temp) / "ota_test"
        command = [os.environ.get("CC", "cc"), "-std=gnu11", "-Wall", "-Wextra",
                   "-Werror", "-Wno-sign-compare", "-fsanitize=address,undefined",
                   "-DSSF_TEST_REAL_MUTEX", "-g", "-pthread"]
        for path in (TEST / "shim", ROOT / "tests/protocol/shim",
                     ROOT / "core", ROOT / "protocol", ROOT / "acquisition",
                     ROOT / "iio"):
            command += ["-I" + str(path)]
        command += [str(ROOT / "protocol" / (name + ".c")) for name in
                    ("modbus", "modbus_request", "modbus_receive", "protocol",
                     "raw_stream", "ota")]
        command += [str(ROOT / "acquisition/ssf_mems_firmware.c"),
                    str(TEST / "ota_test.c"), "-o", str(executable)]
        subprocess.run(command, check=True)
        subprocess.run([str(executable), str(ROOT / "MEMS2.0/Objects/STM32F407IGT6.bin")],
                       check=True)


if __name__ == "__main__":
    main()
