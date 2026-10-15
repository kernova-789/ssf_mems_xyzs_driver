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
        command += [str(ROOT / "protocol" / source) for source in
                    ("model/modbus_table.c", "transport/serial_transport.c", "raw/raw_capture.c",
                     "modbus/modbus.c", "modbus/modbus_request.c", "modbus/modbus_receive.c",
                     "protocol.c", "raw/raw_stream.c", "firmware/ota.c")]
        command += [str(ROOT / "acquisition/ssf_mems_firmware.c"),
                    str(TEST / "ota_test.c"), "-o", str(executable)]
        subprocess.run(command, check=True)
        subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
