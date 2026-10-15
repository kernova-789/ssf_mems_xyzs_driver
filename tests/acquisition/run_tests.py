#!/usr/bin/env python3
"""Run production acquisition code with a deterministic clock/serial harness."""
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
TEST = Path(__file__).resolve().parent


def main():
    with tempfile.TemporaryDirectory(prefix="ssf-mems-acquisition-") as temp:
        executable = Path(temp) / "acquisition_test"
        command = [os.environ.get("CC", "cc"), "-std=gnu11", "-Wall", "-Wextra",
                   "-Werror", "-Wno-sign-compare", "-fsanitize=address,undefined",
                   "-g", "-I" + str(TEST / "shim"),
                   "-I" + str(ROOT / "tests/protocol/shim")]
        command += ["-I" + str(ROOT / path)
                    for path in ("acquisition", "core", "protocol", "iio")]
        command += [str(ROOT / "protocol/modbus_table.c"),
                    str(ROOT / "protocol/serial_transport.c"),
                    str(ROOT / "acquisition/acquisition_policy.c"),
                    str(ROOT / "acquisition/ssf_mems_acquisition.c"),
                    str(TEST / "acquisition_test.c"), "-o", str(executable)]
        subprocess.run(command, check=True)
        subprocess.run([str(executable)], check=True)
        status_executable = Path(temp) / "status_test"
        status_command = command[:command.index(str(ROOT / "protocol/modbus_table.c"))]
        status_command += ["-DSSF_TEST_REAL_MUTEX", "-pthread", "-ffunction-sections",
                           "-fdata-sections", "-Wl,--gc-sections",
                           str(ROOT / "acquisition/ssf_mems_acquisition.c"),
                           str(TEST / "status_test.c"), "-o", str(status_executable)]
        subprocess.run(status_command, check=True)
        subprocess.run([str(status_executable)], check=True, timeout=5)


if __name__ == "__main__":
    main()
