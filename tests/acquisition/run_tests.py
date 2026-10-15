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
        command += [str(ROOT / "protocol/model/modbus_table.c"),
                    str(ROOT / "protocol/transport/serial_transport.c"),
                    str(ROOT / "acquisition/acquisition_policy.c"),
                    str(ROOT / "acquisition/ssf_mems_acquisition.c"),
                    str(TEST / "acquisition_test.c"), "-o", str(executable)]
        subprocess.run(command, check=True)
        subprocess.run([str(executable)], check=True)
        status_executable = Path(temp) / "status_test"
        status_command = command[:command.index(str(ROOT / "protocol/model/modbus_table.c"))]
        status_command += ["-DSSF_TEST_REAL_MUTEX", "-pthread", "-ffunction-sections",
                           "-fdata-sections", "-Wl,--gc-sections",
                           str(ROOT / "acquisition/ssf_mems_acquisition.c"),
                           str(TEST / "status_test.c"), "-o", str(status_executable)]
        subprocess.run(status_command, check=True)
        subprocess.run([str(status_executable)], check=True, timeout=5)
        profile_directory = Path(temp) / "alternate-profile"
        (profile_directory / "model").mkdir(parents=True)
        table_header = (ROOT / "protocol/model/modbus_table.h").read_text()
        table_header = table_header.replace(
            ".default_work_baudrate = SSF_MEMS_BAUDRATE_128000",
            ".default_work_baudrate = SSF_MEMS_BAUDRATE_115200")
        (profile_directory / "model/modbus_table.h").write_text(table_header)
        (profile_directory / "model/modbus_table.c").write_text(
            (ROOT / "protocol/model/modbus_table.c").read_text())
        profile_executable = Path(temp) / "profile_test"
        profile_command = command[:command.index(str(ROOT / "protocol/model/modbus_table.c"))]
        profile_command.insert(1, "-I" + str(profile_directory))
        profile_command += ["-ffunction-sections", "-fdata-sections", "-Wl,--gc-sections",
                            str(profile_directory / "model/modbus_table.c"),
                            str(ROOT / "protocol/protocol.c"),
                            str(ROOT / "acquisition/acquisition_policy.c"),
                            str(ROOT / "acquisition/ssf_mems_acquisition.c"),
                            str(TEST / "profile_test.c"), "-o", str(profile_executable)]
        subprocess.run(profile_command, check=True)
        subprocess.run([str(profile_executable)], check=True)


if __name__ == "__main__":
    main()
