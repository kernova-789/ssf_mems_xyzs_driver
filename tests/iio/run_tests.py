#!/usr/bin/env python3
"""Exercise the production IIO publisher against concurrent buffer changes."""
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
TEST_DIR = Path(__file__).resolve().parent

def main():
    with tempfile.TemporaryDirectory(prefix="ssf-mems-iio-tests-") as temp:
        executable = Path(temp) / "iio_test"
        flags = [os.environ.get("CC", "cc"), "-std=gnu11", "-Wall", "-Wextra", "-Werror",
                 "-Wno-unused-parameter", "-Wno-sign-compare", "-DSSF_TEST_REAL_MUTEX",
                 "-fsanitize=address,undefined", "-g", "-pthread", "-ffunction-sections",
                 "-fdata-sections", "-Wl,--gc-sections"]
        for directory in (TEST_DIR / "shim", ROOT / "tests/protocol/shim",
                          ROOT / "iio", ROOT / "core", ROOT / "protocol", ROOT / "acquisition"):
            flags += ["-I" + str(directory)]
        subprocess.run(flags + [str(TEST_DIR / "iio_test.c"), "-o", str(executable)], check=True)
        subprocess.run([str(executable)], check=True)

if __name__ == "__main__":
    main()
