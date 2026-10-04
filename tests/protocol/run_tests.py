#!/usr/bin/env python3
"""Compile real protocol sources against a single-threaded Linux/serial shim."""
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
TEST_DIR = Path(__file__).resolve().parent
VARIANTS = {
    "baseline": set(),
    "delete_first": {40001},
    "delete_middle": {40002},
    "delete_last": {40029},
    "delete_multiple": {40001, 40002, 40007, 40015, 40028, 40029},
    "delete_all_features": set(range(40001, 40030)),
    "delete_config": {40050},
    "delete_work_parameter": {40064},
    "reverse_table": set(),
}

def configure_table(text, deleted, reverse):
    marker = "static const struct ssf_reg_desc ssf_reg_table[] = {"
    start = text.index(marker) + len(marker)
    end = text.index("\n};", start)
    entries = re.findall(r"    \{\n.*?\n    \},", text[start:end], re.S)
    kept = []
    for entry in entries:
        address = int(re.search(r"\.display_reg = (\d+)", entry).group(1))
        if address not in deleted:
            kept.append(entry)
    if reverse:
        kept.reverse()
    return text[:start] + "\n" + "\n".join(kept) + text[end:]

def main():
    compiler = os.environ.get("CC", "cc")
    with tempfile.TemporaryDirectory(prefix="ssf-mems-protocol-tests-") as temp:
        scratch = Path(temp)
        for name, deleted in VARIANTS.items():
            project = scratch / name
            shutil.copytree(ROOT / "protocol", project / "protocol")
            table_path = project / "protocol" / "modbus_table.h"
            # Mechanical test-fixture transformation; production table is never edited.
            table_path.write_text(configure_table(table_path.read_text(), deleted, name == "reverse_table"))
            executable = project / "protocol_test"
            command = [
                compiler, "-std=gnu11", "-Wall", "-Wextra", "-Werror",
                "-Wno-sign-compare", "-fsanitize=address,undefined", "-g",
                "-I" + str(TEST_DIR / "shim"), "-I" + str(project / "protocol"),
                str(TEST_DIR / "protocol_test.c"), "-o", str(executable),
            ]
            subprocess.run(command, check=True)
            subprocess.run([str(executable), name], check=True)
    print("All table-removal and protocol cases passed.")

if __name__ == "__main__":
    main()
