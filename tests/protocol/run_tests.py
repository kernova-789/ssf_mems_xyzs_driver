#!/usr/bin/env python3
"""Compile modules independently and test them against a Linux/serial shim."""
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
MODULES = ("modbus", "modbus_request", "modbus_receive", "protocol")
HEADERS = (
    "sensor_data.h", "modbus_types.h", "modbus.h", "modbus_table.h",
    "modbus_request.h", "modbus_receive.h", "protocol.h", "core.h",
    "ssf_mems_acquisition.h", "acquisition_policy.h",
)
ALLOWED_DEPENDENCIES = {
    "modbus": set(),
    "modbus_request": {"modbus"},
    "modbus_receive": {"modbus"},
    "protocol": {"modbus", "modbus_request", "modbus_receive"},
}

def check_module_dependencies(objects):
    nm = os.environ.get("NM", "nm")
    owners = {}
    for module, obj in objects.items():
        output = subprocess.check_output([nm, "-g", "--defined-only", str(obj)], text=True)
        for line in output.splitlines():
            fields = line.split()
            if fields and fields[-1].startswith("ssf_"):
                owners[fields[-1]] = module
    for module, obj in objects.items():
        output = subprocess.check_output([nm, "-u", str(obj)], text=True)
        for line in output.splitlines():
            symbol = line.split()[-1]
            if not symbol.startswith("ssf_"):
                continue
            owner = owners.get(symbol)
            if owner not in ALLOWED_DEPENDENCIES[module]:
                raise AssertionError(f"Invalid dependency: {module} -> {symbol} ({owner})")

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
            shutil.copytree(ROOT / "core", project / "core")
            shutil.copytree(ROOT / "acquisition", project / "acquisition")
            table_path = project / "protocol" / "modbus_table.h"
            # Mechanical test-fixture transformation; production table is never edited.
            table_path.write_text(configure_table(table_path.read_text(), deleted, name == "reverse_table"))
            executable = project / "protocol_test"
            flags = [
                compiler, "-std=gnu11", "-Wall", "-Wextra", "-Werror",
                "-Wno-sign-compare", "-fsanitize=address,undefined", "-g",
                "-I" + str(TEST_DIR / "shim"), "-I" + str(project / "protocol"),
                "-I" + str(project / "core"),
                "-I" + str(project / "acquisition"),
            ]
            # Headers must provide their own prerequisites, not rely on include order.
            for header in HEADERS:
                subprocess.run(flags + ["-include", header, "-x", "c", "-fsyntax-only", "/dev/null"], check=True)
            objects = {}
            for module in MODULES:
                obj = project / (module + ".o")
                dependency_file = project / (module + ".d")
                subprocess.run(flags + ["-MMD", "-MF", str(dependency_file), "-c",
                                       str(project / "protocol" / (module + ".c")), "-o", str(obj)], check=True)
                objects[module] = obj
                if module == "modbus":
                    includes = set(re.findall(r"[\w]+\.h", dependency_file.read_text()))
                    forbidden = {"core.h", "protocol.h", "modbus_request.h", "modbus_receive.h"}
                    if includes & forbidden:
                        raise AssertionError(f"Codec includes upper-layer state: {includes & forbidden}")
            check_module_dependencies(objects)
            harness = project / "protocol_test.o"
            subprocess.run(flags + ["-c", str(TEST_DIR / "protocol_test.c"), "-o", str(harness)], check=True)
            subprocess.run(flags + [str(harness)] + [str(obj) for obj in objects.values()] +
                           ["-o", str(executable)], check=True)
            subprocess.run([str(executable), name], check=True)
    print("All independent-compilation, dependency-boundary, table-removal and protocol cases passed.")

if __name__ == "__main__":
    main()
