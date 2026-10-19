#!/usr/bin/env python3
"""Compile modules independently and test them against a Linux/serial shim."""
import os
from pathlib import Path
import re
import shutil
import sys
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
TEST_DIR = Path(__file__).resolve().parent
VARIANTS = {
    "baseline": set(),
    "xyz": set(),
    "delete_first": {40001},
    "delete_middle": {40002},
    "delete_last": {40029},
    "delete_multiple": {40001, 40002, 40007, 40015, 40028, 40029},
    "delete_all_features": set(range(40001, 40030)),
    "delete_config": {40050},
    "delete_work_parameter": {40064},
    "delete_save_register": {40110},
    "delete_finite_start": {40060},
    "reverse_table": set(),
    "feature_scale_x10": set(),
    "feature_switch_layout": set(),
    "short_baud_table": set(),
    "relocate_registers": set(),
    "nonadjacent_length_display": set(),
    "nonadjacent_length_protocol": set(),
}
MODULES = {
    "modbus_table": "model/modbus_table.c",
    "sensor": "../sensor/sensor.c",
    "features": "../sensor/features.c",
    "xyzs": "../sensor/xyzs.c",
    "xyz": "../sensor/xyz.c",
    "serial_transport": "transport/serial_transport.c",
    "raw_capture": "raw/raw_capture.c",
    "modbus": "modbus/modbus.c",
    "modbus_request": "modbus/modbus_request.c",
    "modbus_receive": "modbus/modbus_receive.c",
    "protocol": "protocol.c",
    "raw_stream": "raw/raw_stream.c",
    "ota": "firmware/ota.c",
}
HEADERS = (
    "sensor_data.h", "sensor_fields.h", "sensor.h", "xyz.h", "xyzs.h", "modbus/modbus_types.h",
    "modbus/modbus.h", "model/modbus_table.h", "modbus/modbus_request.h",
    "modbus/modbus_receive.h", "protocol.h", "core.h",
    "ssf_mems_acquisition.h", "acquisition_policy.h", "raw/raw_stream.h",
    "raw/raw_capture.h", "transport/serial_transport.h", "firmware/ota.h",
)
ALLOWED_DEPENDENCIES = {
    "sensor": {"modbus_table"},
    "features": {"sensor", "modbus_request", "protocol"},
    "xyzs": {"features", "raw_capture"},
    "xyz": {"features", "raw_capture"},
    "modbus_table": set(),
    "serial_transport": {"modbus_table"},
    "raw_capture": {"modbus_table", "modbus", "modbus_request", "modbus_receive", "protocol", "serial_transport"},
    "modbus": {"modbus_table"},
    "modbus_request": {"modbus", "modbus_table", "modbus_receive", "serial_transport"},
    "modbus_receive": {"modbus", "modbus_table", "raw_stream", "ota", "serial_transport"},
    "protocol": {"modbus", "modbus_table", "modbus_request", "modbus_receive", "raw_stream", "raw_capture", "ota"},
    "raw_stream": {"modbus", "modbus_table"},
    "ota": {"modbus_request", "modbus_receive", "protocol", "serial_transport"},
}

def check_module_dependencies(objects):
    nm = os.environ.get("NM", "nm")
    for allowed in ALLOWED_DEPENDENCIES.values():
        allowed.add("sensor")
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
            owner = "iio" if symbol == "ssf_mems_iio_publish_raw" else owners.get(symbol)
            if owner not in ALLOWED_DEPENDENCIES[module]:
                raise AssertionError(f"Invalid dependency: {module} -> {symbol} ({owner})")

def configure_table(text, deleted, variant):
    marker = "static const struct ssf_reg_desc ssf_reg_table[] = {"
    start = text.index(marker) + len(marker)
    end = text.index("\n};", start)
    entries = re.findall(r"    \{\n.*?\n    \},", text[start:end], re.S)
    kept = []
    for entry in entries:
        address = int(re.search(r"\.display_reg = (\d+)", entry).group(1))
        if address not in deleted:
            # Keep wire addresses unchanged while moving the display addresses.
            # Any high-level literal address now fails, even though names are stable.
            if variant == "relocate_registers" and address in {
                40052, 40053, 40060, 40061, 40062, 40070, 40102, 40110, 40121
            }:
                entry = entry.replace(f".display_reg = {address}",
                                      f".display_reg = {address + 1000}")
            if address == 40062 and variant == "nonadjacent_length_display":
                entry = entry.replace(".display_reg = 40062", ".display_reg = 41062")
            if address == 40062 and variant == "nonadjacent_length_protocol":
                entry = entry.replace(".protocol_addr = 0x003d", ".protocol_addr = 0x013d")
            if address == 40007 and variant == "feature_scale_x10":
                entry = entry.replace('.name = "temperature",',
                                      '.feature_multiplier = 20,\n        .feature_divisor = 2,\n        .name = "temperature",')
            kept.append(entry)
    if variant == "reverse_table":
        kept.reverse()
    result = text[:start] + "\n" + "\n".join(kept) + text[end:]
    if variant == "feature_switch_layout":
        result = result.replace(".feature_enable_mask = 0xfc00", ".feature_enable_mask = 0x003f")
        result = result.replace(".feature_enable_shift = 10", ".feature_enable_shift = 0")
    if variant == "short_baud_table":
        result = result.replace(".default_work_baudrate = SSF_MEMS_BAUDRATE_128000",
                                ".default_work_baudrate = SSF_MEMS_BAUDRATE_115200")
        result = re.sub(r"static const unsigned int ssf_baudrates\[\] = \{.*?\n};",
                        "static const unsigned int ssf_baudrates[] = {\n"
                        "    9600, 2400, 4800, 9600, 19200, 38400, 57600, 115200,\n};",
                        result, flags=re.S)
    return result

def main():
    compiler = os.environ.get("CC", "cc")
    with tempfile.TemporaryDirectory(prefix="ssf-mems-protocol-tests-") as temp:
        scratch = Path(temp)
        selected = sys.argv[1:] or list(VARIANTS)
        if not sys.argv[1:]:
            check_table_validation(compiler, scratch)
        for name in selected:
            deleted = VARIANTS[name]
            project = scratch / name
            shutil.copytree(ROOT / "protocol", project / "protocol")
            shutil.copytree(ROOT / "core", project / "core")
            shutil.copytree(ROOT / "acquisition", project / "acquisition")
            shutil.copytree(ROOT / "sensor", project / "sensor")
            table_path = project / "sensor/xyzs.c"
            # Mechanical test-fixture transformation; production table is never edited.
            table_path.write_text(configure_table(table_path.read_text(), deleted, name))
            executable = project / "protocol_test"
            flags = [
                compiler, "-std=gnu11", "-Wall", "-Wextra", "-Werror",
                "-Wno-sign-compare", "-fsanitize=address,undefined", "-g",
                "-I" + str(TEST_DIR / "shim"), "-I" + str(project / "protocol"),
                "-I" + str(project / "core"),
                "-I" + str(project / "sensor"),
                "-I" + str(project / "acquisition"),
                "-I" + str(ROOT / "iio"),
            ]
            # Headers must provide their own prerequisites, not rely on include order.
            for header in HEADERS:
                subprocess.run(flags + ["-include", header, "-x", "c", "-fsyntax-only", "/dev/null"], check=True)
            objects = {}
            for module, source in MODULES.items():
                obj = project / (module + ".o")
                dependency_file = project / (module + ".d")
                subprocess.run(flags + ["-MMD", "-MF", str(dependency_file), "-c",
                                       str(project / "protocol" / source), "-o", str(obj)], check=True)
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


def check_table_validation(compiler, scratch):
    fixtures = {
        "valid": (None, None, "valid"),
        "empty_sampling": (".sampling_frequency_count = ARRAY_SIZE(ssf_sampling_frequencies)",
                           ".sampling_frequency_count = 0", "sensor_profile"),
        "empty_bauds": (".baudrate_count = ARRAY_SIZE(ssf_baudrates)",
                        ".baudrate_count = 0", "sensor_profile"),
        "bad_target_baud": (".default_work_baudrate = SSF_MEMS_BAUDRATE_128000",
                            ".default_work_baudrate = SSF_MEMS_BAUDRATE_MAX", "sensor_profile"),
        "bad_sampling_default": (".default_sampling_rate_index = 6",
                                 ".default_sampling_rate_index = 10", "sensor_profile"),
        "bad_raw_limit": (".raw_max_sampling_rate_index = 9",
                          ".raw_max_sampling_rate_index = 10", "sensor_profile"),
        "bad_shift": (".feature_enable_shift = 10", ".feature_enable_shift = 16", "sensor_profile"),
        "sparse_mask": (".feature_enable_mask = 0xfc00", ".feature_enable_mask = 0xa400", "sensor_profile"),
        "bad_axis": (".raw_axis_order = {0, 1, 2}", ".raw_axis_order = {0, 0, 2}", "sensor_profile"),
        "bad_scale": (".raw_accel_scale_nano = 4788403", ".raw_accel_scale_nano = 0", "sensor_profile"),
        "duplicate_name": ('.name = "x_low_freq_velocity_rms"', '.name = "x_high_freq_acc_rms"',
                           "x_high_freq_acc_rms"),
        "empty_name": ('.name = "x_high_freq_acc_rms"', '.name = ""', "unnamed_register"),
        "duplicate_display": (".display_reg = 40002", ".display_reg = 40001", "x_low_freq_velocity_rms"),
        "duplicate_wire": (".protocol_addr = 0x0001", ".protocol_addr = 0x0000", "x_low_freq_velocity_rms"),
        "duplicate_field": (".feature_field = SSF_MEMS_FIELD_X_LOW_FREQ_VELOCITY_RMS",
                            ".feature_field = SSF_MEMS_FIELD_X_HIGH_FREQ_ACC_RMS", "x_low_freq_velocity_rms"),
        "bad_field": (".feature_field = SSF_MEMS_FIELD_TEMPERATURE",
                      ".feature_field = SSF_MEMS_FIELD_MAX", "temperature"),
        "bad_field_type": (".type = SSF_REG_S16", ".type = SSF_REG_U32_HIGH", "temperature"),
        "bad_offset": ("SSF_MEMS_FEATURE_MEMBER(xyzs.x.high_freq_acc_rms_x100, ACCEL)",
                       ".field = {.offset = 1000, .width = 2}", "x_high_freq_acc_rms"),
        "bad_width": ("SSF_MEMS_FEATURE_MEMBER(xyzs.x.high_freq_acc_rms_x100, ACCEL)",
                      ".field = {.offset = 0, .width = 4}", "x_high_freq_acc_rms"),
        "overlapping_member": ("SSF_MEMS_FEATURE_MEMBER(xyzs.x.low_freq_velocity_rms_x100, VELOCITY)",
                               "SSF_MEMS_FEATURE_MEMBER(xyzs.x.high_freq_acc_rms_x100, VELOCITY)",
                               "x_low_freq_velocity_rms"),
        "bad_feature_scale": ("SSF_MEMS_FEATURE_MEMBER(xyzs.x.high_freq_acc_rms_x100, ACCEL)",
                              ".field = {.offset = 0, .width = 2, .scale = 1000}",
                              "x_high_freq_acc_rms"),
        "empty_block": (".reg_count = 29", ".reg_count = 0", "read_all_features"),
    }
    for name, (original, replacement, expected) in fixtures.items():
        project = scratch / ("validation_" + name)
        shutil.copytree(ROOT / "protocol", project / "protocol")
        shutil.copytree(ROOT / "sensor", project / "sensor")
        table_path = project / "sensor/xyzs.c"
        if original is not None:
            source = table_path.read_text()
            assert original in source, (name, original)
            table_path.write_text(source.replace(original, replacement, 1))
        executable = project / "table_test"
        command = [compiler, "-std=gnu11", "-Wall", "-Wextra", "-Werror",
                   "-Wno-sign-compare", "-fsanitize=address,undefined", "-g",
                   "-I" + str(TEST_DIR / "shim"), "-I" + str(project / "protocol"),
                   "-I" + str(project / "sensor"),
                   str(project / "protocol/model/modbus_table.c"),
                   str(project / "sensor/sensor.c"), str(project / "sensor/xyzs.c"),
                   str(TEST_DIR / "model_ops_stub.c"), str(TEST_DIR / "table_test.c"),
                   "-o", str(executable)]
        subprocess.run(command, check=True)
        subprocess.run([str(executable), expected], check=True)

if __name__ == "__main__":
    main()
