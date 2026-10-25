#!/usr/bin/env python3
"""Build a native harness from exact SA11/SA12 production method bodies.

This does not compile the full ESP-IDF firmware. It extracts the selected
actual C++ methods and compiles them against small host-only adapters for the
firmware-owned types and services used by those methods.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
from typing import Any


HERE = Path(__file__).resolve().parent
FW_DEFAULT = HERE.parents[1]
MI_ROOT = HERE.parents[4]
BASELINE_DEFAULT = MI_ROOT / "work/sa11-r1-20261023/fw"
TEMPLATE = HERE / "native_harness.cpp.in"
OUTPUT_DEFAULT = MI_ROOT / "outputs/sa12-r1-offline-2026-10-24/adaptation"


def require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def cpp_block(source: str, signature: str) -> str:
    start = source.find(signature)
    require(start >= 0, f"missing source signature: {signature}")
    opening = source.find("{", start)
    require(opening >= 0, f"missing opening brace: {signature}")
    depth = 0
    i = opening
    state = "normal"
    while i < len(source):
        ch = source[i]
        nxt = source[i + 1] if i + 1 < len(source) else ""
        if state == "line_comment":
            if ch == "\n":
                state = "normal"
        elif state == "block_comment":
            if ch == "*" and nxt == "/":
                state = "normal"
                i += 1
        elif state in ("string", "char"):
            if ch == "\\":
                i += 1
            elif (state == "string" and ch == '"') or (state == "char" and ch == "'"):
                state = "normal"
        else:
            if ch == "/" and nxt == "/":
                state = "line_comment"
                i += 1
            elif ch == "/" and nxt == "*":
                state = "block_comment"
                i += 1
            elif ch == '"':
                state = "string"
            elif ch == "'":
                state = "char"
            elif ch == "{":
                depth += 1
            elif ch == "}":
                depth -= 1
                if depth == 0:
                    return source[start : i + 1]
        i += 1
    raise RuntimeError(f"unterminated block: {signature}")


def function(source: str, signature: str) -> str:
    return cpp_block(source, signature)


def namespace_block(source: str) -> str | None:
    start = source.find("namespace {")
    if start < 0:
        return None
    return cpp_block(source, "namespace {")


def source_bundle(
    fw: Path,
    normal_pid_settings: dict[str, dict[str, int]],
    input_rpm_max: int,
    slip_min: int,
    slip_max: int,
) -> tuple[str, dict[str, str]]:
    adapt_path = fw / "src/adaptation/shift_adaptation.cpp"
    algo_path = fw / "src/shifting_algo/s_algo.cpp"
    stored_path = fw / "src/stored_map.cpp"
    tcc_path = fw / "src/torque_converter.h"
    files = {p.relative_to(fw).as_posix(): p.read_bytes() for p in
             (adapt_path, algo_path, stored_path, tcc_path)}
    adapt = files["src/adaptation/shift_adaptation.cpp"].decode("utf-8")
    algo = files["src/shifting_algo/s_algo.cpp"].decode("utf-8")
    stored = files["src/stored_map.cpp"].decode("utf-8")
    tcc = files["src/torque_converter.h"].decode("utf-8")

    adaptation_methods = [
        function(adapt, "ShiftAdaptationSystem::ShiftAdaptationSystem()"),
        function(adapt, "esp_err_t ShiftAdaptationSystem::save(void)"),
        function(adapt, "int8_t ShiftAdaptationSystem::get_prefill_cycles_offset"),
        function(adapt, "int16_t ShiftAdaptationSystem::get_adapt_spc_offset"),
        function(adapt, "int16_t ShiftAdaptationSystem::get_freeing_torque_offset"),
        function(adapt, "int16_t ShiftAdaptationSystem::get_applying_torque_offset"),
        function(adapt, "void ShiftAdaptationSystem::offset_prefill_cycles"),
        function(adapt, "void ShiftAdaptationSystem::offset_spc_pressure"),
        function(adapt, "void ShiftAdaptationSystem::offset_freeing_trq"),
        function(adapt, "void ShiftAdaptationSystem::offset_applying_trq"),
        function(adapt, "esp_err_t ShiftAdaptationSystem::reset()"),
    ]
    init_shift = "void ShiftAdaptationSystem::init_shift()"
    if init_shift in adapt:
        adaptation_methods.insert(1, function(adapt, init_shift))
    adaptation_namespace = namespace_block(adapt) or ""

    algo_methods = []
    algo_namespace = namespace_block(algo) or ""
    for signature in (
        "uint8_t ShiftingAlgorithm::adapt_p_map_idx()",
        "short ShiftingAlgorithm::calc_correction_trq",
        "void ShiftingAlgorithm::adaptation_step()",
    ):
        extracted = function(algo, signature)
        extracted = extracted.replace("ShiftingAlgorithm::", "TestShiftAlgo::", 1)
        algo_methods.append(extracted)

    stored_methods = [
        function(stored, "esp_err_t StoredMap::replace_data_content"),
        function(stored, "esp_err_t StoredMap::reset_from_flash"),
    ]
    tcc_signature = "esp_err_t save()" if "esp_err_t save()" in tcc else "void save()"
    tcc_method = function(tcc, tcc_signature)

    pieces = [adaptation_namespace, "\n".join(adaptation_methods),
              algo_namespace, "\n".join(algo_methods), "\n".join(stored_methods)]
    generated_methods = "\n\n".join(piece for piece in pieces if piece)
    template = TEMPLATE.read_text(encoding="utf-8")
    adaptation_piece = "\n\n".join(
        piece for piece in (adaptation_namespace, "\n".join(adaptation_methods),
                            algo_namespace, "\n".join(stored_methods)) if piece
    )
    generated = template.replace("@@SHIFT_ADAPTATION_METHODS@@", adaptation_piece)
    generated = generated.replace("@@S_ALGO_METHODS@@", "\n".join(algo_methods))
    generated = generated.replace("@@TCC_SAVE_METHOD@@", tcc_method)
    setup_lines = []
    for name, prefix in (("CRS", "CRS_CURRENT_SETTINGS"), ("REL", "REL_CURRENT_SETTINGS")):
        for suffix in ("p_val_upshift", "i_val_upshift", "d_val_upshift",
                       "p_val_downshift", "i_val_downshift", "d_val_downshift"):
            value = normal_pid_settings[name][suffix]
            setup_lines.append(f"{prefix}.pid_{suffix} = {value};")
    generated = generated.replace("@@NORMAL_PID_SETTINGS@@", "\n    ".join(setup_lines))
    generated = generated.replace("@@NORMAL_INPUT_RPM_MAX@@", str(input_rpm_max))
    generated = generated.replace("@@NORMAL_SLIP_MIN@@", str(slip_min))
    generated = generated.replace("@@NORMAL_SLIP_MAX@@", str(slip_max))
    require("@@" not in generated, "unreplaced source template token")
    return generated, {name: sha256(data) for name, data in files.items()}


def live_evidence() -> dict[str, Any]:
    settings_before_path = MI_ROOT / "outputs/sa11-feliras-2026-10-24/logger-live-attempt01/settings-before-capture.json"
    settings_after_path = MI_ROOT / "outputs/sa11-feliras-2026-10-24/logger-live-attempt01/settings-after-capture.json"
    shift_path = MI_ROOT.parent / "drive_20261023_170642_950893/shift_live.csv"
    tcc_path = MI_ROOT.parent / "drive_20261023_170642_950893/tcc_cycle.csv"
    require(settings_before_path.is_file() and settings_after_path.is_file(), "read-only SA11 settings snapshots missing")
    require(shift_path.is_file() and tcc_path.is_file(), "source drive CSVs missing")

    before = json.loads(settings_before_path.read_text(encoding="utf-8"))
    after = json.loads(settings_after_path.read_text(encoding="utf-8"))
    before_modules = {m["key"]: m["current"]["values"] for m in before["modules"] if m.get("key") in ("CRS", "REL", "ADP")}
    after_modules = {m["key"]: m["current"]["values"] for m in after["modules"] if m.get("key") in ("CRS", "REL", "ADP")}
    require(before_modules == after_modules, "CRS/REL/ADP saved settings changed across the stationary session")
    pid = {key: dict(after_modules[key]) for key in ("CRS", "REL")}
    require(all(f"pid_{field}" in pid["CRS"] for field in
                ("p_val_upshift", "i_val_upshift", "d_val_upshift", "p_val_downshift", "i_val_downshift", "d_val_downshift")),
            "current saved PID settings incomplete: CRS")
    require(all(f"pid_{field}" in pid["REL"] for field in
                ("p_val_upshift", "i_val_upshift", "p_val_downshift", "i_val_downshift")),
            "current saved PID settings incomplete: REL")
    saved_pid_values = {
        "CRS": {field: int(pid["CRS"][f"pid_{field}"]) for field in
                ("p_val_upshift", "i_val_upshift", "d_val_upshift", "p_val_downshift", "i_val_downshift", "d_val_downshift")},
        "REL": {field: int(pid["REL"][f"pid_{field}"]) for field in
                ("p_val_upshift", "i_val_upshift", "p_val_downshift", "i_val_downshift")},
    }
    # REL has no derivative settings in its structure; the runtime method sets
    # its derivative gain to zero for both release styles.
    pid_values = {
        "CRS": dict(saved_pid_values["CRS"]),
        "REL": dict(saved_pid_values["REL"]),
    }
    pid_values["REL"]["d_val_upshift"] = 0
    pid_values["REL"]["d_val_downshift"] = 0

    shift_rows = list(csv.DictReader(shift_path.open("r", newline="", encoding="utf-8")))
    positive_input_rpm = [int(row["input_rpm"]) for row in shift_rows
                          if row.get("input_rpm", "").isdigit() and int(row["input_rpm"]) > 0]
    require(bool(positive_input_rpm), "no positive input rpm observations")
    tcc_rows = list(csv.DictReader(tcc_path.open("r", newline="", encoding="utf-8")))
    valid_tcc = [row for row in tcc_rows if row.get("tcc_input_valid", "").lower() == "true"]
    signed_slip = [int(row["signed_engine_minus_input_rpm"]) for row in valid_tcc]
    require(bool(signed_slip), "no valid signed-slip observations")
    return {
        "pid_settings": pid_values,
        "pid_settings_saved_exact": saved_pid_values,
        "settings_before": {
            "path": str(settings_before_path),
            "sha256": sha256(settings_before_path.read_bytes()),
            "ts": before.get("ts"),
        },
        "settings_after": {
            "path": str(settings_after_path),
            "sha256": sha256(settings_after_path.read_bytes()),
            "ts": after.get("ts"),
        },
        "drive_input": {
            "shift_live_path": str(shift_path),
            "shift_live_sha256": sha256(shift_path.read_bytes()),
            "shift_live_rows": len(shift_rows),
            "positive_input_rpm_count": len(positive_input_rpm),
            "positive_input_rpm_min": min(positive_input_rpm),
            "positive_input_rpm_max": max(positive_input_rpm),
            "positive_input_rpm_unique_count": len(set(positive_input_rpm)),
            "tcc_cycle_path": str(tcc_path),
            "tcc_cycle_sha256": sha256(tcc_path.read_bytes()),
            "tcc_cycle_valid_rows": len(valid_tcc),
            "signed_engine_minus_input_rpm_min": min(signed_slip),
            "signed_engine_minus_input_rpm_max": max(signed_slip),
            "slip_semantics": "Measured converter slip, not the shift PID target-speed error. Used only as a clearly labeled synthetic error envelope for numeric regression coverage.",
        },
    }


def compile_variant(
    fw: Path,
    compiler: str,
    temp_dir: Path,
    tag: str,
    candidate: bool,
    sanitize: bool,
    normal_pid_settings: dict[str, dict[str, int]],
    input_rpm_max: int,
    slip_min: int,
    slip_max: int,
) -> tuple[Path, dict[str, Any]]:
    generated, hashes = source_bundle(fw, normal_pid_settings, input_rpm_max, slip_min, slip_max)
    if candidate:
        generated = "#define SA12_CANDIDATE 1\n" + generated
    cpp_path = temp_dir / f"{tag}.cpp"
    exe_path = temp_dir / tag
    cpp_path.write_text(generated, encoding="utf-8")
    stub_dir = HERE / "stubs"
    src_dir = fw / "src"
    adapt_dir = src_dir / "adaptation"
    flags = [
        "-std=c++17", "-O1", "-g", "-Wall", "-Wextra",
        "-Wno-unused-parameter", "-Wno-unused-function",
        "-I", str(stub_dir), "-I", str(adapt_dir), "-I", str(src_dir),
    ]
    if sanitize:
        flags += ["-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-fno-sanitize-recover=all"]
    command = [compiler, *flags, str(cpp_path), "-o", str(exe_path)]
    result = subprocess.run(command, text=True, capture_output=True)
    require(result.returncode == 0,
            f"compile failed ({tag})\ncommand={command!r}\n{result.stdout}{result.stderr}")
    return exe_path, {
        "source_sha256": hashes,
        "compiler_command": command,
        "compiler_stdout": result.stdout,
        "compiler_stderr": result.stderr,
        "sanitizers": ["address", "undefined"] if sanitize else [],
    }


def run(exe: Path, args: list[str] | None = None) -> dict[str, Any]:
    command = [str(exe), *(args or [])]
    env = os.environ.copy()
    # Apple Clang's bundled ASan runtime does not implement LeakSanitizer.
    env["ASAN_OPTIONS"] = "detect_leaks=0:halt_on_error=1"
    env["UBSAN_OPTIONS"] = "halt_on_error=1:print_stacktrace=1"
    result = subprocess.run(command, text=True, capture_output=True, env=env)
    parsed = None
    for line in reversed(result.stdout.splitlines()):
        try:
            parsed = json.loads(line)
            break
        except json.JSONDecodeError:
            continue
    return {
        "command": command,
        "exit_code": result.returncode,
        "stdout": result.stdout,
        "stderr": result.stderr,
        "json": parsed,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--candidate-fw", type=Path, default=FW_DEFAULT)
    parser.add_argument("--baseline-fw", type=Path, default=BASELINE_DEFAULT)
    parser.add_argument("--output-dir", type=Path, default=OUTPUT_DEFAULT)
    args = parser.parse_args()
    candidate_fw = args.candidate_fw.resolve()
    baseline_fw = args.baseline_fw.resolve()
    require(candidate_fw.is_dir(), f"candidate source missing: {candidate_fw}")
    require(baseline_fw.is_dir(), f"baseline source missing: {baseline_fw}")
    compiler = shutil.which("clang++") or shutil.which("g++")
    require(compiler is not None, "clang++ or g++ required")
    compiler_version = subprocess.run([compiler, "--version"], text=True, capture_output=True)
    evidence = live_evidence()
    normal_pid_settings = evidence["pid_settings"]
    input_rpm_max = evidence["drive_input"]["positive_input_rpm_max"]
    slip_min = evidence["drive_input"]["signed_engine_minus_input_rpm_min"]
    slip_max = evidence["drive_input"]["signed_engine_minus_input_rpm_max"]

    with tempfile.TemporaryDirectory(prefix="sa12-adaptation-native-") as temp_name:
        temp_dir = Path(temp_name)
        candidate_exe, candidate_build = compile_variant(
            candidate_fw, compiler, temp_dir, "sa12-candidate", True, False,
            normal_pid_settings, input_rpm_max, slip_min, slip_max)
        candidate_san_exe, candidate_san_build = compile_variant(
            candidate_fw, compiler, temp_dir, "sa12-sanitized", True, True,
            normal_pid_settings, input_rpm_max, slip_min, slip_max)
        baseline_exe, baseline_build = compile_variant(
            baseline_fw, compiler, temp_dir, "sa11-negative", False, False,
            normal_pid_settings, input_rpm_max, slip_min, slip_max)
        baseline_san_exe, baseline_san_build = compile_variant(
            baseline_fw, compiler, temp_dir, "sa11-sanitized-negative", False, True,
            normal_pid_settings, input_rpm_max, slip_min, slip_max)

        candidate_run = run(candidate_exe)
        candidate_sanitized_run = run(candidate_san_exe)
        baseline_run = run(baseline_exe)
        baseline_oob_run = run(baseline_san_exe, ["oob"])

    require(candidate_run["exit_code"] == 0 and candidate_run["json"],
            "candidate native run failed")
    require(candidate_sanitized_run["exit_code"] == 0 and candidate_sanitized_run["json"],
            "candidate ASan/UBSan run failed")
    require(baseline_run["exit_code"] == 0 and baseline_run["json"],
            "SA11 native negative-control run failed")
    require(baseline_oob_run["exit_code"] != 0 and
            "AddressSanitizer" in baseline_oob_run["stderr"] and
            "heap-buffer-overflow" in baseline_oob_run["stderr"],
            "SA11 sanitizer negative control did not expose index-8 heap OOB")

    candidate_values = candidate_run["json"]
    baseline_values = baseline_run["json"]
    sanitized_values = candidate_sanitized_run["json"]
    require(candidate_values["normal_map_values"] == baseline_values["normal_map_values"],
            "normal map getter/setter behavior diverges between SA11 and SA12")
    require(candidate_values["normal_pid_regression_hash"] ==
            baseline_values["normal_pid_regression_hash"] ==
            sanitized_values["normal_pid_regression_hash"],
            "four-style normal PID output differs between candidate, baseline, and sanitized run")
    expected_cases = 4 * input_rpm_max * 3
    require(candidate_values["normal_pid_regression_cases"] == expected_cases and
            baseline_values["normal_pid_regression_cases"] == expected_cases and
            sanitized_values["normal_pid_regression_cases"] == expected_cases,
            "normal PID regression case count mismatch")
    require(candidate_values["learning_off_maps_unchanged"] is True and
            baseline_values["learning_off_maps_unchanged"] is True and
            sanitized_values["learning_off_maps_unchanged"] is True,
            "learning-off map path changed")
    require(candidate_values["average_fixture_offset"] == 9 and
            baseline_values["average_fixture_offset"] == 10,
            "float abs-average fixture did not distinguish candidate from SA11")
    require(candidate_values["spc_positive_fixture"] == 115 and
            baseline_values["spc_positive_fixture"] == 200,
            "SPC limiter fixture did not distinguish candidate from SA11")
    require(candidate_values["prefill_200_getter"] == 127 and
            baseline_values["prefill_200_getter"] == -56,
            "prefill int8 saturation fixture changed unexpectedly")
    require(candidate_values["stored_map_reset_replace_failure"] == "propagated" and
            baseline_values["stored_map_reset_replace_failure"] == "swallowed",
            "StoredMap actual-source reset error propagation comparison failed")

    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    result_path = output_dir / f"native-actual-source-{stamp}.json"
    markdown_path = output_dir / f"native-actual-source-{stamp}.md"
    proof = {
        "status": "PASS",
        "observed_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "scope": "offline native actual-method extraction; no device/port/network access",
        "candidate_fw": str(candidate_fw),
        "baseline_fw": str(baseline_fw),
        "compiler": compiler,
        "compiler_version": compiler_version.stdout.strip().splitlines()[:3],
        "current_normal_pid_and_drive_evidence": evidence,
        "builds": {
            "candidate_native": candidate_build,
            "candidate_asan_ubsan": candidate_san_build,
            "sa11_negative_control_native": baseline_build,
            "sa11_negative_control_asan_ubsan": baseline_san_build,
        },
        "runs": {
            "candidate_native": candidate_run,
            "candidate_asan_ubsan": candidate_sanitized_run,
            "sa11_negative_control_native": baseline_run,
            "sa11_invalid_index_8_asan_negative_control": baseline_oob_run,
        },
        "verified": {
            "normal_getter_setter_values_equal": candidate_values["normal_map_values"],
            "normal_pid_four_style_regression": {
                "cases_each_build": candidate_values["normal_pid_regression_cases"],
                "candidate_baseline_sanitized_hash": candidate_values["normal_pid_regression_hash"],
                "all_equal": True,
                "pid_gains_from_current_saved_settings": evidence["pid_settings_saved_exact"],
                "effective_coefficients_used_by_source": normal_pid_settings,
                "max_abs_gains": {
                    "p": max(abs(v) for module in normal_pid_settings.values() for k, v in module.items() if k.startswith("p_")),
                    "i": max(abs(v) for module in normal_pid_settings.values() for k, v in module.items() if k.startswith("i_")),
                    "d": max(abs(v) for module in normal_pid_settings.values() for k, v in module.items() if k.startswith("d_")),
                },
                "input_rpm_domain": [1, input_rpm_max],
                "rpm_error_benchmarks": [slip_min, 0, slip_max],
                "caveat": evidence["drive_input"]["slip_semantics"],
            },
            "learning_off_maps_unchanged": True,
            "abs_torque_average_fixture": {
                "pid_sum": 1000,
                "abs_sum": 1005,
                "pid_count": 10,
                "last_abs_input_torque": 1000,
                "float_abs_average_nm": 100.5,
                "candidate_resulting_offset": candidate_values["average_fixture_offset"],
                "sa11_current_sample_over_count_resulting_offset": baseline_values["average_fixture_offset"],
                "integer_average_would_yield_offset": 10,
            },
            "spc_positive_limit": {
                "old_offset": 100,
                "pressure_multiplier": 100,
                "correction": 60,
                "calculated_limit": 200,
                "candidate_result": candidate_values["spc_positive_fixture"],
                "sa11_result": baseline_values["spc_positive_fixture"],
            },
            "prefill_int8_boundary": {
                "stored_int16": 200,
                "candidate_saturated_int8": candidate_values["prefill_200_getter"],
                "sa11_clang_implementation_narrowing": baseline_values["prefill_200_getter"],
                "compatibility_note": "Stored values 0..127 preserve the prior result; 128..200 now saturate at 127 instead of wrapping. This is a boundary/invalid-state correction, not full-range legacy equality.",
            },
            "stored_map_reset": {
                "candidate_replace_failure": candidate_values["stored_map_reset_replace_failure"],
                "sa11_replace_failure": baseline_values["stored_map_reset_replace_failure"],
                "candidate_direct_tests": [
                    "normal replace and save",
                    "add_data replacement failure preserves RAM and skips save",
                    "invalid content length propagates and skips mutation/save",
                    "EEPROM save failure is returned after RAM replacement",
                    "wrong adaptation-map axes shape is rejected by getters/save/reset",
                    "ShiftAdaptationSystem reset propagates first replace/save error at every map position and is not atomic",
                ],
            },
            "candidate_assertions_native": candidate_values["assertions"],
            "candidate_assertions_sanitized": sanitized_values["assertions"],
            "sa11_assertions_native": baseline_values["assertions"],
            "sa11_oob_negative_control": "ASan reports heap-buffer-overflow for legacy prefill getter index 8",
        },
        "limitations": [
            "These are exact extracted production method bodies compiled against host adapters, not a full ESP-IDF target build.",
            "The actual Config App/TCU and hydraulic behavior are not exercised.",
            "The observed engine-input converter slip is not the shift PID target-speed error; its extrema are used only as a synthetic error-domain benchmark. The source drive did not log target_turbine_speed or momentum_ctrl_filtered, so their per-cycle values cannot be replayed exactly.",
            "Apple Clang's bundled ASan runtime lacks LeakSanitizer; ASan/UBSan ran with leak detection disabled.",
            "SA11 prefill narrowing result -56 is the observed clang++ arm64 host result; the old out-of-range signed narrowing is implementation-defined by C++.",
            "Shift and TCC save APIs return honest errors; the gearbox caller propagation is handled separately by the parent task and is not included in this scoped harness.",
            "Multi-map save/reset are fail-fast sequential operations, not atomic; preceding maps may already have changed RAM/EEPROM before a later error.",
        ],
    }
    result_path.write_text(json.dumps(proof, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    markdown = [
        "# SA12 adaptation actual-source native proof",
        "",
        f"- Status: **{proof['status']}** ({stamp})",
        f"- Candidate: `{candidate_fw}`",
        f"- SA11 negative control: `{baseline_fw}`",
        f"- Compiler: `{compiler_version.stdout.strip().splitlines()[0]}`",
        f"- Native candidate assertions: {candidate_values['assertions']}; ASan/UBSan repeat: {sanitized_values['assertions']}",
        f"- SA11 negative-control assertions: {baseline_values['assertions']}",
        "",
        "## Findings",
        "",
        f"- Normal getter/setter fixture matches SA11 exactly: `{candidate_values['normal_map_values']}`.",
        f"- Four-style normal PID regression: {candidate_values['normal_pid_regression_cases']} cases per build, using the current saved CRS/REL gains and the observed input-RPM plus labeled synthetic slip/error envelope; candidate, SA11, and ASan/UBSan hashes match.",
        "- Learning-off path leaves the four adaptation maps unchanged in both source versions.",
        "- Fractional absolute-torque mean 100.5 Nm produces candidate offset 9; SA11 current-sample/count integer division (1000/10 = 100) produces 10. The test distinguishes float accumulation and division from the legacy calculation.",
        "- SPC positive-limit fixture returns 115 at a calculated 200 limit; SA11 incorrectly jumps to 200.",
        "- Prefill map value 200 returns 127 after saturation; SA11 clang host narrowing returns -56. Values within 0..127 retain the same getter result.",
        "- Candidate StoredMap reset propagates replace and save errors; SA11 negative control swallows the replace failure. Candidate direct tests cover valid reset, failed replacement, invalid length, save failure, fail-fast ordering, and non-atomic prior-map effects.",
        "- The SA11 index-8 getter probe is rejected by ASan as a heap-buffer-overflow; the SA12 guarded candidate passes ASan/UBSan.",
        "",
        "## Limits",
        "",
        "The proof compiles exact extracted production methods against small host adapters. It is not a full ESP-IDF build, GUI test, TCU test, or vehicle/hydraulic validation. Save/reset operations are sequential and fail-fast, not multi-map atomic. Gearbox caller-side save fault reporting is outside this task's source scope.",
        "",
        f"Machine-readable evidence: `{result_path}`.",
    ]
    markdown_path.write_text("\n".join(markdown) + "\n", encoding="utf-8")
    print(json.dumps({"status": "PASS", "json": str(result_path), "markdown": str(markdown_path)}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
