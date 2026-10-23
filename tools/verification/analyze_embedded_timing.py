"""Reproducible UART budgets and ARM artifact/stack/call-site audit, not WCET.

Build the firmware with -fstack-usage -fcallgraph-info=su and export compile
commands. Unknown library stack, interrupt nesting and silicon/driver latency
are retained as unknowns. Static call sites are never converted to cycle times.
"""
from pathlib import Path
import argparse
from collections import Counter, defaultdict
import hashlib
import json
import re
import subprocess

ROOT = Path(__file__).resolve().parents[2]


def uart_budget():
    from companionbot_contracts.packets import ReferenceBlockPacket
    from companionbot_control.runtime.embedded_link import EmbeddedFrame
    packet = ReferenceBlockPacket.from_rows(reference_epoch=0, sequence_id=0,
        start_control_tick=25, generated_time_s=0, dt_s=.002, yaw_rate_target_rad_s=0,
        rows=[(0., 0., 0., 0., 0., 0.)] * 25)
    assert len(packet.to_bytes()) == 640
    payloads = {"reference": packet.to_bytes(), "heartbeat": b"", "yaw": bytes(8),
                "safety": bytes(3), "resume": bytes(5), "status": bytes(144)}
    kinds = dict(reference=1, heartbeat=2, yaw=3, safety=4, resume=5, status=128)
    rows = {}
    for name, payload in payloads.items():
        raw = 20 + len(payload)
        worst = raw + raw // 254 + 2
        actual = len(EmbeddedFrame(kinds[name], 77, 0, 100, payload).to_wire())
        assert actual <= worst
        rows[name] = {"payload_bytes": len(payload), "raw_bytes_with_header_crc": raw,
            "worst_encoded_bytes_including_delimiter": worst, "zero_fixture_encoded_bytes": actual,
            "wire_ms_by_baud_8n1": {str(baud): worst * 10000 / baud for baud in (115200, 460800, 921600)}}
    profiles = {
        "current_20Hz_ref_20Hz_heartbeat_20Hz_yaw": {"reference": 20, "heartbeat": 20, "yaw": 20},
        "load_20Hz_ref_100Hz_heartbeat_100Hz_yaw": {"reference": 20, "heartbeat": 100, "yaw": 100},
        "load_plus_20Hz_each_safety_resume": {"reference": 20, "heartbeat": 100, "yaw": 100, "safety": 20, "resume": 20},
        "overload_200Hz_ref_200Hz_heartbeat_200Hz_yaw": {"reference": 200, "heartbeat": 200, "yaw": 200},
    }
    for name, rates in list(profiles.items()):
        demand = sum(rows[kind]["worst_encoded_bytes_including_delimiter"] * rate for kind, rate in rates.items())
        profiles[name] = {"rates_hz": rates, "worst_pi_to_mcu_bytes_s": demand,
            "line_utilization_by_baud": {str(baud): demand * 10 / baud for baud in (115200, 460800, 921600)}}
    return {"protocol": "embedded v2; unchanged 640B ReferenceBlock v1", "format": "8N1 full duplex",
        "baud_configured": 921600, "profiles_are_explicit_not_an_unlimited_input_admission_guarantee": True,
        "frames": rows, "profiles": profiles,
        "status_50Hz_bytes_s": rows["status"]["worst_encoded_bytes_including_delimiter"] * 50,
        "max_generic_frame_bytes": 696,
        "bursts_921600_ms": {"two_references_heartbeat_yaw": (2*664+22+30)*10000/921600,
            "eight_references": 8*664*10000/921600, "eight_pending_plus_one_in_flight": 9*664*10000/921600},
        "receiver": {"dma_bytes": 1024, "rx_ring_usable_bytes": 4095, "parse_budget_bytes_per_poll": 768,
            "dma_full_line_ms": 1024*10000/921600, "half_dma_line_ms": 512*10000/921600,
            "rx_ring_full_line_ms": 4095*10000/921600, "reference_starvation_margin_ms": 12,
            "next_reference_latest_arrival_ms_after_active_start": 38,
            "reference_wire_only_margin_ms": 38-664*10000/921600},
        "br_r_nominal": {"pclk_hz": 170000000, "brr_oversample16": round(170000000/921600),
            "nominal_actual_baud": 170000000/round(170000000/921600),
            "hsi_and_usb_baud_tolerances_tested": False}}


def arm_audit(build, output):
    prefix = ROOT / ".venv/embedded/native/usr/bin/arm-none-eabi-"
    elf = build / "companionbot_g474.elf"
    def run(tool, *arguments):
        return subprocess.check_output([str(prefix) + tool, *map(str, arguments)], text=True)
    disassembly = run("objdump", "-d", "--no-show-raw-insn", elf)
    (output / "firmware_disassembly.txt").write_text(disassembly)
    symbols = run("nm", "-S", "--defined-only", elf)
    (output / "firmware_symbols.txt").write_text(symbols)
    attributes = run("readelf", "-A", elf)
    (output / "arm_attributes.txt").write_text(attributes)
    sections = run("size", "-A", elf)
    (output / "arm_sections.txt").write_text(sections)
    undefined = run("nm", "--undefined-only", elf)
    names = {line.split()[-1] for line in symbols.splitlines() if line.split()}
    functions = defaultdict(lambda: {"instructions": 0, "call_sites": Counter(), "indirect_call_sites": 0})
    current = None
    for line in disassembly.splitlines():
        match = re.match(r"[0-9a-f]+ <([^>]+)>:", line)
        if match:
            current = match[1]
            continue
        instruction = re.match(r"\s+[0-9a-f]+:\s+(\S+)\s*(.*)", line)
        if not instruction or not current:
            continue
        opcode, operands = instruction.groups()
        functions[current]["instructions"] += 1
        target = re.search(r"<([^>+]+)(?:\+[^>]*)?>", operands)
        if opcode in ("bl", "bl.w", "blx"):
            if target:
                functions[current]["call_sites"][target[1]] += 1
            else:
                functions[current]["indirect_call_sites"] += 1
        elif opcode in ("b", "b.w") and target and target[1] != current:
            functions[current]["call_sites"][target[1]] += 1
    stack = {}
    stack_rows = []
    for path in sorted(build.rglob("*.su")):
        for line in path.read_text().splitlines():
            fields = line.split("\t")
            if len(fields) != 3:
                continue
            name = fields[0].rsplit(":", 1)[-1]
            row = {"function": name, "bytes": int(fields[1]), "qualification": fields[2],
                   "object_stack_file": str(path.relative_to(build))}
            stack_rows.append(row)
            if name in names:
                stack[name] = max(stack.get(name, 0), row["bytes"])
    graph = defaultdict(set)
    for path in build.rglob("*.ci"):
        for source, target in re.findall(r'edge: \{ sourcename: "([^"]+)" targetname: "([^"]+)"', path.read_text()):
            graph[source.rsplit(":", 1)[-1]].add(target.rsplit(":", 1)[-1])
    cycles = set()
    def known_path(name, visited=()):
        if name in visited:
            cycles.add(name)
            return 0, [name]
        candidates = [known_path(child, (*visited, name)) for child in graph[name] if child in stack]
        size, chain = max(candidates, default=(0, []), key=lambda item: item[0])
        return stack.get(name, 0) + size, [name, *chain]
    roots = ("cb_app_poll", "cb_app_rx_isr", "cb_estimator_update", "cb_mcu_step", "cb_parser_feed", "cb_mcu_dispatch")
    if not set(roots) <= stack.keys():
        raise RuntimeError("Rebuild every firmware/core C source with -fstack-usage -fcallgraph-info=su")
    paths = {name: dict(zip(("known_stack_bytes_only", "path"), known_path(name))) for name in roots}
    compiled = json.loads((build / "compile_commands.json").read_text())
    commands = {str(Path(row["file"]).relative_to(ROOT)): row["command"] for row in compiled if str(ROOT) in row["file"]}
    math_names = sorted(name for name in names if name.startswith("__aeabi_d") or name in ("atan2", "sin", "cos", "sincos", "fmod", "floor"))
    own = {name: info for name, info in functions.items() if name.startswith("cb_")}
    result = {"elf": str(elf), "sha256": hashlib.sha256(elf.read_bytes()).hexdigest(),
        "silicon_execution_measured": False, "wcet_cycles_proven": None,
        "slot_cycles_at_170MHz": 340000, "undefined_symbols": undefined.splitlines(),
        "hard_float_abi": "Tag_ABI_VFP_args: VFP registers" in attributes,
        "double_and_math_symbols_linked": math_names,
        "malloc_or_free_symbols_linked": sorted(names & {"malloc", "calloc", "realloc", "free", "_malloc_r", "_free_r"}),
        "compile_commands": commands, "linked_source_stack_rows": [row for row in stack_rows if row["function"] in names],
        "known_call_chain_stack_partial": paths, "source_recursion_cycles_found": sorted(cycles),
        "stack_unknowns": "libm/libgcc assembly, indirect calls, exception/FPU stacking and future board_io; partial sums are not a proven bound",
        "largest_source_frames": sorted(stack_rows, key=lambda row: -row["bytes"])[:15],
        "project_functions_static_instruction_and_call_sites": own,
        "call_site_counts_are_not_dynamic_counts_or_cycle_estimates": True}
    assert not result["undefined_symbols"] and result["hard_float_abi"]
    assert not result["malloc_or_free_symbols_linked"]
    assert {"cb_app_poll", "cb_estimator_update", "cb_mcu_step"} <= names
    return result


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument("--build-dir", type=Path, required=True)
    cli.add_argument("--output-dir", type=Path, required=True)
    args = cli.parse_args()
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    budget = uart_budget()
    (output / "uart_budget.json").write_text(json.dumps(budget, indent=2) + "\n")
    arm = arm_audit(args.build_dir.resolve(), output)
    (output / "arm_audit.json").write_text(json.dumps(arm, indent=2) + "\n")
    print(json.dumps({"uart_default_utilization": budget["profiles"]["current_20Hz_ref_20Hz_heartbeat_20Hz_yaw"]["line_utilization_by_baud"],
        "known_stack_paths_partial": arm["known_call_chain_stack_partial"],
        "board_wcet_proven": False}, indent=2))


if __name__ == "__main__":
    main()
