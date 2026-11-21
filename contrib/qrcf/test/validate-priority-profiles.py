#!/usr/bin/env python3
"""Validate QRCF presets with the existing binary; never build or select by PDR.

The default output is a fresh, retained /tmp directory. An explicit --output
must be empty. Historical CSVs are read as references and are never modified.
"""

import argparse
import csv
import hashlib
import json
from pathlib import Path
import shlex
import statistics
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[3]
PRESETS = ("legacy-noq", "staggered-noq", "outcome-immediate", "outcome-bellman")
NUMERIC_SETTINGS = (
    "HelloLower", "HelloUpper", "ErrorMin", "ErrorMax", "K", "WeightSpeed",
    "WeightConnection", "WeightEnergy", "WeightAngle", "Rmax", "Rmin", "ConnectionCap",
)
WIRE_COUNTERS = ("ValueTx", "ValueRx", "BootstrapUpdates", "OutcomeTimeouts", "ArpFailures")


def read_csv(path):
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        assert len(reader.fieldnames) == len(set(reader.fieldnames)), path
        rows = list(reader)
    assert rows, path
    return rows


def write_csv(path, rows):
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def label(preset):
    return "qrcf-aligned-noq" if preset == "legacy-noq" else f"qrcf-aligned-{preset}-v1"


def ratio_change(after, before):
    return 100 * (after / before - 1) if before else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, default=ROOT / "build/scratch/ns3.40-benchmarktest-default")
    parser.add_argument("--output", type=Path, help="Empty destination; defaults to a new /tmp directory")
    parser.add_argument("--nodes", type=int, choices=(20, 50), default=50)
    parser.add_argument("--time", type=float, default=30)
    parser.add_argument("--seeds", nargs="+", type=int, default=(2025, 2026, 2027))
    parser.add_argument("--timeout", type=float, default=180, help="Per-case wall-clock limit, seconds")
    parser.add_argument("--snapshot-dir", type=Path, default=ROOT / "results/qrcf-mainline-cleanup")
    args = parser.parse_args()
    if not args.binary.is_file():
        parser.error(f"Build benchmarktest first; binary does not exist: {args.binary}")
    if args.time < 20 or not args.seeds or len(set(args.seeds)) != len(args.seeds) or min(args.seeds) <= 0:
        parser.error("Use time >= 20 and distinct positive seeds")
    binary = args.binary.resolve()
    output = args.output.resolve() if args.output else Path(tempfile.mkdtemp(prefix="qrcf-priority-profiles-"))
    if args.output:
        output.mkdir(parents=True, exist_ok=True)
        if any(output.iterdir()):
            parser.error(f"Output directory must be empty to prevent mixed CSV runs: {output}")
    print(f"Artifacts: {output}", flush=True)
    commands = []
    artifacts = [binary, ROOT / "build/lib/libns3.40-qrcf-default.so"]

    def build_fingerprints():
        return {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                for path in artifacts if path.is_file()}

    fingerprints = build_fingerprints()
    (output / "manifest.json").write_text(json.dumps({
        "build_sha256": fingerprints, "nodes": args.nodes, "seconds": args.time,
        "seeds": args.seeds, "presets": PRESETS, "default_preset": "outcome-bellman",
    }, indent=2) + "\n")

    def run(name, preset, nodes, duration, seed, extra=(), default_prefix=False):
        assert build_fingerprints() == fingerprints, "Executable/module changed during validation; use a fresh output after building"
        directory = output / name
        directory.mkdir()
        prefix = directory / "run_"
        command = [str(binary), "--algorithm=qrcf", f"--qrcfPreset={preset}",
                   "--profile=legacy", f"--size={nodes}", f"--time={duration}", f"--seed={seed}",
                   "--MinSpeed=20", "--MaxSpeed=60", "--MaxRange=500", "--channelModel=simple", *extra]
        if default_prefix:
            base = "fbcr_benchmark_tests_" if preset == "legacy-noq" else f"qrcf_priority_v1_{preset}_legacy_"
            prefix = directory / base
        else:
            command.append(f"--csvPrefix={prefix}")
        commands.append({"case": name, "cwd": str(directory), "argv": command,
                         "shell_display": shlex.join(command)})
        (output / "commands.json").write_text(json.dumps(commands, indent=2) + "\n")
        print(f"RUN {name}", flush=True)
        try:
            result = subprocess.run(command, cwd=directory, capture_output=True, text=True, timeout=args.timeout)
        except subprocess.TimeoutExpired as error:
            captured = []
            for chunk in (error.stdout, error.stderr):
                captured.append(chunk.decode(errors="replace") if isinstance(chunk, bytes) else (chunk or ""))
            (directory / "stdout.log").write_text("".join(captured))
            raise AssertionError(f"Timed out: {name}; inspect {directory / 'stdout.log'}") from error
        (directory / "stdout.log").write_text(result.stdout + result.stderr)
        assert result.returncode == 0, (name, result.returncode, directory / "stdout.log")
        main_path = Path(f"{prefix}{nodes}nodes.csv")
        side_path = Path(f"{prefix}{nodes}nodes_measurements.csv")
        suffix = "_qrcf_diagnostics.csv" if preset == "legacy-noq" else "_qrcf_priority_v1_diagnostics.csv"
        diagnostic_path = Path(f"{prefix}{nodes}nodes{suffix}")
        main_rows, side_rows, diagnostics = read_csv(main_path), read_csv(side_path), read_csv(diagnostic_path)
        assert len(main_rows) == len(side_rows) == 1 and len(diagnostics) == nodes, name
        side = side_rows[0]
        assert main_rows[0]["Algorithm"] == side["Algorithm"] == label(preset), name
        assert len({d["Node"] for d in diagnostics}) == nodes, name
        assert int(side["Offered"]) > 0 and 0 <= int(side["Delivered"]) <= int(side["Offered"]), name
        assert abs(float(side["PDR(%)"]) - 100 * int(side["Delivered"]) / int(side["Offered"])) < 1e-8, name
        assert all(int(d["InvalidControl"]) == 0 for d in diagnostics), name
        assert all(d["Seed"] == str(seed) and int(d["UAVs"]) == nodes for d in diagnostics), name
        if preset == "legacy-noq":
            assert all(d["Stage"] == "online-noq" and "Preset" not in d for d in diagnostics), name
        else:
            flags = (int(preset.startswith("outcome-")), int(preset == "outcome-bellman"), 1)
            assert all(d["Preset"] == preset and d["Stage"] == f"online-{preset}-v1" for d in diagnostics), name
            assert all(tuple(int(d[k]) for k in ("OutcomeUpdates", "Bootstrap", "HelloJitter")) == flags
                       for d in diagnostics), name
            assert not Path(f"{prefix}{nodes}nodes_qrcf_diagnostics.csv").exists(), name
            if preset in ("staggered-noq", "outcome-immediate"):
                assert sum(int(d["BootstrapUpdates"]) for d in diagnostics) == 0, name
            if preset == "staggered-noq":
                assert sum(int(d["ValueTx"]) + int(d["ValueRx"]) for d in diagnostics) == 0, name
        total = lambda key: sum(int(d.get(key, 0)) for d in diagnostics)
        accepted = total("HelloTx") + total("FeedbackTx") + total("ValueTx")
        tagged_packets, tagged_bytes = total("ControlL3Packets"), total("ControlL3Bytes")
        expected_bytes = 118 * total("HelloTx") + 45 * total("FeedbackTx") + 61 * total("ValueTx")
        assert total("ControlTx") == accepted, (name, "Every accepted control datagram has a known type")
        # Socket acceptance and the IP-Tx trace are different observation
        # points. Compare lengths only when all accepted datagrams reached it;
        # otherwise retain both counts for review rather than inventing bytes.
        if tagged_packets == accepted:
            assert tagged_bytes == expected_bytes, (name, "HELLO=118, FEEDBACK=45, VALUE=61 L3 bytes")
        else:
            print(f"ACCOUNTING {name}: socket_accept={accepted}, tagged_ip_tx={tagged_packets}", flush=True)
        record = {
            "Case": name, "Preset": preset, "Algorithm": label(preset), "Nodes": nodes,
            "Seed": seed, "TimeSeconds": duration, "Offered": int(side["Offered"]),
            "Delivered": int(side["Delivered"]), "AppPDRPct": float(side["PDR(%)"]),
            "AvgDelayMs": float(side["AvgDelay(ms)"]), "AvgHops": float(side["AvgHops"]),
            "SharedL3ControlKbps": float(side["L3ControlThroughput(kbps)"]),
            "SharedL3ControlPackets": int(side["L3ControlPackets"]),
            "TotalEnergyJ": sum(float(d["ConsumedJ"]) for d in diagnostics),
            "HelloTx": total("HelloTx"), "HelloRx": total("HelloRx"),
            "FeedbackTx": total("FeedbackTx"), "FeedbackRx": total("FeedbackRx"),
            "QUpdates": total("QUpdates"), **{key: total(key) for key in WIRE_COUNTERS},
            "SocketControlAccepted": accepted, "TaggedL3ControlPackets": tagged_packets,
            "TaggedL3ControlBytes": tagged_bytes, "ExpectedControlBytesIfAllSent": expected_bytes,
            "WireAccountingComplete": tagged_packets == accepted and tagged_bytes == expected_bytes,
        }
        print(f"PASS {name}: {record['Delivered']}/{record['Offered']} delivered, "
              f"value_tx={record['ValueTx']}, value_rx={record['ValueRx']}", flush=True)
        return {"record": record, "side": side, "diagnostics": diagnostics,
                "paths": (main_path, side_path, diagnostic_path)}

    # Exact historical behavior is a required control, not a numerical target
    # for the new policies. Its default file names and diagnostic schema stay old.
    snapshot = run("legacy_snapshot", "legacy-noq", 20, 20, 2025, default_prefix=True)
    for actual, suffix in zip(snapshot["paths"], (".csv", "_measurements.csv", "_qrcf_diagnostics.csv")):
        reference = args.snapshot_dir / f"after_20nodes{suffix}"
        assert reference.is_file(), reference
        assert actual.read_bytes() == reference.read_bytes(), f"Legacy differs from snapshot: {actual.name}"
    assert not list((output / "legacy_snapshot").glob("*_priority_v1_diagnostics.csv"))

    # The existing Gauss-Markov NormalVelocity is nonzero even when its mean
    # speed is zero. Call this full-coverage/zero-mean-speed, not stationary.
    # Strictly stationary discovery is checked by the C++ priority test suite.
    dense = run("full_coverage_zero_mean_speed", "staggered-noq", 12, 20, 2025,
                extra=("--MinSpeed=0", "--MaxSpeed=0", "--MaxRange=3000"), default_prefix=True)
    assert dense["record"]["HelloRx"] > 0 and dense["record"]["Delivered"] > 0, "Staggering must support discovery and data"
    assert all(int(d["HelloRx"]) > 0 and int(d["OpenDirectedLinks"]) > 0
               for d in dense["diagnostics"]), "Every full-coverage UAV must discover a live neighbor"
    assert not list((output / "full_coverage_zero_mean_speed").glob("fbcr_benchmark_tests_*"))

    cases = []
    for seed in args.seeds:
        paired = [run(f"mobile_{args.nodes}_{seed}_{preset}", preset, args.nodes, args.time, seed)
                  for preset in PRESETS]
        reference = paired[0]
        for case in paired[1:]:
            for key in ("Profile", "Seed", "UAVs", "Time", "MinSpeed", "MaxSpeed", "Channel", "Range",
                        "BsX", "BsY", "BsZ", "Offered", "MeasurementSeconds"):
                assert case["side"][key] == reference["side"][key], (seed, key)
            before = {d["Node"]: d for d in reference["diagnostics"]}
            for row in case["diagnostics"]:
                for key in (*NUMERIC_SETTINGS, "Epsilon", "DestinationLocationService"):
                    assert row[key] == before[row["Node"]][key], (seed, row["Node"], key)
        cases.extend(paired)

    records = [case["record"] for case in cases]
    assert build_fingerprints() == fingerprints, "Executable/module changed during validation"
    write_csv(output / "runs.csv", records)
    pairs = []
    for seed in args.seeds:
        baseline = next(r for r in records if r["Seed"] == seed and r["Preset"] == "legacy-noq")
        for record in (r for r in records if r["Seed"] == seed and r["Preset"] != "legacy-noq"):
            pairs.append({"Nodes": args.nodes, "Seed": seed, "Preset": record["Preset"],
                          "PDRChangePercentagePoints": record["AppPDRPct"] - baseline["AppPDRPct"],
                          "DelayChangePct": ratio_change(record["AvgDelayMs"], baseline["AvgDelayMs"]),
                          "SharedL3ControlChangePct": ratio_change(record["SharedL3ControlKbps"], baseline["SharedL3ControlKbps"]),
                          "EnergyChangePct": ratio_change(record["TotalEnergyJ"], baseline["TotalEnergyJ"]),
                          "ValueTx": record["ValueTx"], "ValueRx": record["ValueRx"],
                          "BootstrapUpdates": record["BootstrapUpdates"]})
    write_csv(output / "paired_changes.csv", pairs)
    means = []
    for preset in PRESETS:
        group = [r for r in records if r["Preset"] == preset]
        row = {"Preset": preset, "Runs": len(group), "Seeds": ";".join(str(r["Seed"]) for r in group)}
        for key in ("AppPDRPct", "AvgDelayMs", "SharedL3ControlKbps", "TotalEnergyJ", *WIRE_COUNTERS):
            row[f"Mean{key}"] = statistics.mean(r[key] for r in group)
        means.append(row)
    write_csv(output / "means.csv", means)
    (output / "checks.json").write_text(json.dumps({
        "legacy_snapshot_byte_identical": True,
        "full_coverage_zero_mean_speed": dense["record"],
        "paired_numeric_settings_and_epsilon_identical": True,
        "wire_l3_bytes": {"hello": 118, "feedback": 45, "value": 61},
        "all_socket_accepted_controls_observed_at_ip_tx": all(
            case["record"]["WireAccountingComplete"] for case in [snapshot, dense, *cases]),
        "default_preset": "outcome-bellman",
        "default_selection": "Specified before experiments; metrics do not choose the default",
        "strict_stationary_topology": "Covered by the C++ priority suite, not this Gauss-Markov benchmark case",
        "seeds": args.seeds,
    }, indent=2) + "\n")
    print(f"PASS all controls and {len(records)} paired runs; summaries: {output}", flush=True)


if __name__ == "__main__":
    main()
