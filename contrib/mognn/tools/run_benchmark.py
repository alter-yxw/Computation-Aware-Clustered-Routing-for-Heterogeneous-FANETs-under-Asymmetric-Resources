#!/usr/bin/env python3
"""Run reproducible MO variants in ca-fbcr-test with a common task executor."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
NS3 = ROOT.parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--nodes', type=int, nargs='+', default=[25])
    parser.add_argument('--seeds', type=int, nargs='+', default=[2025])
    parser.add_argument('--protocols', nargs='+', default=['mognn', 'mognn-baseline', 'mognn-local'],
                        choices=['mognn', 'mognn-baseline', 'mognn-local', 'cafbcr-eng', 'cfr-eng'])
    parser.add_argument('--time', type=float, default=60.)
    parser.add_argument('--drain', type=float, default=20.)
    parser.add_argument('--task-mbit', type=float, default=.1)
    parser.add_argument('--flows', type=int, default=10)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--timeout', type=float, default=900.)
    args = parser.parse_args()
    destination = args.output.resolve()
    if destination.exists() and any(destination.iterdir()):
        parser.error('output must be new or empty; previous evidence is never overwritten')
    executable = NS3 / 'build/scratch/ns3.40-ca-fbcr-test-default'
    if not executable.is_file(): parser.error('Build ca-fbcr-test first')
    destination.mkdir(parents=True, exist_ok=True)
    rows, runs = [], []
    for n in args.nodes:
        for seed in args.seeds:
            for protocol in args.protocols:
                case = destination / f'n{n}-seed{seed}-{protocol}'; case.mkdir()
                cmd = [str(executable), f'--protocol={protocol}', f'--size={n}', f'--seed={seed}',
                       f'--time={args.time}', f'--drainTime={args.drain}', f'--taskMbit={args.task_mbit}',
                       f'--numTestFlows={args.flows}', '--cleanShutdown=true', f'--output={case / "results"}']
                if protocol == 'cfr-eng': cmd += ['--engCompute=destination']
                (case / 'command.json').write_text(json.dumps(cmd, indent=2) + '\n')
                print(f'Running N={n} seed={seed} {protocol}', flush=True)
                with (case / 'simulation.log').open('w') as log:
                    subprocess.run(cmd, cwd=NS3, stdout=log, stderr=subprocess.STDOUT, check=True, timeout=args.timeout)
                files = list((case / 'results').glob('engineering-benchmark_*.csv'))
                if len(files) != 1: raise RuntimeError('Missing benchmark CSV: ' + str(case))
                records = list(csv.DictReader(files[0].open()))
                if len(records) != 1: raise RuntimeError('Expected one benchmark row: ' + str(case))
                record = records[0]; record['case'] = str(case.relative_to(destination)); rows.append(record)
                runs.append({'nodes': n, 'seed': seed, 'protocol': protocol, 'passed': True})
    with (destination / 'summary.csv').open('w') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    manifest = {'passed': True, 'runs': runs, 'generation_s': args.time, 'drain_s': args.drain,
                'task_mbit': args.task_mbit, 'udp_flows': args.flows,
                'binary_sha256': hashlib.sha256(executable.read_bytes()).hexdigest(),
                'model_sha256': hashlib.sha256((ROOT / 'models/BAT800.txt').read_bytes()).hexdigest(),
                'paper_numerical_reproduction': False, 'statistical_superiority_established': False,
                'comparison_scope': 'engineering; CAF/CFR fixed target, MO includes destination selection'}
    manifest['shared_library_sha256'] = {
        p.name: hashlib.sha256(p.read_bytes()).hexdigest()
        for p in (NS3 / 'build/lib').glob('libns3.40-*-default.so')
    }
    (destination / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(destination / 'summary.csv')


if __name__ == '__main__':
    main()
