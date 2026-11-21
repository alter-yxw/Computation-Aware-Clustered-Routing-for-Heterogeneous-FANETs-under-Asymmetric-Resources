#!/usr/bin/env python3
"""Check real-radio scenarios, rejected options and benchmark accounting."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
NS3 = ROOT.parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--benchmark', type=Path, required=True)
    parser.add_argument('--output', type=Path, default=ROOT / 'validation/integration-check.json')
    args = parser.parse_args()
    executable = NS3 / 'build/scratch/ns3.40-ca-fbcr-test-default'
    chain = NS3 / 'build/scratch/ns3.40-mognn-check-default'
    checks = {}
    for mode in ['mognn', 'mognn-baseline', 'mognn-local']:
        result = subprocess.run([str(chain), f'--mode={mode}'], cwd=NS3, text=True, capture_output=True, check=True, timeout=60)
        assert 'PASS' in result.stdout
        checks[mode + '_radio'] = result.stdout.strip()
    negative = {
        'missing_model': ['--protocol=mognn', '--moModel=/tmp/mo-nonexistent-policy.txt'],
        'invalid_slot': ['--protocol=mognn', '--moSlotSeconds=-1'],
        'incompatible_fixed_target': ['--protocol=mognn', '--fixedTaskDestination=true'],
        'incompatible_execution': ['--protocol=mognn', '--engCompute=joint'],
        'mo_option_on_legacy': ['--protocol=olsr', '--moSlotSeconds=.1'],
    }
    for name, options in negative.items():
        result = subprocess.run([str(executable)] + options, cwd=NS3, text=True, capture_output=True, timeout=30)
        assert result.returncode == 1, (name, result.stdout, result.stderr)
        checks[name] = 'rejected: ' + result.stderr.strip()
    with tempfile.TemporaryDirectory(prefix='mo-legacy-check-') as td:
        # The legacy program writes relative outputs; isolate its read-only
        # regression smoke run without changing legacy output semantics.
        result = subprocess.run([str(executable), '--protocol=olsr', '--numCommand=1', '--numRelay=1',
                                 '--numTask=1', '--numSwarm=2', '--numTestFlows=0', '--time=12',
                                 '--enableComputeTask=false', '--cleanShutdown=true'],
                                cwd=td, text=True, capture_output=True, timeout=60)
        assert result.returncode == 0, result.stderr
        checks['legacy_olsr_smoke'] = True
    benchmark = args.benchmark.resolve()
    manifest = json.loads((benchmark / 'manifest.json').read_text())
    assert manifest['passed']
    assert manifest['binary_sha256'] == hashlib.sha256(executable.read_bytes()).hexdigest()
    for name, expected in manifest['shared_library_sha256'].items():
        assert hashlib.sha256((NS3 / 'build/lib' / name).read_bytes()).hexdigest() == expected
    rows = list(csv.DictReader((benchmark / 'summary.csv').open()))
    for row in rows:
        case = benchmark / row['case'] / 'results'
        ledger_paths = list(case.glob('engineering-tasks_*.csv')); assert len(ledger_paths) == 1
        tasks = list(csv.DictReader(ledger_paths[0].open()))
        assert len(tasks) == int(row['TasksGenerated'])
        completed = [t for t in tasks if t['status'] == 'complete' and t['acknowledged'] == '1']
        assert len(completed) == int(row['TasksCompleted'])
        assert all(float(t['remaining_cycles']) == 0 and float(t['compute_s']) > 0 for t in completed)
        if row['Algorithm'] == 'mognn-local': assert all(t['source'] == t['destination'] for t in tasks)
        if row['Algorithm'] == 'mognn':
            assert hashlib.sha256((case / 'mo-model.txt').read_bytes()).hexdigest() == manifest['model_sha256']
        if row['Algorithm'].startswith('mognn'):
            decisions = [d for p in case.glob('mo-decisions-node*.csv') for d in csv.DictReader(p.open())]
            assert len(decisions) == len(tasks)
    checks['benchmark_rows_and_ledgers'] = len(rows)
    report = {'passed': True, 'checks': checks, 'binary_and_shared_libraries_match': True,
              'benchmark': str(benchmark.relative_to(ROOT)), 'paper_numerical_reproduction': False}
    args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
