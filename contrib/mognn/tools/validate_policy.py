#!/usr/bin/env python3
"""Compare C++ predictions, queue weights and decisions to the author pipeline.

Default: independent NumPy K=1 forward using checkpoint tensors + unmodified
author offloading() on three bundled network instances, ten task draws each.
--tensorflow: additionally run the actual author ACOAgent/Spektral forward().
"""
import argparse
import contextlib
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import numpy as np
import scipy.io as sio
import scipy.sparse as sparse
import networkx as nx

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'upstream'))
from offloading_v3 import AdhocCloud
from util import all_pairs_shortest_paths


def read_model():
    tokens = iter((ROOT / 'models/BAT800.txt').read_text().split())
    assert next(tokens) == 'MO_UPSTREAM_V1'
    count, order = int(next(tokens)), int(next(tokens))
    assert order == 1
    layers = []
    for _ in range(count):
        ni, no = int(next(tokens)), int(next(tokens))
        w = np.array([float(next(tokens)) for _ in range(ni * no)]).reshape(ni, no)
        b = np.array([float(next(tokens)) for _ in range(no)])
        layers.append((w, b))
    return layers


def predict(x, layers):
    for i, (w, b) in enumerate(layers):
        x = x @ w + b
        x = np.where(x >= 0, x, x * (0 if i == len(layers) - 1 else float(np.float32(.2))))
    return x[:, 0]


def reference(env, obj, layers, learned, agent):
    physical = [(int(a), int(b), float(env.link_rates[i])) for i, (a, b) in enumerate(env.link_list)]
    virtual = np.flatnonzero(env.proc_bws > 0)
    features = [[0, rate, 0, 0] for a, b, rate in physical]
    features += [[1, env.proc_bws[i], sum(j.arrival_rate * j.ul_data for j in env.jobs if j.source_node == i),
                  int(env.roles[i] == 1)] for i in virtual]
    y = predict(np.asarray(features), layers) if learned else np.array([])
    cpu = np.full(env.num_nodes, np.inf)
    if learned:
        x = y[:env.num_links]
        mu = env.link_rates / (1 + env.cf_degs)
        for _ in range(10):
            mu = env.link_rates / (1 + env.adj_i @ np.clip(x / mu, 0, 1))
        with np.errstate(divide='ignore'):
            link = np.where(mu > x, 1 / (mu - x), env.T * x / (101 * mu))
            xcpu = y[env.num_links:]
            cpu[virtual] = np.where(env.proc_bws[virtual] > xcpu, 1 / (env.proc_bws[virtual] - xcpu),
                                    env.T * xcpu / (100 * env.proc_bws[virtual]))
        if agent is not None:
            # This calls the unchanged upstream TensorFlow forward and compares
            # its gathered physical/virtual weights with our independent reference.
            state, matrix_ts, _ = agent.forward(obj, env)
            # The released NumPy return uses fill_diagonal with fewer than N
            # CPU values when relays exist, causing repetition/misalignment.
            # Its tensor return correctly scatters by comp_nodes. Compare that
            # correctly indexed return; this intentional port fix is documented.
            matrix = matrix_ts.numpy()
            raw = agent.act(state).numpy()[:, 0]
            raw_ordered = list(raw[obj.maps_ol_el]) + list(raw[obj.maps_on_el])
            np.testing.assert_allclose(raw_ordered, y, rtol=1e-10, atol=1e-11)
            np.testing.assert_allclose([matrix[a, b] for a, b, _ in physical], link, rtol=1e-10, atol=1e-11)
            np.testing.assert_allclose(np.diag(matrix), cpu, rtol=1e-10, atol=1e-11)
            for key, a, expected in [('prediction', np.asarray(raw_ordered), y),
                                      ('link_delay', np.asarray([matrix[a, b] for a, b, _ in physical]), link),
                                      ('cpu_delay', np.diag(matrix), cpu)]:
                finite = np.isfinite(expected)
                if finite.any():
                    agent.parity_errors[key] = max(agent.parity_errors.get(key, 0.),
                        float(np.max(np.abs(a[finite] - expected[finite]))))
    else:
        link = 1 / env.link_rates
        cpu[virtual] = 1 / env.proc_bws[virtual]
    for (a, b, _), weight in zip(physical, link): env.graph_c[a][b]['delay'] = weight
    distance = all_pairs_shortest_paths(env.graph_c, 'delay')
    hop = all_pairs_shortest_paths(env.graph_c)
    sp = distance.copy(); np.fill_diagonal(sp, cpu)
    # Execute the author's unchanged decision function, including its cost lower bounds.
    decisions, _ = env.offloading(sp, hop)
    return physical, y, link, cpu, distance, np.asarray(decisions)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tensorflow', action='store_true')
    parser.add_argument('--output', type=Path, default=ROOT / 'validation/policy-parity.json')
    args = parser.parse_args()
    agent = None
    if args.tensorflow:
        from gnn_offloading_agent import ACOAgent
        flags = SimpleNamespace(learning_rate=1e-6, weight_decay=5e-4, epsilon=0,
                                num_layer=5, dropout=0., learning_decay=1.)
        with contextlib.redirect_stdout(io.StringIO()):
            agent = ACOAgent(flags)
            agent.parity_errors = {}
            agent.model.load_weights(str(ROOT / 'upstream/model_ChebConv_BAT800_a5_c5_ACO_agent/cp-0000.ckpt'))
    layers = read_model()
    errors = {'prediction': 0., 'link_delay': 0., 'cpu_delay': 0., 'distance': 0.}
    cases, decisions_checked = 0, 0
    with tempfile.TemporaryDirectory(prefix='mo-policy-') as td:
        driver = Path(td) / 'policy-driver'
        subprocess.run(['g++', '-std=c++17', '-O2', str(ROOT / 'test/policy-driver.cc'),
                        str(ROOT / 'model/mo-policy.cc'), '-o', str(driver)], check=True)
        for filename in sorted((ROOT / 'upstream/data').glob('*.mat')):
            data = sio.loadmat(filename)
            info = data['nodes_info']
            np.random.seed(503)
            env = AdhocCloud(len(info), 1000, 500, cf_radius=0., gtype=str(filename))
            # NetworkX 3 returns csr_array; the author's row indexing expects
            # the NetworkX 2 csr_matrix API. No change to policy calculations.
            env.adj_c = sparse.csr_matrix(env.adj_c)
            env.adj_i = sparse.csr_matrix(env.adj_i)
            env.links_init(data['link_rate'].flatten())
            for i, row in enumerate(info):
                if row[0] == 2: env.add_relay(i)
                elif row[0] == 1: env.add_server(i, float(row[1]))
                else: env.proc_bws[i] = row[1]
            for draw in range(10):
                env.clear_all_jobs()
                mobile = np.flatnonzero(info[:, 0] == 0)
                selected = np.random.permutation(mobile)[:max(1, int(len(mobile) * (.3 + .06 * draw)))]
                for i in selected: env.add_job(int(i), rate=float(np.random.uniform(.015, .075)))
                obj = env.graph_expand()
                for learned in [False, True]:
                    physical, y, link, cpu, distance, decisions = reference(env, obj, layers, learned, agent)
                    arrivals = [sum(j.arrival_rate * j.ul_data for j in env.jobs if j.source_node == i) for i in range(env.num_nodes)]
                    lines = [f'{env.num_nodes} {env.num_links} {int(learned)} 1000 100 1']
                    lines += [f'{env.proc_bws[i]:.17g} {arrivals[i]:.17g} {int(env.roles[i] == 1)}' for i in range(env.num_nodes)]
                    lines += [f'{a} {b} {rate:.17g}' for a, b, rate in physical]
                    result = subprocess.run([str(driver), str(ROOT / 'models/BAT800.txt')], input='\n'.join(lines) + '\n',
                                            text=True, capture_output=True, check=True).stdout.splitlines()
                    actual = [np.fromstring(result[i], sep=' ') for i in range(3)]
                    actual.append(np.vstack([np.fromstring(row, sep=' ') for row in result[3:3 + env.num_nodes]]))
                    for key, a, expected in zip(errors, actual, [y, link, cpu, distance]):
                        np.testing.assert_allclose(a, expected, rtol=1e-10, atol=1e-11)
                        finite = np.isfinite(expected)
                        if finite.any(): errors[key] = max(errors[key], float(np.max(np.abs(a[finite] - expected[finite]))))
                    chosen = np.fromstring(result[-1], dtype=int, sep=' ')
                    np.testing.assert_array_equal(chosen[selected], decisions)
                    decisions_checked += len(selected); cases += 1
        # Disconnected graph: the C++ adaptation must select reachable local service.
        case = '2 0 1 1000 100 1\n1 0 0\n100 0 1\n'
        result = subprocess.run([str(driver), str(ROOT / 'models/BAT800.txt')], input=case,
                                text=True, capture_output=True, check=True).stdout.splitlines()
        assert result[-1].split()[0] == '0'
        bad = Path(td) / 'bad.txt'; bad.write_text('MO_UPSTREAM_V1 5 2\n')
        assert subprocess.run([str(driver), str(bad)], input=case, text=True, capture_output=True).returncode != 0
    report = {'passed': True, 'cases': cases, 'decisions_checked': decisions_checked,
              'network_sizes': [20, 50, 110], 'max_absolute_error': errors,
              'actual_tensorflow_spektral_forward': args.tensorflow,
              'unmodified_author_offloading': True, 'disconnected_graph': True,
              'cpu_mapping': 'explicit node indices; matches upstream tensor, fixes upstream numpy diagonal',
              'unsupported_checkpoint_rejected': True, 'paper_numerical_reproduction': False}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if agent is not None: report['tensorflow_vs_numpy_max_absolute_error'] = agent.parity_errors
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
