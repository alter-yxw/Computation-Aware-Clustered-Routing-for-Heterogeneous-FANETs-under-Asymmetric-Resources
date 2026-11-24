> **Status:** This work is under submission to *Computer Networks* and may be
> published there in the future.

# CA-FBCR

Computation-aware clustering and routing for flying ad hoc networks (FANETs),
implemented as ns-3.40 modules, together with the benchmark entry point and the
three experiment matrices used in the paper.

Repository: <https://github.com/alter-yxw/Computation-Aware-Clustered-Routing-for-Heterogeneous-FANETs-under-Asymmetric-Resources>

CA-FBCR extends a fuzzy-logic cluster-head election and a two-layer beacon
maintenance scheme with:

- heterogeneous node and compute-capacity modelling;
- compute-aware cluster-head election and task-server selection;
- asymmetric-link handling (proxy join, range gating, link evidence);
- fairness and control-overhead accounting across heterogeneous node classes.

The repository is an **overlay** on the official ns-3.40 release. It contains
only the modules, the simulation entry point, the experiment scripts and four
patches against `src/`. It does not vendor a complete ns-3 tree.

## Repository layout

```
contrib/
  cafbcr/     CA-FBCR routing protocol, election, fuzzy/Q-learning Hello,
              compute-task application, scenario configuration
  fanet/      ClusterNode, neighbor table and cluster manager (base classes)
  qrcf/       QRCF baseline
  icra/       ICRA baseline
  cfr/        CFR-ENG baseline (engineering, no PDDQN inference)
  mognn/      MO-GNN baseline: ns-3 adapter plus upstream MIT code and weights
scratch/
  ca-fbcr-test.cc            benchmark entry point (all protocols)
  ca-fbcr-*.h                passive observers and scenario helpers
scripts/
  cafbcr_scale_matrix_v5.sh  node-count matrix
  cafbcr_speed_matrix_v5.sh  speed matrix
  cafbcr_asym_matrix_v5.sh   asymmetry matrix
patches/ns-3.40/             four patches against the official ns-3.40 src/
docs/ns3-changes.md          what each patch changes and why
Result/                      fifth-batch per-run CSV results
  README.md                  file index, row counts and schema
apply_to_ns3.sh              copies this overlay onto a clean ns-3.40 tree
```

## Development environment

- Ubuntu 22.04 LTS, 64 GB RAM;
- GCC 11 with C++17, CMake, Python 3;
- the official **ns-3.40** source tree.

No TensorFlow is needed to run ns-3 simulations. TensorFlow is only required
if you execute the original MO-GNN reference model outside ns-3.

## Quick start

> **Back up the original ns-3.40 tree before applying this overlay.**
> `apply_to_ns3.sh` patches `src/` in place and copies files into `contrib/`,
> `scratch/` and the tree root; it does not provide an automatic rollback.
> Work on a fresh copy, keep an untouched archive, or run `git init` in the
> ns-3.40 tree before applying so you can revert with `git checkout -- .`.

```bash
# 1. Unpack the official release and keep the original tree untouched.
tar -xjf ns-allinone-3.40.tar.bz2
cp -a ns-allinone-3.40/ns-3.40 ns-3.40-cafbcr
cd ns-3.40-cafbcr

# 2. Apply the four src/ patches and copy the overlay.
/path/to/cafbcr-open-source/apply_to_ns3.sh "$PWD"

# 3. Configure and build.
./ns3 configure --build-profile=default --enable-asserts --enable-logs \
    --disable-examples --disable-tests \
    --enable-modules='cafbcr;fanet;qrcf;cfr;icra;mognn;applications;internet;wifi;mobility;energy;flow-monitor;olsr;aodv' \
    -- -DNS3_FETCH_OPTIONAL_COMPONENTS=OFF -DNS3_CCACHE=OFF
./ns3 build ca-fbcr-test -j2

# 4. Run a short CA-FBCR example.
./ns3 run --no-build 'scratch/ca-fbcr-test --protocol=cafbcr --size=100 \
    --time=60 --seed=2026 --RngRun=1 --csvOnly=1 \
    --resultsCsv=Result/example.csv'
```

The result row is appended to `Result/example.csv`. Longer runs use the
matrices below; they create their own CSV, audit log and status file.

## Protocols

`--protocol=` accepts the following values:

| Value | Meaning |
|---|---|
| `cafbcr` | CA-FBCR. `--helloMethod=fixed|fuzzy|qlearning` selects the Hello mode. |
| `qrcf` | QRCF baseline (`--qrcfPreset=...`). |
| `icra` | ICRA baseline. |
| `cfr-eng` | CFR-ENG engineering variant (`--cfrEngCompute=joint|source|destination`). |
| `mognn`, `mognn-baseline`, `mognn-local` | MO-GNN learned policy, no-congestion baseline and all-local variant. |
| `olsr` | OLSR baseline (`--olsrHelloInterval`). |
| `aodv` | Stock AODV, retained for historical diagnostics only. |

The CSV label is `ca-fbcr` for `cafbcr --helloMethod=fuzzy`, `ca-qbcr` for
`qlearning` and `ca-bcr` for `fixed`. The matrices use these three CA variants
plus `qrcf`, `cfr-eng` and `mognn`; `olsr` is the routing baseline. ICRA is
available as an additional baseline but is not part of the three matrices.

The published matrices contain **seven objects rather than the nine used in the
paper**: FBCR (the authors' prior work) and IABC (see the note below) are not
included. The scripts and the counts below describe exactly the seven-object
set they run.

## Reproducing the matrices

The three scripts are intentionally independent. Run one group at a time; each
supports `--plan`, `--check`, `--run` and `--resume`. Start with a zero-node
preflight:

```bash
bash cafbcr_scale_matrix_v5.sh --check --no-build
bash cafbcr_speed_matrix_v5.sh --check --no-build
bash cafbcr_asym_matrix_v5.sh --check --no-build
```

`--check` creates no nodes and no formal CSV. Expected plan sizes for the
public seven-object set are:

| Group | Grid | Runs | Rows |
|---|---|---:|---:|
| Scale | 7 objects x N100/200/300/400/500 | 20 | 700 |
| Speed | 7 objects x six SWARM mean-speed ranges | 20 | 840 |
| Asymmetry | 7 objects x L0-L4, plus CA-only P/R and fixed-range arms | 20 | 860 |

The common reference point (N200, 20-60 m/s, L4, sigma=4) has 140 rows.
Formal runs are author-started and must keep one build ID per CSV:

```bash
bash cafbcr_scale_matrix_v5.sh --resume --no-build
bash cafbcr_speed_matrix_v5.sh --resume --no-build
bash cafbcr_asym_matrix_v5.sh --resume --no-build
```

Each group writes one 46-column main CSV, a same-name `.audit.log` with the
full native rows and commands, and a `.status.tsv`. Failed or interrupted runs
are retained under `.csv.failures/`.

## Published results

`Result/` contains the per-run 46-column CSVs from the fifth benchmark batch
(2026-11-14 to 2026-11-20): the scale, speed and asymmetry matrices, the three
ICRA supplement files, and the server-selection/offloading ablations. See
[Result/README.md](Result/README.md) for the file index, row counts and schema.

The three main matrices are the **nine-object paper run** and therefore include
`FBCR` and `iabc` rows even though their source code is not part of this
release. The scripts in this repository reproduce the seven-object subset.

ICRA was accidentally omitted from the main-batch run list and was collected
separately shortly afterwards; the scenario, environment and code were
unchanged. The three ICRA files are kept separate and are intended to be joined
by configuration identity.

## Scenario summary

Unless overridden by a matrix:

- 350 s simulation, tasks generated on [30, 150) s with `bounded-1s` starts;
- UDP test flows on [30, 150) s, 50 flows, 512-byte payloads, 0.5 s interval;
- LogDistance path loss, exponent 2, correlated log-normal shadowing with
  sigma = 4 dB, no hard distance cutoff;
- heterogeneous node classes: COMMAND, RELAY, TASK and SWARM, each with its own
  compute capacity, battery, TX power and task profile;
- a common fixed task-server assignment across protocols (`--taskSelection=fixed`)
  and a common terminal; native server selection is available separately;
- ARP policy is explicit and identical for every protocol in the published
  matrices: `--arpDeadReprobe=0` (stock ns-3.40 behaviour), three
  retransmissions, 1000 ms wait. The DEAD-entry re-probe is only active for
  MO-GNN runs that opt in to a non-zero interval.

See the `--help` output of `scratch/ca-fbcr-test` for the complete switch set.

## Why stock ns-3.40 is patched

The overlay changes 16 files under `src/`, grouped into four patches. **None of
the changes modifies a routing mechanism or algorithm.**

### AODV and OLSR: statistics interfaces only

The patches add passive counters and accessors for control-packet counts,
HELLO counts, control throughput and link lifetime, so the benchmark can
report control-overhead and link-duration columns. They also correct an
incoming-interface index lookup (device `IfIndex` versus IPv4 interface
index). No routing decision, timer, election rule or forwarding mechanism is
changed.

### WiFi: energy accounting and memory

With the 0.273 A current setting used by the energy model, stock ns-3.40 stops
updating energy consumption at about **122 s of a 200 s run**: the simulation
continues, but the energy counter freezes. The patch fixes the PHY OFF and
energy-model state handling so accounting runs to the end of the simulation.
It also replaces the energy-model switch-off event's `Cancel()` with
`Simulator::Remove()`; this leaves the simulation results unchanged while
substantially reducing DRAM use during long runs.

### ARP: DEAD-entry re-probe (MO-GNN only)

This change was introduced while porting MO-GNN. It adds a configurable
`ArpL3Protocol::DeadReprobeInterval` attribute so a DEAD ARP entry can be
re-probed after a bounded interval instead of being dropped until the dead
timeout in multi-hop paths.

It is **opt-in and has no effect on the other protocols**: the attribute
defaults to 0, which preserves stock ns-3.40 behaviour, and the three published
matrices pass `--arpDeadReprobe=0`. CA-FBCR, QRCF, ICRA, CFR-ENG and OLSR
results do not depend on this patch.

If you do not need to reproduce MO-GNN, you can skip this patch:

```bash
SKIP_ARP_PATCH=1 ./apply_to_ns3.sh /path/to/ns-3.40
```

Keep `--arpDeadReprobe=0` (the default) in that case.

See [docs/ns3-changes.md](docs/ns3-changes.md) for the per-patch file list and
more detail.

## MO-GNN attribution

`contrib/mognn/upstream/` contains the MIT-licensed reference implementation
and the BAT800 checkpoint from
<https://github.com/zhongyuanzhao/multihop-offload> (commit
`524bc4ef6af2a2a8c238b91470d10d2aeda14915`). Its original `LICENSE` and
`PROVENANCE.json` are preserved. The ns-3 adapter and the routing module are
released under GPL-2.0-only, like the rest of this repository. MO-GNN is a port
and interface adaptation, not a numerical reproduction of the paper's figures.

## Scope of this release

Included: CA-FBCR, FANET base classes, QRCF, ICRA, CFR-ENG, MO-GNN, the
benchmark entry point, three matrices, the fifth-batch CSV results and the
ns-3.40 patches.

Not included:

- FBCR (the authors' prior work);
- IABC (see the note below);
- KMeans, LEACH, PAGPSR and the old location-service module;
- internal design documents, development hand-offs and manuscript sources;
- per-run audit logs, status files and failure directories for the published
  CSVs (only the 46-column main CSVs are included);
- publisher PDFs of the cited ICRA and QRCF papers; the module documentation
  gives the bibliographic references only;
- MOGNN validation artefacts (the 27 MB `validation/` directory).

## IABC note

IABC is **not included** in this public release. Its original prototype is
<https://github.com/MuhammadBinSaif/FANET-Routing>. In our paper we refactored
and restructured that prototype into a standard ns-3 module for the benchmark.
The upstream repository does not declare a license, so the refactored module is
withheld here pending clarification.

The corresponding paper is: L. Zhao, M. B. Saif, A. Hawbani, G. Min, S. Peng,
and N. Lin, "A novel improved artificial bee colony and blockchain-based
secure clustering routing scheme for FANET," *China Communications*, vol. 18,
no. 7, pp. 103-116, Jul. 2021, doi: 10.23919/JCC.2021.07.009.

## Verification status

Checked on 2026-11-21 against a clean official ns-3.40 tree:

- the overlay applies and the four patches apply cleanly;
- `ca-fbcr-test` builds with the module list above;
- `--check --no-build` reports 700 / 840 / 860 planned rows, zero saved rows
  and zero nodes created for the scale, speed and asymmetry groups;
- short 20-node runs of `cafbcr`, `qrcf`, `icra`, `cfr-eng`, `olsr` and
  `mognn` complete and write their result CSV.

Module unit-test suites are included but were not re-run for this release
check; enable ns-3 tests and use the standard `test.py` runner to execute them.

## License

GPL-2.0-only, matching ns-3. See [LICENSE](LICENSE). MO-GNN `upstream/` code
remains under the MIT license shipped in that directory.
