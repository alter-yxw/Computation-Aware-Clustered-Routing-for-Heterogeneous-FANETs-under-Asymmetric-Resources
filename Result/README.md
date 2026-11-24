# Fifth-batch benchmark results

Per-run CSV results from the fifth benchmark batch, collected between
2026-11-14 and 2026-11-20. Each row is one simulation run; the files are not
pre-averaged. The common seed is `20261106`; independent repetitions are
`RngRun=1..20`.

Only the main 46-column result CSV of each run group is included. The
corresponding `.audit.log`, `.status.tsv`, failure directories and native
70-column rows are not part of this release.

## File index

| File | Rows | Content |
|---|---:|---|
| `cafbcr_scale_v5.csv` | 900 | Scale matrix, 9 objects x N100/200/300/400/500 x 20 runs |
| `cafbcr_speed_v5.csv` | 1080 | Speed matrix, 9 objects x 6 SWARM mean-speed ranges x 20 runs |
| `cafbcr_asym_v5.csv` | 1060 | Asymmetry matrix, 9 objects x L0-L4 plus CA-only P/R and fixed-range arms |
| `cafbcr_scale_v5_icra.csv` | 100 | ICRA supplement for the scale matrix (ICRA only) |
| `cafbcr_speed_v5_icra.csv` | 120 | ICRA supplement for the speed matrix (ICRA only) |
| `cafbcr_asym_v5_icra.csv` | 100 | ICRA supplement for the asymmetry matrix (ICRA only) |
| `cafbcr_server_selection_n200_v5.csv` | 40 | CA-FBCR native vs fixed server selection, N200 |
| `cafbcr_server_selection_cfr_n200_v5.csv` | 80 | CFR server-selection study, N200 |
| `cafbcr_server_selection_cfr_n500_v5.csv` | 80 | CFR server-selection study, N500 |
| `cafbcr_n50_offloading_shadow_v5.csv` | 240 | N50 offloading/shadowing ablation |

The scale matrix is the complete 900-row version. An earlier 891-row file
from the same batch was incomplete and is deliberately not included.

### ICRA supplement

ICRA was accidentally omitted from the main-batch run list. The three
`*_icra.csv` files were therefore collected separately shortly afterwards
(2026-11-16/17). The scenario parameters, seed, 20 `RngRun` repetitions,
hardware/software environment and build configuration are unchanged, and no
algorithm or protocol code was modified for the supplement. The files are kept
separate from the main matrices so that the original main-batch rows remain
untouched. When combining them, join by configuration identity (`Algorithm`,
`Nodes`, `Speed(m/s)`, `AsymLevel`, `AsymRangeProfile`, `Seed`, `RngRun`).

## Protocol labels

The three main matrices are the **nine-object paper run** and therefore
contain `FBCR` and `iabc` rows, plus the CA ablation labels
`ca-fbcr-noproxy`, `ca-fbcr-norange` and `ca-fbcr-noproxy-norange` in the
asymmetry matrix. FBCR and IABC source code is not part of this repository
(see the top-level README), so the released scripts reproduce the
seven-object subset only.

The `*_icra.csv` files are the separate ICRA supplement described above; they
contain `icra` rows only and are meant to be combined with the main matrix by
configuration identity.

## Schema

All files use the same 46-column clean schema:

```
Algorithm, Nodes, Speed(m/s), AsymLevel, ShadowSigma(dB), AsymRangeProfile,
RngRun, Seed, UDPAttemptPDR(%), UDPMeanDelay(ms), FlowPDR(%),
FlowMeanDelay(ms), AvgHops, ValidFlows, Throughput(kbps), OverHeadRatio,
ControlThroughput(kB/s), HelloRatio, AvgLinkLen(s), TotalEnergy(J),
AvgEnergy(J), DeadNodes, IsClusteringAlgo, TotalClusterNum, AvgClusterNum,
AvgClusteringRatio(%), AvgClusterSize, AvgClusterLifetime(s),
AvgClusterSwitchCount(/node), AvgRoleChangeCount(/node), TasksGenerated,
TasksCompleted, TaskCompletionRate(%), AvgResponseTime(ms),
AvgProcessingDelay(ms), AvgQueueWaitTime(ms), DeadlineMissRate(%),
DeadlineSatisfactionRate(%), TasksOnTime, TasksLate, TasksUnconfirmedDue,
TasksPending, TasksReceived, TasksProcessed, UDPAttempts, UDPReceived
```

The 46-column CSV does not carry a `BuildId`; build identity was recorded in
the original audit logs, which are not included here. Do not merge rows from
different batches or builds when computing statistics.
