# cafbcr

CA-FBCR routing protocol for ns-3.40.

The module provides:

- `cafbcr::RoutingProtocol`: cluster-based IPv4 routing with intra-cluster,
  inter-cluster, greedy and perimeter forwarding;
- `cafbcr::cafbcrElection`: ABC-based cluster-head election over the
  LHTP/energy/degree/distance/compute fitness;
- `cafbcr::FuzzyHelloManager` and `cafbcr::QLearningHelloManager`: adaptive
  Hello intervals;
- `cafbcr::ComputeTaskClientApp` / `ComputeTaskServerApp`: task generation,
  server selection, FIFO service and completion accounting;
- `cafbcr::ScenarioConfig`: heterogeneous UAV node profiles and scenario
  parameters;
- helpers, packet headers, tags and twenty unit-test suites.

The benchmark entry point is `scratch/ca-fbcr-test.cc` in the repository root.

## Hello modes and CSV labels

| `--helloMethod` | CSV label |
|---|---|
| `fuzzy` | `ca-fbcr` |
| `qlearning` | `ca-qbcr` |
| `fixed` | `ca-bcr` |

## Tests

After configuring ns-3 with `--enable-tests`:

```bash
./ns3 build cafbcr-test
./ns3 run 'test-runner --suite=cafbcr'
```
