# cfr

CFR-ENG baseline, adapted to the shared ns-3.40 task contract.

The module contains the CFR packet/routing model and an engineering variant
that combines geographic forwarding with a local compute-time heuristic. It
does **not** perform PDDQN inference, checkpoint loading or training.

Use it through the shared benchmark entry point:

```bash
./ns3 run --no-build 'scratch/ca-fbcr-test --protocol=cfr-eng \
    --taskSelection=fixed --cfrEngCompute=joint'
```

`--cfrEngCompute=joint` (default), `source` and `destination` select the
computation placement. `helper/cfr-eng-local-adapter.*` adapts the local task,
capacity, queue and observation state to the shared client; workload and
capacity both use 16 FLOPs/cycle.

## Reference

N. Lin, J. Huang, A. Hawbani, L. Zhao, H. Tang, Y. Guan, and Y. Sun, "Joint
routing and computation offloading based deep reinforcement learning for
flying ad hoc networks," *Computer Networks*, vol. 249, p. 110514, Jul. 2024,
doi: 10.1016/j.comnet.2024.110514.

```bibtex
@article{lin2024cfr,
  author  = {Na Lin and Jinjiao Huang and Ammar Hawbani and Liang Zhao and
             Hailun Tang and Yunchong Guan and Yunhe Sun},
  title   = {Joint Routing and Computation Offloading Based Deep
             Reinforcement Learning for Flying Ad Hoc Networks},
  journal = {Computer Networks},
  volume  = {249},
  pages   = {110514},
  year    = {2024},
  doi     = {10.1016/j.comnet.2024.110514}
}
```
