# mognn

ns-3.40 adapter for MO-GNN, the congestion-aware multi-hop offloading policy of
Zhao et al. ("Congestion-aware Distributed Task Offloading in Wireless
Multi-hop Networks Using Graph Neural Networks", ICASSP 2024).

Three entry points are exposed through `scratch/ca-fbcr-test.cc`:

| `--protocol` | Behaviour |
|---|---|
| `mognn` | Learned BAT800 policy with the local task executor |
| `mognn-baseline` | No-congestion-aware variant |
| `mognn-local` | All tasks executed locally |

The adapter owns task wire format, descriptor exchange, link-rate estimation
and the local relay/control transport. It does not retrain the model and does
not reproduce the paper's analytical queuing figures; it is an interface and
policy port for FANET benchmarking.

## Reference

Z. Zhao, J. Perazzone, G. Verma, and S. Segarra, "Congestion-aware distributed
task offloading in wireless multi-hop networks using graph neural networks,"
in *Proc. IEEE International Conference on Acoustics, Speech and Signal
Processing (ICASSP)*, 2024, pp. 8951-8955,
doi: 10.1109/ICASSP48485.2024.10447302. arXiv:2312.02471.

```bibtex
@inproceedings{zhao2024mognn,
  author    = {Zhongyuan Zhao and Jake Perazzone and Gunjan Verma and
               Santiago Segarra},
  title     = {Congestion-Aware Distributed Task Offloading in Wireless
               Multi-Hop Networks Using Graph Neural Networks},
  booktitle = {ICASSP 2024 - 2024 IEEE International Conference on
               Acoustics, Speech and Signal Processing (ICASSP)},
  pages     = {8951--8955},
  year      = {2024},
  doi       = {10.1109/ICASSP48485.2024.10447302},
  note      = {arXiv:2312.02471}
}
```

## Upstream

`upstream/` contains the MIT-licensed reference implementation and the BAT800
checkpoint from <https://github.com/zhongyuanzhao/multihop-offload>, commit
`524bc4ef6af2a2a8c238b91470d10d2aeda14915`. The original `LICENSE` and
`PROVENANCE.json` are preserved. The ns-3 adapter code in this directory is
GPL-2.0-only.

## Model file

`models/BAT800.txt` is the exported text model read by `--moModel`.
`tools/` contains optional offline helpers for validation and checkpoint
export; TensorFlow is required only for those helpers, never for ns-3 runs.
