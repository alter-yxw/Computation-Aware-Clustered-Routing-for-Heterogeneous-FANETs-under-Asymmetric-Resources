# fanet

Shared FANET base classes used by CA-FBCR and the clustering baselines:

- `ClusterNode`: per-node state (role, cluster, position, velocity, energy,
  compute capacity, load) and the multi-factor fitness function;
- `Neighbors`: neighbor table with position, velocity, energy and compute
  observations;
- `ClusterManager` / `Cluster`: cluster membership, gateway bookkeeping and
  clustering statistics;
- `PerformanceMetrics`: flow-level and clustering-level metric aggregation;
- `AttackLayer`: optional protocol-agnostic packet-drop layer used by the
  secure-routing benchmark.

`ClusterNode::ComputeFitness` combines LHTP, residual energy, degree, distance
and (in heterogeneous mode) compute capacity. The weights are supplied by the
installed routing protocol.
