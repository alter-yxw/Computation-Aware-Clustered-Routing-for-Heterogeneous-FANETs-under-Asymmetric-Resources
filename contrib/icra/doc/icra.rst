ICRA reproduction and fidelity modes
------------------------------------

This module implements Guo et al., *ICRA: An Intelligent Clustering Routing
Approach for UAV Ad Hoc Networks*, IEEE T-ITS 24(2), 2023. Page numbers below
are PDF page indices, followed by printed pages.

Reference
---------

J. Guo, H. Gao, Z. Liu, F. Huang, J. Zhang, X. Li, and J. Ma, "ICRA: An
intelligent clustering routing approach for UAV ad hoc networks," *IEEE
Transactions on Intelligent Transportation Systems*, vol. 24, no. 2,
pp. 2447-2460, Feb. 2023, doi: 10.1109/TITS.2022.3145857.

.. code-block:: bibtex

   @article{guo2023icra,
     author  = {Jingjing Guo and Huamin Gao and Zhiquan Liu and Feiran Huang
                and Junwei Zhang and Xinghua Li and Jianfeng Ma},
     title   = {{ICRA}: An Intelligent Clustering Routing Approach for {UAV}
                Ad Hoc Networks},
     journal = {IEEE Transactions on Intelligent Transportation Systems},
     volume  = {24},
     number  = {2},
     pages   = {2447--2460},
     year    = {2023},
     doi     = {10.1109/TITS.2022.3145857}
   }

The default ``paper`` preset restores the paper's mechanism within the shared
benchmark scenario. It does not claim to reproduce the original OPNET curves.
``legacy`` retains the prior fixed-weight clustering and routing path so that
mechanism changes can be compared and rolled back without editing shared code.
Only ICRA and ICRA-specific benchmark configuration are changed.

Mechanisms retained from the paper
**********************************

* Equations 7--19: residual/initial energy, degree/(N-1), variance of velocity
  similarity, and mean link holding time. Own motion and energy come from live
  local models; neighbor information comes from received messages.
* Equation 6 uses the strategy received from the GS. Neither the routing
  fitness nor its weights are independently fixed at 0.25 in paper mode.
* Algorithm 2 starts with Q=1 for equal weights and Q=0 for other actions.
  The simplex on the 0.05 grid contains 1771 actions. Unvisited Q values are
  implicit, avoiding allocation of tens of millions of map entries.
* Equations 22--26 are evaluated for individual nodes and then averaged.
  Feedback counters are cumulative on the wire and differenced over each
  observation window. Equality with phi is unstable. Gamma is explicitly zero.
  The unusual exponential denominator in Equation 22 is preserved.
* Equation 27 chooses maximum Q and randomizes equal maxima using an assigned
  ns-3 stream. Paper mode does not add epsilon exploration.
* PDF p6/2452: nodes advertise utility, local maxima declare CH, other nodes
  request membership and wait for acceptance. Exhausted membership attempts
  fall back to CH without BioCluster density or initialization-time rules.
* PDF p6/2452, III-C.5.2 and III-C.5.5: a member losing its CH leaves that
  cluster and resumes Step 3; each new strategy update resumes Step 1, even
  when the weights are unchanged. These are the defaults of
  ``PaperRejoinOnly`` and ``PaperRebuildEveryPolicy`` respectively.
* A CM with a foreign-cluster neighbor requests IFN status from its CH.
  Request and acknowledgement are actual one-hop UDP control messages. The CH
  uses the IFN only after the member confirms its accepted role in a Hello.
  Roles and membership expire when neighbor observations become invalid.
* PDF p9/2455, Section III-E: direct destination first; CH chooses among its
  confirmed IFNs and neighbor CHs; IFN chooses among its CH and foreign-cluster
  neighbors; common CM uses its own CH. Distance is measured to the final
  destination, never to an oracle-provided remote CH.

Paper mode computes next hops from current local observations for each packet.
It does not reuse the legacy ten-second routing cache or the shared weighted
greedy selector. MAC/ARP failure invalidates the local next hop. IP TTL provides
the ordinary bound on loops; no extra perimeter or topology oracle is added.

Explicit completions and remaining assumptions
*********************************************

The following choices fill gaps or contradictions in the source. They must be
reported alongside experiments rather than described as literal paper details.

* Algorithm 1 line 8 prints Wi > Wk, conflicting with the preceding
  maximum-utility CH election. Paper mode follows the self-consistent body:
  join an advertised CH of at least the node's utility and LHTP > sigma.
  Eligible CHs are attempted in descending utility, then node-ID order.
  All tied local maxima may declare CH, as the election text allows.
* Election and JOIN compare the utility published for the current strategy.
  An early CH decision must not cause that CH's subsequent Hello messages to
  replace its published utility with a newly calculated value while other
  nodes are still selecting a CH. Position, velocity, residual energy and
  link eligibility continue to use live observations. The source distinguishes
  utility/declaration tables from the Hello neighbor table; preserving this
  distinction does not change the utility formula or the tie rule.
* Equations 9--18 are planar: speed/angle and LHTP use x/y, while radio-neighbor
  eligibility uses the benchmark's actual 3D distance. Angle differences wrap
  to [-pi,pi]. Variance is neither inverted nor stretched.
* Equation 14 lacks a finite treatment of infinite link lifetime and the
  factor scaling needed by the state bins. Each link lifetime is capped at
  50 seconds, averaged, and divided by 50. This horizon is a modeling choice.
* Equation 20's floor/mod expression is ambiguous. Factors use eleven floor
  bins on [0,1]; Shannon entropy is divided by log2(11), then floor(10H) is the
  state coordinate. This is not a literal modulus operation.
* Hello interval is 0.25 seconds with independent +/-10 ms jitter, neighbor
  timeout 1 second. Cached positions are extrapolated using their advertised
  velocity. Initial Hello phases span 0--250 ms. A full formation uses a
  0.27-second collection window: utility publication is at 0.27--0.37 seconds,
  election at 0.54 seconds, and CH selection at 0.81--1.30 seconds after its
  start. Selection is spread across 20 deterministic node-ID slots with an
  additional 0--15 ms jitter. The full-round budget is the smaller of 1.7
  seconds and the strategy period minus twice the GS delay minus 20 ms.
  Recovery skips utility publication/election, selects at 0.27--0.31 seconds,
  and uses at most 1.2 seconds. These times assume the default Hello interval.
* JOIN has up to three attempts per candidate. Normal waits are 100--140 ms;
  the first attempt to an unresolved ARP destination waits at least the ARP
  wait-reply timeout plus 140 ms, capped by the round's remaining budget minus
  20 ms. A late attempt may therefore have less than one ARP retry interval.
  Default ARP discovery is retained, and no acceptance or address mapping is
  injected. ``PaperArpAwareJoin=true`` applies this longer first wait even for
  resolved destinations; its default is false. Publication, contention jitter,
  acceptance timeouts and finite deadlines are simulator completions.
* Role-change feedback counts CH/IFN/common-CM category changes. Initial
  assignment and temporary Idle states during reconstruction are excluded.
  A CM changing CH without changing category is not given an extra role penalty.
* The destination-location service is ideal: a destination's current position
  is available. Remote cluster membership and CH locations are not consulted.
  The paper does not specify a destination-location distribution protocol.
* The GS is a logical, resource-unlimited controller on a separate ideal
  out-of-band channel. The first participating protocol hosts its computation,
  without becoming a special aerial routing role. Reports and policies are
  serialized, delivered after the configured delay, decoded, and counted.
  The learner consumes received reports rather than reading global node state.
  This channel models neither radio contention, coverage failures, IP/UDP
  overhead nor GS communication energy. These are explicit limitations.
* Reports are sampled at 2,4,6,... seconds. The controller runs after report
  delivery and disseminates the next strategy to each UAV; policy volume is
  counted per recipient rather than as one physical broadcast. The default one-way delay is
  1 ms, leaving an approximately 3 ms action-boundary offset. The first
  complete window is a warm-up. Large GS delays have not been validated as
  a delayed-control learning model.
* Every new policy epoch rebuilds by default, including equal weight vectors;
  duplicate epochs are ignored. A policy received during formation is deferred
  until that round finishes, so its in-flight JOIN and ACCEPT packets retain
  a consistent epoch. A full rebuild retains the committed association for
  forwarding until the new decision commits; loss-of-head recovery first
  leaves the old cluster. Live neighbor observations are retained subject to
  freshness and range checks, while candidacy requires the current epoch.
  These transition semantics fill gaps in the paper's message timing.
  ``PaperRebuildEveryPolicy=false`` and ``PaperRejoinOnly=false`` select older
  diagnostic behaviors. Reconfiguration buffering is bounded to 64 packets and
  one second; it does not provide a general disconnected-network recovery
  protocol.
* In the BS benchmark profile the external data sink is excluded from UAV
  election, degree denominator and learning. Adjacency to that explicit
  external endpoint may qualify a CM as IFN so that CH--IFN--BS delivery remains
  possible. This adapter supports the shared BS workload, which is not the
  paper's original UAV-to-UAV experiment.
* New A2A controls use a common 93-byte full-precision header plus a one-byte
  type. GS reports use 125+1 bytes; policies use 44+1 bytes. These are explicit
  wire formats chosen here, not recovered original packet sizes.

Parameters and benchmark use
****************************

PDF p9/2455 Table II specifies alpha=0.8, gamma=0, sigma=0.1, phi=2 and a
two-second feedback cycle. Paper mode uses these values unless an explicit
benchmark override is supplied. Lambda=0.5 is an explicit completion.
Legacy uses alpha=0.1, sigma=2, phi=3 and the old
ten-second episode. The predictor's radius is supplied from benchmark MaxRange
in paper mode; legacy keeps its old 500 m predictor.

Shared radio, motion, area, traffic, initial energy and battery models are not
replaced with the original paper's setup. In particular, the original
1500 seconds, 10x10 km area, 1 km radius and role-dependent constant power model
are not silently imposed on other baseline algorithms.

From the ns-3 root::

  ./ns3 configure --enable-tests
  cmake --build cmake-cache --target icra-tests scratch_benchmarktest -j 4
  build/utils/ns3.40-icra-tests-default --suite=icra --verbose

  build/scratch/ns3.40-benchmarktest-default --algorithm=icra --icraPreset=paper \
    --icraAudit=true --size=50 --time=30 --seed=2025 --csvPrefix=/tmp/icra-paper-
  build/scratch/ns3.40-benchmarktest-default --algorithm=icra --icraPreset=legacy \
    --size=50 --time=30 --seed=2025 --csvPrefix=/tmp/icra-legacy-

Select ``--profile=bs`` for the existing BS workload, and ``--MaxRange=750``
to check range propagation. ``--icraGsDelay`` sets the one-way ideal GS delay
in seconds and defaults to 0.001. ``--icraAlpha=-1`` preserves the preset's
learning rate; a value in [0,1] is an explicit diagnostic override.
``--icraStreamBase=-1`` preserves automatic stream allocation. A nonnegative
value assigns explicit ICRA streams; record it when comparing runs because it
can change random trajectories. None of these options changes the shared
scenario for another routing algorithm.

Without an explicit CSV prefix, ICRA uses ``icra_<preset>_<profile>_``.
The algorithm column remains ``icra``. Use a separate output prefix/directory
per experiment and record the preset, overrides, seed, binary and library
versions. Do not combine paper and legacy rows without identifying the preset.

``--icraAudit=true`` records live committed membership, singleton and small
clusters, unique application delivery, role changes and control counters in
``*_icra_cluster_samples.csv`` and ``*_icra_cluster_audit.csv``. Current schemas
are ``icra-cluster-samples-v3`` and ``icra-cluster-audit-v3``. Samples occur
every 0.25 seconds and once after the simulation stops; the node audit contains
one final row per node. The sample's global learner fields occur once per
snapshot. AppliedStrategies counts accepted policy epochs, including unchanged
weights; an accepted policy does not prove that a new action was selected.
OptimizerLastReward retains the last successful update's reward.
Serialization counters count transmission attempts (type plus header), not
successful delivery or L3 bytes. Historical flow statistics are sampled 0.1
seconds before the stop, so their counters may differ at the boundary.
The historical control-throughput column retains its historical
unit convention; it is not a corrected bit/s or L3-radio metric.

``clustering-ablation.rst`` defines the current passive metrics and identifies
the older six-case ablation workflow as historical. Its runner and benchmark
variant flags are not available in the current checkout. The earlier
``*_icra_paper_v1_diagnostics.csv`` files under ``fidelity-v1-validation`` are
historical artifacts, not an additional output of the current benchmark.

Reliability and rollback
***********************

Q models have a version and preset marker. Paper models do not silently load
the old feature definitions. Explicit zero Q entries override the implicit
equal-weight prior and survive save/load. Simulator::Destroy clears learner
and controller runtime state. Paper RNG uses ns-3 Seed/Run and either automatic
streams or the explicitly configured stream range.

Runtime rollback is ``--icraPreset=legacy``. It restores the historical
mechanism, while sparse storage and model persistence fixes remain; it does
not promise bit-for-bit identity with all old artifacts. The source patch in
``fidelity-v1-validation/reproduction.patch`` can also be reviewed or reversed,
subject to ordinary patch conflict checks. Never overwrite unrelated later
benchmark changes to restore this version.

Validation
**********

The 2026-10-22 published-utility correction, benchmark wiring updates and
paired multi-seed results are documented in ``refinement-20261022.rst``.
The reproducible passive-audit runner is ``scripts/icra-refinement.py``.

The native ICRA suite replaces the former constant assertions. It exercises
the action grid, prior, formulas, live motion/radius, reported state and
individual feedback, RNG reproducibility, model roundtrip and lifecycle,
control serialization, direct/CH/IFN/CM candidate rules, real JOIN/IFN
handshakes, multi-hop UDP, external-BS exclusion and changed-policy application.
The original validation commands and observed results are recorded under
``fidelity-v1-validation``; those artifacts describe that earlier source
revision. Later changes require their own recorded validation. Smoke tests
establish executability and mechanism
coverage, not statistical superiority of one baseline over another.
