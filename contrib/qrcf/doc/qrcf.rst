QRCF reproduction in ns-3.40
============================

QRCF is an independent IPv4 routing module, not a clustering algorithm. It
implements adaptive neighbor discovery, cylindrical candidate filtering,
destination-indexed local action-value learning, local-minimum feedback and
actual data-outcome learning. The implementation uses destination-aligned
filtering and does not exchange Q tables in Hello messages. ``QrcfHelper``
installs ``qrcf::RoutingProtocol`` without a clustering dependency.

Reference
---------

A. M. Rahmani, A. Haider, M. M. Zaidi, A. Alanazi, S. Alsubai, A. Alqahtani,
M. S. Yousefpoor, E. Yousefpoor, and M. Hosseinzadeh, "QRCF: A new
Q-learning-based routing approach using a smart cylindrical filtering system
in flying ad hoc networks," *Vehicular Communications*, vol. 53, p. 100905,
Jun. 2025, doi: 10.1016/j.vehcom.2025.100905.

.. code-block:: bibtex

   @article{rahmani2025qrcf,
     author  = {Amir Masoud Rahmani and Amir Haider and Monji Mohamed Zaidi and
                Abed Alanazi and Shtwai Alsubai and Abdullah Alqahtani and
                Mohammad Sadegh Yousefpoor and Efat Yousefpoor and
                Mehdi Hosseinzadeh},
     title   = {{QRCF}: A New Q-Learning-Based Routing Approach Using a Smart
                Cylindrical Filtering System in Flying Ad Hoc Networks},
     journal = {Vehicular Communications},
     volume  = {53},
     pages   = {100905},
     year    = {2025},
     doi     = {10.1016/j.vehcom.2025.100905}
   }

The implementation follows the rotation matrix and algorithm diagrams in the
paper.

Run the shared benchmark
------------------------

From the ns-3.40 directory::

  ./ns3 build benchmarktest qrcf-tests -j 4
  ./ns3 run 'benchmarktest --algorithm=qrcf --size=100 --time=200'

The default preset is ``outcome-bellman``. Roll back without rebuilding::

  ./ns3 run 'benchmarktest --algorithm=qrcf --qrcfPreset=legacy-noq --size=100 --time=200' --no-build

``--qrcfPreset`` selects ``legacy-noq``, ``staggered-noq``,
``outcome-immediate`` or ``outcome-bellman``. The first retains the old
``qrcf-aligned-noq`` label, schema, default prefix and behavior. New presets
use labels ``qrcf-aligned-<preset>-v1``, separate default prefixes and
``*_qrcf_priority_v1_diagnostics.csv``. Numerical reproduction constants
are identical across presets. The old cylinder and Q-advertisement switches
remain removed.
``--profile=bs`` uses the same externally powered BS and UAV-to-BS traffic as
the other protocols.
``--AppsOn=false`` runs discovery alone with a separate output prefix.
``--printRoutes=true`` writes destination/next-hop Q tables near simulation end.

All topology, mobility, radio, energy, application and historical metric scopes
come from the existing benchmark. Its defaults include 802.11g, 100 J UAV
batteries, a 2000 x 2000 x 500 m volume and range 500 m. Those differ from the
paper's NS2/802.11n/1000 J/2000 x 2000 x 300 m/range 250 m experiment. The module
comparison under this shared environment does not reproduce the paper's
numerical curves or reported improvement percentages.

Local learning without Q-table broadcasts
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Gamma and the two-byte reserved zero count remain in Hello. No Q-list encoder,
decoder or periodic destination map is restored. ``legacy-noq`` and
``staggered-noq`` use ``Q <- (1-alpha)*Q + alpha*reward`` at action selection,
with a possible second negative update on failure, preserving the old learning
semantics. Gamma does not affect those updates.

The two outcome presets instead train once on an observed receipt or an actual
stack failure. A receiver sends a destination-scoped VALUE message to its
predecessor, carrying its current gamma and maximum local Q among its current
usable filtered actions. A destination sends a terminal VALUE. Both presets
pay for exactly the same format. ``outcome-immediate`` ignores the future term;
``outcome-bellman`` applies Equation (1). No remote Q is read through a global
service or simulator tag. This sparse exchange is an explicit engineering
completion of unspecified exchange/timing details, not a paper-specified
packet format.

Hello remains exactly 118 L3 bytes (90 protocol bytes); feedback remains
45 L3 bytes (17 protocol bytes); VALUE costs 61 L3 bytes (33 protocol bytes).
Historical experiment files remain archived
with their original settings and labels. Their old variant switches are not
supported by the current mainline.

Original equations and authorized corrections
---------------------------------------------

``Model`` implements the paper's algebra, including:

* Configured connection time, Equations (3)-(7), distinct from the true
  relative speed in Equations (19)-(22).
* Own-position prediction and adaptive Hello interval, Equations (8)-(9).
* Geometric position-vector angle, Equations (26)-(30), rather than a physical
  velocity-vector heading angle.
* Candidate-set min-max normalization, four-component reward, learning rate
  exp(-CT_norm), and neighbor-set Jaccard discount, Equations (23)-(34).
* Equation (1) is used by ``outcome-bellman``. The other presets specialize
  its future-value term to zero. Q entries initially equal zero.

The printed Equations (14)-(15) rotate ABOUT z, although the text and Fig. 7
require aligning z WITH the destination direction. The sole implementation
projects the neighbor offset onto the source-to-destination axis and tests
axial height and radial distance. This is equivalent to the intended
full-azimuth cylinder after a proper coordinate transformation; it is an
explicitly authorized correction, not a transcription of Equations (14)-(15).
The literal geometry implementation has been removed.

Filtering starts only with received neighbor entries inside the spherical
communication range. Directly reachable destinations in the valid neighbor
table bypass filtering, as the user explicitly authorized. An expired or
unheard node is never discovered by reading its global position.

Approved engineering settings
-----------------------------

The following numerical values and conventions were approved by the user on
2026-10-18. They are reproduction choices, not constants stated in the paper.
They are exported through ``Settings`` and benchmark command-line options.

======================= =========================== ===========================
Quantity                Initial engineering value   Benchmark option
======================= =========================== ===========================
Hello lower / upper     0.25 / 1.0 s                qrcfHelloLower / qrcfHelloUpper
Prediction thresholds   1 / 5 m                     qrcfErrorMin / qrcfErrorMax
Mixed-unit MD threshold 0.5                         qrcfK
Four reward weights     0.25 each                   qrcfWeightSpeed/Connection/Energy/Angle
Destination / failure   2 / -1                      qrcfRewardMax / qrcfRewardMin
Connection-time cap     60 s                        qrcfConnectionCap
Communication radius    shared benchmark MaxRange   MaxRange
======================= =========================== ===========================

* Initial nominal Hello interval is the midpoint. Prediction uses actual time
  since the preceding own Hello. ``legacy-noq`` starts at t=0 and uses the
  nominal period. Other presets start uniformly in [0,initial HT] and send
  after U[max(HT_L,0.9 HT),HT]. This prevents synchronized broadcast losses
  without exceeding HT_H or changing the paper's validity rule. The 10% jitter
  window is an engineering choice, and the extra sends are metered.
* Equal-valued/singleton min-max domains normalize to 0.5. Zero configured
  relative speed maps to CT=60 s; larger finite CT values are capped at 60 s.
  With no in-range CT samples or all CT samples zero, keep the current period.
* Validity is reset to local HT_H on a nonduplicate Hello; expiry is processed
  at now >= expiry. Duplicate IDs retained in the current neighbor entry do
  not extend validity. Previous observations and distances are retained.
* A still-valid cache entry outside R stays until validity expiry, but is
  omitted from CT statistics and data candidates. Each CT exclusion is counted.
  Gamma history still uses the full valid neighbor table, once per own Hello.
* Missing history or an empty neighbor union gives gamma=0. Otherwise gamma
  is the Jaccard similarity of consecutive neighbor ID sets.
* Same-direction ahead/behind is determined by projection on the faster UAV's
  velocity; tied speed/side-by-side uses alpha1=+1. Different directions use
  consecutive distance observations, falling back to the instantaneous closing
  dot product if history is absent. alpha2 follows the literal MD branch.
* Spherical motion uses radians, azimuth in [0,2pi), polar angle FROM +z,
  and theta=phi=0 for zero speed. MD directly subtracts speed and angle values,
  with no unit rescaling or shortest-angle wrapping.

The pure formula API returns ``std::nullopt`` for undefined domains. Approved
policies live in ``Settings``/``Discovery``. Acos arguments are clamped to
[-1,1] for roundoff. Remaining undefined data-candidate scores are counted and
that attempted forwarding fails; they are never silently replaced by an
arbitrary epsilon or a fabricated reward.

Local online learning and prevention
------------------------------------

The paper's Algorithm 3 does not uniquely define the M-episode loop, repeated
random state draws or their relationship to real transmissions. The user
approved this executable interpretation:

* Each UAV samples epsilon uniformly in [0,1] once using an assigned ns-3 RNG
  stream. Each actual forwarding hop selects one action: with probability
  epsilon, uniformly explore the filtered candidates; otherwise choose the
  highest Q, breaking exact ties uniformly. Null-packet route previews do not
  train or consume random draws. There is no extra virtual M-episode training.
* Each node keeps a separate local Q table per destination, with next-hop
  IPv4 addresses as actions. Ordinary rewards use normalized candidate
  metrics and dynamic alpha. Outcome modes retain action-time alpha/reward
  until the actual one-hop outcome; Bellman future values arrive in VALUE.
* A direct destination gets Rmax, alpha=exp(-0.5) under the approved singleton
  convention and maxNextQ=0.
* If the current filtered set contains no strictly closer candidate, drop the
  packet and actually send a one-hop UDP feedback message to its predecessor.
  Source-side local minima have no predecessor to notify.
* Feedback and actual MAC retry failures penalize the corresponding action
  with Rmin and maxNextQ=0. Outcome modes also observe ARP and IPv4 drops,
  avoid optimistic pre-training, and resolve a token at most once. MAC retry
  exhaustion or ARP resolution failure excludes that neighbor, including
  direct delivery, until a fresh valid Hello. A fresh Hello removes only that
  sender's DEAD ARP cache entry to permit real address resolution again.
* Unanswered outcome tokens expire after max(connectionCap,helloUpper), with
  a lower bound of the actual ARP retry horizon plus one nanosecond.
  Expiry counts an unknown outcome, without changing Q. It is neither proof
  of a failed data transmission nor a protocol-level retransmission timer.
* There is no added protocol-level data retransmission. IPv4 TTL limits loops.
  Q entries persist across temporary neighbor loss; action selection excludes
  currently unavailable neighbors.

The user authorized simulator PacketTags to represent only the previous link
sender and transmission identity, without additional data-frame wire bytes.
No remote Q, gamma, energy or neighbor table is read through a tag or oracle.
The only global information service is the existing GodLocationService for a
requested destination's coordinates; this service is explicitly unmetered,
as approved. Unknown addresses are rejected before querying that service.

Control wire format and accounting
----------------------------------

The original Equation (2) Hello is a 76-byte ``HelloHeader``: uint64 message
ID, uint32 node ID, and eight network-order IEEE double fields. Field widths
are implementation serialization choices. Approved learning information is
an actual wire extension, not free simulator metadata:

* Versioned envelope: magic uint16, version uint8, kind uint8 (4 bytes).
* Fixed extension after the original Hello: gamma double and a reserved zero
  uint16 field, 10 bytes. There are no destination or Q-value entries.
* Failure feedback: transmission token uint64, destination IPv4, reason uint8
  (13 bytes), plus the envelope.
* VALUE: token uint64, destination IPv4, gamma double, maximum-Q double,
  status uint8 (29 bytes), plus the envelope. Status is TRANSIT or TERMINAL;
  finite negative Q is valid. Token, destination and peer must match pending
  state. Terminal status must be sent by the actual destination.

Thus Hello costs 118 IPv4/UDP bytes, feedback 45, and VALUE 61.
Deserialization rejects malformed versions, truncation,
nonzero reserved fields (including old Q-bearing Hellos) and invalid
numerical fields. UDP port 17930 and IP TTL=1 are used for all control types.

Control message counts are recorded at successful socket submission. Control
L3 packet counts and bytes come from the actual IPv4 Tx trace, including any
IP fragmentation. They exclude MAC headers, ARP and MAC retransmissions.
The shared additional-measurement CSV
also retains its existing broader all-IP-control scope for every algorithm.

The historical ``ControlThroughput`` field follows the older protocols'
compatibility convention: sum submitted QRCF protocol payload bytes (including
the learning extension, feedback and VALUE), divide by 1000 and simulation seconds.
It excludes IPv4/UDP headers and does not multiply by 8. Its numerical unit is
kB/s, even where the legacy report labels it kbps. ``ControlL3Bytes`` in QRCF
diagnostics and ``L3ControlThroughput(kbps)`` in the shared sidecar retain their
actual IPv4 byte scope and proper bit conversion. Existing result files are
not retroactively converted; comparisons must use rows with the same convention.

Measurements and known limits
-----------------------------

Historical PDR, delay, throughput, hop, energy and control fields keep the
existing benchmark definitions. QRCF automatically enables its common
application/L3 sidecar so offered datagrams and aggregate delivery remain
visible alongside historical averages of per-flow statistics. This distinction
matters when packets are rejected before IPv4 admission.

The per-node ``*_qrcf_diagnostics.csv`` records settings, variant, epsilon,
gamma, learned destination count, network offered/delivered data, network
Hello/offered ratio, Hello/control/feedback counts, actual control L3 packets
and bytes, Q updates, exploration/exploitation, local minima, final MAC
failures, CT exclusions, candidate counts and Hello interval averages.
Protocol-observed directed neighbor-link lifetimes have completed and still-
open counts; links still open at the horizon remain censored. These records
are independent of FANET's clustering neighbor table.

The shared BasicEnergySource defaults to a 10% low-battery threshold and may
turn off the radio before energy reaches zero. Record the first actual UAV
low-battery crossing and zero-energy crossing separately, using observation
flags and blank times when unobserved. No unapproved choice between them is
called the paper's FND, and no Round-to-seconds conversion is invented.

A stationary, synchronized scene showed persistent Hello losses and zero
delivery under identical Hello clocks. ``legacy-noq`` preserves that timing.
The new priority tests use identical settings at every stationary node and
verify discovery and actual delivery with jitter. Old transport tests select
``legacy-noq`` explicitly. Statistical performance claims, 30-run confidence
intervals and sensitivity studies remain separate work.

Validation
----------

Run all suites::

  ./ns3 run 'qrcf-tests --suite=qrcf --verbose' --no-build
  ./ns3 run 'qrcf-tests --suite=qrcf-routing --verbose' --no-build
  ./ns3 run 'qrcf-tests --suite=qrcf-value-wire --verbose' --no-build
  ./ns3 run 'qrcf-tests --suite=qrcf-priority --verbose' --no-build

Run the paired benchmark controls::

  python3 contrib/qrcf/test/validate-priority-profiles.py --nodes 50

The four core cases cover formulas and aligned geometry, singular domains,
wire encoding/truncation, validity and duplicate IDs, candidate normalization,
Q-table maxima, approved CT/Hello rules and directed link censoring. Three
transport cases use real Wi-Fi and UDP to check:

* A source and destination separated by more than R communicate through an
  intermediate UAV, and both UAVs update destination-specific local Q tables.
  Every node's Hello remains exactly 118 L3 bytes after learning, fixed gamma
  extensions decode, and the source's singleton Q does not exceed its
  immediate reward of 0.5.
* A physically disconnected destination causes an intermediate local minimum;
  actual UDP feedback arrives and the predecessor's selected action turns
  negative.
* A warmed data/ARP path is physically broken; a real MAC retry-limit trace
  fires and penalizes the corresponding action.

VALUE wire tests preserve golden Hello/feedback bytes and reject malformed
receipts. Priority network tests exercise actual multihop Bellman propagation,
destination/action-domain isolation, single failure updates, MAC and cold ARP
failures, recovery and stationary discovery. Fixture constants are not protocol
defaults or performance data. The paired driver checks legacy byte compatibility,
identical epsilon and numerical settings, metered wire costs and several seeds;
it never selects a preset by its PDR.
