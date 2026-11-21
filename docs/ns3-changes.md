# Changes to stock ns-3.40

This release is an overlay on the official ns-3.40 release. The top-level
`CMakeLists.txt`, `README.md` and `LICENSE` are unchanged. The `src/` tree has
**16 modified files**, grouped into four patches under `patches/ns-3.40/`.

All patches are generated against the official `ns-allinone-3.40` tarball and
apply cleanly with `patch -p1` (the `apply_to_ns3.sh` helper does this for you).

| Patch | Files | Nature | Needed to reproduce the paper results |
|---|---|---|---|
| `0001-internet-arp-dead-reprobe.patch` | `src/internet/model/arp-cache.{h,cc}`, `src/internet/model/arp-l3-protocol.{h,cc}` | Adds the `ns3::ArpL3Protocol::DeadReprobeInterval` attribute and a bounded re-probe of DEAD entries | MO-GNN only. Opt-in; the default 0 preserves stock behaviour and the published matrices use 0. |
| `0002-wifi-energy-off-state.patch` | `src/wifi/model/phy-entity.cc`, `wifi-phy.cc`, `wifi-phy-state-helper.cc`, `wifi-radio-energy-model.{h,cc}` | Makes PHY OFF transitions re-entrant and preserves a synchronous energy-depletion OFF state | Yes. Protocols that install an energy model can otherwise assert or resume a depleted radio. |
| `0003-aodv-control-metrics.patch` | `src/aodv/model/aodv-neighbor.{h,cc}`, `aodv-routing-protocol.{h,cc}` | Control-packet counters, link-lifetime statistics, an incoming-interface index fix, and a neighbor accessor | Diagnostic. AODV is retained for historical diagnostics only and is not part of the main comparison. |
| `0004-olsr-control-metrics.patch` | `src/olsr/model/olsr-repositories.h`, `olsr-routing-protocol.{h,cc}` | Control-packet counters, link-lifetime statistics, and the same incoming-interface index fix | Yes. The OLSR baseline's control-overhead and link-lifetime columns use these counters. |

## What each change does

### ARP DEAD-entry re-probe (`internet`, MO-GNN only)

This change was introduced while porting MO-GNN. Stock ns-3.40 drops packets
destined to an ARP entry in the DEAD state until the entry's dead timeout
expires. The patch adds a configurable minimum interval before a DEAD entry may
be re-probed:

```cpp
Config::SetDefault("ns3::ArpL3Protocol::DeadReprobeInterval",
                   TimeValue(Seconds(arpDeadReprobe)));
```

A value of 0 keeps the stock behaviour. The attribute is opt-in, and the three
published matrices pass `--arpDeadReprobe=0`, so CA-FBCR, QRCF, ICRA, CFR-ENG
and OLSR results do not depend on this patch. If MO-GNN reproduction is
not needed, omit `0001-internet-arp-dead-reprobe.patch` and keep
`--arpDeadReprobe=0`.

### WiFi energy-model OFF handling (`wifi`)

With the 0.273 A current setting used by the energy model, stock ns-3.40 stops
updating energy consumption at about 122 s of a 200 s run: the simulation
continues, but the energy counter freezes. The cause is a PHY OFF transition
that is not handled re-entrantly when depletion happens inside a receive, CCA
or transmit callback. The patch:

- detects a synchronous OFF after RX-start and RX-end notifications;
- stops field processing and transmit setup once the PHY is OFF;
- clears the reception context when `SetOffMode()` cancels PHY events;
- distinguishes an explicit PHY ON notification from delayed IDLE
  notifications, so only a real ON resumes a depleted model.

The patch also replaces the energy-model switch-off event's `Cancel()` with
`Simulator::Remove()`. This does not change the simulation results and
substantially reduces DRAM use during long runs.

This is required for any scenario that installs `WifiRadioEnergyModel`; without
it, long runs with depleting nodes can stop accounting energy or abort.

### AODV and OLSR measurement instrumentation (`aodv`, `olsr`)

Both protocols expose:

- total control-packet counts;
- HELLO counts;
- control-packet throughput;
- average link lifetime and symmetric-link duration (OLSR).

The incoming-interface lookup in both protocols is also corrected to compare
the device `IfIndex` against IPv4 interfaces instead of indexing the node's
device vector. The AODV neighbor table gains a read-only accessor.

These counters feed the `ControlOverhead` and average-link-lifetime CSV
columns. OLSR is a published baseline; AODV is kept only as a diagnostic
entry point.

## Verifying the delta

Against a fresh official ns-3.40 tree:

```bash
for p in patches/ns-3.40/*.patch; do
  patch --dry-run -p1 -d /path/to/ns-3.40 < "$p"
done
```

The 16 files above are the complete delta. No other `src/` file differs from
the official release.
