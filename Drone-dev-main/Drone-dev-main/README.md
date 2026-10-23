# Multi-Agent Flight Planner — SAR Coverage Programme

Turns a **real operational area** drawn as KML into flyable QGroundControl `.plan` files
for a **swarm** of multirotors carrying a **SIYI ZR30** gimbal, then launches them together
from one command.

Two knobs matter, and only two:

* **Where** you search comes from the KML — an `OpsArea` inside a `GeoFence`.
* **How well** you image comes from one number, **flight altitude**, from which the ZR30
  geometry re-derives swath, line spacing, ground sample distance and the motion-blur-safe
  cruise speed. No imaging number is ever chosen twice.

Everything else — drone count, scan pattern, overlap — is configuration.

> **This file is how to operate the system.** Why each decision was made, what was measured
> on which date, and every sortie's numbers live in [`../history.md`](../history.md).

---

## Contents

| | |
|---|---|
| **[1. Planning](#1-planning)** | pipeline, camera model, partitioning, guardrail, plan output |
| **[2. Flying in simulation](#2-flying-in-simulation)** | one drone, a swarm, QGroundControl, separation |
| **[3. Hardware operations](#3-hardware-operations)** | the network, companion computers, the ELRS link, bench flight controllers |
| **[4. Commanding a swarm](#4-commanding-a-swarm)** | the launch handshake, preflight contract, fault matrix |
| **[5. Reference](#5-reference)** | tools, traps, troubleshooting, tests, provenance |

---

# 1. Planning

## The pipeline

```
KML (GCSlocation + GeoFence + OpsArea)
   │  kml_parser.py
   ▼
project to local metres (equirectangular @ home)          geometry.py
   │
   ▼
camera model: altitude → swath, GSD, line spacing,        camera.py
              blur-safe cruise speed, altitude ceiling
   │
   ▼
partition OpsArea for N drones                            scanner_tools.py
   ├─ lawnmower : one contiguous strip per drone
   └─ voronoi   : one cell per drone
   │
   ▼
one altitude layer per drone (28 / 30 / 32 m), imaging     agents.py
   recomputed per layer
   │
   ▼
scan each sub-area, lanes along its own long axis,         scanner_tools.py
   corners drawn as circles                                geometry.py
   │
   ▼
GuardrailPlannerAgent — every waypoint AND every leg       agents.py
   inside the GeoFence, no two drones able to meet.
   HARD STOP on failure; no mission is written
   │
   ▼
build .plan: DO_CHANGE_SPEED → NAV_TAKEOFF → lawnmower →   qgc_plan.py
             NAV_RTL (frame 2)
   │
   ├─ mission.plan                  (combined, each item tagged "drone-N")
   ├─ mission_drones/drone-N.plan   (one single-vehicle plan each)
   └─ mission.baseline.plan         (home + fence only)
```

Planning is fully deterministic: no LLM is involved at any stage.

## Module layout

| Path | Role |
|---|---|
| `src/flight_planner/kml_parser.py` | Pulls `GCSlocation` (home), `GeoFence`, `OpsArea` from the KML, ignoring folders/styles/wrappers. |
| `src/flight_planner/geometry.py` | Local-metre projection, point-in-polygon, clipping, scanline fill, rounded turns. |
| `src/flight_planner/camera.py` | **Single-knob camera model** — altitude → spacing, GSD, blur-safe speed, ceiling. |
| `src/flight_planner/models.py` | `Coordinate`, `MissionInput`, `PlannerConfig`, `DronePlan`, `PlanningResult`. |
| `src/flight_planner/scanner_tools.py` | Partitioning (lawnmower strips / voronoi cells) + orientation-aware scan. |
| `src/flight_planner/agents.py` | `FlightPlannerAgent`, `GuardrailPlannerAgent`, `MultiAgentPlanner`. |
| `src/flight_planner/qgc_plan.py` | Writes the QGC `.plan`, splits per drone, embeds the geofence. |
| `src/flight_planner/cli.py` | Wires the run; hard-stops if the guardrail rejects. |

Every knob in the chain is catalogued in [`parameters.yaml`](parameters.yaml), tagged
`active`, `derived`, `constant` or `GAP`. `tests/test_parameters_yaml.py` checks it against
live code, so the reference cannot silently drift.

## The single-knob camera model (`camera.py`)

Pure geometry, no dependencies. Defaults are the ZR30 at 1× zoom, 720p.

```
swath width   W  = 2 · h · tan(HFOV/2)
along-track   L  = 2 · h · tan(VFOV/2)
GSD              = W / image_width
line spacing     = W · (1 − side_overlap)        → coverage width for the scanner
speed cap (blur) = GSD / exposure_time           ← almost always the binding cap
speed cap (fwd)  = L · (1 − fwd_overlap) · fps
cruise speed     = min(caps) · 0.5               (50 % margin)
```

Defaults: HFOV 58.1°, VFOV 35°, 1280×720, 30 fps, 25 % overlaps, 1/1000 s exposure, 0.5 m
target (a person). **At 30 m: 33.3 m swath, 2.60 cm/px, 25.0 m line spacing, 13.0 m/s
cruise.**

```bash
# "how high can I fly and still resolve a person at 3 cm/px?"
PYTHONPATH=src python3 -m flight_planner.cli --ceiling 3.0     # -> 34.6 m
```

`PlannerConfig.lane_spacing_m` and `.cruise_speed_m_s` both read from this model, so
altitude, spacing and speed can never drift out of agreement.

## Coverage and partitioning (`scanner_tools.py`)

* **lawnmower** — the `OpsArea` is split into one contiguous strip per drone along its
  longer axis; each strip is boustrophedon-scanned.
* **voronoi** — one cell per drone, grown by half-plane clipping around seed points; seeds
  are de-duplicated so no two drones inherit the same cell.

Either way each drone gets a **spatially separate** sub-area, keeping the swarm
de-conflicted by geography rather than by mid-air avoidance.

Lanes run along **each sub-area's own long axis** (found by testing every convex-hull edge
for minimum width), so long thin strips get few long passes instead of many short
turn-heavy ones. Lanes are **centred** within the sub-area, not started on its boundary —
the swath is wider than the lane pitch, so a centred block still covers both edges and no
lane degenerates onto the boundary where it would scan nothing.

**Altitude layering.** Each drone gets its own layer, 2 m apart by default (28 / 30 / 32 m
for drones 1–3). `PlannerConfig.for_drone(index)` rebuilds the *whole* imaging profile for
that altitude, so a staggered drone's camera geometry is correct rather than reusing
drone-1's numbers at a different height.

**Turn shaping.** Corners are drawn into the plan as a circle of `turn_radius_m` (0 = half
the drone's lane spacing), rendered as the polygon *around* the circle — a vertex every
45°, each with an acceptance radius of half its leg. `MIN_TURN_RADIUS_M` (3 m) and a
cut-depth cap stop a corner being drawn so tight the camera misses it. An alternative,
`turn_style: "pre_yaw"`, exists but is **not** the default: it was flown and does not
reduce bank, because a multirotor turns by tilting, not by yawing — pre-rotating the nose
just moves deceleration from pitch into roll.

## The guardrail (`agents.py`)

`GuardrailPlannerAgent` projects the KML `GeoFence` to local metres and checks:

- every **waypoint** is inside the fence (point-in-polygon),
- every **leg between waypoints** is inside it (segment-in-polygon),
- the **takeoff transit** home → waypoint 0 and the **RTL return** last → home,
- no two drones' paths come within `MIN_HORIZONTAL_SEPARATION_M` (20 m) while on layers
  closer than `MIN_VERTICAL_SEPARATION_M` (2 m).

It runs on every mission whether or not the LLM is enabled, and the CLI **raises and writes
no mission** if it fails. Planning and safety validation are separate modules on purpose.

> **This rejects missions that older versions accepted.** Transits were previously
> unchecked — on one Pulau Ubin sortie those legs were 269, 585 and 664 m, roughly a third
> of drone-3's flight, inside the fence only by where the GCS happened to sit. A staging
> point outside a concave fence's lobe will now hard-stop. That is correct, not a
> regression. Home is checked **once** and reported as `GCS location is outside the
> GeoFence`, not as 2N leg failures. Grazing a fence vertex is not a breach.

## Plan output (`qgc_plan.py`)

Every mission is complete and self-contained:

* `MAV_CMD_DO_CHANGE_SPEED` (178, frame 2) — **before** takeoff
* `NAV_TAKEOFF` (22) at the first waypoint
* the lawnmower waypoints (16, frame 3 `MAV_FRAME_GLOBAL_RELATIVE_ALT`)
* `NAV_RETURN_TO_LAUNCH` (20, **frame 2 `MAV_FRAME_MISSION`** — frame 3 makes PX4 reject it)

### Why the speed item exists, and why it comes first

A `.plan`'s `cruiseSpeed` field is a **ground-station planning value**. It never reaches the
aircraft, which flies `MPC_XY_CRUISE` — 5 m/s by default. Measured at Pulau Ubin: the plan
asked for 13.0 m/s and the vehicle held a median of **5.00 m/s** for the entire survey, so
the blur-safe speed, the whole output of the camera model, was being discarded.

`DO_CHANGE_SPEED` fixes it, and it goes **before** `NAV_TAKEOFF` because takeoff flies the
vehicle all the way to the first waypoint:

| phase | item after takeoff | item before takeoff |
|---|---|---|
| transit to survey area | median 5.00 m/s | **median 13.00 m/s** |
| time to reach the area | 155 s | **79 s** |
| survey complete at | 443 s | **366 s** |

`MPC_XY_VEL_MAX` must also be raised — PX4's 12.0 default would clamp 13.0. It is 20.0 in
`sar_turn_params.params`. Above roughly 46 m the blur-safe speed exceeds that ceiling and
the planner warns that the airframe, not the optics, has become the limit.

`droneCount` in the plan means **the number of `drone-N.plan` files that will be written**;
`requestedDroneCount` sits beside it so a starved drone stays visible.

## Configure and run

```bash
cp config.example.json config.json      # then edit

PYTHONPATH=src python3 -m flight_planner.cli "ops/SAFTI city.kml" \
    --config config.json --output missions/safti/mission.plan

PYTHONPATH=src python3 -m flight_planner.cli "ops/SAFTI city.kml" --alt 40
```

```json
{
  "number_of_drones": 3,
  "camera_fov_degrees": 58.1,
  "scan_altitude_m": 30.0,
  "scan_overlap_percent": 25.0,
  "scan_type": "lawnmower"
}
```

`--scan-type lawnmower|voronoi` and `--alt <metres>` override the config without editing
it. Outputs: `mission.plan`, `mission_drones/drone-N.plan` and `mission.baseline.plan`.

---
# 2. Flying in simulation

## One drone

```bash
source ~/drone-dev/venv/bin/activate
cd ~/drone-dev/FlightPlanner

# 1. plan a sector
PYTHONPATH=src python3 -m flight_planner.cli "ops/pulau-ubin-west.kml" \
    --output missions/ubin/mission.plan

# 2. see it on satellite imagery before you fly it
python3 tools/plot_mission.py ops/pulau-ubin-west.kml \
    missions/ubin/mission.plan missions/ubin/sector.png

# 3. fly one drone's plan and record the track
python3 tools/fly_mission.py missions/ubin/mission_drones/drone-1.plan \
    --params sar_turn_params.params --track missions/ubin/track.json

# 4. compare what was flown against what was planned
python3 tools/plot_track.py ops/pulau-ubin-west.kml \
    missions/ubin/mission_drones/drone-1.plan \
    missions/ubin/track.json missions/ubin/flown.png
```

`fly_mission.py` uploads through MAVSDK's *QGroundControl plan importer*, so it exercises
the real `.plan` file rather than a hand-built item list — if PX4 accepts the upload, the
file is genuinely valid.

**The flight ends with `TIMEOUT`, and that is normal.** `fly_mission.py` waits for
`current >= total`, and PX4's RTL item never reports as reached. The mission completed;
only the detector gave up.

## A swarm

```bash
tools/sitl_swarm.sh start toapayoh 3     # three x500 over Toa Payoh, headless
tools/sitl_swarm.sh status               # what is running, on which ports
tools/sitl_swarm.sh stop                 # kills PX4 and the Gazebo server
```

`HEADLESS= tools/sitl_swarm.sh start toapayoh 3` also opens the Gazebo 3D window.
`SPACING=10` moves the spawn points apart; `MODEL=gz_x500_gimbal` changes the airframe.

PX4 derives everything from the instance index `-i N`. This is the single most important
thing to get right — three instances launched *without* it are all instance 0, all claim
`MAV_SYS_ID 1`, and all fight over the same ports:

| instance | `MAV_SYS_ID` | working dir | GCS link (QGC) | API link (tooling) | gz model |
|---|---|---|---|---|---|
| 0 | 1 | `rootfs/0` | 18570 | 14540 | `x500_0` |
| 1 | 2 | `rootfs/1` | 18571 | 14541 | `x500_1` |
| 2 | 3 | `rootfs/2` | 18572 | 14542 | `x500_2` |

Each instance gets **its own `rootfs/<i>` and therefore its own dataman**, so a stale
mission or geofence cannot leak between vehicles. The launcher waits for the world's
`/clock` topic before starting instance 1 — if the others race instance 0, you get a
half-built world.

### Flying every drone's mission at once

```bash
for i in 1 2 3; do
  python3 tools/fly_mission.py missions/toapayoh/mission_drones/drone-$i.plan \
    --url "udpin://0.0.0.0:$((14539+i))" \
    --server-port $((50060+i)) \
    --gcs-sysid  $((244+i)) \
    --params sar_turn_params.params \
    --track missions/toapayoh/track-drone$i.json &
done
wait
```

`--server-port` and `--gcs-sysid` exist for exactly this. Each `System()` spawns its own
`mavsdk_server`; without distinct ports the second and third flights fail to bind, and
without distinct sysids every flight presents as the same ground station. This runs happily
alongside QGC because the flights use the **API** ports and QGC uses the **GCS** port.

## Watching them in QGroundControl

Start the swarm **first**, then QGC. It auto-connects on UDP 14550 and every instance
announces itself there with a distinct system id.

```bash
~/drone-dev/QGroundControl.AppImage
```

- **Top bar, left** — a vehicle dropdown (`Vehicle 1 ▾`) appears only when more than one
  vehicle is connected. Quickest check that all N registered.
- **Top right of the Fly view** — `Single` / `Multi-Vehicle` radio buttons. Multi-Vehicle
  gives one card per aircraft with its own attitude ball, compass, mode and Arm button.
  This is native QGC, not a modification.
- **Start Mission** at the top of that panel applies to *every* vehicle at once.

QGC's autoconnect listens on **14550 and nowhere else**, and the port is not configurable
(v4.4.3 has no `udpListenPort` setting). Anything on another port needs a link added by
hand under *Application Settings → Comm Links → Add*; autoconnect will never find it.

## Measuring whether they stayed apart

The guardrail proves the *plan* keeps every pair separated. Measuring the *flight* needs
all vehicles on one clock, which per-flight tracks do not give you — each stamps t=0 at its
own arming moment, and 2 s of skew is 24 m at survey speed.

```bash
python3 tools/swarm_record.py missions/toapayoh/common-clock.json --vehicles 3 --seconds 900
python3 tools/swarm_separation.py missions/toapayoh/common-clock.json
```

`swarm_separation.py` reports, per pair, the closest approach while **both are airborne**
(vehicles parked metres apart on the ground break both limits trivially, for as long as
they sit there) and flags any moment inside *both* limits.

Measured, Toa Payoh, 3 drones at 30/32/28 m, all launched from one point:

| pair | both airborne | min horizontal | verdict |
|---|---|---|---|
| drone-1 v drone-2 | 440 s | **17.8 m** | 1.0 s inside both limits |
| drone-1 v drone-3 | 440 s | 76.8 m | clear |
| drone-2 v drone-3 | 564 s | 93.6 m | clear |

Once dispersed, no pair came within 76 m. The single breach is at the **launch point** —
the known gap: the guardrail reasons about survey legs, not about N aircraft leaving one
GCS point. Launch scheduling is not implemented.

> **`swarm_record.py` and QGC cannot both run.** See *one consumer per link* below.

---

# 3. Hardware operations

## The network

The workstation and the companion computers sit on an **unmanaged switch**. An unmanaged
switch forwards frames and serves nothing — no DHCP, no address of its own — so **every
address is static**. This is the most common cause of "nothing works this morning": if the
cable moves between the office LAN and this switch, whichever end expected DHCP comes up
with no address at all.

| | address | switch port |
|---|---|---|
| workstation | `192.168.144.50` | 5 |
| `jetson-1` | `192.168.144.101` | — |
| `jetson-2` | `192.168.144.102` | 8 |

```bash
tools/switch_link.sh discover     # who is on the wire, by IPv6 link-local
tools/switch_link.sh check        # both ends, addresses and reachability
```

**How to reach a board that has no IPv4 at all.** IPv6 link-local addresses self-assign
with no server, no lease and no configuration, so a board with nothing else still answers:

```bash
ssh Orin_NX@fe80::de4a:9eff:fee1:f2a6%enp130s0
```

One caveat that cost an hour: **"no reply to `ping6 ff02::1`" means no host has *configured*
this link, not that no host is attached.** A board sat in port 8 with `carrier=1` at
1000 Mb while NetworkManager held the device `disconnected` with no profile bound — and a
device NM has not brought up gets no IPv6 link-local either, so it answered nothing. Read
`carrier` on the far side before concluding a board is powered off.

Measured across the switch, workstation to jetson-2: **0% loss, 0.28 ms mean**, scp
**66 MB/s up and 78 MB/s down** with matching checksums. Over office WiFi the same board
gave 4.6–193 ms and 50% loss. Use the wire.

## Bringing up a companion computer

One command gives a board its permanent identity on the switch:

```bash
tools/switch_link.sh discover
tools/switch_link.sh setup fe80::<addr>%enp130s0 192.168.144.101 jetson-1
```

That installs a `switch-static` profile which is **bound to the NIC's burned-in MAC** (so
it can never attach to the wrong interface), refuses MAC randomisation, outranks the
leftover DHCP profile on the same device, and declines a default route — the switch is
isolated, so a default through it would black-hole traffic.

> **Check it persisted.** On Ubuntu 24.04, NetworkManager only *renders* netplan: the live
> profiles in `/run/NetworkManager/system-connections/` are **tmpfs, wiped every boot**. The
> real copy must exist in `/etc/netplan/`. A profile that lives only in `/run` looks perfect
> in `nmcli` and is gone after a power cycle. `switch_link.sh setup` greps `/etc/netplan`
> afterwards and says so either way. Verified by rebooting jetson-2: back on its static
> address unattended in **28 s**.

The agent needs only `pymavlink` beyond the standard library — no MAVSDK on the companion:

```bash
ssh jetson-1 'python3 -m pip install --user --break-system-packages pymavlink'
scp tools/drone_agent.py tools/elrs_link.py jetson-1:~/FlightPlanner/tools/
scp sar_turn_params.params jetson-1:~/FlightPlanner/
```

`--user --break-system-packages` is required: Ubuntu 24.04 is PEP 668 managed and has no
`python3-pymavlink` apt candidate.

## Stopping a flight

Three independent ways, and they are not equal. Reach for them in this order.

**1. The RC transmitter.** The authoritative override. It reaches the flight controller
directly, over a link that does not pass through the companion, the relay, WiFi, ssh or any
of our code -- which is exactly the chain an emergency may already have broken. Nothing
below is a substitute for having a pilot on the sticks.

A switch on the RadioMaster Pocket requests a hand-flown mode and PX4 honours it directly.
Two things have to be true for that, and one of them is **not** true by default:

```bash
python3 tools/rc_takeover.py --fleet fleet-artex-pair.yaml     # read-only; changes nothing
```

`COM_RC_OVERRIDE` is a bitmask -- bit 0 enables stick override in AUTO, bit 1 in OFFBOARD
-- and **PX4 ships it as 1**, so on a stock airframe moving the sticks does not break
OFFBOARD, which is the mode every arming test here uses. Set it to 3. The mode switch
itself needs `RC_MAP_FLTMODE` pointing at the channel and at least one of `COM_FLTMODE1..6`
holding a hand-flown mode -- Manual(0), Altitude(1), Position(2), Acro(6), Stabilized(8),
Position Slow(9); Mission(3), Hold(4) and Return(5) are still the autopilot flying.
`RC_MAP_KILL_SW` maps the kill switch, the one override that does not need the flight
stack's cooperation -- it will drop the aircraft, so it is for a crash in progress.
The handset half (which physical switch, which channel) is EdgeTX configuration on the
Pocket and nothing here can set it. `~/drone-dev/drone-control-scheme` is the running
record of the whole scheme.

**When the pilot takes over, the companion stands down.** The agent watches for the
transition out of OFFBOARD or AUTO into a hand-flown mode, latches "pilot has it", and then
sends nothing further: no setpoints, no mode changes, no automatic abort, and **no disarm
while the aircraft is airborne**. It reports an `rc_takeover` event and keeps out of the
way, because two aborts fighting each other is worse than either.

**2. The guard on the companion** (`tools/safety_rules.py`, carried out by
`drone_agent.py`). Autonomous, needs only the companion-to-aircraft link, and keeps working
with the ground station switched off. It climbs a four-rung ladder and never descends --
recovery is a human decision, not a timeout:

| Rung | What it does | Fires on (defaults) |
|---|---|---|
| WARN | logs, keeps flying | error rate >= 3/min |
| HOLD | AUTO.LOITER: stops the mission, holds position | error rate >= 8/min, battery <= 25% |
| LAND | AUTO.LAND in place, then disarms once down | error rate >= 14/min, battery <= 15%, tilt >= 60 deg, no telemetry 12 s, estimator lost attitude, innovation test failing 3 s |
| DISARM | cuts the motors -- **only** once the aircraft is on the ground | reached by LAND after touchdown |

The error *rate* is the point: PX4 talks constantly and a healthy aircraft still emits the
odd warning, so messages are weighted by MAVLink severity (EMERGENCY 8 ... WARNING 1) and
summed over a sliding 60 s window. One "EKF ..." means nothing; ten in twenty seconds means
the estimator is coming apart. Tune with `--hold-rate`, `--land-rate`,
`--battery-land-pct`, `--tilt-land-deg`; disable for bench diagnosis with `--no-safety`.

A DISARM is never carried out on an aircraft that is still flying -- it is downgraded to
LAND, and when the agent cannot tell whether it is airborne it lands rather than guessing.
The one command that *can* drop an aircraft is `KILL` (force-disarm), and it is refused
unless the agent was started with `--allow-kill`.

**3. The operator on the ground.** Two paths, deliberately:

```bash
# in the pair_arm_test console: type 'a' + ENTER. Fastest -- it already holds the pipes.
# out-of-band, needs nothing but ssh -- survives the orchestrator being wedged or killed:
python3 tools/takeover.py --fleet fleet-artex-pair.yaml --now
```

`takeover.py` writes a flag file beside each agent, which the agent polls several times a
second; the agent also refuses to arm while a flag is set, so clear it with `--clear`
before the next run. An agent reports every rung it climbs as a JSON `safety` event, so
an orchestrator (and one day the GUI) can show what happened and why.

## The ELRS radio link

Each companion joins one RadioMaster Pocket's **TX backpack** over WiFi; the backpack
bridges MAVLink to the aircraft over ELRS.

```bash
# on the Jetson -- must be root; NetworkManager refuses to create a system
# profile for a non-console ssh session
echo <pw> | sudo -S -p '' env ELRS_BSSID=02:70:07:61:E4:CB ELRS_PSK='...' \
    bash tools/jetson_net.sh
tools/jetson_net.sh --status

# make the relay survive a reboot -- without this, a rebooted Jetson looks like a dead
# aircraft: ssh works, but the agent never sees a heartbeat on 127.0.0.1:14540
sudo bash tools/elrs_service.sh
tools/elrs_service.sh --status
```

Three things about these radios that are not obvious:

**Every backpack broadcasts the identical SSID** `ExpressLRS TX Backpack 000000` — the
`000000` is an unset uid, not a serial — and `http://10.0.0.1/config` reports the same
product name on all of them. **The BSSID is the only identity.** Left unpinned,
NetworkManager silently attaches to whichever board is loudest; that happened, and put two
Jetsons on one transmitter with nothing logged anywhere. `artex-registry.yaml` maps each
board, and:

```bash
ELRS_PSK=... python3 tools/elrs_pair.py            # apply the registry's pairing
python3 tools/elrs_pair.py --verify                # check it, change nothing
python3 tools/elrs_pair.py --discover              # what is in range, by BSSID
```

`elrs_pair.py` treats **two Jetsons on one BSSID as a hard failure**, and where a board's
own transmitter is not powered it takes that Jetson *off* the radio rather than leaving it
on someone else's — an idle uplink is safe, a shared one is not.

**The MAVLink ports are asymmetric.** `GET http://10.0.0.1/mavlink` returns
`{"ports":{"listen":14555,"send":14550},"ip":{"gcs":"IP UNSET"}}`. The module accepts GCS
traffic on **14555** and emits vehicle traffic to **14550**, and it will not speak first —
`gcs` stays `IP UNSET` until a packet lands on 14555. A probe sent to 14550 gets nothing
however long it listens, which reads exactly like "no MAVLink support". pymavlink cannot
express "bind one port, send to another", so `tools/elrs_link.py` owns that pair and hands
the agent an ordinary localhost endpoint:

```bash
ssh jetson-1 'cd ~/FlightPlanner && python3 tools/elrs_link.py --agent-port 14540'
python3 tools/elrs_link.py --stats     # module counters, one shot
```

**The backpack serves DHCP *with a default route*.** Accept it and the Jetson's default
points at a radio that routes nowhere — apt, DNS and NTP all stop, while ssh from the
ground LAN keeps working because that is a connected route, so the box looks healthy until
it isn't. `jetson_net.sh` sets `ipv4.never-default yes`, which keeps the subnet route and
discards the default.

Measured uplink, `MISSION_ITEM_INT` at 49 bytes on the wire: **clean to 100 msg/s
(4.9 kB/s), 5.8% loss at 200 msg/s.** A 76-item mission clears this hop in 0.76 s. That
measures WiFi plus module ingest only — **the RF hop is not yet measured** and is certainly
narrower.

> A Jetson has **one** wireless interface, so it cannot hold the office network and a
> backpack at once. Ethernet carries C2, WiFi is reserved for the radio. Before a demo,
> make the backpack win: `sudo nmcli con modify elrs-backpack connection.autoconnect-priority 30`.

## Real flight controllers on the bench

Identical boards ship with identical identity. The MicoAir743v2 units here are all
`MAV_SYS_ID 1` and all report USB serial `0`, so `/dev/serial/by-id/` collapses to one
symlink. Only `/dev/serial/by-path/` separates them, by physical hub socket — and `ttyACM`
numbering follows plug order, not port order.

Connect three to one QGC as they are and you get **one** vehicle, not three: QGC keys
vehicles by system id and flaps between them with *"Vehicle 1 Switching communication to
secondary link"*. When configuring boards this is a hazard, not a cosmetic problem — a
parameter write can land on the wrong aircraft. **Configure one board at a time.**

```bash
# pass-through: one board, its own port, identity untouched
python3 tools/mavlink_bridge.py --serial /dev/ttyACM0 --udp 127.0.0.1:14551

# rewrite: board stays MAV_SYS_ID 1 internally, appears to the GCS as vehicle 2
python3 tools/mavlink_bridge.py --serial /dev/ttyACM1 --udp 127.0.0.1:14550 --as-sysid 2
```

All three arrangements were measured against three boards:

| arrangement | our own tooling | one shared QGC |
|---|---|---|
| same id, one port per board | 3 separable streams | one vehicle, link-flapping |
| **distinct ids (bridge-rewritten), all on 14550** | **3 separable streams** | **3 vehicles** |
| distinct ids, on ports other than 14550 | 3 separable streams | nothing — *Disconnected* |

So the only arrangement that puts several aircraft on one QGC screen is **ids rewritten in
the bridge, all delivered to 14550**, with no aircraft reconfigured. The alternative —
writing distinct `MAV_SYS_ID` to each board (1–250, `@reboot_required`) — is permanent and
equally valid. **Identity uniqueness is only required where streams merge**; a consumer
that owns one vehicle on its own link never needs it.

> **Not yet proven:** the bridge's telemetry direction is measured — three boards, zero CRC
> failures — but **no command has ever crossed a rewritten id**. Upload a mission to a
> rewritten vehicle and confirm it lands on the right board before trusting this near an
> aircraft.

---
# 4. Commanding a swarm

Flying several aircraft from one command needs more than a plan each. This is the protocol
that gets every drone into the air together, and refuses to get any of them into the air
alone.

## Running it

```bash
# against the SITL swarm
tools/sitl_swarm.sh start toapayoh 2
python3 tools/swarm_deploy.py --fleet fleet.yaml --plans-dir missions/toapayoh/mission_drones

# against the real Jetsons
python3 tools/swarm_deploy.py --fleet fleet-jetson.yaml --plans-dir missions/<site>/mission_drones

# plan from a KML in the same command
python3 tools/swarm_deploy.py --fleet fleet.yaml --kml ops/pulau-ubin-west.kml

# put each drone's plan on its aircraft and read it back -- no params, no preflight,
# never arms (refuses an armed vehicle). Success = the autopilot returned the same mission.
python3 tools/swarm_deploy.py --fleet fleet.yaml --plans-dir ... --upload-only

# handshake -> push plans -> prepare (barrier) -> workstation authorises ARM -> agents arm,
# hold --arm-hold-s, then disarm. Never starts the mission. --qgc opens QGroundControl on
# the workstation (observer only; it is not on the automation path).
python3 tools/swarm_deploy.py --fleet fleet.yaml --plans-dir ... --arm-only --qgc

# offboard arm with no GPS/mission (props OFF only): one aircraft. The agent runs ON the
# companion, so this is an ssh away -- the link is localhost from the Jetson's point of view.
ssh jetson-1 'cd ~/FlightPlanner && python3 tools/drone_agent.py offboard-arm \
    --link udpout://127.0.0.1:14540 --expect-sysid 115 --slot 1 --hold-s 10'

# EMERGENCY: stop every aircraft in the fleet. Works even if the orchestrator is wedged
# or gone -- it writes a flag file each agent polls. Use the RC sticks first if you have them.
python3 tools/takeover.py --fleet fleet-artex-pair.yaml --now           # land, then disarm
python3 tools/takeover.py --fleet fleet-artex-pair.yaml --action hold    # just stop the mission
python3 tools/takeover.py --fleet fleet-artex-pair.yaml --check          # is a flag set?
python3 tools/takeover.py --fleet fleet-artex-pair.yaml --clear          # before the next run

# two aircraft armed together, all-or-nobody: each Jetson reaches OFFBOARD and reports
# READY; ARM is sent to all only when ALL are ready, hold, disarm, confirm disarmed.
# Reports the arm-instant spread. Use --skip-status against SITL (it latches its first client).
# Refuses to start if any Jetson's link to its backpack is worse than 10% loss / 150 ms --
# a lost ARM is a non-event, a lost DISARM is not.
python3 tools/pair_arm_test.py --fleet fleet-artex-pair.yaml --hold-s 10

# bench run: verify everything, never arm
python3 tools/swarm_deploy.py --fleet fleet.yaml --plans-dir ... --dry-run

# bring everyone home
python3 tools/swarm_deploy.py --fleet fleet.yaml --abort
```

`fleet.yaml` maps each planner **slot** (`drone-N.plan`) to a vehicle's permanent
**sysid**, a **host**, and a **link**:

```yaml
drones:
  - slot: 1
    sysid: 1
    host: jetson-1
    remote_dir: ~/FlightPlanner
    link: "udpout://127.0.0.1:14540"      # the Jetson's own ELRS relay
    set:
      RTL_RETURN_ALT: 35
```

**`link` is the only line that changes** between a simulated vehicle, a board on USB
(maintenance only) and a flight controller behind an ELRS radio. For SITL it points at the
workstation's GCS port; for a live flight it is `127.0.0.1` in front of `elrs_link.py`.

## Which side speaks MAVLink

**The companion, always.** `swarm_deploy.py` does not import pymavlink at all.

```
WORKSTATION                    │ ETH switch │              JETSON                 │ WiFi │  POCKET
swarm_deploy.py                │            │  drone_agent.py                     │      │
  · plans from KML             │            │    · heartbeats, 1 Hz, continuous   │      │
  · scp the .plan              │─── SSH ────│    · param_set                      │      │
  · "START <token> <at>"       │   JSON     │    · mission_count/item_int/ack     │ UDP  │
  · "STATUS" / "ABORT"         │   lines    │    · command_long (arm, start)      │──────│──RF──▶
  ← {"ready":true,"token":…}   │            │  elrs_link.py (127.0.0.1)           │      │
```

The switch carries SSH and file copies; the radio carries MAVLink. They never mix. Mission
upload is stop-and-wait — cost is `items × round-trip` — so it belongs next to the radio,
not two hops away. And the agent owning the link is what makes the failsafe possible.

## The sequence

```
 orchestrator                         agent (one per vehicle, stays alive)
      │ ssh: prepare --resident ────────▶ verify sysid, clear mission, load params,
      │                                   upload, reset index, preflight contract
      │ ◀──── {ready, token: run7:a3f9…}
      │                                   ── holds the link, heartbeating ──
      │ BARRIER  all ready, or nobody arms
      │ stdin: START run7:a3f9… <epoch> ▶ check token, arm, MISSION_START
      │ ◀──── {started, t_epoch}
      │ stdin: STATUS  (every 10 s) ────▶ mode, waypoint, altitude, battery
      │ stdin: ABORT ───────────────────▶ RTL
```

Three design points, each from something that went wrong:

**Agents stay resident.** PX4 will not arm for a ground station it has not heard from
recently — the arm returns `MAV_RESULT_TEMPORARILY_REJECTED`, and only a statustext sweep
reveals *"Arming denied: Resolve system health failures first"*. An agent that exits after
`prepare` must rebuild that relationship from cold at exactly the wrong moment: cheap on
localhost UDP, expensive on a radio. One connection, prepare through landing.

**A run token binds START to what was prepared.** The token is the run id plus a hash of
the plan actually uploaded. Without it, `start` has no memory that `prepare` happened — it
would arm a vehicle that was never verified, or one whose plan changed in between. The
barrier checks every drone *said* ready; the token checks the thing about to arm is the
thing that was checked.

**Each agent holds its own failsafe.** If the orchestrator goes silent for `--failsafe-s`
(default 60 s) after launch, the agent returns its aircraft to launch unprompted. Without
it, a workstation that dies after takeoff leaves aircraft flying with no supervisor.

## The preflight contract

The checks a `.plan` cannot express. Every one came from a real setting found on real
hardware:

| check | why |
|---|---|
| identity | the sysid answering matches the slot handed over |
| mission_uploaded | read back the count, do not trust the ack |
| fence_ceiling | boards were found with `GF_MAX_VER_DIST` 5–7 m and `GF_ACTION` = Land |
| rtl_altitude | `RTL_RETURN_ALT` above the mission's top layer, or RTL descends into it |
| rc_loss_action | `NAV_RCL_ACT` not Land — on an ELRS link, RC loss is also C2 loss |
| gps_fix / position_estimate | no position, no AUTO mission |
| battery | above `MIN_BATTERY_FRACTION` (0.30) |

Per-drone `set:` overrides in the fleet file are how **RTL separation** is applied — each
aircraft returns at its own altitude, so two coming home never collapse onto one layer.
PX4 never descends to return, so without this the low drone climbs onto the high drone's
layer. Flown: *"RTL: start return at 36 m"* and *"at 46 m"*, ten metres apart, against both
collapsing onto 30 m before the fix.

## Testing the handshake

```bash
tools/sitl_swarm.sh start toapayoh 2
python3 tools/test_handshake.py                    # the whole matrix
python3 tools/test_handshake.py --only dead_link   # one case
```

A handshake is defined by what it does when things go wrong, and the happy path exercises
none of that. Every case injects a **real** condition — writes an actual parameter to an
actual vehicle, points at an actual dead port, lets an actual process go silent:

| case | injected | must happen |
|---|---|---|
| `happy_path` | — | both READY, barrier passes |
| `barrier_refuses_on_unready` | `GF_MAX_VER_DIST=7` on one vehicle | refused, names `fence_ceiling`, **nobody** arms |
| `unreachable_host` | a host that does not resolve | barrier refuses, no traceback |
| `dead_link` | a UDP port with no vehicle | prepare times out, barrier refuses |
| `wrong_vehicle_answers` | slot 1's link pointed at vehicle 2 | identity check refuses |
| `token_mismatch` | `START` with a token never issued | agent refuses |
| `ssh_localhost` | the remote path over real ssh | prepare runs remotely, both READY |
| `agent_failsafe` | launch, then go silent | agent RTLs alone, vehicle in `AUTO.RTL` |

`unreachable_host` found a real defect on its first run: `push()` used `check=True`, so an
unreachable agent crashed the orchestrator with a traceback instead of failing the barrier.
That is what the matrix is for.

---

# 5. Reference

## Tools

| Path | Role |
|---|---|
| `tools/plot_mission.py` | Draws a plan over satellite imagery — fence, OpsArea, per-drone lanes, ingress. |
| `tools/plot_track.py` | Flown track against the plan; cross-track and altitude hold. |
| `tools/plot_exclusion.py` | Highlights legs crossing an excluded notch. |
| `tools/viz_voronoi.py` | Debug view of the voronoi partition. |
| `tools/fly_mission.py` | Uploads a `.plan` to PX4 via MAVSDK and flies it. One vehicle. |
| `tools/sitl_swarm.sh` | Launches N PX4 SITL instances into **one** Gazebo world. |
| `tools/swarm_record.py` | Records every SITL vehicle's position on one wall clock. |
| `tools/swarm_separation.py` | Closest approach per pair against the guardrail's limits. |
| `tools/drone_agent.py` | One vehicle's agent: prepare, **upload**, **arm/disarm** (resident `ARM <token>` / `DISARM` verbs), start, status, abort. **Runs on the companion.** |
| `tools/swarm_deploy.py` | Orchestrator: plan, push, prepare, **barrier**, launch together, watch. |
| `tools/test_handshake.py` | Fault-injection matrix for the handshake, against live SITL. |
| `tools/switch_link.sh` | Find and address a companion on the unmanaged switch, over IPv6 link-local. |
| `tools/jetson_net.sh` | Puts a companion in its two-network role: ETH to the LAN, WiFi to a backpack. |
| `tools/elrs_link.py` | Straddles the backpack's asymmetric MAVLink ports. **Runs on the companion.** |
| `tools/elrs_service.sh` | Installs `elrs_link.py` as a systemd unit so the relay comes back after a reboot. **Runs on the companion.** |
| `tools/pair_arm_test.py` | Arms every aircraft in a fleet at one instant, or nobody: link gate, status, READY barrier, hold, disarm. Type `a` to abort all. |
| `tools/safety_rules.py` | The abort ladder -- WARN/HOLD/LAND/DISARM -- as pure, tested logic. **Runs on the companion.** |
| `tools/takeover.py` | The operator's abort switch: sets the ABORT flag on every companion over ssh, orchestrator or no orchestrator. |
| `tools/rc_takeover.py` | Read-only check that the pilot's RC switch really can take each aircraft back. |
| `tools/elrs_pair.py` | Pins each companion to exactly one backpack by BSSID, and proves it. |
| `tools/mavlink_bridge.py` | Puts a serial flight controller on UDP; can rewrite its system id. |
| `tools/qgc_capture/` | Scripted QGC screenshots (X11 clicks; fragile by nature). |

Config files: `config.json` (planner), `fleet.yaml` (all-local SITL fleet),
`fleet-jetson.yaml` (the real two-Jetson fleet), `artex-registry.yaml` (which backpack is
which), `parameters.yaml` (every knob), `sar_turn_params.params` (PX4 vehicle params).

## Traps

Each of these cost real time. They are in rough order of how often they recur.

**One consumer per link.** PX4 SITL starts its GCS link with no remote specified, meaning
*announce to 127.0.0.1:14550 until somebody talks to me, then speak to that partner alone*
— permanently, even after that partner exits. So anything attaching to `18570+i` silently
blinds QGroundControl, and restarting the swarm is the only cure. Measured, same listener
both times:

| | heartbeats on 14550 |
|---|---|
| after something had touched 18570-2 | **none** |
| fresh swarm, nothing attached | sysid 1: 21, sysid 2: 20, sysid 3: 20 |

**Leave `18570+i` for QGroundControl; use `14540+i` for tooling.** The same holds on
serial: two processes on one flight controller each read half the byte stream and both
break. It is also why a companion that must serve a GCS, an agent and a logger at once
needs a MAVLink router.

**Stale mission index.** PX4 persists the mission *and the current item index* in dataman
across restarts. A vehicle whose last run ended on the RTL item resumes **there** — it
"returns" from 1 m and disarms without ever taking off. Always
`clear_mission()` → `upload` → `set_current_mission_item(0)`.

**Under Gazebo, `PX4_HOME_LAT/LON` is ignored.** Home comes from the world SDF's
`<spherical_coordinates>`. If the world origin and the KML sector disagree, every waypoint
is kilometres away and PX4 rejects the mission with *"Geofence violation for waypoint 1"*.
Make a world per site and select it with `PX4_GZ_WORLD=<name>`.

**Stale dataman across sites.** A geofence from a previous session will reject a perfectly
valid new mission. `rm build/px4_sitl_default/rootfs/dataman` when changing sites.

**Run the SITL binary with `-d`.** Without it the `pxh>` prompt redraws into a non-TTY
forever — a background run wrote a 2.7 GB log in minutes.

**Never `pkill -f` a pattern that appears in your own command line.** It matches the shell
running the command and kills it mid-script. Kill by numeric PID:

```bash
ps -eo pid,cmd | awk '/mavlink[_]bridge/ {print $1}' | while read p; do kill "$p"; done
```

**`nmcli dev wifi list` serves a stale cache.** It reported a dead access point at a frozen
signal for minutes after the radio was switched off. For "is this AP actually present", only
`iw dev <if> scan` is trustworthy.

**`MIS_YAWMODE` does not exist in PX4 v1.16** — it is `MPC_YAW_MODE`, and the numeral does
not carry over. The survey flies value **3** ("along trajectory"); 4 ("yaw first") stops
dead at every corner.

## Troubleshooting

| symptom | cause | fix |
|---|---|---|
| QGC shows no vehicles | something attached to `18570+i` | restart the swarm; nothing less works |
| QGC shows one vehicle instead of N | instances launched without `-i` | use `sitl_swarm.sh` |
| Second/third flight fails to start | `mavsdk_server` port clash | distinct `--server-port` per flight |
| A port is busy with nothing running | orphaned `mavsdk_server` | `ss -lunp \| grep 14540`, kill by PID |
| Vehicle "returns to launch" from 1 m | stale mission index in dataman | clear, upload, reset index to 0 |
| Arming denied, health failures | no GCS heartbeat reaching PX4 | the agent heartbeats; bare pymavlink does not |
| Bridge dies: *multiple access on port* | QGC opened the same tty | the bridge takes `TIOCEXCL`; start it before QGC |
| Real board connected but not on the map | no GPS fix — bench boards have none | expected; check satellite count in QGC |
| Gazebo world half-built | instances raced to start the server | use `sitl_swarm.sh`, it waits for `/clock` |
| Jetson unreachable after a reboot | profile lived only in `/run` | check `/etc/netplan`; see *Bringing up a companion* |
| Jetson unreachable, cable is in | NM never brought the device up | check far-side `carrier`, not just `ping6` |
| Two drones on one transmitter | backpack profile not pinned to a BSSID | `elrs_pair.py`; every backpack shares one SSID |
| Jetson loses apt/DNS after joining a backpack | accepted the backpack's default route | `ipv4.never-default yes` |

## Testing

```bash
python3 -m pip install -e ".[dev]"
python3 -m pytest -q
```

Covers KML parsing (including missing-geofence rejection), both scanners, guardrail
accept/reject with transits, `.plan` structure, per-drone split and the parameter reference
against live code. None of them needs hardware or a simulator; `test_handshake.py` is separate and does need live SITL.

## Provenance and limitations

This build consolidates the ground-station **Multi-Agent Flight Planner** (KML in,
multi-drone partition, deterministic guardrail) with the
**drone-dev** coverage workspace (altitude-driven camera physics, blur-safe speed, complete
takeoff→lawnmower→RTL missions proven in SITL). The retired single-drone pipeline is in
`../legacy/planning/`. What changed from each source, and why, is recorded in
[`../history.md`](../history.md).

Known limitations:

* **Launch scheduling is not implemented.** N aircraft leaving one GCS point is the one
  measured breach of both separation limits.
* **RTL always returns at `MPC_XY_CRUISE`** — PX4 calls `reset_cruising_speed()` on RTL, so
  the mission's speed item does not apply to the return leg.
* **Corner speed is capped by geometry, not tuning.** PX4's per-waypoint limit is
  `sqrt(MPC_ACC_HOR × param2 × tan(bend/2))`. Since `turn_radius_m` derives from lane
  spacing, which derives from altitude, **altitude indirectly caps turn speed far below the
  blur-safe cruise the same altitude produces** — so duration estimates from cruise speed
  run ~50 % optimistic.
* **No steady bank.** PX4 re-plans a straight line at every waypoint switch, so a drawn
  circle arrives as one ~16° pulse per vertex. A true continuous bank needs offboard
  setpoints, not waypoints.
* **The partition splits the OpsArea's long axis.** On SAFTI (305 × 721 m, 3 drones) that
  gives 30 lanes over 7.57 km; splitting the short axis gives 15 lanes over 8.45 km — half
  the turns for ~12 % more scan length. Worth revisiting now turn cost is measured.
* **The RF hop is unmeasured**, and backpack **downlink is unproven** — `packets_down` has
  never been non-zero on any board.
* Projection is equirectangular; AEQD is marginally more accurate but negligible at SAR
  scale. `camera.py` models one nadir gimbal; zoom and oblique framing are out of scope.
