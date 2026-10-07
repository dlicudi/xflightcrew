# Ground conformance check

```
xvl conform
xvl conform --only mcp,yoke
xvl conform --prepare
```

Every control channel the live profile declares, moved on a PARKED aircraft
through the code's own seam, read back, and put back. One row per channel,
PASS / FAIL / SKIPPED with the numbers read, printed as a markdown table and
written as one blackbox `conform` record each (`channel`, `verdict`,
`detail`, `reads`, `profile`). Exit status 1 on any FAIL or a refusal. Run it
after loading a new airframe or editing its profile, BEFORE the first flight:
on 2026-10-07 the Zibo 737 took about nine flights to find bugs a two minute
probe on the ground each would have shown (yoke ratios overwritten, takeoff
trim at 8.35 units, levers snapping to the hardware throttle, a beta reverse
write undone, MCP toggles, a heading window that must be written, a V/S
window quantum).

It is a second process beside the sim (like `xvl sortie`): it resolves the
profile the way the app does (`XCOPILOT_AIRCRAFT`, `auto` detects the loaded
model) and installs that airframe's facts in its own process only
(`flight_state::install_for_check`); it begins no flight and stands nothing
down in a running xvl.

## --prepare

A cold aircraft cannot show every channel: the Zibo's six MCP modes all
stayed off with the IRS not aligned, and its reversers deployed on the APU
and the electric pumps and never stowed (2026-10-07). `--prepare` brings the
aircraft up first, the way full auto's crew does from cold, saying each
milestone on stdout (the crew's lines and every switch with it):

1. Refused unless parked (on the ground, stopped, park brake set), unless
   the profile declares `boeing_start`, and with under 500 kg of fuel
   aboard (`sim/cockpit2/fuel/fuel_quantity` summed: a reload left the
   tanks empty and the APU never came up).
2. The battery: the profile's bus commands (`tools::bus_power_on_cmds`), the
   bus read live.
3. The preflight (`preflight::run`), the APU and its generators first
   through the start's own APU steps, as full auto runs it.
4. The engine start (`jet_start::BoeingStart::run`) unless every engine
   already runs; engines not running after it ends the prepare.
5. The IRS alignment waited on (`preflight::wait_aligned`, bounded by the
   profile's `timeout_secs`), then the `after_aligned` steps.

Parked is read again between the stages: a moving aircraft sets the park
brake and ends it. A failed prepare is a `prepare` FAIL record and exit 1;
the check does not run.

## The rules

- **Parked or refused**: on the ground, stopped (`sim::ground`), park brake
  set (the add-on's own position where the profile declares
  `park_brake_toggle`). A failed read refuses too. The groundspeed is read
  again before every probe: a moving aircraft stops the check, sets the park
  brake (toe brakes held) and fails a `parked` row.
- **Read first, put back, read back.** A probe that could not restore says
  `NOT restored` in its row.
- **Nothing moves the aircraft.** Thrust only from idle (lever at or under
  0.05, else SKIPPED) and only to 0.08; reverse only from idle, a 1 s hold;
  the park brake released only under the plugin's toe brakes
  (`xcopilot/taxi_brake` held) and set again at once.
- **A failed read is a FAIL with "no reading"**, never a guess.
- **The reversers end stowed or the check says so**: wherever the reverse
  was probed, the last row (`reversers_stowed`) reads every engine's deploy
  ratio, and a deployed one (or no reading) is a WARNING row, printed again
  under the table and counted for the exit status.

## The channels

| Row | Area | Seam | PASS when |
|---|---|---|---|
| `profile` | profile | `aircraft::profile_references`, `detect::missing_profile_names` | every name the profile relies on exists in the sim |
| `mcp_window_heading`, `_altitude`, `_vs`, `_airspeed` | mcp | `autoflight::set_dial_on` / `read_dial_on` (the declared window: written, quantum snapped, or a knob stepped) | reads the test value, then the original, within the knob step, the quantum's half or 1 (a Mach window 0.001). A V/S window reading 0 with V/S off is SKIPPED (blank) |
| `mcp_mode_<mode>` | mcp | `autoflight::press_logged_on`, `read_status_on` or the declared lamp | the lamp changes on a press (waited up to 2 s, lamps ramp in) and comes back on a second press (a toggle left where it was). Any other mode the presses moved is named. Some modes refuse to engage on the ground: read the detail. Where the profile declares `preflight.align`, every IRS mode must read `want` first, else every mode row is SKIPPED "IRS not aligned: modes need an aligned platform" and nothing is pressed (`--prepare` aligns it) |
| `mcp_switch_<switch>` | mcp | `autoflight::ensure_switch_on` (FD, FO FD, A/T arm, where declared) | flips and flips back |
| `flaps` | flaps | `tools::flaps::set_handle_with` / `read_handle_with` | the next detent and back, within 0.02 |
| `speedbrake` | speedbrake | `tools::controls::speedbrake_set_with` / `speedbrake_read_with` (stock sense) | the flight detent (0.5) and back, within 0.05 |
| `trim` | trim | declared `takeoff_trim`: a 1 s blip of its switch, wound back on `takeoff_trim::next_hold`; else the stock trim ratio a 0.1 step | the gauge moves the switch's way and comes back within 0.1 units (0.02 stock). The table's figure for the live CG and weight is printed beside it |
| `yoke_pitch`, `yoke_roll` | yoke | `yoke::arm` (the airframe's route), then `xcopilot/pitch` / `xcopilot/roll` held at +/-0.5 | the elevator (`elevator1_deg`) or aileron (`aileron1_deg`) element that moved most moves 2 deg each way, opposite ways, aft giving POSITIVE `elevator1_deg`, and recentres within 1 deg on release. The row names the element judged and every element that moved 0.5 deg (`elements moved: 8 +13.6, 9 +13.6`), flagged when they disagree in sign. Aft positive is the stock array's sense: the Zibo flew nose up on `+xcopilot/pitch` all day on 2026-10-07 and its stock `elevator1_deg` reads the opposite sign of its own `laminar/B738/controls/elevator_deg` |
| `taxi_steer` | steer | `xcopilot/taxi_steer` held at +/-0.5 | `tire_steer_command_deg` moves 1 deg each way and recentres |
| `tiller` | steer | `tiller::Tiller` armed and written with `xcopilot/taxi_steer` beside it, the same command, as auto taxi drives them (the Zibo's `zibo_yaw` tiller rides the pedal channel) | the nosewheel turns a quarter of the declared `max_deg` each way (5 deg undeclared) and recentres |
| `throttle` | throttle | `aircraft::throttle_dataref()` | 0.08 read 0.4 s AND 1.5 s after the write (a lever snapped back to the hardware reads right once), then idle again; the hardware axis is printed where declared |
| `reverse` | reverse | the declared `reverse_hold_command`, held 1 s; else `throttle_beta_rev_ratio` written -1.2 per engine and put back | deploy ratio from 0.5 (the furthest of the engines) or the lever in reverse while held, then stowed within 8 s (polled). Not deployed with no engine running is SKIPPED (no hydraulics); deployed and not stowed with the engines off is a FAIL "deployed but did not stow with engines off (stows on engine hydraulics)" |
| `park_brake` | brake | `tools::controls::drive_park_brake`, `set_park_brake_confirmed` | reads released, then set again |

A channel the profile does not declare is a SKIPPED row saying so.

## Timing

Each probe is a few seconds; the whole check on the Zibo is about 90 s. The
mode buttons dominate (two presses, each waited on). `--prepare` adds the
start and the IRS alignment, 5 to 17 minutes by latitude on the Zibo.
