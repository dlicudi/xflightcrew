# AI traffic

With `XCOPILOT_TRAFFIC=ours`, XFlightCrew runs its own traffic, controlled by
the same tower that controls you.

## What flies

- GA circuits
- airline arrivals and departures, with the airlines and aircraft types that
  suit the field
- turnarounds on stands: an aircraft arrives, parks and leaves as its next
  flight
- go-arounds, holds and taxi in

You hear their calls when your COM1 is on their frequency. When you taxi for
yourself, the copilot stops behind traffic ahead.

## Timetables

At Gibraltar (LXGB) and Malaga (LEMG) a built-in timetable reproduces the
airlines and routes that fly there, by weekday and season. The airport is
already busy when you start: arrivals on approach, aircraft taxiing, others
boarding. The panel's Board tab shows the departures and arrivals.

Elsewhere the traffic is generated from a seed, so the same day plays out
the same way. `XCOPILOT_TRAFFIC_FILE` takes your own YAML schedule instead.

## Drawing the aircraft

Traffic is drawn through XPMP2 using the CSL models you have installed:
LiveTraffic's folder by default, or the folder named by
`XCOPILOT_TRAFFIC_CSL` in X-Plane's environment. If LiveTraffic holds
X-Plane's TCAS slots, the aircraft still draw but will not show on TCAS.

## Settings

| Variable | Default | |
|---|---|---|
| `XCOPILOT_TRAFFIC` | `off` | `ours` for this traffic, `real` to narrate X-Plane or LiveTraffic targets, `off` |
| `XCOPILOT_TRAFFIC_DENSITY` | `normal` | `off`, `light` or `normal` |
| `XCOPILOT_TRAFFIC_FILE` | | your own YAML schedule |
| `XCOPILOT_TRAFFIC_CSL` | LiveTraffic's | your CSL folder, read by the plugin from X-Plane's environment |

## Limitations

The traffic pauses under time acceleration, does not yet account for wind,
and fetches no online schedules.
