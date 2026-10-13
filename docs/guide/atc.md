# ATC

The controllers run inside the app. They hear you when COM1 is tuned to
their frequency and what you say is a radio call; otherwise you are talking
to the crew. Frequencies, taxi routes and approaches come from X-Plane's own
`apt.dat`, `earth_nav.dat` and CIFP data.

## Positions

Delivery, Ground, Tower and Approach are separate stations where the airport
publishes them, each on its own frequency, with handoffs between them. ATIS
has its own frequency. Phraseology follows the region: ICAO and UK CAP 413 in
Europe, FAA in the US.

## What they handle

- IFR clearances, with a departure procedure that joins your filed route
- pushback and taxi, with named routes ("taxi to holding point Alpha one via
  Bravo")
- line up and wait, takeoff, circuits
- radar contact, climbs and handoffs en route
- instrument approaches, landing clearances, go-arounds
- taxi in to a stand

## Rules they keep

- Runway clearances must be read back. `XCOPILOT_ATC_READBACK=false` turns
  this into coaching only.
- A misheard call gets "say again", never a clearance.
- Wake turbulence spacing between departures, and departures fitted into
  gaps between arrivals.
- No landing clearance onto an occupied runway.
- Calls you miss while a push-to-talk key is held are repeated.

## Taxi guidance

The copilot gives turn-by-turn taxi guidance along the cleared route ("next
left, onto Sierra five") and calls runway crossings.

## Settings

`XCOPILOT_ATC_AIRPORT` sets the starting field (default `LXGB`).
`XCOPILOT_MODE=copilot` turns the simulated ATC off, for use with online ATC.
See [configuration](configuration.md).
