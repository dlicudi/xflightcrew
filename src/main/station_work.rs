//! Stage 1b of the command loop: the station work done between taking a
//! line and routing it, on every tick, idle included.
use super::*;

impl Session {
    /// The loop's own work between taking a line and routing it, done on the
    /// idle tick too: the mode, the station a keyed call or the monitor
    /// armed, the tower's syncs, then the crew's queued radio calls.
    pub(crate) async fn station_work(&mut self, config: &Config, input: &Option<Input>) {
        self.follow_mode(config).await;
        self.arm_keyed_station(input).await;
        self.take_pending_station(config);
        self.sync_tower(config).await;
        self.drain_traffic_calls();
        self.drain_crew_radio(input).await;
    }

    /// Our own traffic's calls (stage D), answered on each flight's own
    /// desk after the desks are synced with what the ticks said
    /// (`traffic_link::drain_calls`). Nothing unless `XCOPILOT_TRAFFIC=ours`.
    pub(crate) fn drain_traffic_calls(&mut self) {
        if crate::traffic_link::TrafficMode::now() != crate::traffic_link::TrafficMode::Ours {
            return;
        }
        if let Some(st) = self.atc_station.as_mut() {
            crate::traffic_link::note_station_runway(st.runway_in_use());
            let replies = crate::traffic_link::drain_calls(st);
            crate::traffic_link::overhear_replies(&self.voice, replies);
        }
    }

    /// A mode change (panel toggle or the poller's pick) reconciled: the world
    /// side torn down or brought up, the crew's and the instructor's edges.
    pub(crate) async fn follow_mode(&mut self, config: &Config) {
        let xplane = &self.xplane;
        let voice = &self.voice;
        let reporter = &self.reporter;
        // Reconcile a mode change (panel toggle or the poller's pick) — on
        // the idle tick too, so a flip lands within ~2 s of the click even
        // in a silent cockpit. The world side is torn down/brought up here;
        // the monitor's ticks and the routing gates below read mode::* live.
        self.sim_radio_sync
            .follow(
                xplane,
                self.xp_radio_muted && mode::atc_enabled(),
                tts::AUDIO_BUS.load(std::sync::atomic::Ordering::Relaxed),
            )
            .await;
        let now_mode = mode::get();
        if now_mode != self.applied_mode {
            tracing::info!(
                from = self.applied_mode.as_str(),
                to = now_mode.as_str(),
                "mode change"
            );
            blackbox::record(
                "mode",
                serde_json::json!({ "from": self.applied_mode.as_str(), "to": now_mode.as_str() }),
            );
            // World side.
            if mode::atc_enabled() && self.atc_station.is_none() {
                self.atc_station = install_home_station(&config, &voice);
                // The radio soundscape is reclaimed by `sim_radio_sync`.
                let line = match self.atc_station.as_ref() {
                    Some(st) => format!(
                        "ATC is on. {} is up.",
                        party_line::spoken_airport(st.airport_name())
                    ),
                    None => "ATC is on, no station configured.".to_string(),
                };
                reporter.log(ui::LogKind::Info, line.clone());
                voice.alert(Priority::Advisory, line);
            } else if !mode::atc_enabled() && self.atc_station.is_some() {
                self.atc_station = None;
                // Retire every published fact (party line + monitor side
                // state) and cut whatever the station was saying mid-word.
                party_line::Station::uninstall();
                tts::stop_channel(tts::Channel::Radio);
                // X-Plane gets its radio voice back through `sim_radio_sync`:
                // the pilot may be switching to the sim's OWN ATC.
                reporter.log(
                    ui::LogKind::Info,
                    "ATC off; the radio belongs to the real world now".to_string(),
                );
                voice.alert(
                    Priority::Advisory,
                    "ATC is off. Radio is all yours.".to_string(),
                );
            }
            // Crew side (edges only: AtcOnly is the one mode without a crew).
            if self.applied_mode == mode::Mode::AtcOnly && now_mode != mode::Mode::AtcOnly {
                self.atc_only_noted = false;
                voice.alert(Priority::Advisory, "Copilot back with you.".to_string());
            } else if now_mode == mode::Mode::AtcOnly && self.applied_mode != mode::Mode::AtcOnly {
                // Leaving the crew behind: cut any copilot speech and evict
                // queued crew lines too. Radio traffic stays on the air.
                voice.dismiss_crew();
            }
            // The instructor's edges: hands come off everything that is
            // running (the airframe-swap stand-down, since a lesson and a
            // loop cannot share the aircraft), and a lesson does not
            // outlive the mode that allowed it.
            if now_mode == mode::Mode::Instructor && self.applied_mode != mode::Mode::Instructor {
                flight_state::stand_down();
                self.instructor_noted = false;
                voice.alert(
                    Priority::Advisory,
                    "Instructor mode. My hands are off the controls. Your aircraft.".to_string(),
                );
            } else if self.applied_mode == mode::Mode::Instructor
                && now_mode != mode::Mode::Instructor
            {
                if instructor::abandon("mode_change") {
                    voice.alert(
                        Priority::Advisory,
                        "Lesson over, the mode changed.".to_string(),
                    );
                }
                if now_mode != mode::Mode::AtcOnly {
                    voice.alert(Priority::Advisory, "Copilot back with you.".to_string());
                }
            }
            self.applied_mode = now_mode;
        }
    }

    /// A keyed call on a frequency another in-range field covers arms that
    /// field's station for this very call.
    pub(crate) async fn arm_keyed_station(&mut self, input: &Option<Input>) {
        let xplane = &self.xplane;
        // You call it, you get it: a keyed transmission on a frequency the
        // active station doesn't cover, but another in-range field does, arms
        // the swap so THAT field answers this very call — the ATC tab lets the
        // pilot tune any in-range field, and calling Badajoz on 122.100 must
        // not be answered by dead air and a note about Gibraltar (2026-07-10).
        // BEFORE the pending-station consumption below, so the swap lands for
        // this very utterance.
        if mode::atc_enabled() && matches!(input, Some(Input::Spoken(_, true))) {
            if let Ok(v) = xplane.get_float(datarefs::COM1_FREQ_KHZ).await {
                let com1 = v.round() as i64;
                let covered = self
                    .atc_station
                    .as_ref()
                    .is_some_and(|st| st.covers_khz(com1));
                if !covered {
                    if let Some((lat, lon, alt_ft)) = position(xplane).await {
                        party_line::arm_station_for_freq(com1, lat, lon, alt_ft, Instant::now());
                    }
                }
            }
        }
    }

    /// A region switch the monitor armed: the new station built and installed
    /// before this utterance is routed.
    pub(crate) fn take_pending_station(&mut self, config: &Config) {
        let voice = &self.voice;
        let reporter = &self.reporter;
        // A region switch armed by the monitor: rebuild the station BEFORE
        // this utterance is routed (consumed at the loop top it lagged one
        // call behind, seen live 2026-07-09) — and on the IDLE tick too:
        // input is already captured, and a silent cockpit must still swap
        // (the Heathrow handoff sat unconsumed for minutes, 2026-07-10, and
        // the party line waited on it).
        if let Some((icao, apt)) =
            party_line::take_pending_station().filter(|_| mode::atc_enabled())
        {
            let name = apt.name.clone();
            match party_line::Station::from_airport(apt) {
                Some(mut st) => {
                    reporter.log(ui::LogKind::Info, format!("ATC region: now working {name}"));
                    st.set_enforce_readback(config.atc_readback);
                    st.set_dialect(party_line::dialect_for(&icao));
                    // Its SIDs, for the IFR clearance.
                    st.set_procedures(
                        &config.cifp_dir,
                        &config.fix_data_path,
                        &config.nav_data_path,
                        &icao,
                    );
                    st.install();
                    // The new field's taxi network, in the background.
                    party_line::spawn_taxi_net_load(config.apt_data_path.clone(), icao.clone());
                    // And its ILS chart data, same deal.
                    party_line::spawn_ils_load(
                        config.nav_data_path.clone(),
                        config.cifp_dir.clone(),
                        icao.clone(),
                    );
                    // A new field, a new crew: per-airport voices from the
                    // pool, with the country's own accent (by ICAO prefix)
                    // manning the tower and the ATIS — unless we're back
                    // home, where the curated voices belong.
                    if self.home_airport_name.as_deref() == Some(&name) {
                        voice.set_station_cast(0, Vec::new());
                    } else {
                        voice.set_station_cast(
                            crate::voice::station_voice_seed(&name),
                            crate::voice::local_voices_on_disk(&icao),
                        );
                    }
                    self.atc_station = Some(st);
                }
                None => {
                    tracing::warn!(%name, "region handoff to a field without a tower; keeping the old station")
                }
            }
        }
    }

    /// The tower FSM kept in step with what the monitor already said on
    /// frequency: readback enforcement, a released takeoff, a line-up, a
    /// go-around.
    pub(crate) async fn sync_tower(&mut self, config: &Config) {
        let xplane = &self.xplane;
        let reporter = &self.reporter;
        // Readback enforcement lives INSIDE the tower FSM, so unlike the
        // settings the loop reads per tick, it has to be pushed. Done here and
        // not only at station install, or the Settings screen would show a
        // knob that quietly waits for the next airport.
        if let Some(st) = self.atc_station.as_mut() {
            st.set_enforce_readback(config.atc_readback);
        }

        // A held takeoff clearance the monitor released once the final
        // cleared: sync the tower FSM (state + departure handoff) with what
        // was already said on frequency.
        if party_line::take_pending_takeoff_clearance() {
            if let Some(st) = self.atc_station.as_mut() {
                st.apply_monitor_takeoff_clearance(xplane.view()).await;
                reporter.log(ui::LogKind::Info, "tower: held takeoff clearance released");
            }
        }

        // The monitor sequenced a held departure onto the runway while the
        // wake clock runs: same sync, one state earlier. The hold stays armed
        // — the release is still to come, now with the aircraft actually at
        // the front of the queue.
        if party_line::take_pending_line_up() {
            if let Some(st) = self.atc_station.as_mut() {
                st.apply_monitor_line_up(xplane.view()).await;
                reporter.log(
                    ui::LogKind::Info,
                    "tower: held departure sequenced, line up and wait",
                );
            }
        }

        // A go-around the monitor ordered (traffic on the runway): the
        // landing clearance is void in the FSM too, so the coach's next
        // line is the go-around acknowledgment, never the dead clearance's
        // readback (seen live 2026-07-14: SAY kept "cleared to land" while
        // the tower shouted go around, and the pilot dutifully read it back).
        if party_line::take_pending_go_around() {
            // The pilot's "going around" after it is the readback, which the
            // crew already flies (`go_around::note_radio_call`).
            go_around::note_tower_order();
            if let Some(st) = self.atc_station.as_mut() {
                st.apply_monitor_go_around();
                reporter.log(
                    ui::LogKind::Info,
                    "tower: go-around ordered, landing clearance void",
                );
            }
        }
    }
}
