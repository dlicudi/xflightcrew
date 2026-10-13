//! Stage 4 of the command loop: what the crew says on its own account
//! (the queued radio calls) and the turn recorded once it is spoken.
use super::*;

/// What one utterance's calls did, gathered across them.
#[derive(Default)]
pub(crate) struct TurnRecord {
    // Readbacks from this utterance's actions, recorded as the assistant
    // turn afterwards so the next command can refer back to what was done.
    pub(crate) readbacks: Vec<String>,
    pub(crate) failed: bool,
    // Recovery for everything this utterance does, gathered so one "belay that"
    // undoes the whole set of commands, not just the last tool.
    pub(crate) reverts: Vec<(String, String, f64, bool)>,
    pub(crate) disengage_ap: bool,
    pub(crate) inverse: Vec<(String, serde_json::Value)>,
}

impl Session {
    /// The crew's queued radio calls, drained every tick (idle included) and
    /// transmitted as if the pilot keyed the radio, each routed before it is
    /// spoken; the tower's reply after it.
    pub(crate) async fn drain_crew_radio(&mut self, input: &Option<Input>) {
        let xplane = &self.xplane;
        let voice = &self.voice;
        let reporter = &self.reporter;
        // Radio calls the copilot makes during an auto circuit (position
        // reports; the tower answers and, on the final call, clears us to
        // land). Crew -> world: the copilot transmits exactly as if the pilot
        // keyed the radio. Drained every tick (idle included) so a report made
        // while the cockpit is silent still goes out. In copilot-only mode or
        // with no station, the calls are discarded — external ATC owns the
        // radio then, the same rule a keyed transmission follows below.
        // A pilot's line in hand (or waiting on the channel) is never held
        // behind the crew's radio work: the waits below give way to it, the
        // lines still play in order. LEMG (2026-09-28, sortie 1790586620):
        // "my controls" was taken at +0.0 and routed at +26.1 s, after the
        // crew's "request taxi" and the tower's whole taxi clearance.
        let pilot_waiting = || loop_owed(input.is_some());
        // The waits on the frequency give way to a go-around queued behind
        // them too: it goes out next (`crew_radio::priority_pending`).
        let give_way = || pilot_waiting() || crew_radio::priority_pending();
        let calls = crew_radio::take_crew_radio();
        let superseded =
            crew_radio::superseded(&calls.iter().map(|q| (q.holds, q.at)).collect::<Vec<_>>());
        // A discarded transmission is never silent. The queue is drained
        // unconditionally, so a failed gate loses the call for good — and
        // the caller (full auto) reads the resulting silence as the TOWER
        // not answering, which sent it hunting a clearance that was never
        // asked for (Heathrow 2026-07-27).
        // A readback that cannot go out is OUT for the crew waiting on it
        // (`crew_radio::readback_out`): it is gone either way.
        let delivered = mode::atc_enabled() && voice.radio_powered() && self.atc_station.is_some();
        if !delivered {
            for q in &calls {
                if let crew_radio::CrewCall::Text(t) = &q.call {
                    if q.readback != crew_radio::Readback::No {
                        crew_radio::note_readback_out(t);
                    }
                }
            }
        }
        if !calls.is_empty() && mode::atc_enabled() && !voice.radio_powered() {
            tracing::warn!(
                calls = calls.len(),
                "crew radio call dropped: the radio is unpowered"
            );
        }
        if !calls.is_empty() && mode::atc_enabled() && voice.radio_powered() {
            if let Some(st) = self.atc_station.as_mut() {
                for (queued, superseded) in calls.into_iter().zip(superseded) {
                    if superseded {
                        blackbox::record(
                            "crew_radio",
                            serde_json::json!({
                                "event": "crew_radio_stale",
                                "call": crew_radio::call_words(&queued.call),
                                "why": "superseded",
                                "age_s": (queued.at.elapsed().as_secs_f64() * 10.0).round() / 10.0,
                            }),
                        );
                        continue;
                    }
                    // A position report dies with the position it reports,
                    // judged as it is about to go out (each call waits
                    // behind the one before it): LEMG's "5 mile final"
                    // went out 93 s late, in the flare (#143).
                    let age_s = queued.at.elapsed().as_secs_f64();
                    if queued.holds != crew_radio::Holds::Always {
                        let facts = crew_radio_facts(xplane).await;
                        if let Some(why) = crew_radio::stale_reason(queued.holds, age_s, &facts) {
                            tracing::info!(
                                target: "xflightcrew::radio",
                                call = %crew_radio::call_words(&queued.call),
                                why,
                                age_s,
                                "crew radio call dropped: no longer true"
                            );
                            blackbox::record(
                                "crew_radio",
                                serde_json::json!({
                                    "event": "crew_radio_stale",
                                    "call": crew_radio::call_words(&queued.call),
                                    "why": why,
                                    "age_s": (age_s * 10.0).round() / 10.0,
                                    "on_ground": facts.on_ground,
                                    "agl_ft": facts.agl_ft.map(|h| h.round()),
                                    "gs_kts": facts.gs_kts.map(|g| g.round()),
                                }),
                            );
                            continue;
                        }
                    }
                    // A copy of the tower's suggested readback goes out only
                    // while the tower still waits for those words: a copy
                    // queued twice went out in the takeoff roll (LEMG 13,
                    // 2026-09-28, #172).
                    if let (crew_radio::Readback::OwedCopy, crew_radio::CrewCall::Text(t)) =
                        (queued.readback, &queued.call)
                    {
                        let owed = crate::party_line::readback_owed();
                        if crew_radio::owed_copy_stale(t, owed.as_deref()) {
                            crew_radio::note_readback_out(t);
                            blackbox::record(
                                "crew_radio",
                                serde_json::json!({
                                    "event": "crew_radio_stale",
                                    "call": t,
                                    "why": "not_owed",
                                    "age_s": (age_s * 10.0).round() / 10.0,
                                }),
                            );
                            continue;
                        }
                    }
                    blackbox::record(
                        "crew_radio",
                        serde_json::json!({
                            "event": "crew_radio_sent",
                            "call": crew_radio::call_words(&queued.call),
                            "age_s": (age_s * 10.0).round() / 10.0,
                            "readback": queued.readback != crew_radio::Readback::No,
                        }),
                    );
                    let queued_readback = queued.readback != crew_radio::Readback::No;
                    let call = queued.call;
                    // A check-in is rendered NOW, with the frequency quiet
                    // (bounded), so the altitude it names is the one being
                    // passed as it is spoken.
                    let call = match call {
                        crew_radio::CrewCall::Text(t) => t,
                        crew_radio::CrewCall::CheckIn {
                            callsign,
                            to,
                            climb_to_ft,
                        } => {
                            let quiet = std::time::Instant::now();
                            while voice.radio_on_air()
                                && quiet.elapsed().as_secs_f64() < 20.0
                                && !pilot_waiting()
                            {
                                tokio::time::sleep(std::time::Duration::from_millis(200)).await;
                            }
                            let alt_ft =
                                xplane.get_float(datarefs::ALTITUDE_FT).await.unwrap_or(0.0);
                            // Above the flight's transition altitude the
                            // levels are flight levels (issue #97).
                            let ta = crate::tools::instruments::transition_in_force_ft(18_000.0);
                            full_auto::departure_check_in_above(
                                &callsign,
                                &to,
                                alt_ft,
                                climb_to_ft,
                                ta,
                            )
                        }
                    };
                    // The copilot transmits (its own voice), then the tower.
                    // Both sides logged so the full ATC transcript shows in
                    // the log, not just the panel.
                    tracing::info!(target: "xflightcrew::radio", copilot = %call, "tx");
                    // ROUTED before it is spoken: `say` blocks until the
                    // frequency is quiet, and a handoff readback queued
                    // behind a party-line exchange was routed 19 s later,
                    // after full auto had retuned, so Gibraltar DEPARTURE
                    // heard "Gibraltar Departure on one three zero decimal
                    // four" and said "say again" (08:23Z, 2026-09-15). The
                    // station that was tuned when the crew keyed is the one
                    // that hears it; the audible order is unchanged.
                    let routed = st
                        .handle_transmission(&call.to_ascii_lowercase(), xplane.view())
                        .await;
                    // The reply answers the state as it was routed: what the
                    // controller says or orders before it is spoken
                    // supersedes it (`crew_radio::reply_superseded`).
                    let routed_at = Instant::now();
                    let mut orders = crate::party_line::GoAroundOrders::arm();
                    if queued_readback {
                        crew_radio::note_readback_out(&call);
                    }
                    efb::feed::crew_transmits(&format!("{call}."), queued_readback);
                    say_or(
                        &voice,
                        &reporter,
                        ui::LogKind::Readback,
                        &format!("{call}."),
                        &give_way,
                    );
                    if let Some((reply, who)) = routed {
                        let last_to_us = voice.last_controller_line().map(|l| l.at);
                        if !reply.is_empty()
                            && crew_radio::reply_superseded(
                                routed_at,
                                last_to_us,
                                orders.take(),
                                crate::party_line::readback_owed().is_some(),
                            )
                        {
                            tracing::info!(target: "xflightcrew::radio", station = %who.tag(), reply = %reply, "reply superseded, not spoken");
                            blackbox::record(
                                "crew_radio",
                                serde_json::json!({
                                    "event": "reply_superseded",
                                    "reply": reply,
                                    "held_s":
                                        (routed_at.elapsed().as_secs_f64() * 10.0).round() / 10.0,
                                }),
                            );
                        } else if !reply.is_empty() {
                            tracing::info!(target: "xflightcrew::radio", station = %who.tag(), reply = %reply, "rx");
                            reporter.log(ui::LogKind::Readback, format!("{}: {reply}", who.tag()));
                            voice.say_station_until(Priority::Readback, who, &reply, &give_way);
                        }
                    }
                }
            }
        }
    }

    /// The turn recorded once its calls have run: the conversation's report,
    /// the undo stack, the history and the subject the next bare value
    /// follow-up anchors on.
    pub(crate) fn record_turn(&mut self, u: &mut Utterance, turn: TurnRecord) {
        let command = &u.command;
        if !turn.readbacks.is_empty() {
            self.conversation.report(
                &command,
                &turn.readbacks.join(" "),
                turn.failed,
                Instant::now(),
            );
        }

        // Push this utterance's recovery onto the undo stack (newest on top), bounded.
        if !turn.reverts.is_empty() || turn.disengage_ap || !turn.inverse.is_empty() {
            self.undo_stack.push(UndoSnapshot {
                label: command.clone(),
                reverts: turn.reverts,
                disengage_ap: turn.disengage_ap,
                inverse: turn.inverse,
            });
            if self.undo_stack.len() > UNDO_STACK_MAX {
                self.undo_stack.remove(0);
            }
        }

        // Record this turn for back-reference context, then bound the history by
        // count and age so stale state can't accumulate.
        if !turn.readbacks.is_empty() {
            // Anchor for the next bare-value follow-up: the subject of this
            // command if it named a system (covers rewritten follow-ups too,
            // which are explicit by now, so a chain 80 → 70 → 60 stays anchored).
            if looks_like_command(&command) {
                let subj = command_subject(&command);
                if !subj.is_empty() {
                    self.last_subject = Some(subj);
                }
            }
            self.history
                .push((command.clone(), turn.readbacks.join(" "), Instant::now()));
            let excess = self.history.len().saturating_sub(HISTORY_MAX);
            if excess > 0 {
                self.history.drain(0..excess);
            }
            self.history.retain(|(_, _, t)| t.elapsed() < HISTORY_TTL);
        }
    }
}
