//! Stage 3 of the command loop, the part taken on words alone before the
//! grammar: the pre-grammar shortcuts' answers (`question_shortcut`, in its
//! order, pinned by `the_loop_asks_every_shortcut_in_order`), the cabin's
//! line, the taxi route and the crew desk.
use super::*;

impl Session {
    /// The fuel question, answered from the tanks and the plan.
    pub(crate) async fn answer_fuel(
        &mut self,
        u: &mut Utterance,
        shortcut: Option<Shortcut>,
    ) -> Line {
        let xplane = &self.xplane;
        let voice = &self.voice;
        let reporter = &self.reporter;
        let live = &self.live;
        let profile = &self.profile;
        let config = &*u.config;
        let command = &u.command;
        if shortcut == Some(Shortcut::Fuel) {
            // The landing-fuel projection needs the destination, which is only
            // populated by a plan fetch — after a restart it's empty until
            // something asks. Prime it (cached, cheap) so "will we have enough
            // fuel" projects instead of just reading the tanks.
            if !config.simbrief_id.trim().is_empty() {
                let _ = ensure_plan(&config, &mut self.fp_cache, PlanFetch::Cached).await;
            }
            let line =
                fuel_report(xplane, &profile, Some(&live), is_endurance_query(&command)).await;
            self.conversation.observe(&command, &line, Instant::now());
            say(&voice, &reporter, ui::LogKind::Readback, &line);
            return Line::Spent;
        }
        Line::Open
    }

    /// An interphone call addressed to the cabin, or on its open line.
    pub(crate) fn cabin_call(&mut self, u: &mut Utterance) -> Line {
        let voice = &self.voice;
        let reporter = &self.reporter;
        let profile = &self.profile;
        let cabin_state = &self.cabin_state;
        let command = &u.command;
        // Interphone: an utterance ADDRESSED to the cabin ("cabin crew,
        // prepare for landing") is theirs, answered deterministically —
        // cabin calls are rare and formulaic, so the LLM never sees them.
        // The address is the routing (no third PTT; see crate::cabin). On an
        // airframe with no cabin crew the copilot answers honestly instead
        // of letting the model improvise a flight attendant.
        // A bare "cabin crew?" leaves the line open: the next utterance is
        // theirs without the address, when it is a cabin call.
        let addressed = cabin::Cabin::addressed(&command);
        let on_open_line = match addressed {
            Some(_) => None,
            None => cabin_state
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .take_open_line(&command, Instant::now()),
        };
        if addressed.is_some() || on_open_line.is_some() {
            if cabin::aboard(profile.cabin_crew) {
                let reply = match on_open_line {
                    Some(reply) => reply,
                    None => cabin_state
                        .lock()
                        .unwrap_or_else(|e| e.into_inner())
                        .handle(addressed.as_deref().unwrap_or(""), Instant::now()),
                };
                blackbox::record(
                    "cabin",
                    serde_json::json!({ "route": "call", "call": &command, "reply": &reply }),
                );
                reporter.status(ui::Phase::Speaking);
                reporter.log(ui::LogKind::Readback, format!("CAB: {reply}"));
                voice.say_cabin(Priority::Readback, &reply);
                reporter.status(ui::Phase::Listening);
            } else {
                say(
                    &voice,
                    &reporter,
                    ui::LogKind::Readback,
                    "No cabin crew on this airframe. It's just us up front.",
                );
            }
            return Line::Spent;
        }
        Line::Open
    }

    /// A ground navigation question answered off the published taxi route.
    pub(crate) async fn taxi_question(&mut self, u: &mut Utterance) -> Line {
        let xplane = &self.xplane;
        let voice = &self.voice;
        let reporter = &self.reporter;
        let profile = &self.profile;
        let registry = &self.registry;
        let command = &u.command;
        // Ground navigation questions ("which way?", "say the taxi route")
        // answered deterministically off the published taxi route — the FO
        // reading the airport diagram, never the LLM. Position reads happen
        // BEFORE the route lock: no guard held across an await.
        // Only ON THE GROUND: airborne, "where are we going?" is about the
        // flight plan, and the taxi reading answered it with "we don't have a
        // taxi clearance yet" at 1,000 ft (LEMG, 2026-09-12). A failed read
        // keeps the old ground reading.
        let on_ground_now = || async {
            xplane
                .get_float(datarefs::ON_GROUND_ANY)
                .await
                .map(|v| v > 0.5)
                .unwrap_or(true)
        };
        if is_taxi_query(&command)
            && grammar::try_parse(&command, &registry, &profile).is_none()
            && on_ground_now().await
        {
            let pos = match (
                xplane.get_float(datarefs::LATITUDE).await,
                xplane.get_float(datarefs::LONGITUDE).await,
            ) {
                (Ok(lat), Ok(lon)) => Some((lat, lon)),
                _ => None,
            };
            let line = party_line::with_taxi_route(|route| match (route, pos) {
                (Some(w), Some((lat, lon))) => taxi_guide::describe(w, lat, lon),
                (Some(w), None) if !w.via.is_empty() => {
                    format!("The cleared route is via {}.", w.via)
                }
                (Some(_), None) => {
                    "We have a taxi clearance, but I can't read our position right now.".to_string()
                }
                (None, _) => "We don't have a taxi clearance yet. Call the tower and request taxi."
                    .to_string(),
            });
            say(&voice, &reporter, ui::LogKind::Readback, &line);
            return Line::Spent;
        }
        Line::Open
    }

    /// The crew desk on words alone: the release guard, the radio work and the
    /// shortcuts' answers, all on one lowered line.
    pub(crate) async fn desk(&mut self, u: &mut Utterance, shortcut: Option<Shortcut>) -> Line {
        // Live systems check + static reference limits — often phrased as commands,
        // not questions ("check all systems", "performance limits"), so they're
        // routed here, not behind the question gate.
        let tl = u.command.to_ascii_lowercase();
        if self.release_or_radio(&tl).await.spent() {
            return Line::Spent;
        }
        if self.answer_shortcut(u, shortcut, &tl).await.spent() {
            return Line::Spent;
        }
        self.answer_flight_question(u, shortcut, &tl).await
    }

    /// The release guard, then the station's radio work: a content-routed
    /// call, named-station tuning, the frequency briefing, the radio coach.
    pub(crate) async fn release_or_radio(&mut self, tl: &str) -> Line {
        let xplane = &self.xplane;
        let voice = &self.voice;
        let reporter = &self.reporter;
        let dual_ptt = self.dual_ptt;
        // FIRST, before anything else: while the copilot holds a control
        // authority (speed/taxi hold, mixture manager), any release-shaped
        // utterance lets go — even a garbled one. Whisper mangled "my
        // throttle" seven ways in a row live while the governor kept
        // taxiing; when we hold something, ambiguity resolves toward
        // giving it back. Naming a lever releases just that one; a generic
        // "my controls" / "let go" releases everything held.
        {
            // The one read of what the crew holds and the one door that
            // lets each go (`flight_state::Held`), in its own order.
            let held = flight_state::Held::now();
            if held.any()
                && release_intent(&tl)
                // "Put us on final, my controls" is a new flight the
                // pilot will fly, not a hand-back of this one.
                && !grammar::approach_setup_request(&tl)
                && !grammar::enroute_setup_request(&tl)
            {
                let released = held.release_to_pilot(&tl);
                if !released.is_empty() {
                    crate::flight::note_pilot_takeover("my controls");
                    let line = format!("Copy — {}.", released.join("; "));
                    say(&voice, &reporter, ui::LogKind::Readback, &line);
                    return Line::Spent;
                }
            }
        }
        // ATC station: a radio call answered by the tower when COM1 is tuned
        // to it. After the release guard above (handing controls back always
        // wins), before everything else — "request taxi" is a call to the
        // world, not a cockpit command. Falls through untouched when not
        // tuned or not a radio call. CONTENT routing only exists for setups
        // with no radio key: with a dedicated transmit button bound
        // (dual_ptt) this utterance came over the INTERCOM, and physical
        // routing beats content routing in both directions ("requesting
        // clearance" to the copilot was answered by the tower, 2026-07-12).
        // And either way the radio must be powered and the airframe not
        // wreckage — a dead panel can't transmit (fail open, like the
        // receive gate).
        if let Some(st) = self.atc_station.as_mut() {
            // "Put us on a ten mile final" carries the radio word
            // "final": it is the crew's reposition, never a final report.
            if !grammar::approach_setup_request(&tl)
                && !grammar::enroute_setup_request(&tl)
                && try_route_content_atc(st, &tl, dual_ptt, &voice, &reporter, xplane).await
            {
                return Line::Spent;
            }
            // Named-station tuning ("tune the tower", "contact approach") —
            // a COPILOT action: resolve the name against the station's
            // apt.dat frequencies, write COM1, confirm what's actually set.
            match st.tune_request(&tl) {
                Some(party_line::Tune::Found { label, khz }) => {
                    let line = match xplane.set_int(datarefs::COM1_FREQ_KHZ, khz).await {
                        Ok(()) => {
                            let mhz = xplane
                                .get_float(datarefs::COM1_FREQ_KHZ)
                                .await
                                .map(|v| v / 1000.0)
                                .unwrap_or(khz as f64 / 1000.0);
                            format!("Roger, com one {mhz:.3} for {label}.")
                        }
                        Err(e) => {
                            tracing::warn!("COM1 write failed: {e:#}");
                            "Couldn't set the radio.".to_string()
                        }
                    };
                    say(&voice, &reporter, ui::LogKind::Readback, &line);
                    return Line::Spent;
                }
                Some(party_line::Tune::Missing(msg)) => {
                    say(&voice, &reporter, ui::LogKind::Readback, &msg);
                    return Line::Spent;
                }
                None => {}
            }
            // Frequency briefing ("what frequencies are here", "who can I
            // talk to") — crew knowledge answered from the loaded apt.dat
            // rows plus live COM1, after the tuning verbs so those win.
            if party_line::freq_query(&tl) {
                let line = st.freq_briefing(xplane.view()).await;
                say(&voice, &reporter, ui::LogKind::Readback, &line);
                return Line::Spent;
            }
            // Radio coach ("what do I say?"): the copilot reads the
            // tower's own state machine and hands the pilot the exact
            // next call — deterministic, and always a call the parser
            // routes. A learning tool, not an LLM guess.
            if party_line::coach_query(&tl) {
                let on_ground = xplane
                    .get_float(datarefs::ON_GROUND_ANY)
                    .await
                    .map(|v| v > 0.5)
                    .unwrap_or(true);
                let callsign = crate::crew_radio::crew_callsign(xplane)
                    .await
                    .unwrap_or_else(|| "your callsign".to_string());
                let dist = match (position(xplane).await, party_line::active_field()) {
                    (Some((lat, lon, _)), Some((_, flat, flon, _))) => {
                        Some(nav::haversine_nm(lat, lon, flat, flon))
                    }
                    _ => None,
                };
                let com1 = xplane
                    .get_float(datarefs::COM1_FREQ_KHZ)
                    .await
                    .ok()
                    .map(|v| v.round() as i64);
                // "Should we check in with ATC?" wants a yes or a no ahead
                // of the call, not just the phraseology.
                let yes_no = party_line::coach_yes_no(&tl);
                let line = match party_line::suggest_call(&callsign, on_ground, dist, com1) {
                    Some(s) => match (s.first, s.call) {
                        (Some(f), Some(c)) => {
                            let lead = if yes_no { "Yes. " } else { "" };
                            format!("{lead}{f} Then key the radio and say: {c}")
                        }
                        (Some(f), None) if yes_no => format!("No call owed right now. {f}"),
                        (Some(f), None) => f,
                        (None, Some(c)) => {
                            let lead = if yes_no { "Yes. " } else { "" };
                            format!("{lead}Key the radio and say: {c}")
                        }
                        (None, None) => "No station on frequency to call.".to_string(),
                    },
                    None => "No station on frequency to call.".to_string(),
                };
                say(&voice, &reporter, ui::LogKind::Readback, &line);
                return Line::Spent;
            }
        }
        Line::Open
    }

    /// The reminder memory, the plan check, the systems answers and the local
    /// flight planner.
    pub(crate) async fn answer_shortcut(
        &mut self,
        u: &mut Utterance,
        shortcut: Option<Shortcut>,
        tl: &str,
    ) -> Line {
        let xplane = &self.xplane;
        let voice = &self.voice;
        let reporter = &self.reporter;
        let profile = &self.profile;
        let registry = &self.registry;
        let reminders = &self.reminders;
        let config = &*u.config;
        let command = &u.command;
        // Reminder memory: cancel or list what's armed ("cancel the reminders",
        // "do we have a reminder set?"). Checked before set_reminder so "cancel"
        // wins over arming.
        {
            if shortcut == Some(Shortcut::ReminderCancel) {
                let n = reminders.cancel_all();
                let line = match n {
                    0 => "No reminders to cancel.".to_string(),
                    1 => "Cancelled the reminder.".to_string(),
                    _ => format!("Cancelled {n} reminders."),
                };
                say(&voice, &reporter, ui::LogKind::Readback, &line);
                return Line::Spent;
            }
            // A mention of reminders with no arming cue is a recall ("do we have a
            // reminder?"). If it names a condition ("set a reminder to wake me at
            // 5000 feet"), fall through to arm it instead.
            if shortcut == Some(Shortcut::ReminderList) {
                let list = reminders.active();
                let line = if list.is_empty() {
                    "No reminders set.".to_string()
                } else if list.len() == 1 {
                    format!("One reminder, {}.", list[0])
                } else {
                    format!("{} reminders: {}.", list.len(), list.join(", "))
                };
                say(&voice, &reporter, ui::LogKind::Readback, &line);
                return Line::Spent;
            }
        }
        // "Check the flight plan": the whole-flight audit, nothing armed. Before the
        // systems check, whose "check ... aircraft" these words can have.
        if shortcut == Some(Shortcut::PlanCheck) {
            reporter.status(ui::Phase::Thinking);
            let line = plan::plan_check_report(&config, xplane, &mut self.fp_cache).await;
            say(&voice, &reporter, ui::LogKind::Readback, &line);
            return Line::Spent;
        }
        if shortcut == Some(Shortcut::SystemsCheck) {
            let line = systems_check(xplane, &profile).await;
            say(&voice, &reporter, ui::LogKind::Readback, &line);
            return Line::Spent;
        }
        // Power-loss diagnosis ("why have we lost power?", "the engine quit") —
        // went unrecognized live. Phrased as a question, but what it wants is
        // the engine's fuel/ignition chain read and the culprit named.
        if shortcut == Some(Shortcut::PowerLoss) {
            let line = power_loss_diagnosis(xplane, &profile).await;
            say(&voice, &reporter, ui::LogKind::Readback, &line);
            return Line::Spent;
        }
        if let Some(line) = (shortcut == Some(Shortcut::ReferenceLimits))
            .then(|| reference_limits(&profile, &tl))
            .flatten()
        {
            self.conversation
                .reference(&profile.display_name, &line, Instant::now());
            say(&voice, &reporter, ui::LogKind::Readback, &line);
            return Line::Spent;
        }
        if shortcut == Some(Shortcut::SystemsSurvey) {
            let line = systems::current().line();
            say(&voice, &reporter, ui::LogKind::Readback, &line);
            return Line::Spent;
        }
        if let Some(line) = (shortcut == Some(Shortcut::AircraftInfo))
            .then(|| aircraft_info(&profile, &tl))
            .flatten()
        {
            self.conversation
                .reference(&profile.display_name, &line, Instant::now());
            say(&voice, &reporter, ui::LogKind::Readback, &line);
            return Line::Spent;
        }
        // The local flight planner: "plan a flight to Malaga", "divert to
        // Gibraltar", or the short answer to its own "to where?". Sits
        // before the reminder/TOD/ETA branches, which key on words a
        // destination request never carries. Deterministic: apt.dat,
        // the profile, and the sim's terrain probe. A plan on the shelf
        // authorises nothing; "load it" is a separate word.
        {
            let asked = self
                .plan_pending
                .is_some_and(|t| t.elapsed() < PLAN_ASK_WINDOW);
            // The shortcuts before it have all answered: here it is this
            // one, a later one, or none, and the short answer to "to
            // where?" comes ahead of the later ones as it always did.
            let planned = (shortcut == Some(Shortcut::PlanFlight))
                .then(|| flightplan::parse_request(&tl))
                .flatten();
            let req = planned.or_else(|| {
                // An answer is a place, short, and not a command in its own right.
                let words = tl.split_whitespace().count();
                let reply = tl.trim().strip_prefix("to ").unwrap_or(tl.trim());
                (asked && words <= 4 && grammar::try_parse(&command, &registry, &profile).is_none())
                    .then(|| flightplan::parse_destination(reply))
                    .flatten()
                    .map(|d| flightplan::PlanRequest {
                        dest: Some(d),
                        load: false,
                    })
            });
            if let Some(req) = req {
                self.plan_pending = None;
                let line = match req.dest {
                    None => {
                        self.plan_pending = Some(Instant::now());
                        "To where?".to_string()
                    }
                    Some(_) => {
                        reporter.status(ui::Phase::Thinking);
                        plan_flight(&config, xplane, &profile, &mut self.fp_cache, &req).await
                    }
                };
                self.conversation.observe(&command, &line, Instant::now());
                say(&voice, &reporter, ui::LogKind::Readback, &line);
                return Line::Spent;
            }
        }
        Line::Open
    }

    /// The flight's questions: descent, weather, ETA, the runway, the
    /// configuration, traffic, the course, the fixes and the plan itself.
    pub(crate) async fn answer_flight_question(
        &mut self,
        u: &mut Utterance,
        shortcut: Option<Shortcut>,
        tl: &str,
    ) -> Line {
        let xplane = &self.xplane;
        let voice = &self.voice;
        let reporter = &self.reporter;
        let live = &self.live;
        let profile = &self.profile;
        let config = &*u.config;
        let command = &u.command;
        // "wake me / remind me / let me know / tell me when ..." is a reminder to
        // arm, not a status query to answer now. Let it fall through to the tool
        // layer so it doesn't get hijacked by the nav-progress branch below (both
        // mention "next waypoint").
        // Time/distance to top of descent ("how long before TOD"). Guarded by
        // is_reminder so "wake me at top of descent" still arms the reminder.
        // "top of descent" is reliably misheard ("when is top of descent" came in
        // as "1 is top descent"), so match top+descent loosely rather than the
        // exact phrase. This also keeps such an utterance out of the vertical-speed
        // grammar, which would otherwise read "descend 1 feet per minute".
        // "Put us enroute 30 miles before top of descent" is the enroute
        // setup, not a question (answered "Top of descent about 893 miles
        // ahead" at LEMG, 2026-09-28, and the setup never ran).
        if shortcut == Some(Shortcut::TopOfDescent) {
            let line = time_to_tod(&config, xplane, &mut self.fp_cache).await;
            self.conversation.observe(&command, &line, Instant::now());
            say(&voice, &reporter, ui::LogKind::Readback, &line);
            return Line::Spent;
        }
        // Weather somewhere ELSE ("what weather can we expect on arrival",
        // "weather at LEMG"): the sim's datarefs describe the air we are in,
        // and reading them back for an arrival question is a confident wrong
        // answer (seen live 45 nm out of Malaga, 2026-09-12). The real-weather
        // METAR file is the honest source; without one, say so.
        if shortcut == Some(Shortcut::RemoteWeather) {
            if let Some(target) =
                remote_weather_target(&tl, self.fp_cache.as_ref().map(|(_, fp)| fp))
            {
                let line = match target {
                    RemoteWeather::NoPlan => {
                        "No flight plan loaded, so I have no destination to look up.".to_string()
                    }
                    RemoteWeather::Station { icao, label } => {
                        match weather::briefing(&crate::config::xplane_root(), &icao, &label) {
                            Ok(l) | Err(l) => l,
                        }
                    }
                };
                self.conversation.observe(&command, &line, Instant::now());
                say(&voice, &reporter, ui::LogKind::Readback, &line);
                return Line::Spent;
            }
        }
        // ETA to the destination ("time before we land", "when do we get in",
        // "how far before we land"): distance to the destination airport over
        // groundspeed. Beats the next-waypoint branch below (`eta_question`).
        if shortcut == Some(Shortcut::Eta) {
            let line = eta_to_destination(&config, xplane, &mut self.fp_cache).await;
            self.conversation.observe(&command, &line, Instant::now());
            say(&voice, &reporter, ui::LogKind::Readback, &line);
            return Line::Spent;
        }
        // Destination field elevation — from the plan's destination record,
        // never the LLM (which answered "9999 feet": our own cruise altitude).
        if shortcut == Some(Shortcut::DestinationElevation) {
            if !config.simbrief_id.trim().is_empty() {
                let _ = ensure_plan(&config, &mut self.fp_cache, PlanFetch::Cached).await;
            }
            let dest = monitor::DESTINATION
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .clone();
            let line = match dest {
                Some(d) => format!(
                    "Field elevation at the destination is about {:.0} feet.",
                    (d.elev_ft / 50.0).round() * 50.0
                ),
                None => "No destination on file — load the flight plan first.".to_string(),
            };
            say(&voice, &reporter, ui::LogKind::Readback, &line);
            return Line::Spent;
        }
        // Runway choice — answered from apt.dat + the wind, never the LLM
        // (which once invented "runway 18" at a field that only has 10/28).
        // "Do we have enough runway for takeoff?": the takeoff's own fit
        // and pavement rule, said as two numbers. Ahead of the runway
        // ADVICE below, whose "use"/"should" would otherwise take it.
        if shortcut == Some(Shortcut::TakeoffRunway) {
            reporter.status(ui::Phase::Thinking);
            let line = answers::takeoff_runway_answer(xplane, &profile).await;
            say(&voice, &reporter, ui::LogKind::Readback, &line);
            return Line::Spent;
        }
        if shortcut == Some(Shortcut::RunwayAdvice) {
            reporter.status(ui::Phase::Thinking);
            let line = runway_advice(&config, xplane, &mut self.fp_cache).await;
            say(&voice, &reporter, ui::LogKind::Readback, &line);
            return Line::Spent;
        }
        // "Are we good for landing?" — the landing-config check. Phrased as a
        // question so "set flaps for landing" (a command) never lands here.
        if shortcut == Some(Shortcut::LandingReadiness) {
            let line = landing_readiness(xplane, &profile).await;
            say(&voice, &reporter, ui::LogKind::Readback, &line);
            return Line::Spent;
        }
        // "Do we need any lights on?" / "are we configured?" — the SOP table
        // answers. Guarded so light *commands* ("nav lights on") never match.
        if shortcut == Some(Shortcut::ConfigAdvice) {
            let line = config_advice(xplane, &profile).await;
            say(&voice, &reporter, ui::LogKind::Readback, &line);
            return Line::Spent;
        }
        // "Any traffic around?" — the TCAS scan, spoken as clock positions.
        if shortcut == Some(Shortcut::Traffic) {
            let line = traffic_report(xplane).await;
            say(&voice, &reporter, ui::LogKind::Readback, &line);
            return Line::Spent;
        }
        // "Are we on course / on track?" — HSI deviation + waypoint, computed
        // (the LLM once answered this with the single word "heading").
        if shortcut == Some(Shortcut::OnCourse) {
            let line = on_course_report(xplane, Some(&live)).await;
            say(&voice, &reporter, ui::LogKind::Readback, &line);
            return Line::Spent;
        }
        // A NAMED fix ("how much left for PIMOS", "what's after PIMOS"):
        // the ident is matched fuzzily against the loaded plan, because
        // whisper spells a fix it has never seen phonetically ("pimmons"
        // for PIMOS, live 2026-09-12). Only a plan already on the shelf is
        // consulted: a stray "for X" must never trigger a SimBrief fetch.
        if shortcut == Some(Shortcut::NamedFix) {
            if let Some(q) = self
                .fp_cache
                .as_ref()
                .and_then(|(_, fp)| fix_query(&tl, &fp.waypoints))
            {
                let line = fix_answer(&q, xplane).await;
                self.conversation.observe(&command, &line, Instant::now());
                say(&voice, &reporter, ui::LogKind::Readback, &line);
                return Line::Spent;
            }
        }
        // Navigation progress (often a command, not a question): ETA / distance /
        // time to the next waypoint.
        if shortcut == Some(Shortcut::NavProgress) {
            let line = nav_progress(xplane, Some(&live)).await;
            self.conversation.observe(&command, &line, Instant::now());
            say(&voice, &reporter, ui::LogKind::Readback, &line);
            return Line::Spent;
        }
        // "route" is reliably misheard as "root"/"road" (both meaningless in a
        // cockpit), so accept them as synonyms.
        // Load the SimBrief route into the GPS/FMS (checked before the
        // brief, so "load the flight plan" loads rather than reads it back),
        // never the load SHEET, which is `load_aircraft`'s.
        if shortcut == Some(Shortcut::LoadRoute) {
            reporter.status(ui::Phase::Thinking);
            let line =
                load_route_to_fms(&config, xplane, &mut self.fp_cache, plan_fetch_for(&tl)).await;
            say(&voice, &reporter, ui::LogKind::Readback, &line);
            return Line::Spent;
        }
        // "That's the wrong flight plan": the pilot has regenerated it on
        // SimBrief; fetch again and say whether anything actually changed
        // (whisper: "wrong flag plan", live 2026-09-14).
        if shortcut == Some(Shortcut::WrongPlan) {
            reporter.status(ui::Phase::Thinking);
            let before = self.fp_cache.as_ref().map(|(_, fp)| fp.clone());
            let line = match ensure_plan(&config, &mut self.fp_cache, PlanFetch::Forced).await {
                Err(msg) => msg,
                Ok(fetched) => plan_change_line(
                    before.as_ref(),
                    self.fp_cache.as_ref().map(|(_, fp)| fp),
                    fetched,
                ),
            };
            say(&voice, &reporter, ui::LogKind::Readback, &line);
            return Line::Spent;
        }
        // SimBrief flight plan (cached fetch) + advice. Route, full brief, or
        // refresh. Not when the intent is to *fly* it ("follow/navigate/track"
        // is the navigate tool; the word "fly" is the full-auto SimBrief
        // mission — both handled via the grammar/LLM path below).
        if shortcut == Some(Shortcut::PlanBrief) {
            reporter.status(ui::Phase::Thinking);
            // "route" is reliably misheard as "root"/"road".
            let want_route = tl.contains("route") || tl.contains("root") || tl.contains("road");
            let line = flight_plan_report(
                &config,
                xplane,
                &mut self.fp_cache,
                plan_fetch_for(&tl),
                want_route,
                reporter.status_sink(),
            )
            .await;
            say(&voice, &reporter, ui::LogKind::Readback, &line);
            return Line::Spent;
        }
        Line::Open
    }
}
