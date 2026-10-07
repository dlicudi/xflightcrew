//! Stage 1 of the command loop: take a line (push-to-talk or continuous
//! capture, the typed line from the panel, terminal or EFB) and the
//! airframe it is for.
use super::*;

impl Session {
    /// The line taken: a typed line or a spoken capture, logged and mirrored,
    /// with the settings it is routed under. `None` is the idle tick.
    pub(crate) fn take_line(
        &mut self,
        input: Option<Input>,
        config: std::sync::Arc<Config>,
    ) -> Option<Utterance> {
        let reporter = &self.reporter;
        let explicit = self.explicit;
        // Input is spoken (gated like before) or typed in the TUI. A typed
        // command is deliberate, so it skips the confidence/wake/domain gates and
        // goes straight to the model.
        // Confidence of a spoken utterance, applied *after* the abort check so a
        // panic word lands even if whisper wasn't sure. `None` for typed input.
        let mut spoken_confidence: Option<f32> = None;
        // True when the RADIO PTT carried this utterance: it was TRANSMITTED,
        // so it belongs to whatever station COM1 is tuned to, never the copilot.
        let mut from_radio = false;
        let (command, from_text) = match input {
            None => return None,
            Some(Input::Typed(text, taken_at)) => {
                tracing::info!(typed = %text, "typed command");
                // From the channel to routing: the station work above ran
                // between them, and "consumed" alone hid 26 s of it (#135).
                blackbox::record(
                    "typed",
                    serde_json::json!({ "event": "routed", "text": &text,
                                        "held_ms": taken_at.elapsed().as_millis() as u64 }),
                );
                reporter.log(ui::LogKind::Heard, format!("⌨  \"{text}\""));
                efb::feed::note_pilot(&text, false);
                (text, true)
            }
            Some(Input::Spoken(heard, radio)) => {
                tracing::info!(transcript = %heard.text, confidence = heard.confidence, radio, "heard utterance");
                reporter.heard(&heard.text, heard.confidence, radio);
                efb::feed::note_pilot(&heard.text, radio);
                spoken_confidence = Some(heard.confidence);
                from_radio = radio;
                // Blackbox every spoken capture with its confidence, so the drop /
                // confirm / act thresholds can be tuned against the real distribution
                // after a flight (none was logged before, so the bands are set blind).
                blackbox::record(
                    "heard",
                    serde_json::json!({
                        "transcript": &heard.text, "confidence": heard.confidence,
                        "radio": radio, "explicit": explicit,
                    }),
                );
                // Continuous mode acts on the transcript directly — the downstream
                // domain gate (`looks_like_command`) filters non-commands.
                (heard.text, false)
            }
        };
        Some(Utterance {
            config,
            command,
            from_text,
            spoken_confidence,
            from_radio,
            force_confirm_low: false,
        })
    }

    /// The airframe the line is for: everything profile-derived rebuilt on a
    /// swap, and the old airframe's conversation forgotten.
    pub(crate) fn follow_airframe(&mut self) {
        // Aircraft switched? Rebuild everything profile-derived before acting on this
        // command, so the tools, limits, prompt, and checklists match what's actually
        // in the sim. (The poller swaps the shared profile; the monitor reacts on its
        // own.) The switch itself is announced by the poller.
        if self.live_profile.generation() != self.active_gen {
            self.active_gen = self.live_profile.generation();
            self.profile = self.live_profile.get();
            self.registry = Registry::for_aircraft(&self.profile);
            self.checklists = flow::install(&self.profile);
            instructor::install(&self.profile);
            self.stt.set_prompt(stt::build_prompt(&self.profile));
            self.stt.set_radio_prompt(format!(
                "{} {}",
                stt::build_prompt(&self.profile),
                tower::STT_PROMPT
            ));
            // The old airframe's conversation dies with it: a back-reference
            // must never nudge a value commanded on a different aircraft.
            self.dialogue.clear();
            self.conversation.clear();
            self.history.clear();
            self.last_subject = None;
            forget_last_readback();
            self.voice.forget_speech();
            tracing::info!(aircraft = %self.profile.display_name, "reconfigured for new aircraft");
        }
    }
}
