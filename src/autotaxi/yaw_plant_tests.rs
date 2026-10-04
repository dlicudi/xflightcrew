    pub struct Plant {
        pub k: f64,
        pub d: f64,
        pub delay_s: f64,
        pub tau_s: f64,
        /// Heading error, degrees (+ = the course is right of the nose).
        pub err: f64,
        /// Yaw rate, deg/s (+ = nose right, reducing a + error).
        pub yaw: f64,
        brake: f64,
        t: f64,
        hist: Vec<(f64, f64)>,
    }

    impl Plant {
        pub fn new(k: f64, d: f64, err: f64, yaw: f64) -> Self {
            Plant { k, d, delay_s: 0.3, tau_s: 0.3, err, yaw, brake: 0.0, t: 0.0, hist: Vec::new() }
        }

        /// Hold the signed brake command `cmd` for `dt` seconds.
        pub fn run(&mut self, cmd: f64, authority: f64, dt: f64) {
            self.hist.push((self.t, cmd));
            let h = 0.01;
            let end = self.t + dt;
            while self.t < end - 1e-9 {
                let delayed = self
                    .hist
                    .iter()
                    .rev()
                    .find(|(tt, _)| *tt <= self.t - self.delay_s)
                    .map_or(0.0, |(_, c)| *c);
                self.brake += (delayed - self.brake) * h / self.tau_s;
                self.yaw += (self.k * self.brake * authority - self.d * self.yaw) * h;
                self.err -= self.yaw * h;
                self.t += h;
            }
        }
    }

    /// The signed command of a (left, right) pair.
    pub fn signed((l, r): (f64, f64)) -> f64 {
        r - l
    }
