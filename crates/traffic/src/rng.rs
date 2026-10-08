//! A tiny deterministic generator: the same seed is the same traffic, on
//! every machine, forever. Not for anything that needs real randomness.

/// Marsaglia's xorshift64 (13, 7, 17).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct XorShift64 {
    state: u64,
}

impl XorShift64 {
    /// A zero seed would stay zero forever; it is replaced by a fixed
    /// odd constant.
    pub fn new(seed: u64) -> Self {
        Self {
            state: if seed == 0 {
                0x9E37_79B9_7F4A_7C15
            } else {
                seed
            },
        }
    }

    pub fn next_u64(&mut self) -> u64 {
        let mut x = self.state;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.state = x;
        x
    }

    /// Uniform in [0, 1), 53 bits.
    pub fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Uniform in [lo, hi).
    pub fn range_f64(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.next_f64()
    }

    /// Uniform in 0..n (0 when n is 0).
    pub fn below(&mut self, n: u64) -> u64 {
        if n == 0 {
            0
        } else {
            self.next_u64() % n
        }
    }

    /// A child generator for one aircraft, so adding an aircraft never
    /// changes another's numbers.
    pub fn fork(&mut self) -> XorShift64 {
        XorShift64::new(self.next_u64() ^ 0xD1B5_4A32_D192_ED03)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_same_seed_is_the_same_sequence() {
        let mut a = XorShift64::new(42);
        let mut b = XorShift64::new(42);
        let va: Vec<u64> = (0..100).map(|_| a.next_u64()).collect();
        let vb: Vec<u64> = (0..100).map(|_| b.next_u64()).collect();
        assert_eq!(va, vb);
        assert_ne!(XorShift64::new(43).next_u64(), va[0]);
    }

    #[test]
    fn zero_is_a_seed_too_and_floats_stay_in_range() {
        let mut r = XorShift64::new(0);
        for _ in 0..1000 {
            let f = r.next_f64();
            assert!((0.0..1.0).contains(&f));
            let g = r.range_f64(5.0, 6.0);
            assert!((5.0..6.0).contains(&g));
            assert!(r.below(7) < 7);
        }
        assert_eq!(r.below(0), 0);
    }
}
