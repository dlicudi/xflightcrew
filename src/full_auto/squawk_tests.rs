    use super::squawk_owed;

    /// The Malaga clearance came back after the wait gave up: the code is
    /// owed until the transponder shows it, and not once it does.
    #[test]
    fn a_late_squawk_is_owed_until_dialed() {
        assert_eq!(squawk_owed(None, Some(7000.0)), None);
        assert_eq!(squawk_owed(Some(4503), Some(7000.0)), Some(4503));
        assert_eq!(squawk_owed(Some(4503), None), Some(4503));
        assert_eq!(squawk_owed(Some(4503), Some(4503.0)), None);
    }
