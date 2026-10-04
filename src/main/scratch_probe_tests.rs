    #[test]
    #[ignore]
    fn probe_phrases() {
        let profile = crate::aircraft::load("cessna_172sp").unwrap();
        let registry = crate::tools::Registry::for_aircraft(&profile);
        for t in std::env::var("PROBE").unwrap_or_default().split('|') {
            let m = crate::grammar::normalize(&crate::phrase::normalize_numbers(&crate::phrase::fix_mishears(t)));
            let r = crate::grammar::try_parse(&m, &registry, &profile);
            println!("PROBE {t:?} -> {m:?} -> {:?}", r.map(|c| c.iter().map(|x| format!("{} {}", x.name, x.arguments)).collect::<Vec<_>>()));
        }
    }
