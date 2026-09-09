#[test]
fn online_locked_cannot_weaken_policy() {
    let t = trybuild::TestCases::new();
    t.compile_fail("tests/ui/*.rs");
}
