#[test]
fn online_session_time_is_not_caller_supplied() {
    let t = trybuild::TestCases::new();
    t.compile_fail("tests/ui/*.rs");
}
