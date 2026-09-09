#[test]
fn public_surface_cannot_mint_or_ack_certified_commands() {
    let t = trybuild::TestCases::new();
    t.compile_fail("tests/ui/*.rs");
}
