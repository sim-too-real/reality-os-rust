#[test]
fn public_surface_cannot_forge_write_token() {
    let t = trybuild::TestCases::new();
    t.compile_fail("tests/ui/*.rs");
}
