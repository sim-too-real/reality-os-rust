//! Unit tests do not prove two-UID isolation.
//! The measurable harness is `scripts/hil-os-users-test.sh`.

#[test]
fn two_user_isolation_is_not_claimed_here() {
    let script = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../scripts/hil-os-users-test.sh"
    );
    assert!(
        std::path::Path::new(script).is_file(),
        "deployment harness missing: {script}"
    );
}

#[test]
#[ignore = "requires root + realityos-authority/autonomy users; run scripts/hil-os-users-ci.sh"]
fn two_user_harness_must_be_run_as_os_test() {
    panic!("invoke scripts/hil-os-users-ci.sh; do not fake this in-process");
}
