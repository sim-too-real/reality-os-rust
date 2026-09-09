//! Test-only crash injection. Enabled by feature `hil-faults`.
//! Production builds compile this to a no-op. Not a public authority API.

#[cfg(feature = "hil-faults")]
pub fn crash_if(point: &str) {
    if std::env::var("REALITYOS_HIL_CRASH").ok().as_deref() == Some(point) {
        eprintln!("hil_crash:{point}");
        std::process::exit(77);
    }
}

#[cfg(not(feature = "hil-faults"))]
#[inline]
pub fn crash_if(_point: &str) {}
