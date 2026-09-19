//! Byte/transport faults on the shipped peer + production Protocol 2.0 parser.

use std::cell::RefCell;
use std::rc::Rc;

use realityos_metal::protocol::{
    decode_status, decode_status_scan, encode_ping, encode_write, ProtocolError, ADDR_TORQUE_ENABLE,
};
use realityos_virtual_metal::faults::{corrupt_crc_bytes, FaultKind, FaultSchedule};
use realityos_virtual_metal::peer::VirtualSerialPeer;
use realityos_virtual_metal::VirtualXl330;

fn peer() -> (Rc<RefCell<VirtualXl330>>, VirtualSerialPeer) {
    let d = Rc::new(RefCell::new(VirtualXl330::xl330_m288()));
    let p = VirtualSerialPeer::new(d.clone());
    (d, p)
}

#[test]
fn reusable_peer_feeds_complete_instruction_and_returns_status() {
    let (_d, mut p) = peer();
    let status = p.exchange(&encode_ping(1));
    let st = decode_status(&status).expect("status");
    assert_eq!(st.id, 1);
    assert_eq!(st.params.len(), 3);
    let t = p.transcript();
    assert!(!t.tx.is_empty());
    assert!(!t.rx.is_empty());
}

#[test]
fn corrupt_outgoing_crc_mutates_on_wire_crc_and_parser_fails() {
    let (_d, mut p) = peer();
    p.set_transport_schedule(FaultSchedule::once(FaultKind::CorruptOutgoingCrc));
    let status = p.exchange(&encode_ping(1));
    assert_eq!(
        decode_status(&status),
        Err(ProtocolError::BadCrc),
        "production parser must see a bad CRC, not a hand-built high-level error"
    );
    let clean = {
        let (_d2, mut p2) = peer();
        p2.exchange(&encode_ping(1))
    };
    assert_ne!(status, clean);
    assert_ne!(status.last(), clean.last());
}

#[test]
fn device_corrupt_outgoing_crc_also_mutates_process_bytes() {
    let mut d = VirtualXl330::xl330_m288();
    d.set_fault_schedule(FaultSchedule::once(FaultKind::CorruptOutgoingCrc));
    let status = d.process(&encode_ping(1));
    assert_eq!(decode_status(&status), Err(ProtocolError::BadCrc));
}

#[test]
fn drop_status_is_empty_on_the_byte_path() {
    let (_d, mut p) = peer();
    p.set_transport_schedule(FaultSchedule::once(FaultKind::DropStatus));
    let status = p.exchange(&encode_ping(1));
    assert!(status.is_empty());
}

#[test]
fn truncate_status_is_visible_as_too_short_or_bad_crc() {
    let (_d, mut p) = peer();
    p.set_transport_schedule(FaultSchedule::once(FaultKind::TruncateStatus { keep: 6 }));
    let status = p.exchange(&encode_ping(1));
    assert!(status.len() <= 6);
    assert!(decode_status(&status).is_err());
}

#[test]
fn wrong_status_id_decodes_with_different_id() {
    let (_d, mut p) = peer();
    p.set_transport_schedule(FaultSchedule::once(FaultKind::WrongStatusId));
    let status = p.exchange(&encode_ping(1));
    let st = decode_status(&status).expect("crc valid, wrong id");
    assert_ne!(st.id, 1);
}

#[test]
fn silent_and_disconnect_yield_no_bytes() {
    let (_d, mut p) = peer();
    p.set_transport_schedule(FaultSchedule::once(FaultKind::DeviceSilent));
    assert!(p.exchange(&encode_ping(1)).is_empty());
    let (_d, mut p) = peer();
    p.set_transport_schedule(FaultSchedule::once(FaultKind::Disconnect));
    assert!(p.exchange(&encode_ping(1)).is_empty());
    assert!(!p.is_connected());
}

#[test]
fn garbage_prefix_fails_decode_status_but_scan_finds_frame() {
    let (_d, mut p) = peer();
    p.set_transport_schedule(FaultSchedule::once(FaultKind::GarbagePrefix));
    let status = p.exchange(&encode_ping(1));
    assert!(
        decode_status(&status).is_err(),
        "a complete junk frame before the status must fail destuff-at-first-header"
    );
    let st = decode_status_scan(&status).expect("scan past junk frame");
    assert_eq!(st.id, 1);
}

#[test]
fn delay_status_records_delay_for_timeout_path() {
    let (_d, mut p) = peer();
    p.set_transport_schedule(FaultSchedule::once(FaultKind::DelayStatus { ms: 250 }));
    let _ = p.exchange(&encode_ping(1));
    assert_eq!(p.last_delay_ms(), 250);
}

#[test]
fn split_status_returns_first_chunk_then_rest() {
    let (_d, mut p) = peer();
    p.set_transport_schedule(FaultSchedule::once(FaultKind::SplitStatusAcrossReads {
        first: 4,
    }));
    p.push(&encode_ping(1));
    let a = p.read(64);
    let b = p.read(64);
    assert_eq!(a.len(), 4);
    assert!(!b.is_empty());
    let mut all = a;
    all.extend_from_slice(&b);
    assert!(decode_status(&all).is_ok() || decode_status_scan(&all).is_ok());
}

#[test]
fn duplicate_and_suffix_still_expose_a_status_frame() {
    let (_d, mut p) = peer();
    p.set_transport_schedule(FaultSchedule::once(FaultKind::DuplicateStatus));
    let dup = p.exchange(&encode_ping(1));
    assert!(decode_status_scan(&dup).is_ok());
    let (_d, mut p) = peer();
    p.set_transport_schedule(FaultSchedule::once(FaultKind::GarbageSuffix));
    let s = p.exchange(&encode_ping(1));
    assert!(decode_status_scan(&s).is_ok());
}

#[test]
fn corrupt_crc_helper_is_not_a_noop() {
    let pkt = encode_write(1, ADDR_TORQUE_ENABLE, &[1]);
    let bad = corrupt_crc_bytes(pkt.clone());
    assert_ne!(bad.last(), pkt.last());
    assert_eq!(decode_status(&bad), Err(ProtocolError::BadCrc));
}
