//! URDF / MJCF → EmbodimentGraph. Unsupported features become diagnostics.

use std::collections::BTreeMap;

use sha2::{Digest, Sha256};

use crate::{
    CapabilityManifest, EmbodimentError, EmbodimentGraph, FrameSpec, JointSpec, LinkSpec,
};

const UNSUPPORTED: &[&str] = &[
    "mimic",
    "transmission",
    "gazebo",
    "collision",
    "visual",
    "mesh",
    "tendon",
    "equality",
    "contact",
    "actuator",
    "sensor",
    "plugin",
    "ros2_control",
    "include",
    "loop_joint",
    "ball",
];

pub fn import_urdf(xml: &str, kind: &str) -> Result<EmbodimentGraph, EmbodimentError> {
    import_xml(xml, kind, "urdf")
}

pub fn import_mjcf(xml: &str, kind: &str) -> Result<EmbodimentGraph, EmbodimentError> {
    import_xml(xml, kind, "mjcf")
}

fn import_xml(xml: &str, kind: &str, dialect: &str) -> Result<EmbodimentGraph, EmbodimentError> {
    if xml.trim().is_empty() {
        return Err(EmbodimentError::Parse("empty_model".into()));
    }
    let mut diagnostics = Vec::new();
    for tag in UNSUPPORTED {
        if contains_tag(xml, tag) {
            diagnostics.push(format!("unsupported:{tag}"));
        }
    }

    let mut joints = Vec::new();
    let mut links = Vec::new();
    let mut frames = vec![FrameSpec {
        name: "world".into(),
        parent: String::new(),
    }];
    let mut parent = "world".to_string();

    if dialect == "urdf" {
        for (attrs, inner) in scan_elements(xml, "link") {
            let name = attrs.get("name").cloned().unwrap_or_else(|| "link".into());
            let mass = child_attr(&inner, "mass", "value").and_then(|s| s.parse().ok());
            links.push(LinkSpec {
                name,
                parent_joint: None,
                mass_kg: mass,
                com_m: None,
                inertia_diag: None,
            });
        }
    }

    for (attrs, inner) in scan_elements(xml, "joint") {
        let name = attrs
            .get("name")
            .cloned()
            .unwrap_or_else(|| format!("joint_{}", joints.len()));
        let jtype = attrs
            .get("type")
            .map(String::as_str)
            .unwrap_or(if dialect == "mjcf" { "hinge" } else { "revolute" });
        if matches!(jtype, "fixed" | "ball" | "free" | "planar") {
            diagnostics.push(format!("dropped_{jtype}_joint:{name}"));
            continue;
        }
        let (q_min, q_max, tau_max, dq_max) = if dialect == "mjcf" {
            mjcf_limits(&attrs, &mut diagnostics, &name)
        } else {
            urdf_limits(&inner, jtype, &mut diagnostics, &name)
        };
        let axis_child = child_attr(&inner, "axis", "xyz");
        let axis = parse_xyz(
            attrs
                .get("axis")
                .map(String::as_str)
                .or(axis_child.as_deref())
                .unwrap_or("0 0 1"),
        );
        let origin_raw = child_attr(&inner, "origin", "xyz").or_else(|| attrs.get("pos").cloned());
        let origin = parse_xyz(origin_raw.as_deref().unwrap_or("0 0 0"));
        let mapped_type = match jtype {
            "slide" | "prismatic" => "prismatic",
            "continuous" => "continuous",
            _ => "revolute",
        };
        let child = child_attr(&inner, "child", "link").unwrap_or_else(|| format!("link_{name}"));
        let p = child_attr(&inner, "parent", "link").unwrap_or_else(|| parent.clone());
        joints.push(JointSpec {
            name: name.clone(),
            q_min,
            q_max,
            tau_max,
            dq_max,
            axis,
            joint_type: mapped_type.into(),
            origin_xyz: origin,
        });
        if !links.iter().any(|l| l.name == child) {
            links.push(LinkSpec {
                name: child.clone(),
                parent_joint: Some(name.clone()),
                mass_kg: None,
                com_m: None,
                inertia_diag: None,
            });
        } else if let Some(l) = links.iter_mut().find(|l| l.name == child) {
            l.parent_joint = Some(name.clone());
        }
        frames.push(FrameSpec {
            name: name.clone(),
            parent: p,
        });
        parent = child;
    }

    if joints.is_empty() {
        return Err(EmbodimentError::Parse("no_actuated_joints".into()));
    }
    if links.iter().any(|l| l.mass_kg.is_none()) {
        diagnostics.push("inertia_unspecified".into());
    }

    let id = scan_elements(xml, "robot")
        .first()
        .and_then(|(a, _)| a.get("name"))
        .cloned()
        .or_else(|| {
            scan_elements(xml, "mujoco")
                .first()
                .and_then(|(a, _)| a.get("model"))
                .cloned()
        })
        .unwrap_or_else(|| format!("imported_{dialect}"));

    Ok(EmbodimentGraph {
        capabilities: CapabilityManifest::from_kind(kind),
        id,
        kind: kind.into(),
        source: dialect.into(),
        source_hash: hex::encode(Sha256::digest(xml.as_bytes())),
        joints,
        links,
        frames,
        diagnostics,
    })
}

fn urdf_limits(
    inner: &str,
    jtype: &str,
    diagnostics: &mut Vec<String>,
    name: &str,
) -> (f64, f64, f64, f64) {
    if jtype == "continuous" {
        diagnostics.push(format!("continuous_joint:{name}"));
        return (-1.0e9, 1.0e9, 1.0, 1.0);
    }
    let limits = scan_elements(inner, "limit");
    let Some((attrs, _)) = limits.first() else {
        diagnostics.push(format!("missing_limit:{name}"));
        return (-1.0, 1.0, 1.0, 1.0);
    };
    let q_min = parse_f64(attrs.get("lower"), -1.0);
    let q_max = parse_f64(attrs.get("upper"), 1.0);
    let tau_max = parse_f64(attrs.get("effort"), 1.0).abs();
    let dq_max = parse_f64(attrs.get("velocity"), 1.0).abs();
    (q_min, q_max, tau_max.max(1e-9), dq_max.max(1e-9))
}

fn mjcf_limits(
    attrs: &BTreeMap<String, String>,
    diagnostics: &mut Vec<String>,
    name: &str,
) -> (f64, f64, f64, f64) {
    if let Some(range) = attrs.get("range") {
        let parts: Vec<f64> = range
            .split_whitespace()
            .filter_map(|s| s.parse().ok())
            .collect();
        if parts.len() == 2 {
            return (parts[0], parts[1], 1.0, 1.0);
        }
    }
    diagnostics.push(format!("missing_range:{name}"));
    (-1.0, 1.0, 1.0, 1.0)
}

fn parse_f64(raw: Option<&String>, default: f64) -> f64 {
    raw.and_then(|s| s.parse().ok())
        .filter(|v: &f64| v.is_finite())
        .unwrap_or(default)
}

fn parse_xyz(raw: &str) -> [f64; 3] {
    let p: Vec<f64> = raw
        .split_whitespace()
        .filter_map(|s| s.parse().ok())
        .collect();
    if p.len() == 3 && p.iter().all(|v| v.is_finite()) {
        [p[0], p[1], p[2]]
    } else {
        [0.0, 0.0, 1.0]
    }
}

fn child_attr(inner: &str, tag: &str, attr: &str) -> Option<String> {
    scan_elements(inner, tag)
        .first()
        .and_then(|(a, _)| a.get(attr).cloned())
}

fn contains_tag(xml: &str, name: &str) -> bool {
    let a = format!("<{name}");
    let b = format!("<{name} ");
    let c = format!("<{name}>");
    let d = format!("<{name}/");
    xml.contains(&a) && (xml.contains(&b) || xml.contains(&c) || xml.contains(&d) || xml.contains(&a))
}

fn scan_elements(xml: &str, name: &str) -> Vec<(BTreeMap<String, String>, String)> {
    let open = format!("<{name}");
    let mut out = Vec::new();
    let mut i = 0;
    while let Some(rel) = xml[i..].find(&open) {
        let start = i + rel;
        let after = start + open.len();
        let next = xml.as_bytes().get(after).copied();
        if !matches!(next, Some(b' ' | b'\n' | b'\t' | b'\r' | b'>' | b'/')) {
            i = after;
            continue;
        }
        let Some(rel_gt) = xml[after..].find('>') else {
            break;
        };
        let gt = after + rel_gt;
        let open_tag = &xml[start..=gt];
        let attrs = parse_attrs(open_tag);
        if open_tag.trim_end().ends_with("/>") {
            out.push((attrs, String::new()));
            i = gt + 1;
            continue;
        }
        let close = format!("</{name}>");
        if let Some(p) = xml[gt + 1..].find(&close) {
            let inner = xml[gt + 1..gt + 1 + p].to_string();
            out.push((attrs, inner));
            i = gt + 1 + p + close.len();
        } else {
            i = gt + 1;
        }
    }
    out
}

fn parse_attrs(tag: &str) -> BTreeMap<String, String> {
    let mut map = BTreeMap::new();
    let mut rest = tag;
    while let Some(eq) = rest.find('=') {
        let key = rest[..eq]
            .split_whitespace()
            .last()
            .unwrap_or("")
            .to_string();
        let after = rest[eq + 1..].trim_start();
        let quote = after.chars().next();
        if quote == Some('"') || quote == Some('\'') {
            let q = quote.unwrap();
            if let Some(end) = after[1..].find(q) {
                if !key.is_empty() {
                    map.insert(key, after[1..1 + end].to_string());
                }
                rest = &after[1 + end + 1..];
                continue;
            }
        }
        break;
    }
    map
}

#[cfg(test)]
mod tests {
    use super::*;
    use realityos_kernel::Capability;

    #[test]
    fn urdf_import_keeps_limits_and_diagnostics() {
        let xml = r#"
        <robot name="arm_fixture">
          <link name="base"><inertial><mass value="1.0"/></inertial></link>
          <link name="l1"><inertial><mass value="0.4"/></inertial></link>
          <joint name="j1" type="revolute">
            <parent link="base"/>
            <child link="l1"/>
            <axis xyz="0 0 1"/>
            <limit lower="-1.2" upper="1.2" effort="25" velocity="3"/>
          </joint>
          <joint name="lock" type="fixed"><parent link="l1"/><child link="tool"/></joint>
          <gazebo>x</gazebo>
          <transmission name="t1"/>
        </robot>
        "#;
        let g = import_urdf(xml, "serial_arm").unwrap();
        assert_eq!(g.dof(), 1);
        assert!((g.joints[0].q_min + 1.2).abs() < 1e-12);
        assert!(g.diagnostics.iter().any(|d| d.contains("unsupported:gazebo")));
        assert!(g.diagnostics.iter().any(|d| d.contains("dropped_fixed_joint")));
        assert!(g.requires(Capability::SerialArm));
    }

    #[test]
    fn mjcf_import_does_not_silent_drop_tendon() {
        let xml = r#"
        <mujoco model="quad_fix">
          <worldbody>
            <body name="torso">
              <joint name="hx" type="hinge" axis="1 0 0" range="-0.4 0.4"/>
              <joint name="hy" type="hinge" axis="0 1 0" range="-0.8 0.8"/>
            </body>
          </worldbody>
          <tendon/>
        </mujoco>
        "#;
        let g = import_mjcf(xml, "quadruped").unwrap();
        assert_eq!(g.dof(), 2);
        assert!(g.diagnostics.iter().any(|d| d == "unsupported:tendon"));
        assert!(g.requires(Capability::FloatingBase));
        assert!(!g.id.contains("unitree"));
    }
}
