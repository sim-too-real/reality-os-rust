use super::BenchmarkError;
use crate::bundle::RobotBundle;
use crate::normalize::{ContactPairRecord, ExcludedBodyPairRecord, GeomRecord, RobotManifest};
use realityos_semantics::embodiment::EmbodimentModel;
use realityos_semantics::geometry::{CollisionRole, PrimitiveShape, RigidGeometry};
use realityos_semantics::transform::Se3;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::path::{Path, PathBuf};

pub const GEOMETRY_SCHEMA_VERSION: u32 = 2;

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct MeshAssetDigest {
    pub reference: String,
    pub sha256: String,
    pub query_vertices_sha256: String,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CollisionGeomRecord {
    pub geom_id: i32,
    pub name: String,
    pub body: String,
    pub geom_type: String,
    pub size: [f64; 3],
    pub friction: [f64; 3],
    pub local_pose: Se3,
    pub contype: i32,
    pub conaffinity: i32,
    pub mesh_asset: Option<MeshAssetDigest>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "status", rename_all = "snake_case")]
pub enum GeometryGateResult {
    Complete,
    UnsupportedPresent { geom_names: Vec<String> },
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CollisionGeometryManifest {
    pub schema_version: u32,
    pub robot_id: String,
    pub bundle_hash: String,
    pub model_hash: String,
    pub geoms: Vec<CollisionGeomRecord>,
    pub contact_pairs: Vec<ContactPairRecord>,
    pub excluded_body_pairs: Vec<ExcludedBodyPairRecord>,
    pub result: GeometryGateResult,
    pub digest: String,
}

pub fn validate_collision_geometry(
    bundle: &RobotBundle,
    manifest: &RobotManifest,
    model: &EmbodimentModel,
) -> Result<CollisionGeometryManifest, BenchmarkError> {
    if bundle.manifest.robot_id != manifest.robot_id || model.robot_id != manifest.robot_id {
        return Err(BenchmarkError::Geometry(format!(
            "robot identity mismatch: bundle={}, manifest={}, semantic={}",
            bundle.manifest.robot_id, manifest.robot_id, model.robot_id
        )));
    }
    if manifest.source_hash != bundle.source_hash
        || model.source_bundle_hash != manifest.source_hash
        || model.model_hash != manifest.model_hash
    {
        return Err(BenchmarkError::Geometry(
            "bundle or semantic model hash does not match inspection manifest".into(),
        ));
    }
    if !manifest.geometry_inspection_complete {
        return Err(BenchmarkError::Geometry(
            "inspector omitted collision geometry or contact-rule tables".into(),
        ));
    }

    let mut geom_ids = BTreeSet::new();
    let mut geom_names = BTreeSet::new();
    for geom in &manifest.geoms {
        if geom.geom_id < 0 || !geom_ids.insert(geom.geom_id) {
            return Err(BenchmarkError::Geometry(format!(
                "invalid or duplicate inspected geom id {} for {}",
                geom.geom_id, geom.name
            )));
        }
        if geom.name.trim().is_empty() || !geom_names.insert(geom.name.clone()) {
            return Err(BenchmarkError::Geometry(format!(
                "empty or duplicate inspected geom name {:?}",
                geom.name
            )));
        }
    }

    let mut inspected: Vec<&GeomRecord> = manifest
        .geoms
        .iter()
        .filter(|geom| {
            geom.contype != 0
                || geom.conaffinity != 0
                || manifest
                    .contact_pairs
                    .iter()
                    .any(|pair| pair.geom1_id == geom.geom_id || pair.geom2_id == geom.geom_id)
        })
        .collect();
    inspected.sort_by_key(|geom| geom.geom_id);
    let mut semantic: BTreeMap<&str, &RigidGeometry> = BTreeMap::new();
    for geom in model
        .collision_geoms
        .iter()
        .filter(|geom| geom.collision_role != CollisionRole::Visual)
    {
        if semantic.insert(&geom.id, geom).is_some() {
            return Err(BenchmarkError::Geometry(format!(
                "duplicate semantic collision geom {}",
                geom.id
            )));
        }
    }

    if semantic.len() != inspected.len() {
        return Err(BenchmarkError::Geometry(format!(
            "collision geom count mismatch: inspected={}, semantic={}",
            inspected.len(),
            semantic.len()
        )));
    }

    let mut geometry_records = Vec::with_capacity(inspected.len());
    let mut unsupported = Vec::new();
    for geom in inspected {
        if geom.body.trim().is_empty() {
            return Err(BenchmarkError::Geometry(format!(
                "collision geom {} has no body attachment",
                geom.name
            )));
        }
        if !geom
            .size
            .iter()
            .all(|size| size.is_finite() && *size >= 0.0)
        {
            return Err(BenchmarkError::Geometry(format!(
                "collision geom {} has invalid size {:?}",
                geom.name, geom.size
            )));
        }
        let friction = geom.friction.ok_or_else(|| {
            BenchmarkError::Geometry(format!(
                "collision geom {} has no inspected friction parameters",
                geom.name
            ))
        })?;
        if friction
            .iter()
            .any(|coefficient| !coefficient.is_finite() || *coefficient < 0.0)
        {
            return Err(BenchmarkError::Geometry(format!(
                "collision geom {} has invalid friction {:?}",
                geom.name, friction
            )));
        }
        let local_pose = Se3::try_new(geom.pos, geom.quat).map_err(|error| {
            BenchmarkError::Geometry(format!(
                "collision geom {} has invalid local pose: {error}",
                geom.name
            ))
        })?;
        let semantic_geom = semantic.get(geom.name.as_str()).ok_or_else(|| {
            BenchmarkError::Geometry(format!(
                "collision geom {} is missing from semantic model",
                geom.name
            ))
        })?;
        if semantic_geom.owner_body != geom.body
            || !same_pose(semantic_geom.local_pose, local_pose)
            || !same_shape(geom, &semantic_geom.shape)
            || semantic_geom.collision_role == CollisionRole::Visual
        {
            return Err(BenchmarkError::Geometry(format!(
                "collision geom {} differs between inspection and semantic model",
                geom.name
            )));
        }
        if !semantic_geom.shape.query_supported() {
            unsupported.push(geom.name.clone());
        }
        let mesh_asset = if geom.geom_type == "mesh" {
            Some(hash_mesh_asset(bundle, geom)?)
        } else {
            if geom.mesh_asset.is_some() || !geom.mesh_vertices.is_empty() {
                return Err(BenchmarkError::Geometry(format!(
                    "non-mesh geom {} declares mesh data",
                    geom.name
                )));
            }
            None
        };
        geometry_records.push(CollisionGeomRecord {
            geom_id: geom.geom_id,
            name: geom.name.clone(),
            body: geom.body.clone(),
            geom_type: geom.geom_type.clone(),
            size: geom.size,
            friction,
            local_pose,
            contype: geom.contype,
            conaffinity: geom.conaffinity,
            mesh_asset,
        });
    }

    let canonical_pairs = canonical_contact_pairs(manifest, &geom_ids)?;
    let canonical_exclusions = canonical_exclusions(manifest)?;
    let result = if unsupported.is_empty() {
        GeometryGateResult::Complete
    } else {
        GeometryGateResult::UnsupportedPresent {
            geom_names: unsupported,
        }
    };
    let mut admitted = CollisionGeometryManifest {
        schema_version: GEOMETRY_SCHEMA_VERSION,
        robot_id: manifest.robot_id.clone(),
        bundle_hash: bundle.source_hash.clone(),
        model_hash: manifest.model_hash.clone(),
        geoms: geometry_records,
        contact_pairs: canonical_pairs,
        excluded_body_pairs: canonical_exclusions,
        result,
        digest: String::new(),
    };
    let canonical = serde_json::to_vec(&admitted)?;
    let mut digest = Sha256::new();
    digest.update(b"realityos.manipulation_benchmark/collision_geometry/2\0");
    digest.update(canonical);
    admitted.digest = hex::encode(digest.finalize());
    Ok(admitted)
}

fn hash_mesh_asset(
    bundle: &RobotBundle,
    geom: &GeomRecord,
) -> Result<MeshAssetDigest, BenchmarkError> {
    let reference = geom
        .mesh_asset
        .as_deref()
        .filter(|reference| !reference.trim().is_empty())
        .ok_or_else(|| {
            BenchmarkError::Geometry(format!(
                "collision mesh {} has no source asset reference",
                geom.name
            ))
        })?;
    let candidate = PathBuf::from(reference);
    let path = if candidate.is_absolute() {
        candidate
    } else {
        bundle
            .model_path
            .parent()
            .unwrap_or(&bundle.root)
            .join(candidate)
    };
    let canonical_path = fs::canonicalize(&path).map_err(|error| {
        BenchmarkError::Geometry(format!(
            "collision mesh {} asset {} cannot be resolved: {error}",
            geom.name, reference
        ))
    })?;
    let listed = bundle.asset_files.iter().any(|asset| {
        fs::canonicalize(asset)
            .map(|asset| asset == canonical_path)
            .unwrap_or(false)
    });
    if !listed {
        return Err(BenchmarkError::Geometry(format!(
            "collision mesh {} asset {} is not in the robot bundle asset list",
            geom.name, reference
        )));
    }
    let bytes = fs::read(&canonical_path).map_err(|error| {
        BenchmarkError::Geometry(format!(
            "collision mesh {} asset {} cannot be read: {error}",
            geom.name, reference
        ))
    })?;
    if geom.mesh_vertices.len() < 4
        || geom
            .mesh_vertices
            .iter()
            .flatten()
            .any(|coordinate| !coordinate.is_finite())
    {
        return Err(BenchmarkError::Geometry(format!(
            "collision mesh {} has missing or invalid compiled hull vertices",
            geom.name
        )));
    }
    let mut hull_hasher = Sha256::new();
    hull_hasher.update((geom.mesh_vertices.len() as u64).to_le_bytes());
    for vertex in &geom.mesh_vertices {
        for coordinate in vertex {
            hull_hasher.update(coordinate.to_bits().to_le_bytes());
        }
    }
    Ok(MeshAssetDigest {
        reference: normalize_asset_reference(reference),
        sha256: hex::encode(Sha256::digest(bytes)),
        query_vertices_sha256: hex::encode(hull_hasher.finalize()),
    })
}

fn normalize_asset_reference(reference: &str) -> String {
    Path::new(reference)
        .components()
        .filter_map(|component| match component {
            std::path::Component::Normal(value) => Some(value.to_string_lossy().into_owned()),
            std::path::Component::ParentDir => Some("..".into()),
            std::path::Component::CurDir => None,
            std::path::Component::RootDir | std::path::Component::Prefix(_) => {
                Some(component.as_os_str().to_string_lossy().into_owned())
            }
        })
        .collect::<Vec<_>>()
        .join("/")
}

fn canonical_contact_pairs(
    manifest: &RobotManifest,
    geom_ids: &BTreeSet<i32>,
) -> Result<Vec<ContactPairRecord>, BenchmarkError> {
    let mut pairs = manifest.contact_pairs.clone();
    let mut seen = BTreeSet::new();
    for pair in &mut pairs {
        if pair.geom1_id == pair.geom2_id
            || !geom_ids.contains(&pair.geom1_id)
            || !geom_ids.contains(&pair.geom2_id)
        {
            return Err(BenchmarkError::Geometry(format!(
                "contact pair references invalid geoms {} and {}",
                pair.geom1_id, pair.geom2_id
            )));
        }
        if pair.geom1_id > pair.geom2_id {
            std::mem::swap(&mut pair.geom1_id, &mut pair.geom2_id);
        }
        if !seen.insert((pair.geom1_id, pair.geom2_id)) {
            return Err(BenchmarkError::Geometry(format!(
                "duplicate contact pair {} and {}",
                pair.geom1_id, pair.geom2_id
            )));
        }
    }
    pairs.sort();
    Ok(pairs)
}

fn canonical_exclusions(
    manifest: &RobotManifest,
) -> Result<Vec<ExcludedBodyPairRecord>, BenchmarkError> {
    let bodies: BTreeSet<&str> = manifest
        .bodies
        .iter()
        .map(|body| body.name.as_str())
        .chain(std::iter::once("world"))
        .collect();
    let mut pairs = manifest.excluded_body_pairs.clone();
    let mut seen = BTreeSet::new();
    for pair in &mut pairs {
        if pair.body1.trim().is_empty()
            || pair.body2.trim().is_empty()
            || pair.body1 == pair.body2
            || !bodies.contains(pair.body1.as_str())
            || !bodies.contains(pair.body2.as_str())
        {
            return Err(BenchmarkError::Geometry(format!(
                "excluded body pair references invalid bodies {:?} and {:?}",
                pair.body1, pair.body2
            )));
        }
        if pair.body1 > pair.body2 {
            std::mem::swap(&mut pair.body1, &mut pair.body2);
        }
        if !seen.insert((pair.body1.clone(), pair.body2.clone())) {
            return Err(BenchmarkError::Geometry(format!(
                "duplicate excluded body pair {} and {}",
                pair.body1, pair.body2
            )));
        }
    }
    pairs.sort();
    Ok(pairs)
}

fn same_pose(left: Se3, right: Se3) -> bool {
    const EPSILON: f64 = 1e-9;
    let translation_matches = left
        .xyz
        .iter()
        .zip(right.xyz)
        .all(|(left, right)| (left - right).abs() <= EPSILON);
    let direct_rotation_matches = left
        .quat_wxyz
        .iter()
        .zip(right.quat_wxyz)
        .all(|(left, right)| (left - right).abs() <= EPSILON);
    let inverse_rotation_matches = left
        .quat_wxyz
        .iter()
        .zip(right.quat_wxyz)
        .all(|(left, right)| (left + right).abs() <= EPSILON);
    translation_matches && (direct_rotation_matches || inverse_rotation_matches)
}

fn same_shape(record: &GeomRecord, shape: &PrimitiveShape) -> bool {
    const EPSILON: f64 = 1e-9;
    let close = |left: f64, right: f64| (left.abs() - right.abs()).abs() <= EPSILON;
    match (record.geom_type.as_str(), shape) {
        ("sphere", PrimitiveShape::Sphere { radius }) => close(record.size[0], *radius),
        ("box", PrimitiveShape::Box { half_extents }) => record
            .size
            .iter()
            .zip(half_extents)
            .all(|(left, right)| close(*left, *right)),
        (
            "capsule",
            PrimitiveShape::Capsule {
                radius,
                half_length,
            },
        )
        | (
            "cylinder",
            PrimitiveShape::Cylinder {
                radius,
                half_length,
            },
        ) => close(record.size[0], *radius) && close(record.size[1], *half_length),
        ("plane", PrimitiveShape::Plane { .. }) => true,
        ("mesh", PrimitiveShape::ConvexMesh { vertices }) => {
            record.mesh_vertices.len() == vertices.len()
                && record
                    .mesh_vertices
                    .iter()
                    .zip(vertices)
                    .all(|(left, right)| left == right)
        }
        (kind, PrimitiveShape::Unsupported { kind: unsupported }) => kind == unsupported,
        _ => false,
    }
}
