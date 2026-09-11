//! The brick bundle (`openbricks-brick-bundle/1`): exact LDraw meshes,
//! mass properties for a uniform solid, and connection features, as
//! written by `openbricks_sim.bricks.ldraw` and shipped in the wheel.

use base64::Engine;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::io::Read;

#[derive(Deserialize, Serialize, Clone, Debug)]
pub struct Bundle {
    #[serde(default)]
    pub format: String,
    #[serde(default)]
    pub source: String,
    #[serde(default)]
    pub parts: BTreeMap<String, PartRecord>,
    #[serde(default)]
    pub missing: Vec<String>,
}

#[derive(Deserialize, Serialize, Clone, Debug)]
pub struct PartRecord {
    pub name: String,
    #[serde(default)]
    pub ldraw: String,
    pub mesh: MeshRecord,
    pub bbox: [[f64; 3]; 2],
    #[serde(default)]
    pub volume_mm3: f64,
    pub com: [f64; 3],
    pub inertia_per_g: [[f64; 3]; 3],
    #[serde(default)]
    pub mass_model: String,
    #[serde(default)]
    pub connectors: Vec<Connector>,
    #[serde(default = "one")]
    pub mass_g: f64,
    #[serde(default)]
    pub source: String,
    #[serde(default)]
    pub source_note: String,
    #[serde(default)]
    pub density_g_cm3: Option<f64>,
}

fn one() -> f64 {
    1.0
}

#[derive(Deserialize, Serialize, Clone, Debug)]
pub struct MeshRecord {
    pub verts: usize,
    pub tris: usize,
    pub pos: String,
    pub nrm: String,
    pub idx: String,
    #[serde(default)]
    pub idx32: bool,
}

#[derive(Deserialize, Serialize, Clone, Debug, PartialEq)]
pub struct Connector {
    pub kind: String,
    pub centre: [f64; 3],
    pub axis: [f64; 3],
    pub length: f64,
    #[serde(default)]
    pub r: f64,
}

/// A mesh ready for the GPU: positions in mm, unit normals, triangles.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct MeshData {
    pub positions: Vec<[f32; 3]>,
    pub normals: Vec<[f32; 3]>,
    pub indices: Vec<u32>,
}

impl MeshRecord {
    pub fn decode(&self) -> Result<MeshData, String> {
        let b64 = base64::engine::general_purpose::STANDARD;
        let pos = b64.decode(&self.pos).map_err(|e| format!("mesh positions: {e}"))?;
        let nrm = b64.decode(&self.nrm).map_err(|e| format!("mesh normals: {e}"))?;
        let idx = b64.decode(&self.idx).map_err(|e| format!("mesh indices: {e}"))?;
        if pos.len() % 6 != 0 || nrm.len() % 3 != 0 || pos.len() / 6 != nrm.len() / 3 {
            return Err("mesh positions and normals disagree".into());
        }
        let positions: Vec<[f32; 3]> = pos
            .chunks_exact(6)
            .map(|c| {
                [
                    i16::from_le_bytes([c[0], c[1]]) as f32 / 100.0,
                    i16::from_le_bytes([c[2], c[3]]) as f32 / 100.0,
                    i16::from_le_bytes([c[4], c[5]]) as f32 / 100.0,
                ]
            })
            .collect();
        let normals: Vec<[f32; 3]> = nrm
            .chunks_exact(3)
            .map(|c| [c[0] as i8 as f32 / 127.0, c[1] as i8 as f32 / 127.0, c[2] as i8 as f32 / 127.0])
            .collect();
        let indices: Vec<u32> = if self.idx32 {
            idx.chunks_exact(4).map(|c| u32::from_le_bytes([c[0], c[1], c[2], c[3]])).collect()
        } else {
            idx.chunks_exact(2).map(|c| u16::from_le_bytes([c[0], c[1]]) as u32).collect()
        };
        if !indices.len().is_multiple_of(3) || indices.iter().any(|&i| i as usize >= positions.len()) {
            return Err("mesh indices out of range".into());
        }
        Ok(MeshData {
            positions,
            normals,
            indices,
        })
    }
}

/// Load a bundle from `.json.zlib` (the wheel's file) or plain `.json`
/// (what `openbricks bricks convert` writes).
pub fn load_bundle(path: &std::path::Path) -> Result<Bundle, String> {
    let bytes = std::fs::read(path).map_err(|e| format!("{}: {e}", path.display()))?;
    parse_bundle(&bytes, path.to_string_lossy().ends_with(".zlib"))
}

pub fn parse_bundle(bytes: &[u8], compressed: bool) -> Result<Bundle, String> {
    let text = if compressed {
        let mut out = String::new();
        flate2::read::ZlibDecoder::new(bytes)
            .read_to_string(&mut out)
            .map_err(|e| format!("not a zlib bundle: {e}"))?;
        out
    } else {
        String::from_utf8(bytes.to_vec()).map_err(|e| format!("bundle is not UTF-8: {e}"))?
    };
    let bundle: Bundle = serde_json::from_str(&text).map_err(|e| format!("bundle JSON: {e}"))?;
    Ok(bundle)
}

impl Bundle {
    /// Later bundles win on the same part number.
    pub fn merge(&mut self, other: Bundle) {
        self.parts.extend(other.parts);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;

    fn record(idx32: bool) -> MeshRecord {
        // one triangle: (0,0,0) (10,0,0) (0,10,0) mm, normal +z
        let b64 = base64::engine::general_purpose::STANDARD;
        let mut pos = Vec::new();
        for v in [[0i16, 0, 0], [1000, 0, 0], [0, 1000, 0]] {
            for c in v {
                pos.extend_from_slice(&c.to_le_bytes());
            }
        }
        let nrm = [0i8, 0, 127, 0, 0, 127, 0, 0, 127].map(|v| v as u8);
        let idx: Vec<u8> = if idx32 {
            vec![0, 0, 0, 0, 1, 0, 0, 0, 2, 0, 0, 0]
        } else {
            vec![0, 0, 1, 0, 2, 0]
        };
        MeshRecord {
            verts: 3,
            tris: 1,
            pos: b64.encode(pos),
            nrm: b64.encode(nrm),
            idx: b64.encode(idx),
            idx32,
        }
    }

    #[test]
    fn decodes_16_and_32_bit_indices() {
        for idx32 in [false, true] {
            let m = record(idx32).decode().unwrap();
            assert_eq!(m.positions[1], [10.0, 0.0, 0.0]);
            assert_eq!(m.normals[2], [0.0, 0.0, 1.0]);
            assert_eq!(m.indices, vec![0, 1, 2]);
        }
    }

    #[test]
    fn rejects_out_of_range_indices() {
        let mut r = record(false);
        r.idx = base64::engine::general_purpose::STANDARD.encode([0u8, 0, 1, 0, 9, 0]);
        assert!(r.decode().is_err());
    }

    #[test]
    fn loads_compressed_and_plain_bundles() {
        let json = r#"{"format":"openbricks-brick-bundle/1","parts":{"1":{"name":"One","mesh":{"verts":0,"tris":0,"pos":"","nrm":"","idx":""},"bbox":[[0,0,0],[1,1,1]],"com":[0,0,0],"inertia_per_g":[[1,0,0],[0,1,0],[0,0,1]],"mass_g":2.5}}}"#;
        let plain = parse_bundle(json.as_bytes(), false).unwrap();
        assert_eq!(plain.parts["1"].mass_g, 2.5);
        let mut enc = flate2::write::ZlibEncoder::new(Vec::new(), flate2::Compression::fast());
        enc.write_all(json.as_bytes()).unwrap();
        let zipped = parse_bundle(&enc.finish().unwrap(), true).unwrap();
        assert_eq!(zipped.parts["1"].name, "One");
        let dir = std::env::temp_dir().join(format!("ob-bundle-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("b.json");
        std::fs::write(&path, json).unwrap();
        assert_eq!(load_bundle(&path).unwrap().parts.len(), 1);
        assert!(load_bundle(&dir.join("missing.json")).is_err());
        let mut a = plain.clone();
        let mut other = plain.clone();
        other.parts.get_mut("1").unwrap().mass_g = 9.0;
        a.merge(other);
        assert_eq!(a.parts["1"].mass_g, 9.0);
    }
}
