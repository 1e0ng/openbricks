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
    /// The sets the bundle holds every part of, by set id.
    #[serde(default)]
    pub sets: BTreeMap<String, SetInfo>,
    /// The LEGO colours the parts come in, by LDraw colour id: what a
    /// brick's `color` draws with.
    #[serde(default)]
    pub colors: BTreeMap<u32, ColorInfo>,
}

/// A LEGO colour, as Rebrickable's tables name and paint it.
#[derive(Deserialize, Serialize, Clone, Debug, Default, PartialEq)]
pub struct ColorInfo {
    #[serde(default)]
    pub name: String,
    /// Six hex digits of sRGB.
    #[serde(default)]
    pub rgb: String,
    #[serde(default)]
    pub trans: bool,
}

impl ColorInfo {
    pub fn rgb_u8(&self) -> Option<[u8; 3]> {
        if self.rgb.len() != 6 {
            return None;
        }
        let h = u32::from_str_radix(&self.rgb, 16).ok()?;
        Some([((h >> 16) & 0xFF) as u8, ((h >> 8) & 0xFF) as u8, (h & 0xFF) as u8])
    }
}

/// A LEGO set the bundle holds complete.
#[derive(Deserialize, Serialize, Clone, Debug, Default, PartialEq)]
pub struct SetInfo {
    #[serde(default)]
    pub name: String,
    #[serde(default)]
    pub year: u32,
    #[serde(default)]
    pub pieces: u32,
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
    /// How many of it each set holds, by set id.
    #[serde(default)]
    pub sets: BTreeMap<String, u32>,
    /// The numbers it goes by in set inventories where LDraw names it differently.
    #[serde(default)]
    pub aliases: Vec<String>,
    /// The colours it comes in (LDraw colour ids), each with the LEGO
    /// element numbers that name the part in that colour.
    #[serde(default)]
    pub colors: BTreeMap<u32, Vec<String>>,
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
    /// Millimetres per position step; 0.01 (bricks) when absent.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub scale: Option<f64>,
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
    /// Texture coordinates, one per position, when the mesh is textured.
    pub uvs: Vec<[f32; 2]>,
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
        let step = self.scale.unwrap_or(0.01) as f32;
        let positions: Vec<[f32; 3]> = pos
            .as_chunks::<6>()
            .0
            .iter()
            .map(|c| {
                [
                    i16::from_le_bytes([c[0], c[1]]) as f32 * step,
                    i16::from_le_bytes([c[2], c[3]]) as f32 * step,
                    i16::from_le_bytes([c[4], c[5]]) as f32 * step,
                ]
            })
            .collect();
        let normals: Vec<[f32; 3]> = nrm
            .as_chunks::<3>()
            .0
            .iter()
            .map(|c| [c[0] as i8 as f32 / 127.0, c[1] as i8 as f32 / 127.0, c[2] as i8 as f32 / 127.0])
            .collect();
        let indices: Vec<u32> = if self.idx32 {
            idx.as_chunks::<4>()
                .0
                .iter()
                .map(|c| u32::from_le_bytes([c[0], c[1], c[2], c[3]]))
                .collect()
        } else {
            idx.as_chunks::<2>()
                .0
                .iter()
                .map(|c| u16::from_le_bytes([c[0], c[1]]) as u32)
                .collect()
        };
        if !indices.len().is_multiple_of(3) || indices.iter().any(|&i| i as usize >= positions.len()) {
            return Err("mesh indices out of range".into());
        }
        Ok(MeshData {
            positions,
            normals,
            indices,
            uvs: Vec::new(),
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
    /// Later bundles win on the same part number, and bring their sets
    /// and colours.
    pub fn merge(&mut self, other: Bundle) {
        self.parts.extend(other.parts);
        self.sets.extend(other.sets);
        self.colors.extend(other.colors);
    }

    /// What a colour id draws as, when the palette has it.
    pub fn color_rgb(&self, id: u32) -> Option<[u8; 3]> {
        self.colors.get(&id).and_then(ColorInfo::rgb_u8)
    }

    /// A colour as a picker lists it: id, name and swatch.
    pub fn color_choice(&self, id: u32) -> Option<(u32, String, [u8; 3])> {
        let c = self.colors.get(&id)?;
        Some((id, c.name.clone(), c.rgb_u8()?))
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
            scale: None,
        }
    }

    #[test]
    fn the_position_step_travels_with_the_record() {
        use base64::Engine;
        let b64 = base64::engine::general_purpose::STANDARD;
        let pos = b64.encode(
            [100i16, 0, 0, 0, 100, 0, 0, 0, 100]
                .iter()
                .flat_map(|v| v.to_le_bytes())
                .collect::<Vec<u8>>(),
        );
        let nrm = b64.encode([0i8, 0, 127, 0, 0, 127, 0, 0, 127].iter().map(|v| *v as u8).collect::<Vec<u8>>());
        let idx = b64.encode([0u16, 1, 2].iter().flat_map(|v| v.to_le_bytes()).collect::<Vec<u8>>());
        let mut rec = MeshRecord {
            verts: 3,
            tris: 1,
            pos,
            nrm,
            idx,
            idx32: false,
            scale: None,
        };
        assert_eq!(rec.decode().unwrap().positions[0], [1.0, 0.0, 0.0], "0.01 mm steps by default");
        rec.scale = Some(0.1);
        assert_eq!(rec.decode().unwrap().positions[1], [0.0, 10.0, 0.0], "coarser scenery steps");
        assert!(
            !serde_json::to_string(&MeshRecord {
                scale: None,
                ..rec.clone()
            })
            .unwrap()
            .contains("scale")
        );
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
        // without sets or aliases a record has none; with them, they read and merge
        assert!(plain.sets.is_empty() && plain.parts["1"].sets.is_empty() && plain.parts["1"].aliases.is_empty());
        let with_sets = json.replace(
            r#""mass_g":2.5}}"#,
            r#""mass_g":2.5,"sets":{"45811":4},"aliases":["41250"]}},"sets":{"45811":{"name":"WRO Brick Set","year":2016,"pieces":724}}"#,
        );
        let b = parse_bundle(with_sets.as_bytes(), false).unwrap();
        assert_eq!(b.parts["1"].sets["45811"], 4);
        assert_eq!(b.parts["1"].aliases, vec!["41250".to_string()]);
        assert_eq!(
            b.sets["45811"],
            SetInfo {
                name: "WRO Brick Set".into(),
                year: 2016,
                pieces: 724
            }
        );
        a.merge(b);
        assert_eq!(a.sets.len(), 1);
        assert_eq!(a.parts["1"].sets["45811"], 4);
        // colours: the palette by LDraw id, a part's colours with their element numbers
        let with_colors = json.replace(
            r#""mass_g":2.5}}"#,
            r#""mass_g":2.5,"colors":{"72":["4210687","32278199"],"4":["4163147"]}}},"colors":{"72":{"name":"Dark Bluish Gray","rgb":"6C6E68","trans":false},"4":{"name":"Red","rgb":"C91A09"},"41":{"name":"Trans-Light Blue","rgb":"AEEFEC","trans":true}}"#,
        );
        let c = parse_bundle(with_colors.as_bytes(), false).unwrap();
        assert_eq!(c.parts["1"].colors[&72], vec!["4210687".to_string(), "32278199".to_string()]);
        assert_eq!(c.color_rgb(72), Some([0x6C, 0x6E, 0x68]));
        assert_eq!(c.color_choice(4), Some((4, "Red".to_string(), [0xC9, 0x1A, 0x09])));
        assert!(c.colors[&41].trans && !c.colors[&4].trans);
        assert_eq!(c.color_rgb(9999), None, "not in the palette");
        assert_eq!(
            ColorInfo {
                rgb: "xyz".into(),
                ..Default::default()
            }
            .rgb_u8(),
            None
        );
        assert!(plain.parts["1"].colors.is_empty() && plain.colors.is_empty());
        a.merge(c);
        assert_eq!(a.colors.len(), 3);
    }
}
