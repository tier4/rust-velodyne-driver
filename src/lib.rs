use core::cmp::min;
use std::collections::HashMap;

use serde::Deserialize;

mod sample_packet;

const AZIMUTH_TO_DEGREE: f64 = 0.01;
const PI: f64 = std::f64::consts::PI;

#[derive(Deserialize)]
struct Data {
    distance: u16,
    intensity: u8,
}

type ChannelData = [Data; 16];

#[derive(Deserialize)]
struct DataBlock {
    header: u16,
    azimuth: u16,
    sequence0: ChannelData,
    sequence1: ChannelData,
}

#[derive(Deserialize)]
struct RawData {
    blocks: [DataBlock; 12],
    timestamp: [u8; 4],
    factory_bytes: [u8; 2],
}

#[derive(Deserialize)]
struct LaserConfig {
    dist_correction: f64,
    dist_correction_x: f64,
    dist_correction_y: f64,
    focal_distance: f64,
    focal_slope: f64,
    horiz_offset_correction: f64,
    laser_id: usize,
    rot_correction: f64,
    vert_correction: f64,
    vert_offset_correction: f64,
}

#[derive(Deserialize)]
struct VLP16Config {
    lasers: [LaserConfig; 16],
    num_lasers: usize,
    distance_resolution: f64,
}

struct SinCosTables {
    vert_sin: HashMap<usize, f64>,
    vert_cos: HashMap<usize, f64>,
    distance_resolution: f64,
}

fn make_sin_cos_tables(
    iter: impl Iterator<Item = (usize, f64)>,
) -> (HashMap<usize, f64>, HashMap<usize, f64>) {
    let mut sin = HashMap::new();
    let mut cos = HashMap::new();
    for (id, angle) in iter {
        sin.insert(id, f64::sin(angle));
        cos.insert(id, f64::cos(angle));
    }
    (sin, cos)
}

impl SinCosTables {
    fn calc_points(&self, sequence: &[Data], rotation: f64) {
        let cosr = f64::cos(rotation);
        let sinr = f64::sin(rotation);
        for (channel, s) in sequence.iter().enumerate() {
            if s.distance == 0 {
                continue;
            }
            let distance = (s.distance as f64) * self.distance_resolution;

            let sinv = *(self.vert_sin).get(&channel).unwrap();
            let cosv = *(self.vert_cos).get(&channel).unwrap();
            let (x, y, z) = calc_xyz(distance, sinv, cosv, sinr, cosr);
        }
    }
}

fn make_rot_tables(lasers: &[LaserConfig]) -> (HashMap<usize, f64>, HashMap<usize, f64>) {
    let iter = lasers
        .iter()
        .map(|laser| (laser.laser_id, laser.rot_correction));
    make_sin_cos_tables(iter)
}

fn make_vert_tables(lasers: &[LaserConfig]) -> (HashMap<usize, f64>, HashMap<usize, f64>) {
    let iter = lasers
        .iter()
        .map(|laser| (laser.laser_id, laser.vert_correction));
    make_sin_cos_tables(iter)
}

fn calc_xyz(distance: f64, sinv: f64, cosv: f64, sinr: f64, cosr: f64) -> (f64, f64, f64) {
    let z = distance * sinv;
    let k = distance * cosv;
    let x = k * sinr;
    let y = k * cosr;
    (x, y, z)
}

fn degree_to_radian(degree: f64) -> f64 {
    degree * PI / 180.
}

fn calc_degree_diff(blocks: &[DataBlock], index: usize) -> f64 {
    let n = min(blocks.len() - 2, index);

    let degree0: f64 = (blocks[n + 0].azimuth as f64) * AZIMUTH_TO_DEGREE;
    let degree1: f64 = (blocks[n + 1].azimuth as f64) * AZIMUTH_TO_DEGREE;

    if degree0 <= degree1 {
        degree1 - degree0
    } else {
        (degree1 + 360.) - degree0
    }
}

fn calc_angles(blocks: &[DataBlock], index: usize) -> (f64, f64) {
    let degree_diff = calc_degree_diff(blocks, index);
    let degree_curr = AZIMUTH_TO_DEGREE * (blocks[index].azimuth as f64);
    let degree_next = degree_curr + 0.5 * degree_diff;

    let rotation0 = degree_to_radian(degree_curr);
    let rotation1 = degree_to_radian(degree_next);
    (rotation0, rotation1)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sample_packet::sample_packet;
    use bincode::deserialize;

    #[test]
    fn test_parse_packets() -> Result<(), Box<dyn std::error::Error>> {
        let f = std::fs::File::open("VLP16db.yaml")?;
        let config: VLP16Config = serde_yaml::from_reader(f)?;
        let (vert_sin, vert_cos) = make_vert_tables(&config.lasers);
        // let (rot_sin, rot_cos) = make_rot_tables(&config.lasers);

        let tables = SinCosTables {
            vert_sin: vert_sin,
            vert_cos: vert_cos,
            distance_resolution: config.distance_resolution,
        };

        let data: RawData = deserialize(&sample_packet).unwrap();
        for (i, block) in data.blocks.iter().enumerate() {
            let (rotation0, rotation1) = calc_angles(&data.blocks, i);
            tables.calc_points(&block.sequence0, rotation0);
            tables.calc_points(&block.sequence1, rotation1);
        }

        Ok(())
    }

    #[test]
    fn test_read_vlp16_yaml() -> Result<(), Box<dyn std::error::Error>> {
        let f = std::fs::File::open("VLP16db.yaml")?;
        let config: VLP16Config = serde_yaml::from_reader(f)?;
        make_vert_tables(&config.lasers);
        make_rot_tables(&config.lasers);

        Ok(())
    }

    #[test]
    fn test_calc_angles() {
        let data: RawData = deserialize(&sample_packet).unwrap();

        // blocks[ 0].azimuth = 35695
        // blocks[ 1].azimuth = 35733
        // blocks[ 2].azimuth = 35773
        // blocks[ 3].azimuth = 35813
        // blocks[ 4].azimuth = 35853
        // blocks[ 5].azimuth = 35891
        // blocks[ 6].azimuth = 35931
        // blocks[ 7].azimuth = 35971
        // blocks[ 8].azimuth = 11
        // blocks[ 9].azimuth = 49
        // blocks[10].azimuth = 90
        // blocks[11].azimuth = 129

        let (rotation0, rotation1) = calc_angles(&data.blocks, 0);
        let expected0 = PI * 0.01 * 35695. / 180.;
        let expected1 = PI * 0.01 * (35695. + 0.5 * (35733. - 35695.)) / 180.0;
        assert!(f64::abs(rotation0 - expected0) < 1e-10);
        assert!(f64::abs(rotation1 - expected1) < 1e-10);

        let (rotation0, rotation1) = calc_angles(&data.blocks, 7);
        let expected0 = PI * 0.01 * 35971. / 180.;
        let expected1 = PI * 0.01 * (35971. + 0.5 * (11. + 36000. - 35971.)) / 180.0;
        assert!(f64::abs(rotation0 - expected0) < 1e-10);
        assert!(f64::abs(rotation1 - expected1) < 1e-10);

        let (rotation0, rotation1) = calc_angles(&data.blocks, 11);
        let expected0 = PI * 0.01 * 129. / 180.;
        let expected1 = PI * 0.01 * (129. + 0.5 * (129. - 90.)) / 180.0;
        assert!(f64::abs(rotation0 - expected0) < 1e-10);
        assert!(f64::abs(rotation1 - expected1) < 1e-10);
    }
}
