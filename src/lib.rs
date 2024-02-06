use core::cmp::min;
use std::collections::HashMap;

use serde::Deserialize;

mod sample_packet;

pub use crate::sample_packet::SAMPLE_PACKET;

const AZIMUTH_TO_DEGREE: f64 = 0.01;
const PI: f64 = std::f64::consts::PI;

#[derive(Deserialize)]
pub struct Data {
    distance: u16,
    intensity: u8,
}

type ChannelData = [Data; 16];

#[derive(Deserialize)]
pub struct DataBlock {
    pub header: u16,
    pub azimuth: u16,
    pub sequence0: ChannelData,
    pub sequence1: ChannelData,
}

#[derive(Deserialize)]
pub struct RawData {
    pub blocks: [DataBlock; 12],
    pub timestamp: [u8; 4],
    pub factory_bytes: [u8; 2],
}

#[derive(Deserialize)]
pub struct LaserConfig {
    pub dist_correction: f64,
    pub dist_correction_x: f64,
    pub dist_correction_y: f64,
    pub focal_distance: f64,
    pub focal_slope: f64,
    pub horiz_offset_correction: f64,
    pub laser_id: usize,
    pub rot_correction: f64,
    pub vert_correction: f64,
    pub vert_offset_correction: f64,
}

#[derive(Deserialize)]
pub struct VLP16Config {
    pub lasers: [LaserConfig; 16],
    pub num_lasers: usize,
    pub distance_resolution: f64,
}

pub type Point = (f64, f64, f64);

pub trait PointProcessor {
    fn process(&mut self, point: &Point);
}

pub struct SinCosTables {
    pub vert_sin: HashMap<usize, f64>,
    pub vert_cos: HashMap<usize, f64>,
    pub distance_resolution: f64,
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

fn calc_point(distance: f64, sinv: f64, cosv: f64, sinr: f64, cosr: f64) -> Point {
    let z = distance * sinv;
    let k = distance * cosv;
    let x = k * sinr;
    let y = k * cosr;
    (x, y, z)
}

pub struct RotationCalculator {
    pub radian0: f64,
    pub radian1: f64,
    pub sequence_index: usize,
}

const FIRING_INTERVAL: f64 = 2.304; // [micro second]
const OFFSET_TIME: f64 = 55.296; // [micro second]
impl RotationCalculator {
    fn at(&self, channel: usize) -> f64 {
        let d = self.radian1 - self.radian0;
        let offset = OFFSET_TIME * (self.sequence_index as f64);
        let t = FIRING_INTERVAL * (channel as f64);
        self.radian0 + d * (offset + t) / (OFFSET_TIME * 2.)
    }
}

impl SinCosTables {
    pub fn calc_points<T: PointProcessor>(
        &self,
        point_processor: &mut T,
        rotation: &RotationCalculator,
        sequence: &[Data],
    ) {
        for (channel, s) in sequence.iter().enumerate() {
            let r = rotation.at(channel);
            let cosr = f64::cos(r);
            let sinr = f64::sin(r);
            if s.distance == 0 {
                continue;
            }
            let distance = (s.distance as f64) * self.distance_resolution;

            let sinv = *(self.vert_sin).get(&channel).unwrap();
            let cosv = *(self.vert_cos).get(&channel).unwrap();
            let point = calc_point(distance, sinv, cosv, sinr, cosr);
            point_processor.process(&point);
        }
    }
}

fn make_rot_tables(lasers: &[LaserConfig]) -> (HashMap<usize, f64>, HashMap<usize, f64>) {
    let iter = lasers
        .iter()
        .map(|laser| (laser.laser_id, laser.rot_correction));
    make_sin_cos_tables(iter)
}

pub fn make_vert_tables(lasers: &[LaserConfig]) -> (HashMap<usize, f64>, HashMap<usize, f64>) {
    let iter = lasers
        .iter()
        .map(|laser| (laser.laser_id, laser.vert_correction));
    make_sin_cos_tables(iter)
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

pub fn calc_angles(blocks: &[DataBlock], index: usize) -> (f64, f64) {
    let degree_diff = calc_degree_diff(blocks, index);
    let degree0 = AZIMUTH_TO_DEGREE * (blocks[index].azimuth as f64);
    let degree1 = degree0 + degree_diff;

    let radian0 = degree_to_radian(degree0);
    let radian1 = degree_to_radian(degree1);
    (radian0, radian1)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sample_packet::SAMPLE_PACKET;
    use bincode::deserialize;

    #[test]
    fn test_rotation_calculator() {
        let data: RawData = deserialize(&SAMPLE_PACKET).unwrap();

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

        {
            let block_index = 0;
            let channel = 4;
            let (radian0, radian1) = calc_angles(&data.blocks, block_index);

            let r0 = RotationCalculator {
                radian0,
                radian1,
                sequence_index: 0,
            };
            let r1 = RotationCalculator {
                radian0,
                radian1,
                sequence_index: 1,
            };

            let firinig_time = FIRING_INTERVAL * (channel as f64);
            let t: f64 = 35695. * AZIMUTH_TO_DEGREE;
            let dt: f64 = (35733. - 35695.) * AZIMUTH_TO_DEGREE;

            let e0 = t + dt * firinig_time / (OFFSET_TIME * 2.);
            let e1 = t + dt * (OFFSET_TIME + firinig_time) / (OFFSET_TIME * 2.);

            assert!(f64::abs(r0.at(channel) - degree_to_radian(e0)) < 1e-10);
            assert!(f64::abs(r1.at(channel) - degree_to_radian(e1)) < 1e-10);
        }

        {
            let block_index = 7;
            let channel = 8;
            let (radian0, radian1) = calc_angles(&data.blocks, block_index);

            let r0 = RotationCalculator {
                radian0,
                radian1,
                sequence_index: 0,
            };
            let r1 = RotationCalculator {
                radian0,
                radian1,
                sequence_index: 1,
            };

            let firinig_time = FIRING_INTERVAL * (channel as f64);
            let t: f64 = 35971. * AZIMUTH_TO_DEGREE;
            let dt: f64 = (36000. + 11. - 35971.) * AZIMUTH_TO_DEGREE;

            let e0 = t + dt * firinig_time / (OFFSET_TIME * 2.);
            let e1 = t + dt * (OFFSET_TIME + firinig_time) / (OFFSET_TIME * 2.);

            assert!(f64::abs(r0.at(channel) - degree_to_radian(e0)) < 1e-10);
            assert!(f64::abs(r1.at(channel) - degree_to_radian(e1)) < 1e-10);
        }

        {
            let block_index = 11;
            let channel = 5;
            let (radian0, radian1) = calc_angles(&data.blocks, block_index);

            let r0 = RotationCalculator {
                radian0,
                radian1,
                sequence_index: 0,
            };
            let r1 = RotationCalculator {
                radian0,
                radian1,
                sequence_index: 1,
            };

            let firinig_time = FIRING_INTERVAL * (channel as f64);
            let t: f64 = 129. * AZIMUTH_TO_DEGREE;
            let dt: f64 = (129. - 90.) * AZIMUTH_TO_DEGREE;

            let e0 = t + dt * firinig_time / (OFFSET_TIME * 2.);
            let e1 = t + dt * (OFFSET_TIME + firinig_time) / (OFFSET_TIME * 2.);

            assert!(f64::abs(r0.at(channel) - degree_to_radian(e0)) < 1e-10);
            assert!(f64::abs(r1.at(channel) - degree_to_radian(e1)) < 1e-10);
        }
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
        let data: RawData = deserialize(&SAMPLE_PACKET).unwrap();

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
        let expected1 = PI * 0.01 * 35733. / 180.0;
        assert!(f64::abs(rotation0 - expected0) < 1e-10);
        assert!(f64::abs(rotation1 - expected1) < 1e-10);

        let (rotation0, rotation1) = calc_angles(&data.blocks, 7);
        let expected0 = PI * 0.01 * 35971. / 180.;
        let expected1 = PI * 0.01 * (35971. + 11. + (36000. - 35971.)) / 180.0;
        assert!(f64::abs(rotation0 - expected0) < 1e-10);
        assert!(f64::abs(rotation1 - expected1) < 1e-10);

        let (rotation0, rotation1) = calc_angles(&data.blocks, 11);
        let expected0 = PI * 0.01 * 129. / 180.;
        let expected1 = PI * 0.01 * (129. + (129. - 90.)) / 180.0;
        assert!(f64::abs(rotation0 - expected0) < 1e-10);
        assert!(f64::abs(rotation1 - expected1) < 1e-10);
    }
}
