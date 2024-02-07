use core::cmp::min;
use std::collections::HashMap;

use serde::Deserialize;

mod sample_packet;

pub use crate::sample_packet::SAMPLE_PACKET;
use bincode::deserialize;

const AZIMUTH_TO_DEGREE: f64 = 0.01;
const PI: f64 = std::f64::consts::PI;

#[derive(Deserialize)]
pub struct Data {
    pub distance: u16,
    pub intensity: u8,
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
    fn process(&mut self, channel: usize, point: &Point);
}

pub struct SinCosTables {
    pub sin: HashMap<usize, f64>,
    pub cos: HashMap<usize, f64>,
}

fn make_sin_cos_tables(iter: impl Iterator<Item = (usize, f64)>) -> SinCosTables {
    let mut sin = HashMap::new();
    let mut cos = HashMap::new();
    for (id, angle) in iter {
        sin.insert(id, f64::sin(angle));
        cos.insert(id, f64::cos(angle));
    }
    SinCosTables { sin, cos }
}

pub struct RotationCalculator {
    pub radian0: f64,
    pub radian1: f64,
}

const CHANNELS_PER_SEQUENCE: usize = 16;
const FIRING_INTERVAL: f64 = 2.304; // [micro second]
const OFFSET_TIME: f64 = 55.296; // [micro second]
impl RotationCalculator {
    fn at(&self, sequence_index: usize, channel: usize) -> f64 {
        let d = self.radian1 - self.radian0;
        let offset = OFFSET_TIME * (sequence_index as f64);
        let t = FIRING_INTERVAL * (channel as f64);
        self.radian0 + d * (offset + t) / (OFFSET_TIME * 2.)
    }

    pub fn get(&self, sequence_index: usize) -> [f64; 16] {
        let mut rotations = [0.; 16];
        for c in 0..CHANNELS_PER_SEQUENCE {
            rotations[c] = self.at(sequence_index, c);
        }
        rotations
    }
}

impl SinCosTables {
    fn get(&self, channel: usize) -> (f64, f64) {
        let sin = *(self.sin).get(&channel).unwrap();
        let cos = *(self.cos).get(&channel).unwrap();
        (sin, cos)
    }

    fn project(&self, channel: usize, distance: f64) -> (f64, f64) {
        let (sin, cos) = self.get(channel);
        (distance * sin, distance * cos)
    }
}

pub fn calc_points<T: PointProcessor>(
    point_processor: &mut T,
    sin_cos_table: &SinCosTables,
    distances: &[f64],
    rotations: &[f64],
) {
    assert_eq!(distances.len(), rotations.len());
    for (channel, (distance, radian)) in distances.iter().zip(rotations.iter()).enumerate() {
        if *distance == 0. {
            continue;
        }
        let (z, xy_distance) = sin_cos_table.project(channel, *distance);
        let x = xy_distance * f64::sin(*radian);
        let y = xy_distance * f64::cos(*radian);
        point_processor.process(channel, &(x, y, z));
    }
}

fn make_rot_tables(lasers: &[LaserConfig]) -> SinCosTables {
    let iter = lasers
        .iter()
        .map(|laser| (laser.laser_id, laser.rot_correction));
    make_sin_cos_tables(iter)
}

pub fn make_vert_tables(lasers: &[LaserConfig]) -> SinCosTables {
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

pub struct DistanceCalculator {
    pub resolution: f64,
}

impl DistanceCalculator {
    pub fn new(resolution: f64) -> Self {
        DistanceCalculator { resolution }
    }

    pub fn calculate(&self, sequence: &[Data]) -> Vec<f64> {
        sequence
            .iter()
            .map(|s: &Data| (s.distance as f64) * self.resolution)
            .collect::<Vec<f64>>()
    }
}

pub fn rotation_calculator_new(blocks: &[DataBlock], index: usize) -> RotationCalculator {
    let degree_diff = calc_degree_diff(blocks, index);
    let degree0 = AZIMUTH_TO_DEGREE * (blocks[index].azimuth as f64);
    let degree1 = degree0 + degree_diff;

    let radian0 = degree_to_radian(degree0);
    let radian1 = degree_to_radian(degree1);
    RotationCalculator { radian0, radian1 }
}

pub struct PointCloudCalculator {
    distance: DistanceCalculator,
    tables: SinCosTables,
}

impl PointCloudCalculator {
    pub fn new(config: &VLP16Config) -> Self {
        let distance = DistanceCalculator::new(config.distance_resolution);
        let tables = make_vert_tables(&config.lasers);
        PointCloudCalculator { distance, tables }
    }

    pub fn calculate<T: PointProcessor>(&self, point_processor: &mut T, bytes: &[u8; 1206]) {
        let data: RawData = deserialize(bytes).unwrap();
        for (block_index, block) in data.blocks.iter().enumerate() {
            let r = rotation_calculator_new(&data.blocks, block_index);
            let rotations0 = r.get(0);
            let rotations1 = r.get(1);
            let distances0 = self.distance.calculate(&block.sequence0);
            let distances1 = self.distance.calculate(&block.sequence1);

            calc_points(point_processor, &self.tables, &distances0, &rotations0);
            calc_points(point_processor, &self.tables, &distances1, &rotations1);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sample_packet::SAMPLE_PACKET;

    #[test]
    fn test_distance_calculator() {
        let data: RawData = deserialize(&SAMPLE_PACKET).unwrap();

        // sequence[ 0].distance = 393
        // sequence[ 1].distance = 699
        // sequence[ 2].distance = 438
        // sequence[ 3].distance = 721
        // sequence[ 4].distance = 514
        // sequence[ 5].distance = 731
        // sequence[ 6].distance = 581
        // sequence[ 7].distance = 744
        // sequence[ 8].distance = 574
        // sequence[ 9].distance = 769
        // sequence[10].distance = 585
        // sequence[11].distance = 1140
        // sequence[12].distance = 678
        // sequence[13].distance = 1149
        // sequence[14].distance = 719
        // sequence[15].distance = 1166

        let d = DistanceCalculator::new(0.5);
        let result = d.calculate(&data.blocks[0].sequence0);
        let expected = [
            0.5 * 393.,
            0.5 * 699.,
            0.5 * 438.,
            0.5 * 721.,
            0.5 * 514.,
            0.5 * 731.,
            0.5 * 581.,
            0.5 * 744.,
            0.5 * 574.,
            0.5 * 769.,
            0.5 * 585.,
            0.5 * 1140.,
            0.5 * 678.,
            0.5 * 1149.,
            0.5 * 719.,
            0.5 * 1166.,
        ];

        assert_eq!(result, expected);
    }

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
            let r = rotation_calculator_new(&data.blocks, block_index);

            let firinig_time = FIRING_INTERVAL * (channel as f64);
            let t: f64 = 35695. * AZIMUTH_TO_DEGREE;
            let dt: f64 = (35733. - 35695.) * AZIMUTH_TO_DEGREE;

            let e0 = t + dt * firinig_time / (OFFSET_TIME * 2.);
            let e1 = t + dt * (OFFSET_TIME + firinig_time) / (OFFSET_TIME * 2.);

            assert!(f64::abs(r.at(0, channel) - degree_to_radian(e0)) < 1e-10);
            assert!(f64::abs(r.at(1, channel) - degree_to_radian(e1)) < 1e-10);
        }

        {
            let block_index = 7;
            let channel = 8;
            let r = rotation_calculator_new(&data.blocks, block_index);

            let firinig_time = FIRING_INTERVAL * (channel as f64);
            let t: f64 = 35971. * AZIMUTH_TO_DEGREE;
            let dt: f64 = (36000. + 11. - 35971.) * AZIMUTH_TO_DEGREE;

            let e0 = t + dt * firinig_time / (OFFSET_TIME * 2.);
            let e1 = t + dt * (OFFSET_TIME + firinig_time) / (OFFSET_TIME * 2.);

            assert!(f64::abs(r.at(0, channel) - degree_to_radian(e0)) < 1e-10);
            assert!(f64::abs(r.at(1, channel) - degree_to_radian(e1)) < 1e-10);
        }

        {
            let block_index = 11;
            let channel = 5;
            let r = rotation_calculator_new(&data.blocks, block_index);

            let firinig_time = FIRING_INTERVAL * (channel as f64);
            let t: f64 = 129. * AZIMUTH_TO_DEGREE;
            let dt: f64 = (129. - 90.) * AZIMUTH_TO_DEGREE;

            let e0 = t + dt * firinig_time / (OFFSET_TIME * 2.);
            let e1 = t + dt * (OFFSET_TIME + firinig_time) / (OFFSET_TIME * 2.);

            assert!(f64::abs(r.at(0, channel) - degree_to_radian(e0)) < 1e-10);
            assert!(f64::abs(r.at(1, channel) - degree_to_radian(e1)) < 1e-10);
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

    struct PointAccumulator {
        pub points: [Option<Point>; CHANNELS_PER_SEQUENCE],
    }

    impl PointAccumulator {
        fn new() -> Self {
            PointAccumulator {
                points: [None; CHANNELS_PER_SEQUENCE],
            }
        }
    }

    impl PointProcessor for PointAccumulator {
        fn process(&mut self, channel: usize, p: &Point) {
            self.points[channel] = Some(*p);
        }
    }

    #[test]
    fn test_calc_points() -> Result<(), Box<dyn std::error::Error>> {
        let f = std::fs::File::open("VLP16db.yaml")?;
        let config: VLP16Config = serde_yaml::from_reader(f)?;

        let sin_cos_table = make_vert_tables(&config.lasers);

        let mut point_accumulator = PointAccumulator::new();

        let rotations = [
            0.10, 0.32, 0.54, 0.76, 0.98, 1.20, 1.42, 1.64, 1.86, 2.08, 2.30, 2.52, 2.74, 2.96,
            3.18, 3.40,
        ];

        let distances = [
            0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.0, // distance 0.0 will be skipped
            1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7,
        ];

        calc_points(
            &mut point_accumulator,
            &sin_cos_table,
            &distances,
            &rotations,
        );

        for (c, point) in point_accumulator.points.iter().enumerate() {
            let Some((x, y, z)) = point else {
                assert_eq!(c, 7); // only seventh element should be skipped
                continue;
            };
            assert_ne!(c, 7); // only seventh element should be skipped

            let distance = distances[c];
            let (sinv, cosv) = sin_cos_table.get(c);
            let x_expected = distance * cosv * f64::sin(rotations[c]);
            let y_expected = distance * cosv * f64::cos(rotations[c]);
            let z_expected = distance * sinv;

            assert!(f64::abs(x - x_expected) < 1e-10);
            assert!(f64::abs(y - y_expected) < 1e-10);
            assert!(f64::abs(z - z_expected) < 1e-10);
        }

        Ok(())
    }
}
