use std::collections::HashMap;
use std::net::UdpSocket;
use std::fs::File;
use core::net::IpAddr;
use core::net::Ipv4Addr;
use core::net::SocketAddr;
use core::cmp::min;
use csv;

use bincode::deserialize;
use serde::Deserialize;

mod sample_packet;
use crate::sample_packet::sample_packet;

const AZIMUTH_TO_DEGREE: f64 = 0.01;

fn read_socket() -> std::io::Result<()> {
    let ip_addr = IpAddr::V4(Ipv4Addr::new(0, 0, 0, 0));
    let socket_addr = SocketAddr::new(ip_addr, 2368);
    let socket = UdpSocket::bind(socket_addr)?;

    loop {
        let mut buf = [0u8; 1206];
        let (amount, src) = socket.recv_from(&mut buf)?;
    }

    Ok(())
}


#[derive(Deserialize, Debug)]
struct Data {
    distance: u16,
    intensity: u8,
}

type ChannelData = [Data; 16];

#[derive(Deserialize, Debug)]
struct DataBlock {
    header: u16,
    azimuth: u16,
    sequence0: ChannelData,
    sequence1: ChannelData
}

#[derive(Deserialize, Debug)]
struct RawData {
    blocks: [DataBlock; 12],
    timestamp: [u8; 4],
    factory_bytes: [u8; 2]
}

#[derive(Deserialize, Debug)]
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
    vert_offset_correction: f64
}

#[derive(Deserialize, Debug)]
struct VLP16Config {
    lasers: [LaserConfig; 16],
    num_lasers: usize,
    distance_resolution: f64
}

fn polar_to_cartesian(distance: f64, angle: f64) -> (f64, f64) {
    let x = distance * f64::cos(angle);
    let y = distance * f64::sin(angle);
    (x, y)
}

fn make_sin_cos_tables(iter: impl Iterator<Item = (usize, f64)>) -> (HashMap<usize, f64>, HashMap<usize, f64>) {
    let mut sin = HashMap::new();
    let mut cos = HashMap::new();
    for (id, angle) in iter {
        sin.insert(id, f64::sin(angle));
        cos.insert(id, f64::cos(angle));
    }
    (sin, cos)
}

fn make_rot_tables(lasers: &[LaserConfig]) -> (HashMap<usize, f64>, HashMap<usize, f64>) {
    let iter = lasers.iter().map(|laser| (laser.laser_id, laser.rot_correction));
    make_sin_cos_tables(iter)
}

fn make_vert_tables(lasers: &[LaserConfig]) -> (HashMap<usize, f64>, HashMap<usize, f64>) {
    let iter = lasers.iter().map(|laser| (laser.laser_id, laser.vert_correction));
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
    let pi = std::f64::consts::PI;
    degree * pi / 180.
}

fn calc_angle_diff(blocks: &[DataBlock], index: usize) -> f64 {
    let n = min(blocks.len() - 2, index);

    let angle0: f64 = (blocks[n+0].azimuth as f64) * AZIMUTH_TO_DEGREE;
    let angle1: f64 = (blocks[n+1].azimuth as f64) * AZIMUTH_TO_DEGREE;

    if angle0 <= angle1 {
        angle1 - angle0
    } else {
        (angle1 + 360.) - angle0
    }
}

fn calc_angles(blocks: &[DataBlock], index: usize) -> (f64, f64) {
    let angle_diff = calc_angle_diff(blocks, index) as f64;
    let angle_curr = 0.01 * (blocks[index].azimuth as f64);
    let angle_next = 0.01 * (angle_curr + 0.5 * angle_diff);

    let rotation0 = degree_to_radian(angle_curr);
    let rotation1 = degree_to_radian(angle_next);
    (rotation0, rotation1)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_read_record() -> std::io::Result<()> {
        let file = File::open("scans/velodyne1.csv")?;

        let mut reader = csv::ReaderBuilder::new()
            .has_headers(false)
            .from_reader(file);

        Ok(())
    }

    #[test]
    fn test_parse_packets() -> Result<(), Box<dyn std::error::Error>> {
        let f = std::fs::File::open("VLP16db.yaml")?;
        let config: VLP16Config = serde_yaml::from_reader(f)?;
        let (vert_sin, vert_cos) = make_vert_tables(&config.lasers);
        let (rot_sin, rot_cos) = make_rot_tables(&config.lasers);

        let data: RawData = deserialize(&sample_packet).unwrap();
        for (i, block) in data.blocks.iter().enumerate() {
            let (rotation0, rotation1) = calc_angles(&data.blocks, i);

            let cosr0 = f64::cos(rotation0);
            let sinr0 = f64::sin(rotation0);
            for (channel, s) in block.sequence0.iter().enumerate() {
                if s.distance == 0 {
                    continue;
                }
                let distance = (s.distance as f64) * config.distance_resolution;

                let sinv = *vert_sin.get(&channel).unwrap();
                let cosv = *vert_cos.get(&channel).unwrap();
                let (x, y, z) = calc_xyz(distance, sinv, cosv, sinr0, cosr0);
                println!("distance = {:3.5}", distance);
                println!("x, y, z = {}, {}, {}", x, y, z);
            }

            let cosr1 = f64::cos(rotation1);
            let sinr1 = f64::sin(rotation1);
            for (channel, s) in block.sequence1.iter().enumerate() {
                if s.distance == 0 {
                    continue;
                }
                let distance = (s.distance as f64) * config.distance_resolution;

                let sinv = *vert_sin.get(&channel).unwrap();
                let cosv = *vert_cos.get(&channel).unwrap();
                let (x, y, z) = calc_xyz(distance, sinv, cosv, sinr1, cosr1);
                println!("distance = {:3.5}", distance);
                println!("x, y, z = {}, {}, {}", x, y, z);
            }
        }

        Ok(())
    }

    #[test]
    fn test_read_vlp16_yaml() -> Result<(), Box<dyn std::error::Error>>  {
        let f = std::fs::File::open("VLP16db.yaml")?;
        let config: VLP16Config = serde_yaml::from_reader(f)?;
        make_vert_tables(&config.lasers);
        make_rot_tables(&config.lasers);

        Ok(())
    }

    #[test]
    fn test_calc_angles() {
        let data: RawData = deserialize(&sample_packet).unwrap();
        println!("angles = {:?}", calc_angles(&data.blocks, 0));
        for b in data.blocks {
            println!("b.azimuth = {:?}", b.azimuth);
        }
    }
}
