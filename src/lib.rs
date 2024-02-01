use std::collections::HashMap;
use std::net::UdpSocket;
use std::fs::File;
use core::net::IpAddr;
use core::net::Ipv4Addr;
use core::net::SocketAddr;
use csv;

use bincode::deserialize;
use serde::Deserialize;

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
    laser_id: u32,
    rot_correction: f64,
    vert_correction: f64,
    vert_offset_correction: f64
}

#[derive(Deserialize, Debug)]
struct VLP16Config {
    lasers: [LaserConfig; 16],
    num_lasers: u32,
    distance_resolution: f64
}

fn make_sin_cos_tables(iter: impl Iterator<Item = (u32, f64)>) -> (HashMap<u32, f64>, HashMap<u32, f64>) {
    let mut sin = HashMap::new();
    let mut cos = HashMap::new();
    for (id, angle) in iter {
        sin.insert(id, f64::sin(angle));
        cos.insert(id, f64::cos(angle));
    }
    (sin, cos)
}

fn make_rot_tables(lasers: &[LaserConfig]) -> (HashMap<u32, f64>, HashMap<u32, f64>) {
    let iter = lasers.iter().map(|laser| (laser.laser_id, laser.rot_correction));
    make_sin_cos_tables(iter)
}

fn make_vert_tables(lasers: &[LaserConfig]) -> (HashMap<u32, f64>, HashMap<u32, f64>) {
    let iter = lasers.iter().map(|laser| (laser.laser_id, laser.vert_correction));
    make_sin_cos_tables(iter)
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

        // for record in reader.deserialize() {
        //     let vector: RawData = record?;
        //     // println!("vector = {:3?}", &vector[..20]);
        // }

        Ok(())
    }

    #[test]
    fn test_zero_copy_struct_new() {
        let data = [
            255, 238, 250, 28, 149, 1, 54, 64, 4, 5, 202, 1, 26, 76, 4, 7, 4, 2, 11, 70,
            4, 7, 86, 2, 9, 85, 4, 7, 191, 2, 7, 79, 4, 7, 113, 3, 4, 76, 4, 6,
            106, 4, 20, 89, 4, 6, 67, 4, 7, 87, 4, 6, 150, 1, 52, 58, 4, 5, 200, 1,
            26, 78, 4, 6, 6, 2, 13, 72, 4, 8, 84, 2, 9, 81, 4, 7, 193, 2, 7, 77,
            4, 7, 123, 3, 4, 72, 4, 6, 101, 4, 20, 93, 4, 6, 63, 4, 7, 90, 4, 6,
            255, 238, 34, 29, 157, 1, 49, 59, 4, 5, 198, 1, 27, 70, 4, 7, 8, 2, 15, 70,
            4, 8, 84, 2, 9, 76, 4, 8, 191, 2, 9, 79, 4, 7, 119, 3, 4, 68, 4, 5,
            98, 4, 18, 81, 4, 6, 65, 4, 6, 94, 4, 6, 157, 1, 49, 63, 4, 5, 198, 1,
            27, 66, 4, 6, 8, 2, 12, 74, 4, 8, 88, 2, 9, 77, 4, 7, 187, 2, 10, 77,
            4, 6, 129, 3, 5, 72, 4, 6, 97, 4, 17, 84, 4, 6, 63, 4, 6, 102, 4, 6,
            255, 238, 74, 29, 157, 1, 49, 59, 4, 5, 204, 1, 25, 70, 4, 6, 3, 2, 11, 71,
            4, 6, 90, 2, 9, 79, 4, 7, 195, 2, 9, 73, 4, 7, 129, 3, 4, 76, 4, 7,
            100, 4, 13, 87, 4, 6, 61, 4, 6, 92, 4, 6, 153, 1, 51, 58, 4, 5, 204, 1,
            25, 70, 4, 5, 4, 2, 11, 66, 4, 7, 90, 2, 8, 81, 4, 7, 192, 2, 7, 73,
            4, 7, 121, 3, 4, 74, 4, 6, 99, 4, 15, 87, 4, 6, 53, 4, 7, 86, 4, 6,
            255, 238, 114, 29, 155, 1, 50, 52, 4, 5, 208, 1, 23, 74, 4, 5, 6, 2, 11, 68,
            4, 7, 90, 2, 9, 77, 4, 6, 202, 2, 7, 73, 4, 6, 125, 3, 5, 66, 4, 6,
            91, 4, 15, 86, 4, 6, 55, 4, 6, 88, 4, 6, 156, 1, 50, 56, 4, 5, 200, 1,
            26, 70, 4, 5, 7, 2, 12, 62, 4, 8, 88, 2, 9, 69, 4, 7, 197, 2, 7, 75,
            4, 6, 137, 3, 5, 62, 4, 6, 88, 4, 13, 79, 4, 6, 61, 4, 6, 86, 4, 6,
            255, 238, 154, 29, 162, 1, 47, 55, 4, 5, 208, 1, 26, 60, 4, 6, 5, 2, 12, 68,
            4, 7, 96, 2, 9, 71, 4, 7, 194, 2, 7, 71, 4, 6, 135, 3, 4, 66, 4, 6,
            86, 4, 11, 78, 4, 6, 55, 4, 6, 90, 4, 6, 154, 1, 51, 57, 4, 5, 204, 1,
            25, 64, 4, 5, 4, 2, 11, 66, 4, 7, 96, 2, 9, 75, 4, 7, 199, 2, 7, 64,
            4, 8, 131, 3, 4, 66, 4, 6, 75, 4, 9, 81, 4, 6, 49, 4, 6, 86, 4, 6,
            255, 238, 194, 29, 149, 1, 60, 57, 4, 5, 208, 1, 23, 64, 4, 6, 8, 2, 11, 60,
            4, 8, 92, 2, 9, 70, 4, 8, 202, 2, 7, 67, 4, 8, 131, 3, 5, 64, 4, 5,
            81, 4, 14, 79, 4, 6, 51, 4, 6, 82, 4, 6, 157, 1, 49, 47, 4, 6, 208, 1,
            23, 62, 4, 7, 12, 2, 11, 58, 4, 8, 92, 2, 8, 71, 4, 7, 197, 2, 7, 69,
            4, 6, 137, 3, 4, 62, 4, 6, 75, 4, 11, 79, 4, 6, 55, 4, 6, 78, 4, 6,
            255, 238, 234, 29, 159, 1, 49, 55, 4, 5, 205, 1, 27, 64, 4, 6, 10, 2, 11, 62,
            4, 8, 90, 2, 8, 65, 4, 7, 198, 2, 7, 71, 4, 7, 147, 3, 5, 58, 4, 6,
            101, 4, 15, 79, 4, 6, 57, 4, 6, 86, 4, 6, 162, 1, 47, 53, 4, 6, 204, 1,
            27, 56, 4, 7, 14, 2, 11, 62, 4, 9, 100, 2, 9, 67, 4, 8, 199, 2, 7, 69,
            4, 7, 151, 3, 5, 64, 4, 6, 121, 4, 12, 69, 4, 6, 53, 4, 6, 84, 4, 6,
            255, 238, 18, 30, 161, 1, 48, 51, 4, 6, 208, 1, 26, 57, 4, 8, 9, 2, 11, 65,
            4, 10, 96, 2, 8, 70, 4, 8, 203, 2, 6, 59, 4, 8, 147, 3, 5, 60, 4, 6,
            127, 4, 11, 75, 4, 6, 53, 4, 7, 86, 4, 6, 155, 1, 50, 49, 4, 7, 212, 1,
            20, 62, 4, 8, 12, 2, 11, 56, 4, 11, 102, 2, 8, 70, 4, 8, 207, 2, 6, 63,
            4, 7, 151, 3, 5, 62, 4, 6, 127, 4, 11, 76, 4, 6, 55, 4, 9, 76, 4, 6,
            255, 238, 58, 30, 158, 1, 49, 44, 4, 8, 208, 1, 23, 63, 4, 10, 12, 2, 11, 55,
            4, 11, 96, 2, 9, 65, 4, 10, 209, 2, 6, 64, 4, 8, 169, 3, 5, 54, 4, 7,
            120, 4, 11, 77, 4, 6, 53, 4, 11, 76, 4, 6, 157, 1, 49, 45, 4, 13, 200, 1,
            26, 58, 4, 12, 17, 2, 11, 59, 4, 11, 96, 2, 8, 57, 4, 10, 209, 2, 6, 66,
            4, 8, 147, 3, 4, 58, 4, 6, 104, 4, 11, 72, 4, 6, 63, 4, 20, 84, 4, 6,
            255, 238, 99, 30, 161, 1, 48, 48, 4, 21, 206, 1, 25, 58, 4, 31, 12, 2, 11, 62,
            4, 12, 100, 2, 8, 59, 4, 11, 205, 2, 6, 63, 4, 10, 153, 3, 4, 60, 4, 6,
            64, 4, 32, 69, 4, 7, 68, 4, 38, 76, 4, 6, 163, 1, 47, 51, 4, 36, 208, 1,
            23, 62, 4, 69, 9, 2, 11, 59, 4, 32, 106, 2, 8, 59, 4, 14, 214, 2, 11, 55,
            4, 11, 137, 3, 5, 66, 4, 7, 90, 4, 11, 69, 4, 8, 51, 4, 41, 80, 4, 7,
            255, 238, 139, 30, 157, 1, 49, 44, 4, 36, 214, 1, 21, 57, 4, 55, 11, 2, 11, 59,
            4, 47, 100, 2, 8, 74, 4, 53, 205, 2, 6, 63, 4, 21, 133, 3, 5, 62, 4, 9,
            126, 4, 9, 76, 4, 10, 46, 4, 37, 77, 4, 8, 159, 1, 49, 39, 4, 40, 212, 1,
            22, 56, 4, 46, 16, 2, 11, 50, 4, 37, 98, 2, 8, 62, 4, 30, 214, 2, 6, 61,
            4, 38, 137, 3, 4, 61, 4, 48, 142, 4, 10, 69, 4, 22, 50, 4, 87, 90, 4, 14,
            255, 238, 179, 30, 163, 1, 47, 37, 4, 75, 202, 1, 26, 64, 4, 100, 18, 2, 11, 52,
            4, 44, 100, 2, 8, 54, 4, 39, 212, 2, 6, 62, 4, 35, 139, 3, 4, 54, 4, 14,
            157, 4, 14, 77, 4, 23, 49, 4, 27, 83, 4, 35, 163, 1, 47, 44, 4, 29, 208, 1,
            23, 0, 0, 255, 18, 2, 11, 57, 4, 81, 104, 2, 8, 58, 4, 43, 211, 2, 6, 60,
            4, 37, 139, 3, 4, 56, 4, 12, 163, 4, 13, 67, 4, 26, 0, 0, 42, 77, 4, 22,
            179, 84, 81, 76, 55, 34
        ];
        let foobar: DataBlock = deserialize(&data).unwrap();
        println!("{:0X?}", foobar);
    }

    #[test]
    fn test_read_vlp16_yaml() -> Result<(), Box<dyn std::error::Error>>  {
        let f = std::fs::File::open("VLP16db.yaml")?;
        let config: VLP16Config = serde_yaml::from_reader(f)?;
        println!("config = {:?}", config.lasers);

        print_type_of(&config.lasers);
        make_vert_tables(&config.lasers);
        make_rot_tables(&config.lasers);

        Ok(())
    }
}
