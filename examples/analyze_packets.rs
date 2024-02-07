use core::convert::AsRef;
use core::net::IpAddr;
use core::net::Ipv4Addr;
use core::net::SocketAddr;
use std::fs::File;
use std::net::UdpSocket;
use std::path::Path;

use bincode::deserialize;
use csv::Writer;

use velodyne_driver;
use velodyne_driver::{
    calc_points, make_vert_tables, rotation_calculator_new, Data, Point, PointProcessor, RawData,
    VLP16Config,
};

struct CSVWriter {
    writer: Writer<File>,
}

impl CSVWriter {
    fn from_path<P: AsRef<Path>>(path: P) -> Result<Self, std::io::Error> {
        let writer = Writer::from_path(path)?;
        Ok(CSVWriter { writer })
    }
}

impl PointProcessor for CSVWriter {
    fn process(&mut self, _: usize, point: &Point) {
        let (x, y, z) = point;
        let strs = [x.to_string(), y.to_string(), z.to_string()];
        self.writer.write_record(&strs).unwrap();
    }
}

fn make_socket() -> Result<UdpSocket, std::io::Error> {
    let ip_addr = IpAddr::V4(Ipv4Addr::new(0, 0, 0, 0));
    let socket_addr = SocketAddr::new(ip_addr, 2368);
    UdpSocket::bind(socket_addr)
}

fn extract_distances(sequence: &[Data], resolution: f64) -> Vec<f64> {
    sequence
        .iter()
        .map(|s: &Data| (s.distance as f64) * resolution)
        .collect::<Vec<f64>>()
}

fn main() -> Result<(), std::io::Error> {
    let f = std::fs::File::open("VLP16db.yaml")?;
    let config: VLP16Config = serde_yaml::from_reader(f).unwrap();

    let tables = make_vert_tables(&config.lasers);

    let socket = make_socket()?;
    for i in 0..720 {
        let mut writer = CSVWriter::from_path(format!("points/points{:03}.csv", i)).unwrap();

        let mut buf = [0u8; 1206];
        socket.recv_from(&mut buf)?;

        let data: RawData = deserialize(&buf).unwrap();
        for (block_index, block) in data.blocks.iter().enumerate() {
            let r = rotation_calculator_new(&data.blocks, block_index);
            let rotations0 = r.get(0);
            let rotations1 = r.get(1);
            let distances0 = extract_distances(&block.sequence0, config.distance_resolution);
            let distances1 = extract_distances(&block.sequence1, config.distance_resolution);

            calc_points(&mut writer, &tables, &distances0, &rotations0);
            calc_points(&mut writer, &tables, &distances1, &rotations1);
        }
    }

    Ok(())
}
