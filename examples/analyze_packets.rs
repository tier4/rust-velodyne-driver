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
    calc_angles, make_vert_tables, Point, PointProcessor, RawData, RotationCalculator,
    SinCosTables, VLP16Config,
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
    fn process(&mut self, point: &Point) {
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

fn main() -> Result<(), std::io::Error> {
    let f = std::fs::File::open("VLP16db.yaml")?;
    let config: VLP16Config = serde_yaml::from_reader(f).unwrap();
    let (vert_sin, vert_cos) = make_vert_tables(&config.lasers);
    let tables = SinCosTables {
        vert_sin: vert_sin,
        vert_cos: vert_cos,
        distance_resolution: config.distance_resolution,
    };

    let socket = make_socket()?;
    for i in 0..720 {
        let mut writer = CSVWriter::from_path(format!("points/points{:03}.csv", i)).unwrap();

        let mut buf = [0u8; 1206];
        socket.recv_from(&mut buf)?;

        let data: RawData = deserialize(&buf).unwrap();
        for (block_index, block) in data.blocks.iter().enumerate() {
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
            tables.calc_points(&mut writer, &r0, &block.sequence0);
            tables.calc_points(&mut writer, &r1, &block.sequence1);
        }
    }

    Ok(())
}
