use core::convert::AsRef;
use core::net::IpAddr;
use core::net::Ipv4Addr;
use core::net::SocketAddr;
use std::fs::File;
use std::net::UdpSocket;
use std::path::Path;

use csv::Writer;

use velodyne_driver;
use velodyne_driver::{Point, PointCloudCalculator, PointProcessor, VLP16Config};

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

fn main() -> Result<(), std::io::Error> {
    let f = std::fs::File::open("VLP16db.yaml")?;
    let config: VLP16Config = serde_yaml::from_reader(f).unwrap();
    let calculator = PointCloudCalculator::new(&config);

    let socket = make_socket()?;
    for i in 0..75 {
        let mut writer = CSVWriter::from_path(format!("points/points{:03}.csv", i)).unwrap();

        let mut buf = [0u8; 1206];
        socket.recv_from(&mut buf)?;

        calculator.calculate(&mut writer, &buf);
    }

    Ok(())
}
