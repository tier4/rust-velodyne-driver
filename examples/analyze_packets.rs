use core::convert::AsRef;
use core::net::IpAddr;
use core::net::Ipv4Addr;
use core::net::SocketAddr;
use std::net::UdpSocket;
use std::path::Path;

use hdf5;
use ndarray::{Array, Array3};
use velodyne_driver::{
    Point, PointCloudCalculator, PointProcessor, VLP16Config, CHANNELS_PER_SEQUENCE, N_SEQUENCES,
    VLP16_PACKET_DATA_SIZE,
};

const COMPRESSION_LEVEL: u8 = 9;

struct HDF5Writer {
    file: hdf5::File,
}

impl HDF5Writer {
    fn from_path<P: AsRef<Path>>(path: P) -> Result<Self, hdf5::Error> {
        let file = hdf5::File::create(path)?;
        Ok(HDF5Writer { file })
    }
}

struct Scan {
    pub points: Array3<f64>,
}

impl Default for Scan {
    fn default() -> Self {
        Scan {
            points: Array::zeros((N_SEQUENCES, CHANNELS_PER_SEQUENCE, 3)),
        }
    }
}

impl PointProcessor for Scan {
    fn process(&mut self, sequence_index: usize, channel: usize, point: &Point) {
        let (x, y, z) = point;
        self.points[[sequence_index, channel, 0]] = *x;
        self.points[[sequence_index, channel, 1]] = *y;
        self.points[[sequence_index, channel, 2]] = *z;
    }
}

impl HDF5Writer {
    fn write(&self, dataset_name: &str, points: &Array3<f64>) -> hdf5::Result<()> {
        #[cfg(feature = "blosc")]
        blosc_set_nthreads(2);
        let builder = self.file.new_dataset_builder();
        #[cfg(feature = "blosc")]
        let builder = builder.blosc_zstd(COMPRESSION_LEVEL, true);
        builder.with_data(points).create(dataset_name)?;
        Ok(())
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
    let mut buf = [0u8; VLP16_PACKET_DATA_SIZE];

    let writer = HDF5Writer::from_path("points/points.hdf5").unwrap();

    for i in 0..(75 * 300) {
        socket.recv_from(&mut buf)?;
        let scan_name = format!("{:08}", i);
        println!("scan_name = {scan_name}");

        let mut scan = Scan::default();
        calculator.calculate(&mut scan, &buf);
        writer.write(&scan_name, &scan.points).unwrap();
    }

    Ok(())
}
