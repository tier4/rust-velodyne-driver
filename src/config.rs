use alloc::format;
use alloc::string::String;
use alloc::vec;
use alloc::vec::Vec;
use yaml_peg::repr::{RcRepr, Repr};
use yaml_peg::Seq;

#[derive(Debug)]
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

#[derive(Debug)]
pub struct VLP16Config {
    pub lasers: [LaserConfig; 16],
    pub num_lasers: usize,
    pub distance_resolution: f64,
}

fn read_float<R: Repr>(node: &yaml_peg::Node<R>) -> Result<f64, String> {
    match node.as_float() {
        Ok(v) => return Ok(v),
        Err(pos) => return Err(format!("Faild to read a float value at position {}.", pos)),
    }
}

fn read_int<R: Repr>(node: &yaml_peg::Node<R>) -> Result<i64, String> {
    match node.as_int() {
        Ok(v) => return Ok(v),
        Err(pos) => return Err(format!("Faild to read a int value at position {}.", pos)),
    }
}

fn read_seq<R: Repr>(node: &yaml_peg::Node<R>) -> Result<Seq<R>, String> {
    match node.as_seq() {
        Ok(v) => return Ok(v),
        Err(pos) => return Err(format!("Faild to read a sequence at position {}.", pos)),
    }
}

pub fn parse_config(config_str: &str) -> Result<VLP16Config, String> {
    let configs = yaml_peg::parse::<RcRepr>(config_str).unwrap();
    let config = &configs[0];

    let distance_resolution = read_float(&config["distance_resolution"])?;
    let num_lasers = read_int(&config["num_lasers"])?;
    let lasers = read_seq(&config["lasers"])?;

    assert_eq!(num_lasers, 16);
    assert_eq!(lasers.len(), 16);

    let mut laser_configs: Vec<LaserConfig> = vec![];
    for row in lasers.iter() {
        let c = LaserConfig {
            dist_correction: read_float(&row["dist_correction"])?,
            dist_correction_x: read_float(&row["dist_correction_x"])?,
            dist_correction_y: read_float(&row["dist_correction_y"])?,
            focal_distance: read_float(&row["focal_distance"])?,
            focal_slope: read_float(&row["focal_slope"])?,
            horiz_offset_correction: read_float(&row["horiz_offset_correction"])?,
            laser_id: read_int(&row["laser_id"])? as usize,
            rot_correction: read_float(&row["rot_correction"])?,
            vert_correction: read_float(&row["vert_correction"])?,
            vert_offset_correction: read_float(&row["vert_offset_correction"])?,
        };
        laser_configs.push(c);
    }

    Ok(VLP16Config {
        lasers: laser_configs.try_into().unwrap(),
        num_lasers: (num_lasers as usize),
        distance_resolution: distance_resolution,
    })
}
