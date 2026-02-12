//! Proof of concept for ADS-B reception using beamforming of a two antenna system.
//!
//! This program was designed for experimentation and testing. It provides limited
//! value as is. However, it might serve as a solid foundation or at the least an
//! idea for a future project.
//!
//! Thanks to Salvatore Sanfilippo <antirez@gmail.com> and <https://github.com/antirez/dump1090/>
//!
//! Thanks to Malcolm Robb <support@attavionics.com> and <https://github.com/MalcolmRobb/dump1090/>
//!
//! Thanks to <https://github.com/flightaware/dump1090>

use std::sync::{Arc, Mutex};
use std::io::Read;
use std::net::TcpStream;
use bytemuck::bytes_of;
use std::time::{Duration, Instant};
use std::thread;
use std::sync::mpsc::{channel, Receiver, Sender};
use std::collections::HashMap;
use clap::Parser;
use std::fs::File;
use std::io::prelude::*;
use std::f32::consts::PI;
use bytemuck::cast_slice;
use num::complex::Complex;
use nalgebra as na;

mod crc;
mod constants;
mod stream;
mod pipemgmt;
mod cpr;
mod entity;
mod decode;

use pipemgmt::{ThreadTxMessage, PipeManagement};
use entity::Entity;
use decode::{Message, MessageSpecific};
use decode::process_result;

use constants::*;

/// Serialize the common elements of a message to a file.
fn write_message_to_file(file: &mut File, m: &Message) {
    /*
        The message format.

        u16; message byte count
        [u8]; message bytes as array
        u16; sample count
        [i16]; samples as array
        u64; sample stream index
        f32; snr
        u8; count of theta values
        [f32]; theta values as array
        u8; count of amplitude values
        [f32]; amplitude values as array
    */
    // u16; message byte count
    file.write_all(bytes_of(&(m.common.msg.len() as u16))).unwrap();
    // u8; message bytes
    file.write_all(&m.common.msg).unwrap();
    // u16; sample count
    file.write_all(bytes_of(&(m.common.samples.len() as u16))).unwrap();
    // i16; samples
    for x in 0..m.common.samples.len() {
        file.write_all(bytes_of(&m.common.samples[x])).unwrap();
    }
    // u64; global sample stream offset
    file.write_all(bytes_of(&m.common.ndx)).unwrap();
    // f32; snr
    file.write_all(bytes_of(&m.common.snr)).unwrap();
    let thetas = &m.common.thetas;
    // u8; count of theta values
    file.write_all(bytes_of(&(thetas.len() as u8))).unwrap();
    for theta in thetas {
        // f32; theta value
        file.write_all(bytes_of(theta)).unwrap();
    }
    let amplitudes = &m.common.amplitudes;
    // u8; count of amplitude values
    file.write_all(bytes_of(&(m.common.amplitudes.len() as u8))).unwrap();
    for amp in amplitudes {
        // f32; amplitude value
        file.write_all(bytes_of(amp)).unwrap();
    }
}

/// The command line arguments for the program using the crate Clap.
#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    /// Number of threads to use.
    #[arg(short, long)]
    thread_count: u32,

    /// A file to write messages to.
    #[arg(short, long)]
    file_output: Option<String>,

    /// A file to read samples from.
    #[arg(short, long)]
    file_input: String,

    /// TCP address to output raw messages to.
    #[arg(short, long)]
    net_raw_out: Option<String>,
}

use stream::ProcessStreamResult;

fn cft(theta: f64) -> Complex<f64> {
    Complex::new(theta.cos(), theta.sin())
}

fn process_stream_lcmv(
    u8_buffer: &[u8],
    streams: usize,
    bit_error_table: &HashMap<u32, u16>,
    seen: &Arc<Mutex<HashMap<u32, Instant>>>
) -> Vec<Message> {
    if streams != 4 {
        panic!("Exactly four streams are supported for process_stream_lcmv.")
    }

    let buffer: &[i16] = cast_slice(u8_buffer);
    let mut iq: Vec<Vec<Complex<f64>>> = Vec::new();

    for x in 0..streams {
        iq.push(Vec::new());
    }

    let mul = streams * 2;
    for x in 0..buffer.len() / mul {
        let chunk = &buffer[x * mul..x * mul + mul];
        for y in 0..streams {
            iq[y].push(Complex::new(chunk[y * 2 + 0] as f64 / 2049.0, chunk[y * 2 + 1] as f64 / 2049.0));
        }
    }

    let mut cov: Vec<Complex<f64>> = Vec::new();

    for x in 0..4 {
        for y in 0..4 {
            let mut sum = Complex::new(0.0f64, 0.0f64);
            for z in 0..iq[x].len() {
                sum += iq[x][z] * iq[y][z].conj();
            }
            cov.push(sum / iq[x].len() as f64);
        }
    }

    let R = na::Matrix4::new(
        cov[0],  cov[1],  cov[2],  cov[3],
        cov[4],  cov[5],  cov[6],  cov[7],
        cov[8],  cov[9],  cov[10], cov[11],
        cov[12], cov[13], cov[14], cov[15]
    ); 
    
    let R_inv = R.try_inverse().unwrap();

    let slice_count = 300;
    let pi = std::f64::consts::PI;

    let theta_slice = pi / (slice_count as f64 - 1.0f64);
    
    let d = 0.4f64;

    let mut messages: Vec<Message> = Vec::new();
    let mut hm: HashMap<usize, Message> = HashMap::new();

    for lobe_ndx in 0..slice_count {
        let lobe_theta = lobe_ndx as f64 * theta_slice - pi * 0.5;
        for null_ndx in 0..slice_count {
            let null_theta = null_ndx as f64 * theta_slice - pi * 0.5;

            let C = na::Matrix4x2::new(
                cft(2.0f64 * pi * d * 0.0 * lobe_theta.sin()),
                cft(2.0f64 * pi * d * 0.0 * null_theta.sin()),

                cft(2.0f64 * pi * d * 1.0 * lobe_theta.sin()),
                cft(2.0f64 * pi * d * 1.0 * null_theta.sin()),

                cft(2.0f64 * pi * d * 2.0 * lobe_theta.sin()),
                cft(2.0f64 * pi * d * 2.0 * null_theta.sin()),

                cft(2.0f64 * pi * d * 3.0 * lobe_theta.sin()),
                cft(2.0f64 * pi * d * 3.0 * null_theta.sin())
            );
            
            let f = na::Matrix2x1::new(Complex::new(1.0, 0.0), Complex::new(0.0, 0.0));

            //         2x4         4x4     4x2
            //let q = (C.adjoint() * R_inv * C).try_inverse().unwrap();  

            let inner = match (C.adjoint() * R_inv * C).try_inverse() {
                Some(v) => v,
                None => {
                    // np.linalg.pinv needed...
                    continue;
                },
            };        

            let w = R_inv * C * inner * f;

            let wx = w[(0, 0)].conj();
            let wy = w[(1, 0)].conj();
            let wz = w[(2, 0)].conj();
            let ww = w[(3, 0)].conj();

            let a = &iq[0];
            let b = &iq[1];
            let c = &iq[2];
            let d = &iq[3];

            let mut mag: Vec<f64> = Vec::with_capacity(a.len());

            assert!(a.len() == b.len());
            assert!(a.len() == c.len());
            assert!(a.len() == d.len());

            for x in 0..a.len() {
                let sample = a[x] * wx + b[x] * wy + c[x] * wz + d[x] * ww;
                //let sample = a[x]; // + b[x] + c[x] + d[x];
                mag.push((sample.re * sample.re + sample.im * sample.im).sqrt());
            }

            for x in 0..mag.len() - constants::MODES_PREAMBLE_SAMPLES - constants::MODES_LONG_MSG_SAMPLES {
                let p = &mag[x..x + constants::MODES_PREAMBLE_SAMPLES];
                let valid: bool = (p[0] > p[1]) && (p[1] < p[2]) && (p[2] > p[3]) && (p[3] < p[0]) && 
                                (p[4] < p[0]) && (p[5] < p[0]) && (p[6] < p[0]) && (p[7] > p[8]) &&
                                (p[8] < p[9]) && (p[9] > p[6]);
                if !valid {
                    continue;
                }

                let high = (p[0] + p[2] + p[7] + p[9]) / 6.0f64;

                if (p[4] >= high) || (p[5] >= high) {
                    continue;
                }

                if (p[11] > high) || (p[12] > high) || (p[13] > high) || (p[14] > high) {
                    continue;
                }

                let snr = (p[0] - p[1]) + (p[2] - p[3]) + (p[7] - p[6]) + (p[9] - p[8]);

                let samples = &mag[x + constants::MODES_PREAMBLE_SAMPLES..x + constants::MODES_PREAMBLE_SAMPLES + constants::MODES_LONG_MSG_SAMPLES];

                let mut thebyte: u8 = 0;
                let mut msg: Vec<u8> = Vec::new();

                for y in 0..samples.len() / 2 {
                    let a = samples[y * 2 + 0];
                    let b = samples[y * 2 + 1];

                    if a > b {
                        thebyte |= 1;
                    }

                    if y & 7 == 7 {
                        msg.push(thebyte);
                    }

                    thebyte = thebyte << 1;            
                }

                match decode::process_result(
                    ProcessStreamResult {
                        snr: snr as f32,
                        msg: msg,
                        samples: Vec::new(),
                        ndx: x,
                        thetas: Vec::new(),
                        amplitudes: Vec::new(),
                        pipe_ndx: 0,
                    },
                    bit_error_table,
                    seen
                ) {
                    Ok(message) => {
                        match hm.get(&x) {
                            None => {
                                println!("message");
                                hm.insert(x, message);
                            },
                            Some(om) => {
                                if om.common.snr < message.common.snr {
                                    hm.insert(x, message);
                                }
                            },
                        }
                    },
                    Err(v) => (),
                }
            }
        }
    }

    //let s1 = na::Matrix4x1::new(Complex::new(1.0, 1.0), Complex::new(1.0, 1.0), Complex::new(1.0, 1.0), Complex::new(1.0, 1.0));
    //let C = na::Matrix4x2::new(Complex::new(1.0, 1.0), Complex::new(1.0, 1.0), Complex::new(1.0, 1.0), Complex::new(1.0, 1.0), Complex::new(1.0, 1.0), Complex::new(1.0, 1.0), Complex::new(1.0, 1.0), Complex::new(1.0, 1.0));
    //let R_inv = na::Matrix4::new(Complex::new(1.0, 1.0), Complex::new(1.0, 1.0), Complex::new(1.0, 1.0), Complex::new(1.0, 1.0), Complex::new(1.0, 1.0), Complex::new(1.0, 1.0), Complex::new(1.0, 1.0), Complex::new(1.0, 1.0), Complex::new(1.0, 1.0), Complex::new(1.0, 1.0), Complex::new(1.0, 1.0), Complex::new(1.0, 1.0), Complex::new(1.0, 1.0), Complex::new(1.0, 1.0), Complex::new(1.0, 1.0), Complex::new(1.0, 1.0));
    //let inner = na::Matrix2::new(1.0, 1.0, 1.0, 1.0);
    //let f = na::Matrix2x1::new(Complex::new(1.0, 0.0), Complex::new(0.0, 0.0));

    // conjugate-transpose

    for (ndx, message) in hm.into_iter() {
        messages.push(message);
    }

    messages
}

fn main() {
    let args = Args::parse();

    let thread_count: u32 = args.thread_count;

    println!("Hello, world!");

    println!("Using {} threads.", thread_count);

    let server_addr = "127.0.0.1:7878";

    let mut pipe_mgmt = PipeManagement::new(thread_count as usize, 0);

    let mut rxs: Vec<Receiver<Vec<Message>>> = Vec::new();
    let mut txs: Vec<Sender<Vec<u8>>> = Vec::new();

    let seen: Arc<Mutex<HashMap<u32, Instant>>> = Arc::new(Mutex::new(HashMap::new()));

    let bit_error_table = crc::modes_init_error_info();

    let bit_error_table = crc::modes_init_error_info();

    let mut file = match args.file_output {
        Some(v) => {
            Some(File::create(v).unwrap())
        },
        None => None,
    };

    let mut entities: HashMap<u32, Entity> = HashMap::new();

    let mut sample_index: u64 = 0;

    let mut buffer_time_elapsed_avg = 0.0f64;

    let mut stat_aiac: u64 = 0;
    let mut stat_spm: u64 = 0;
    let mut stat_apm: u64 = 0;
    let mut stat_avm: u64 = 0;
    let mut stat_avms: u64 = 0;
    let mut stat_start = Instant::now();
    let stat_gstart = Instant::now();

    let mut net_raw_out_stream: Option<TcpStream> = match args.net_raw_out {
        None => None,
        Some(addr) => match TcpStream::connect(addr.clone()) {
            Ok(stream) => {
                println!("connected to --net-raw-out {}", addr);
                Some(stream)
            },
            Err(e) => {
                println!("{}", e);
                panic!("failed to connect to --net-raw-out")
            },
        },
    };

    let mut fin = match File::open(args.file_input) {
        Ok(file) => file,
        Err(msg) => {
            panic!("error opening file: {}", msg);
        },
    };

    let mut count = 0usize;
    let mut read = 0.0f64;

    loop {
        let mut buf: Vec<u8> = vec![0; MODES_LONG_MSG_SAMPLES * 512 * (4 * 4)];

        match fin.read_exact(&mut buf) {
            Ok(_) => {
                let messages = process_stream_lcmv(
                    &buf,
                    4,
                    &bit_error_table,
                    &seen
                );

                count += messages.len();
                read += buf.len() as f64;
                println!("count: {} read: {}", count, read / 16.0f64 / 2e6f64);
            },
            Err(..) => break,
        }
    }

    println!("exiting");
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_getbits() {
    }
}