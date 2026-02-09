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

    /// A file prefix to write messages.
    #[arg(short, long)]
    file_output: Option<String>,

    /// TCP address to output raw messages to.
    #[arg(short, long)]
    net_raw_out: Option<String>,

    #[arg(short, long)]
    #[clap(default_value_t = 0.5e-4f32)]
    mu: f32,
}

use bytemuck::cast_slice;
use num::complex::Complex;
use stream::ProcessStreamResult;

fn process_stream_lms(
    u8_buffer: &[u8],
    streams: usize,
    bit_error_table: &HashMap<u32, u16>,
    seen: &Arc<Mutex<HashMap<u32, Instant>>>,
    mu: f32
) -> Vec<Message> {
    let buffer: &[i16] = cast_slice(u8_buffer);
    let mut iq: Vec<Vec<Complex<f32>>> = Vec::new();
    let mut messages: Vec<Message> = Vec::new();

    for x in 0..streams {
        iq.push(Vec::new());
    }

    let mul = streams * 2;
    for x in 0..buffer.len() / mul {
        let chunk = &buffer[x * mul..x * mul + mul];
        for y in 0..streams {
            iq[y].push(Complex::new(chunk[y * 2 + 0] as f32 / 2049.0, chunk[y * 2 + 1] as f32 / 2049.0));
        }
    }

    let soi_bits = vec![1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0];
    let mut soi: Vec<Complex<f32>> = Vec::new();
    for x in 0..soi_bits.len() {
        soi.push(Complex::new(soi_bits[x] as f32, 0.0));
    }

    let mut samples: Vec<f32> = vec![0.0f32; MODES_LONG_MSG_SAMPLES];

    for x in 0..buffer.len() / mul - MODES_PREAMBLE_SAMPLES - MODES_LONG_MSG_SAMPLES {
        let mut w_lms = vec![Complex::new(0.0f32, 0.0f32); streams];

        for i in 0..constants::MODES_PREAMBLE_SAMPLES {
            let soi_sample = soi[i];
            let mut sum = Complex::new(0.0f32, 0.0f32);
            
            for y in 0..streams {
                sum += w_lms[y].conj() * iq[y][x + i];
            }

            let error = soi_sample - sum;
            
            for y in 0..streams {
                w_lms[y] += mu * error.conj() * iq[y][x + i];
            }
        }

        for i in 0..constants::MODES_LONG_MSG_SAMPLES {
            let mut sum = Complex::new(0.0f32, 0.0f32);
            
            for y in 0..streams {
                sum += w_lms[y].conj() * iq[y][x + constants::MODES_PREAMBLE_SAMPLES + i];
            }

            samples[i] = sum.norm();
        }

        let mut thebyte: u8 = 0;
        let mut msg: Vec<u8> = Vec::new();

        for y in 0..samples.len() / 2 {
            let a: f32 = samples[y * 2 + 0];
            let b: f32 = samples[y * 2 + 1];

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
                snr: 0.0,
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
                messages.push(message);
            },
            Err(_) => (),
        }
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

    match TcpStream::connect(server_addr) {
        Ok(mut stream) => {
            println!("connected");
            // We are expecting TWO interleaved streams from TWO antennas.
            let mut read: usize = 0;
            
            let mut short_buffer = vec![0; 1];
            // Read the number of streams.
            let streams = match stream.read(&mut short_buffer[0..1]) {
                Ok(bytes_read) if bytes_read > 0 => {
                    short_buffer[0] as usize
                },
                Ok(_) => {
                    panic!("Sample stream TCP connection returned zero bytes.");
                },
                Err(e) => {
                    panic!("Error: {}", e);
                }
            };

            for x in 0..thread_count as usize {
                let (atx, brx) = channel();
                let (btx, arx) = channel();
                txs.push(atx);
                rxs.push(arx);

                let seen_thread = seen.clone();

                thread::spawn(move || {
                    println!("spawned");
                    let bit_error_table = crc::modes_init_error_info();

                    loop {
                        let buffer = brx.recv().unwrap();
                        let messages = process_stream_lms(
                            &buffer,
                            streams,
                            &bit_error_table,
                            &seen_thread,
                            args.mu
                        );
                        btx.send(messages).unwrap();
                    }
                });
            }


            let mut buffer: Vec<u8> = vec![0; MODES_LONG_MSG_SAMPLES * 1024 * (streams * 4)];

            println!("working with {} streams", streams);

            let sps: f64 = 2e6f64;
            let buffer_time: f64 = buffer.len() as f64 / (streams as f64 * 4.0f64) /  sps;
            println!("reading stream");
            // TODO: Take the tail end of the buffer and prefix it to the
            // next buffer incase a message is across the two buffers.
            while match stream.read(&mut buffer[read..]) {
                Ok(bytes_read) if bytes_read > 0 => {
                    read += bytes_read;
                    //println!("read bytes {}", bytes_read);
                    if read == buffer.len() {
                        //println!("sending buffer to threads");
                        let start = Instant::now();
                        
                        //pipe_mgmt.send_buffer_to_all(&buffer, streams);
                        
                        let bytes_per_strip = 4 * streams;
                        let sample_count = buffer.len() / bytes_per_strip;
                        let chunk_size: usize = sample_count / thread_count as usize;
                        let rem: usize = sample_count % thread_count as usize;
                        
                        for i in 0..thread_count as usize - 1 {
                            let chunk_slice: &[u8] = &buffer[i * chunk_size * bytes_per_strip..(i * chunk_size + chunk_size + MODES_LONG_MSG_SAMPLES) * bytes_per_strip];
                            let chunk: Vec<u8> = chunk_slice.to_vec();
                            txs[i].send(chunk).unwrap();
                        }

                        let i = thread_count as usize - 1;
                        let chunk_slice = &buffer[i * chunk_size * bytes_per_strip..(i * chunk_size + chunk_size + rem) * bytes_per_strip];
                        let chunk: Vec<u8> = chunk_slice.to_vec();
                        txs[i].send(chunk).unwrap();

                        let mut messages: Vec<Message> = Vec::new();

                        for i in 0..thread_count as usize {
                            let msgs = rxs[i].recv().unwrap();
                            for msg in msgs {
                                messages.push(msg);
                            }
                        }

                        //let messages = process_stream_lms(
                        //    &buffer,
                        //    streams,
                        //    &bit_error_table,
                        //    &seen
                        //);

                        for message in &messages {
                            match message.specific {
                                MessageSpecific::AircraftIdenAndCat { .. } => stat_aiac += 1,
                                MessageSpecific::SurfacePositionMessage { .. } => stat_spm += 1,
                                MessageSpecific::AirbornePositionMessage { .. } => stat_apm += 1,
                                MessageSpecific::AirborneVelocityMessage { .. } => stat_avm += 1,
                                MessageSpecific::AirborneVelocityMessageShort { .. } => stat_avms += 1,
                                _ => (),
                            }

                            // This is used to send the raw data in HEX format over a socket. The
                            // primary use case for this is when providing the argument --net-raw-out
                            // for a dump1090 instance running in --net-only mode so you can have a
                            // webpage map of the aircraft.
                            match net_raw_out_stream {
                                None => (),
                                Some(ref mut stream) => {
                                    let msg = message.common.msg.clone();
                                    let hex_string: String = msg.iter().map(
                                        |byte| format!("{:02X}", byte)
                                    ).collect();
                                    let line = format!("*{};\n", hex_string);
                                    if line.is_ascii() {
                                        let ascii_bytes = line.as_bytes();
                                        stream.write(ascii_bytes).unwrap();
                                    }
                                },
                            }

                            // This is used when the --file-output argument is specified. It writes the
                            // raw messages and associated data to a file in a serialized format. See
                            // the function `write_message_to_file` for a detailed overview of the
                            // format used.
                            match message.specific {
                                MessageSpecific::Other => (),
                                _ => {
                                    match &mut file {
                                        None => (),
                                        Some(file) => {
                                            write_message_to_file(file, &message);
                                        },
                                    }
                                },
                            }                            
                        }

                        let items = messages.into_iter().map(|x| (x.common.ndx, x)).collect();

                        // The message have been demodulated and decoded. Process them to produce
                        // an aircraft entity with attached information. This also computes a
                        // steering vector to try to track the aircraft with the antenna system.
                        entity::process_messages(
                            items,
                            &mut entities,
                            sample_index,
                            &mut pipe_mgmt,
                            1.0,
                            1
                        );

                        if (Instant::now() - stat_start).as_secs() > 5 {
                            stat_start = Instant::now();
                            let elapsed_dur: Duration = stat_gstart.elapsed();
                            let elapsed = elapsed_dur.as_secs() as f64 + elapsed_dur.subsec_micros() as f64 / 1e6f64;
                            println!("Type                          Total/PerSecond");
                            println!("AircraftIdenAndCat            {}/{:.1}", stat_aiac, stat_aiac as f64 / elapsed);
                            println!("SurfacePositionMessage        {}/{:.1}", stat_spm, stat_spm as f64 / elapsed);
                            println!("AirbornePositionMessage       {}/{:.1}", stat_apm, stat_apm as f64 / elapsed);
                            println!("AirborneVelocityMessage       {}/{:.1}", stat_avm, stat_avm as f64 / elapsed);
                            println!("AirborneVelocityMessageShort  {}/{:.1}", stat_avms, stat_avms as f64 / elapsed);
                            println!("====== AIRCRAFT ========");
                            let keys: Vec<u32> = entities.keys().map(|x| *x).collect();
                            for addr in keys {
                                let last_update = entities.get(&addr).unwrap().last_update;
                                let delta = sample_index - last_update;
                                // If we have not heard from an entity in roughly 10 seconds then
                                // remove it from the list and make sure to unset any pipe that
                                // was assigned to it.
                                if delta / 2_000_000u64 > 60 {
                                    pipe_mgmt.unset_addr(addr);
                                    entities.remove(&addr);
                                    println!("removed addr {:6x}", addr);
                                }
                            }
                            
                            println!(
                                "ADDR   FLIGHT    ALT        LAT       LON      COUNT"
                            );

                            for (addr, ent) in entities.iter() {
                                let flight = match &ent.flight {
                                    None => String::from(" "),
                                    Some(v) => v.into_iter().collect::<String>(),
                                };

                                println!(
                                    "{:6x} {:>8} {:>8.1} {:>10.4} {:>10.4} {:0>7}",
                                    addr,
                                    flight,
                                    ent.alt.unwrap_or(0.0),
                                    ent.lat.unwrap_or(0.0),
                                    ent.lon.unwrap_or(0.0),
                                    ent.message_count,
                                );
                            }

                            println!("buffer-time-elapsed-average: {} buffer-time:{}", buffer_time_elapsed_avg, buffer_time);
                        }
                        
                        {
                            let elapsed_dur: Duration = start.elapsed();
                            let cur_elapsed = elapsed_dur.as_secs() as f64 + elapsed_dur.subsec_micros() as f64 / 1e6f64;
                            
                            if buffer_time_elapsed_avg == 0.0 {
                                buffer_time_elapsed_avg = cur_elapsed;
                            } else {
                                buffer_time_elapsed_avg = buffer_time_elapsed_avg * 0.9999 + cur_elapsed * 0.0001;
                            }
                            
                            if cur_elapsed > buffer_time * 0.95 {
                                println!("elapsed:{} buffer_time:{} TOO SLOW!!! REDUCE CYCLES!!!", cur_elapsed, buffer_time);
                            }
                        }

                        sample_index += buffer.len() as u64 / (streams * 4) as u64;
                        read = 0;
                    }

                    true
                },
                Ok(_) => false,
                Err(e) => {
                    eprintln!("error: {}", e);
                    false
                },
            } {

            }
            
        },
        Err(e) =>  {
            eprintln!("failed to connect: {}", e);
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