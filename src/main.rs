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
use std::sync::mpsc::{channel, Receiver};
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

    /// Number of cycles per thread.
    #[arg(short, long)]
    cycle_count: u32,

    /// A file prefix to write messages.
    #[arg(short, long)]
    file_output: Option<String>,

    /// TCP address to output raw messages to.
    #[arg(short, long)]
    net_raw_out: Option<String>,

    /// Uniform Linear Array mode. Provide the spacing of the elements in wavelength. Use 0.5 for half a wavelength.
    #[arg(short, long)]
    ula_spacing_wavelength: Option<f32>,

    /// Scales the SNR's weight in the weighted average.
    #[arg(short, long)]
    #[clap(default_value_t = 40.0)]
    snr_scaler: f32,

    /// The depth or count of the items used in the rolling average calculation for the tracking steering vector.
    #[arg(short, long)]
    #[clap(default_value_t = 3)]
    weighted_avg_depth: usize,

    /// If set this will cause the amplitudes to be randomized helping to change the beam pattern.
    #[arg(short, long)]
    #[clap(default_value_t = false)]
    randomize_amplitudes: bool,
}

fn main() {
    let args = Args::parse();

    let thread_count: u32 = args.thread_count;
    let cycle_count: u32 = args.cycle_count;

    println!("Hello, world!");

    println!("Using {} threads and {} cycles.", thread_count, cycle_count);

    let server_addr = "127.0.0.1:7878";

    let mut pipe_mgmt = PipeManagement::new(thread_count as usize, cycle_count as usize);

    let mut rxs: Vec<Receiver<Vec<Message>>> = Vec::new();
    let seen: Arc<Mutex<HashMap<u32, Instant>>> = Arc::new(Mutex::new(HashMap::new()));

    for x in 0..thread_count as usize {
        let (atx, brx) = channel();
        let (btx, arx) = channel();
        pipe_mgmt.push_tx(atx);
        rxs.push(arx);

        let seen_thread = seen.clone();

        let base_pipe_ndx: usize = x * cycle_count as usize;

        thread::spawn(move || {
            println!("spawned");
            let bit_error_table = crc::modes_init_error_info();
            let mut pipe_theta: Vec<Option<Vec<f32>>> = vec![None; cycle_count as usize];
            let mut pipe_amps: Vec<Option<Vec<f32>>> = vec![None; cycle_count as usize];

            loop {
                match brx.recv().unwrap() {
                    ThreadTxMessage::Buffer(buffer, streams) => {
                        btx.send(stream::process_buffer(
                            &buffer,
                            &bit_error_table,
                            &pipe_theta,
                            &pipe_amps,
                            streams,
                            &seen_thread,
                            base_pipe_ndx,
                            args.randomize_amplitudes
                        )).unwrap();
                    },
                    ThreadTxMessage::SetWeights(pipe_ndx, thetas, amps) => {
                        pipe_theta[pipe_ndx] = Some(thetas);
                        pipe_amps[pipe_ndx] = amps;
                        
                    },
                    ThreadTxMessage::UnsetWeights(pipe_ndx) => {
                        pipe_theta[pipe_ndx] = None;
                        pipe_amps[pipe_ndx] = None;
                    },
                }
            }
        });
    }

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

            let mut buffer: Vec<u8> = vec![0; MODES_LONG_MSG_SAMPLES * 1024 * (streams * 4)];

            // This sets up the pipes for uniform linear array (ULA) mode. It consumes all of the pipes
            // currently so there won't be any left to try to track airplanes.
            match args.ula_spacing_wavelength {
                None => (),
                Some(spacing) => {
                    // `spacing` is the distance of each element from the other element in wavelengths
                    let total_pipes = thread_count * cycle_count;

                    // We are going to map -PI/2 to PI/2 to the total pipes.
                    let slice = PI / (total_pipes as f32 - 1.0);

                    for n in 0..total_pipes {
                        let theta = slice * n as f32 - PI * 0.5f32;
                        
                        let mut thetas: Vec<f32> = Vec::with_capacity(streams - 1);

                        // The first theta is always zero so we don't even calculate it. Also,
                        // the code that uses these thetas already expects the first element to
                        // have a theta of zero so we only need to pass minus one the number of
                        // thetas.
                        for element_index in 1..streams {
                            // The conjugate of the phase difference. We have to reverse that]
                            // phase different so they all recieve signals aligned in phase from
                            // this direction `theta`.
                            let shift = -spacing * PI * 2.0f32 * theta.sin() * element_index as f32;
                            thetas.push(shift);
                        }

                        // This communicates with the threads using a global pipe index.
                        pipe_mgmt.set_pipe_to_theta(n as usize, 0, Some(thetas), None);
                    }
                },
            }


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

                        let mut hm: HashMap<u64, Message> = HashMap::new();
                        
                        pipe_mgmt.send_buffer_to_all(&buffer, streams);

                        //println!("getting data from threads");
                        for rx in &rxs {
                            //println!("reading from one thread");
                            for message in rx.recv().unwrap() {
                                // We are highly likely to get the same message from multiple
                                // threads. We should take the highest SNR of any duplicates.
                                match hm.get(&message.common.ndx) {
                                    Some(other) => {
                                        // Compare the SNR (signal to noise) ratio
                                        // and replace the existing if better.
                                        if other.common.snr < message.common.snr {
                                            hm.insert(message.common.ndx, message);    
                                        }
                                    },
                                    None => {
                                        // This was the first time we saw a message at
                                        // this `message.ndx` (index) in the sample
                                        // stream.
                                        hm.insert(message.common.ndx, message);
                                    },
                                }
                            }
                        }

                        let mut items: Vec<(u64, Message)> = hm.into_iter().collect();
                        // They might be out of order so sort them to ensure they are ordered.
                        items.sort_by(|a, b| (&a.0).cmp(&b.0));
                        
                        // Update all indices to be global offsets. They come as offsets
                        // into the buffer but since we track the total offset across all
                        // buffers add them with the base `sample_index`.
                        for (_, message) in &mut items {
                            message.common.ndx += sample_index;
                        }

                        for (_, message) in &items {
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

                        // The message have been demodulated and decoded. Process them to produce
                        // an aircraft entity with attached information. This also computes a
                        // steering vector to try to track the aircraft with the antenna system.
                        entity::process_messages(
                            items,
                            &mut entities,
                            sample_index,
                            &mut pipe_mgmt,
                            args.snr_scaler,
                            args.weighted_avg_depth
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
                                "ADDR   FLIGHT    ALT        LAT       LON      COUNT    INBEAM  STEERING VECTOR"
                            );

                            for (addr, ent) in entities.iter() {
                                let flight = match &ent.flight {
                                    None => String::from(" "),
                                    Some(v) => v.into_iter().collect::<String>(),
                                };

                                println!(
                                    "{:6x} {:>8} {:>8.1} {:>10.4} {:>10.4} {:0>7} {:>7} {:?}",
                                    addr,
                                    flight,
                                    ent.alt.unwrap_or(0.0),
                                    ent.lat.unwrap_or(0.0),
                                    ent.lon.unwrap_or(0.0),
                                    ent.message_count,
                                    // How many messages were picked from the calculated steering vector.
                                    ent.inbeam,
                                    ent.theta_avg(args.snr_scaler)
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