//! Proof of concept for ADS-B reception using beamforming of a two antenna system.
//!
//! This program was designed for experimentation and testing. It provides limited
//! value as is. However, it might serve as a solid foundation or at the least an
//! idea for a future project.
//!
//! Thanks to Salvatore Sanfilippo <antirez@gmail.com> and https://github.com/antirez/dump1090/
//! Thanks to Malcolm Robb <support@attavionics.com> and https://github.com/MalcolmRobb/dump1090/
//! Thanks to https://github.com/flightaware/dump1090

use std::io::Read;
use std::net::TcpStream;
use bytemuck::bytes_of;
use std::time::{Duration, Instant};
use std::thread;
use std::sync::mpsc::{channel, Sender, Receiver};
use std::collections::HashMap;
use std::result::Result;
use clap::Parser;
use std::fs::File;
use std::io::prelude::*;
use std::fmt;

mod crc;
mod constants;
mod stream;

use constants::*;

struct MessageCommon {
    /// The bytes that comprise the message after demodulation.
    msg: Vec<u8>,
    /// The raw I/Q samples.
    samples: Vec<i16>,
    /// The sample index the message was found at.
    ndx: usize,
    /// The signal to noise ratio computed.
    snr: f32,
    /// The theta used during beamforming.
    theta: f32,
    /// The amplitude from antenna A used during beamforming.
    amplitude_a: f32,
    /// The amplitude from antenna B used during beamforming.
    amplitude_b: f32,
}

impl fmt::Debug for MessageCommon {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        f.debug_struct("MessageCommon").finish()
    }
}

/// Represents a message after demodulation and decoding.
#[derive(Debug)]
struct Message {
    common: MessageCommon,
    /// Any data specific to this message. For example, this
    /// might contain fields specific to a message type.
    specific: MessageSpecific,
}

#[derive(Debug)]
struct DfHeader1 {
    capability: u8,
    addr: u32,
    metype: u8,
    mesub: u8,
}

/// This is a good place to put anything specific if any
/// decoding was done on the message. You could put fields
/// specific to each message type here.
#[derive(Debug)]
enum MessageSpecific {
    AircraftIdenAndCat {
        hdr: DfHeader1,
        aircraft_type: u8,
        flight: Vec<char>,
    },
    SurfacePositionMessage {
        hdr: DfHeader1,
        movement: u8,
        ground_track: u8,
        f_flag: bool,
        t_flag: bool,
        raw_lat: u32,
        raw_lon: u32,
    },
    AirbornePositionMessage {
        hdr: DfHeader1,
        f_flag: bool,
        t_flag: bool,
        altitude: f32,
        raw_lat: u32,
        raw_lon: u32,
    },
    AirborneVelocityMessage {
        hdr: DfHeader1,
        ew_dir: u8,
        ew_velocity: u16,
        ns_dir: u8,
        ns_velocity: u16,
        vert_rate_source: u8,
        vert_rate_sign: u8,
        vert_rate: u16,
        velocity: f32,
        heading: f32,
    },
    AirborneVelocityMessageShort {
        hdr: DfHeader1,
        heading: f32,
    },
    Other,
}

enum MessageErrorReason {
    /// This happens when the message can not be decoded because of errors.
    BitErrors,
}

fn decode_ac12_field(msg: &[u8]) -> f32 {
    if msg[5] & 1 == 1 {
        let n = (((msg[5] as u32) >> 1) << 4) | ((msg[6] as u32 & 0xf0) >> 4);
        n as f32 * 25.0 - 1000.0
    } else {
        0.0f32
    }
}

/// Process the stream result and do any decoding that is needed.
fn process_result(result: stream::ProcessStreamResult, bit_error_table: &HashMap<u32, u16>) -> Result<Message, MessageErrorReason> {
    let mut msg = result.msg;

    let is_long: bool = ((msg[0] >> 3) & 0x10) == 0x10;

    match is_long {
        true => (),
        false => {
            while msg.len() > MODES_SHORT_MSG_BYTES {
                msg.pop();
            }
        },
    }

    let msgtype = msg[0] >> 3;
    let mut crc_syndrome = crc::modes_checksum(&msg);
    let mut crc_ok = crc_syndrome == 0u32;

    if !crc_ok && (msgtype == 11 || msgtype == 17 || msgtype == 18) {
        let nfixed = crc::fix_bit_errors(&mut msg, bit_error_table);
        
        if nfixed == 0 {
            return Err(MessageErrorReason::BitErrors);
        }

        crc_syndrome = crc::modes_checksum(&msg);
        crc_ok = crc_syndrome == 0;
    }

    // Here is where you want to do your decoding. I just simply
    // copied the parameters over and set a marker for each message
    // type.

    let ca = msg[0] & 7;
    let aa1 = msg[1];
    let aa2 = msg[2];
    let aa3 = msg[3];
    let metype = msg[4] >> 3;
    let mesub = msg[4] & 7;
    let fs = msg[0] & 7;
    let dr = msg[1] >> 3 & 31;
    let um = ((msg[1] & 7) << 3) | (msg[2] >> 5);

    let addr = ((msg[1] as u32) << 16) | ((msg[2] as u32) << 8) | msg[3] as u32;

    let identity: u32;
    {
        let a = ((msg[3] & 0x80) >> 5) |
                ((msg[2] & 0x02) >> 0) |
                ((msg[2] & 0x08) >> 3);
        let b = ((msg[3] & 0x02) << 1) |
                ((msg[3] & 0x08) >> 2) |
                ((msg[3] & 0x20) >> 5);
        let c = ((msg[2] & 0x01) << 2) |
                ((msg[2] & 0x04) >> 1) |
                ((msg[2] & 0x10) >> 4);
        let d = ((msg[3] & 0x01) << 2) |
                ((msg[3] & 0x04) >> 1) |
                ((msg[3] & 0x10) >> 4);
        identity = a as u32 * 1000 + b as u32 * 100 + c as u32 * 10 + d as u32;
    }

    let common = MessageCommon {
        msg: msg.clone(),
        ndx: result.ndx,
        snr: result.snr,
        theta: result.theta,
        samples: result.samples,
        amplitude_a: result.amplitude_a,
        amplitude_b: result.amplitude_b,
    };

    match msgtype {
        17 | 18 => {
            let hdr = DfHeader1 {
                capability: ca,
                addr: addr,
                metype: metype,
                mesub: mesub,
            };

            if metype >= 1 && metype <= 4 {
                let ais_charset_unicode = "?ABCDEFGHIJKLMNOPQRSTUVWXYZ????? ???????????????0123456789??????";
                let ais_charset: Vec<char> = ais_charset_unicode.chars().collect();
                let f0 = ais_charset[(msg[5] as usize) >> 2];
                let f1 = ais_charset[((msg[5] as usize & 3) << 4) | (msg[6] as usize >> 4)];
                let f2 = ais_charset[((msg[6] as usize & 15) << 2) | (msg[7] as usize >> 6)];
                let f3 = ais_charset[msg[7] as usize & 63];
                let f4 = ais_charset[msg[8] as usize >> 2];
                let f5 = ais_charset[((msg[8] as usize & 3) << 4) | (msg[9] as usize >> 4)];
                let f6 = ais_charset[((msg[9] as usize & 15) << 2) | (msg[10] as usize >> 6)];
                let f7 = ais_charset[msg[10] as usize & 63];
                Ok(Message {
                    common: common,
                    specific: MessageSpecific::AircraftIdenAndCat {
                        hdr: hdr,
                        aircraft_type: metype - 1,
                        flight: vec![f0, f1, f2, f3, f4, f5, f6, f7],
                    }
                })
            } else if metype >=5 && metype <= 8 {
                Ok(Message {
                    common: common,
                    specific: MessageSpecific::SurfacePositionMessage {
                        hdr: hdr,
                        movement: ((msg[4] & 0x07) << 4) | (msg[5] >> 4),
                        ground_track: ((msg[5] & 0x07) << 4) | (msg[6] >> 4),
                        f_flag: (msg[6] >> 2) & 1 == 1,
                        t_flag: (msg[6] >> 3) & 1 == 1,
                        raw_lat: ((msg[6] as u32 & 3) << 15) |
                                 ((msg[7] as u32) << 7) |
                                 ((msg[8] as u32) >> 1),
                        raw_lon: (((msg[8] as u32) & 1) << 16) |
                                 ((msg[9] as u32) << 8) |
                                 msg[10] as u32,
                    },
                })
            } else if metype >= 9 && metype <= 18 {
                Ok(Message {
                    common: common,
                    specific: MessageSpecific::AirbornePositionMessage {
                        hdr: hdr,
                        t_flag: msg[6] & (1 << 2) != 0,
                        f_flag: msg[6] & (1 << 3) != 0,
                        altitude: decode_ac12_field(&msg),
                        raw_lat: ((msg[6] as u32 & 3) << 15) |
                                 ((msg[7] as u32) << 7) |
                                 (msg[8] as u32 >> 1),
                        raw_lon: ((msg[8] as u32 & 1) << 16) |
                                 ((msg[9] as u32) << 8) |
                                 msg[10] as u32,
                    },
                })
            } else if metype == 19 && mesub >= 1 && mesub <= 4 {
                if mesub == 1 || mesub == 2 {
                    let ew_dir: u8 = (msg[5] & 4) >> 2;
                    let ew_velocity = ((msg[5] as u16 & 3) << 8) | msg[6] as u16;
                    let ns_dir: u8 = (msg[7] & 0x80) >> 7;
                    let ns_velocity = ((msg[7] as u16 & 0x7f) << 3) | ((msg[8] as u16 & 0xe0) >> 5);
                    let vert_rate_source = (msg[8] & 0x10) >> 4;
                    let vert_rate_sign = (msg[8] & 0x8) >> 3;
                    let vert_rate = ((msg[8] as u16 & 7) << 6) | ((msg[9] as u16 & 0xfc) >> 2);
                    let velocity = (
                        ew_velocity as f32 * ew_velocity as f32 + 
                        ns_velocity as f32 * ns_velocity as f32
                    ).sqrt();
                    let mut heading;

                    if velocity > 0.0 {
                        let mut ewv = ew_velocity as f32;
                        let mut nsv = ns_velocity as f32;
                        if ew_dir == 1 {
                            ewv *= -1.0;
                        }
                        if ns_dir == 1 {
                            nsv *= -1.0;
                        }

                        heading = ewv.atan2(nsv) * 360.0 / (std::f32::consts::PI * 2.0);
                        if heading < 0.0 {
                            heading += 360.0;
                        }
                    } else {
                        heading = 0.0;
                    }

                    Ok(Message {
                        common: common,
                        specific: MessageSpecific::AirborneVelocityMessage {
                            hdr: hdr,
                            ew_dir: ew_dir,
                            ew_velocity: ew_velocity,
                            ns_dir: ns_dir,
                            ns_velocity: ns_velocity,
                            vert_rate_source: vert_rate_source,
                            vert_rate_sign: vert_rate_sign,
                            vert_rate: vert_rate,
                            velocity: velocity,
                            heading: heading,
                        },
                    })
                } else if mesub == 3 || mesub == 4 {
                    let heading = (360.0 / 128.0) * 
                                  ((((msg[5] as u16 & 3) << 5) |
                                  (msg[6] as u16 >> 3))) as f32;
                    Ok(Message {
                        common: common,
                        specific: MessageSpecific::AirborneVelocityMessageShort {
                            hdr: hdr,
                            heading: heading,
                        },
                    })
                } else {
                    Ok(Message {
                        common: common,
                        specific: MessageSpecific::Other,
                    })                    
                }
            } else {
                Ok(Message {
                    common: common,
                    specific: MessageSpecific::Other,
                })                
            }

        },
        _ => Ok(Message {
            common: common,
            specific: MessageSpecific::Other,
        }),
    }
}

fn write_message_to_file(file: &mut File, m: &Message) {
    file.write_all(bytes_of(&(m.common.msg.len() as u16))).unwrap();
    file.write_all(&m.common.msg).unwrap();
    file.write_all(bytes_of(&(m.common.samples.len() as u16))).unwrap();
    for x in 0..m.common.samples.len() {
        file.write_all(bytes_of(&m.common.samples[x])).unwrap();
    }
    file.write_all(bytes_of(&m.common.ndx)).unwrap();
    file.write_all(bytes_of(&m.common.snr)).unwrap();
    file.write_all(bytes_of(&m.common.theta)).unwrap();
    file.write_all(bytes_of(&m.common.amplitude_a)).unwrap();
    file.write_all(bytes_of(&m.common.amplitude_b)).unwrap();
}

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
}

fn main() {
    let args = Args::parse();

    let thread_count: u32 = args.thread_count;
    let cycle_count: u32 = args.cycle_count;

    println!("Hello, world!");

    println!("Using {} threads and {} cycles.", thread_count, cycle_count);

    let server_addr = "127.0.0.1:7878";

    let mut txs: Vec<Sender<Vec<u8>>> = Vec::new();
    let mut rxs: Vec<Receiver<Vec<Message>>> = Vec::new();

    for _ in 0..thread_count {
        let (atx, brx) = channel();
        let (btx, arx) = channel();
        txs.push(atx);
        rxs.push(arx);

        thread::spawn(move || {
            println!("spawned");
            let bit_error_table = crc::modes_init_error_info();
            loop {
                let buffer = brx.recv().unwrap();
                btx.send(stream::process_buffer(&buffer, &bit_error_table, cycle_count)).unwrap();
            }
        });
    }

    let mut file = match args.file_output {
        Some(v) => {
            Some(File::create(v).unwrap())
        },
        None => None,
    };

    let mut df11total: usize = 0;
    let mut df17total: usize = 0;
    let mut df18total: usize = 0;
    let mut dfothertotal: usize = 0;
    let mut dftotal: usize = 0;
    let stat_start = Instant::now();
    let mut stat_display = Instant::now();

    match TcpStream::connect(server_addr) {
        Ok(mut stream) => {
            println!("connected");
            // We are expecting TWO interleaved streams from TWO antennas.
            let mut buffer: Vec<u8> = vec![0; MODES_LONG_MSG_SAMPLES * 1024 * 8];
            let mut read: usize = 0;

            let sps: f64 = 2e6f64;
            let buffer_time: f64 = buffer.len() as f64 / 8.0f64 /  sps;
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

                        let mut hm: HashMap<usize, Message> = HashMap::new();
                        
                        for tx in &txs {
                            // Send buffer to one thread.
                            tx.send(buffer.clone()).unwrap();
                        }

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

                        let mut items: Vec<(usize, Message)> = hm.into_iter().collect();
                        items.sort_by(|a, b| (&a.0).cmp(&b.0));

                        // Now, `hm` contains a list of final messages that are potentially
                        // unordered. Do some statistical output to validate proof of concept.
                        for (_, message) in items {
                            dftotal += 1;
                            /*
                            match message.specific {
                                MessageSpecific::Df11 => { df11total += 1; },
                                MessageSpecific::Df17 => { df17total += 1; },
                                MessageSpecific::Df18 => { df18total += 1; },
                                MessageSpecific::Other => { dfothertotal += 1; },
                            }
                            */

                            // Other generates too much because it isn't error checked.
                            match message.specific {
                                MessageSpecific::Other => (),
                                MessageSpecific::AircraftIdenAndCat {
                                    hdr: _,
                                    aircraft_type: _,
                                    flight: _,
                                } => {
                                    println!("{:?}", message);
                                },
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
                        
                        if stat_display.elapsed().as_secs() > 5 {
                            let elapsed_dur: Duration = stat_start.elapsed();
                            let elapsed = elapsed_dur.as_secs() as f64 + elapsed_dur.subsec_micros() as f64 / 1e6f64;
                            // Do per second calculations.
                            let df11ps = df11total as f64 / elapsed;
                            let df17ps = df17total as f64 / elapsed;
                            let df18ps = df18total as f64 / elapsed;
                            let dfotherps = dfothertotal as f64 / elapsed;
                            let dftotalps = dftotal as f64 / elapsed;

                            println!("========= BOTH ANTENNAS =============");
                            println!(" DF11: {:.1} {}", df11ps, df11total);
                            println!(" DF17: {:.1} {}", df17ps, df17total);
                            println!(" DF18: {:.1} {}", df18ps, df18total);
                            println!("OTHER: {:.1} {}", dfotherps, dfothertotal);
                            println!("ALL  : {:.1} {}", dftotalps, dftotal);
                            stat_display = Instant::now();
                        }
                        
                        {
                            let elapsed_dur: Duration = start.elapsed();
                            let elapsed = elapsed_dur.as_secs() as f64 + elapsed_dur.subsec_micros() as f64 / 1e6f64; 
                            if elapsed > buffer_time * 0.95 {
                                println!("elapsed:{} buffer_time:{} TOO SLOW!!! REDUCE CYCLES!!!", elapsed, buffer_time);
                            } else {
                                println!("elapsed:{} buffer_time:{}", elapsed, buffer_time);
                            }
                        }

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