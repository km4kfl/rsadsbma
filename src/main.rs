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

mod crc;
mod constants;
mod stream;

use constants::*;

/// Represents a message after demodulation and decoding.
struct Message {
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
    /// Any data specific to this message. For example, this
    /// might contain fields specific to a message type.
    specific: MessageSpecific,
}

/// This is a good place to put anything specific if any
/// decoding was done on the message. You could put fields
/// specific to each message type here.
enum MessageSpecific {
    Df11,
    Df18,
    Df17,
    Other,
}

enum MessageErrorReason {
    /// This happens when the message can not be decoded because of errors.
    BitErrors,
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
    let crc_syndrome = crc::modes_checksum(&msg);
    let crc_ok = crc_syndrome == 0u32;

    if !crc_ok && (msgtype == 11 || msgtype == 17 || msgtype == 18) {
        let nfixed = crc::fix_bit_errors(&mut msg, bit_error_table);
        
        if nfixed == 0 {
            return Err(MessageErrorReason::BitErrors);
        }
    }

    // Here is where you want to do your decoding. I just simply
    // copied the parameters over and set a marker for each message
    // type.

    match msgtype {
        11 => Ok(Message {
            msg: msg,
            ndx: result.ndx,
            snr: result.snr,
            theta: result.theta,
            samples: result.samples,
            amplitude_a: result.amplitude_a,
            amplitude_b: result.amplitude_b,
            specific: MessageSpecific::Df11,
        }),
        17 => Ok(Message {
            msg: msg,
            ndx: result.ndx,
            snr: result.snr,
            theta: result.theta,
            samples: result.samples,
            amplitude_a: result.amplitude_a,
            amplitude_b: result.amplitude_b,        
            specific: MessageSpecific::Df17,
        }),
        18 => Ok(Message {
            msg: msg,
            ndx: result.ndx,
            snr: result.snr,
            theta: result.theta,
            samples: result.samples,
            amplitude_a: result.amplitude_a,
            amplitude_b: result.amplitude_b,        
            specific: MessageSpecific::Df18,
        }),
        _ => Ok(Message {
            msg: msg,
            ndx: result.ndx,
            snr: result.snr,
            theta: result.theta,
            samples: result.samples,
            amplitude_a: result.amplitude_a,
            amplitude_b: result.amplitude_b,        
            specific: MessageSpecific::Other,
        }),
    }
}

fn write_message_to_file(file: &mut File, m: &Message) {
    file.write_all(bytes_of(&(m.msg.len() as u16))).unwrap();
    file.write_all(&m.msg).unwrap();
    file.write_all(bytes_of(&(m.samples.len() as u16))).unwrap();
    for x in 0..m.samples.len() {
        file.write_all(bytes_of(&m.samples[x])).unwrap();
    }
    file.write_all(bytes_of(&m.ndx)).unwrap();
    file.write_all(bytes_of(&m.snr)).unwrap();
    file.write_all(bytes_of(&m.theta)).unwrap();
    file.write_all(bytes_of(&m.amplitude_a)).unwrap();
    file.write_all(bytes_of(&m.amplitude_b)).unwrap();
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
                                match hm.get(&message.ndx) {
                                    Some(other) => {
                                        // Compare the SNR (signal to noise) ratio
                                        // and replace the existing if better.
                                        if other.snr < message.snr {
                                            hm.insert(message.ndx, message);    
                                        }
                                    },
                                    None => {
                                        // This was the first time we saw a message at
                                        // this `message.ndx` (index) in the sample
                                        // stream.
                                        hm.insert(message.ndx, message);
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
                            match message.specific {
                                MessageSpecific::Df11 => { df11total += 1; },
                                MessageSpecific::Df17 => { df17total += 1; },
                                MessageSpecific::Df18 => { df18total += 1; },
                                MessageSpecific::Other => { dfothertotal += 1; },
                            }

                            // Other generates too much because it isn't error checked.
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
