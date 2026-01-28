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
use bytemuck::{bytes_of, cast_slice};
use rand::Rng;
use std::time::{Duration, Instant};
use std::thread;
use std::sync::mpsc::{channel, Sender, Receiver};
use std::collections::HashMap;
use std::result::Result;
use clap::Parser;
use std::fs::File;
use std::io::prelude::*;

const MODES_PREAMBLE_US: usize =  8;
const MODES_PREAMBLE_SAMPLES: usize = MODES_PREAMBLE_US * 2;
const MODES_LONG_MSG_BYTES: usize = 14;
const MODES_SHORT_MSG_BYTES: usize = 7;
const MODES_LONG_MSG_BITS: usize = MODES_LONG_MSG_BYTES * 8;
//const MODES_SHORT_MSG_BITS: usize = MODES_SHORT_MSG_BYTES * 8;
const MODES_LONG_MSG_SAMPLES: usize = MODES_LONG_MSG_BITS * 2;
//const MODES_SHORT_MSG_SAMPLES: usize = MODES_LONG_MSG_BITS * 2;

const MODES_CHECKSUM_TABLE: [u32; 112] = [
    0x3935ea, 0x1c9af5, 0xf1b77e, 0x78dbbf, 0xc397db, 0x9e31e9, 0xb0e2f0, 0x587178,
    0x2c38bc, 0x161c5e, 0x0b0e2f, 0xfa7d13, 0x82c48d, 0xbe9842, 0x5f4c21, 0xd05c14,
    0x682e0a, 0x341705, 0xe5f186, 0x72f8c3, 0xc68665, 0x9cb936, 0x4e5c9b, 0xd8d449,
    0x939020, 0x49c810, 0x24e408, 0x127204, 0x093902, 0x049c81, 0xfdb444, 0x7eda22,
    0x3f6d11, 0xe04c8c, 0x702646, 0x381323, 0xe3f395, 0x8e03ce, 0x4701e7, 0xdc7af7,
    0x91c77f, 0xb719bb, 0xa476d9, 0xadc168, 0x56e0b4, 0x2b705a, 0x15b82d, 0xf52612,
    0x7a9309, 0xc2b380, 0x6159c0, 0x30ace0, 0x185670, 0x0c2b38, 0x06159c, 0x030ace,
    0x018567, 0xff38b7, 0x80665f, 0xbfc92b, 0xa01e91, 0xaff54c, 0x57faa6, 0x2bfd53,
    0xea04ad, 0x8af852, 0x457c29, 0xdd4410, 0x6ea208, 0x375104, 0x1ba882, 0x0dd441,
    0xf91024, 0x7c8812, 0x3e4409, 0xe0d800, 0x706c00, 0x383600, 0x1c1b00, 0x0e0d80,
    0x0706c0, 0x038360, 0x01c1b0, 0x00e0d8, 0x00706c, 0x003836, 0x001c1b, 0xfff409,
    0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
    0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
    0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000
];

fn modes_compute_crc(msg: &[u8]) -> u32 {
    let bits: usize = msg.len() * 8;

    let offset: usize = if bits == 112 {
        0
    } else {
        112 - 56
    };

    let mut crc: u32 = 0u32;

    for j in 0..bits - 24 {
        let bytendx: usize = j / 8;
        let bitndx: usize = j % 8;
        let bitmask: u8 = 1 << (7 - bitndx);

        if msg[bytendx] & bitmask != 0 {
            crc = crc ^ MODES_CHECKSUM_TABLE[offset + j];
        }
    }

    crc & 0xffffff
}

fn modes_checksum(msg: &[u8]) -> u32 {
    let crc = modes_compute_crc(msg);
    let sz = msg.len();
    let rem = ((msg[sz - 3] as u32) << 16) | ((msg[sz - 2] as u32) << 8) | msg[sz - 1] as u32;
    return (crc ^ rem) & 0xffffff;
}

fn modes_init_error_info() -> HashMap<u32, u16> {
    let mut msg: Vec<u8> = vec![0; MODES_LONG_MSG_BYTES];
    let mut bit_error_table = HashMap::new();

    for i in 5..MODES_LONG_MSG_BITS {
        let bytepos0: usize = i >> 3;
        let mask0: u8 = 1 << (7 - (i & 7));
        msg[bytepos0] = msg[bytepos0] & mask0;
        let crc0 = modes_checksum(&msg);

        bit_error_table.insert(crc0, i as u16);

        for j in i + 1..MODES_LONG_MSG_BITS {
            let bytepos1: usize = j >> 3;
            let mask1: u8 = 1 << (7 - (j & 7));
            msg[bytepos1] = msg[bytepos1] ^ mask1;
            let crc1 = modes_checksum(&msg);

            bit_error_table.insert(crc1, i as u16 | ((j as u16) << 8));

            msg[bytepos1] = msg[bytepos1] & mask1;
        }

        msg[bytepos0] = msg[bytepos0] & mask0;
    }

    bit_error_table
}

fn fix_bit_errors(msg: &mut [u8], bit_error_table: &HashMap<u32, u16>) -> u8 {
    let syndrome = modes_checksum(msg);
    let offset: usize = MODES_LONG_MSG_BITS - msg.len() * 8;
    match bit_error_table.get(&syndrome) {
        Some(pei) => {
            let a = (pei & 0xff) as usize;
            let b = ((pei >> 8) & 0xff) as usize;

            if b != 0 {
                if offset > a {
                    return 0;
                }
                if offset > b {
                    return 0;
                }
                let bitpos0 = a - offset;
                let bitpos1 = a - offset;
                msg[bitpos0 >> 3] = msg[bitpos0 >> 3] ^ (1 << (7 - (bitpos0 & 7)));
                msg[bitpos1 >> 3] = msg[bitpos1 >> 3] ^ (1 << (7 - (bitpos1 & 7)));
                2
            } else {
                if offset > a {
                    return 0;
                }
                let bitpos0 = a - offset;
                msg[bitpos0 >> 3] = msg[bitpos0 >> 3] ^ (1 << (7 - (bitpos0 & 7)));
                1
            }
        },
        None => 0,
    }
}

/// Represents a message after demodulation but before decoding.
struct ProcessStreamResult {
    snr: f32,
    msg: Vec<u8>,
    samples: Vec<i16>,
    ndx: usize,
    theta: f32,
    amplitude_a: f32,
    amplitude_b: f32,
}

/// Demodulates a message using `stream`.
///
/// The `i16stream`, `theta`, `amplitude_a`, and `amplitude_b` are simply copied
/// into the `ProcessStreamResult`. There is some offset magic with multiple of 4
/// with `i16stream` but nothing major.
///
/// The `stream` is expected to be the magnitude of the incoming I/Q stream. Likely,
/// this function is called after performing the beamforming and combining the two
/// antenna sample streams into a magnitude stream.
fn process_stream_mfloat32(
    stream: &[f32],
    i16stream: &[i16],
    theta: f32,
    amplitude_a: f32,
    amplitude_b: f32) -> Vec<ProcessStreamResult> {
    let mut results: Vec<ProcessStreamResult> = Vec::new();

    for x in 0..stream.len() - MODES_PREAMBLE_SAMPLES - MODES_LONG_MSG_SAMPLES - 1 {
        let snr: f32;

        {
            let p = &stream[x..x + MODES_PREAMBLE_SAMPLES];
            let valid: bool = (p[0] > p[1]) && (p[1] < p[2]) && (p[2] > p[3]) && (p[3] < p[0]) && 
                            (p[4] < p[0]) && (p[5] < p[0]) && (p[6] < p[0]) && (p[7] > p[8]) &&
                            (p[8] < p[9]) && (p[9] > p[6]);
            if !valid {
                continue;
            }

            let high: f32 = (p[0] + p[2] + p[7] + p[9]) / 6.0f32;

            if (p[4] >= high) || (p[5] >= high) {
                continue;
            }

            if (p[11] > high) || (p[12] > high) || (p[13] > high) || (p[14] > high) {
                continue;
            }

            snr = (p[0] - p[1]) + (p[2] - p[3]) + (p[7] - p[6]) + (p[9] - p[8]);
        }

        let payload = &stream[x + MODES_PREAMBLE_SAMPLES..x + MODES_PREAMBLE_SAMPLES + MODES_LONG_MSG_SAMPLES];   

        let mut thebyte: u8 = 0;
        let mut msg: Vec<u8> = Vec::new();

        for y in 0..MODES_LONG_MSG_SAMPLES / 2 {
            let a: f32 = payload[y * 2 + 0];
            let b: f32 = payload[y * 2 + 1];
            
            if a > b {
                thebyte |= 1;
            }

            if y & 7 == 7 {
                msg.push(thebyte);
            }

            thebyte = thebyte << 1;
        }

        results.push(ProcessStreamResult {
            snr: snr,
            msg: msg,
            samples: (&i16stream[x * 4..x * 4 + (MODES_PREAMBLE_SAMPLES + MODES_LONG_MSG_SAMPLES) * 4]).to_vec(),
            ndx: x,
            theta: theta,
            amplitude_a: amplitude_a,
            amplitude_b: amplitude_b,
        });
    }

    results
}

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
fn process_result(result: ProcessStreamResult, bit_error_table: &HashMap<u32, u16>) -> Result<Message, MessageErrorReason> {
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
    let crc_syndrome = modes_checksum(&msg);
    let crc_ok = crc_syndrome == 0u32;

    if !crc_ok && (msgtype == 11 || msgtype == 17 || msgtype == 18) {
        let nfixed = fix_bit_errors(&mut msg, bit_error_table);
        
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

/// Does a single beamforming operation on the interleaved two antenna stream.
///
/// This function expects that `u8_buffer` is a stream of `i16` values where
/// the I/Q pairs for the two antennas are interleaved. Such that they are in
/// the format: ABCDABCDABCDABCD...
///
/// Where A is the real value (I) of antenna 1, B is the imaginary value (Q) of
/// antenna 1, C is the real value (I) of antenna 2, and D is the imaginary value (Q)
/// of antenna 2. The pattern just continues. Therefore, the function expects
/// the length to a multiple of 8. However, any odd bytes are just ignored.
fn process_buffer_single(
    u8_buffer: &[u8],
    bit_error_table: &HashMap<u32, u16>,
    theta: f32,
    amplitude_a: f32,
    amplitude_b: f32
) -> Vec<Message> {
    let buffer: &[i16] = cast_slice(u8_buffer);
    let mut mbuffer: Vec<f32> = Vec::with_capacity(buffer.len() / 4);
    let ri = theta.cos();
    let rq = theta.sin();
    for x in 0..buffer.len() / 4 {
        let ai: f32 = buffer[x*4+0] as f32 / 2049.0 * amplitude_a;
        let aq: f32 = buffer[x*4+1] as f32 / 2049.0 * amplitude_a;
        let bi: f32 = buffer[x*4+2] as f32 / 2049.0 * amplitude_b;
        let bq: f32 = buffer[x*4+3] as f32 / 2049.0 * amplitude_b;
        // multiply two complex numbers
        // a = bi
        // b = bq
        // c = ri
        // d = rq
        // (a + ib)(c + id) = (ac - bd) + i(ad + bc)
        // rotates b then adds b to a
        let ci = bi * ri - bq * rq + ai;
        let cq = bi * rq + bq * ri + aq;

        mbuffer.push((ci * ci + cq * cq).sqrt());
    }

    let results = process_stream_mfloat32(&mbuffer, &buffer, theta, amplitude_a, amplitude_b);

    let mut out: Vec<Message> = Vec::new();

    for result in results {
        match process_result(result, bit_error_table) {
            Ok(message) => out.push(message),
            Err(_) => (),
        }        
    }

    out
}

fn process_buffer(u8_buffer: &[u8], bit_error_table: &HashMap<u32, u16>, cycle_count: u32) -> Vec<Message> {
    let buffer: &[i16] = cast_slice(u8_buffer);
    let mut mbuffer: Vec<f32> = Vec::with_capacity(buffer.len() / 4);
    let mut rng = rand::thread_rng();
    let mut hm: HashMap<usize, ProcessStreamResult> = HashMap::new();

    for _ in 0..cycle_count {
        let theta: f32 = rng.r#gen::<f32>() * std::f32::consts::PI * 2.0f32 - std::f32::consts::PI;
        let amplitude: f32 = rng.r#gen();
        let ri = theta.cos();
        let rq = theta.sin();
        for x in 0..buffer.len() / 4 {
            let ai: f32 = buffer[x*4+0] as f32 / 2049.0;
            let aq: f32 = buffer[x*4+1] as f32 / 2049.0;
            let bi: f32 = buffer[x*4+2] as f32 / 2049.0 * amplitude;
            let bq: f32 = buffer[x*4+3] as f32 / 2049.0 * amplitude;
            // multiply two complex numbers
            // a = bi
            // b = bq
            // c = ri
            // d = rq
            // (a + ib)(c + id) = (ac - bd) + i(ad + bc)
            // rotates b then adds b to a
            let ci = bi * ri - bq * rq + ai;
            let cq = bi * rq + bq * ri + aq;

            mbuffer.push((ci * ci + cq * cq).sqrt());
        }

        let results = process_stream_mfloat32(&mbuffer, &buffer, theta, 1.0, amplitude);

        for result in results {
            match hm.get(&result.ndx) {
                Some(other) => {
                    if other.snr < result.snr {
                        hm.insert(result.ndx, result);
                    }
                },
                None => {
                    hm.insert(result.ndx, result);
                },
            }
        }

        mbuffer.clear();
    }

    let mut out: Vec<Message> = Vec::new();

    for (_, result) in hm {
        match process_result(result, bit_error_table) {
            Ok(message) => out.push(message),
            Err(_) => (),
        }
    }

    out
}

fn write_message_to_file(file: &mut File, m: &Message) {
    file.write_all(bytes_of(&(m.msg.len() as u16)));
    file.write_all(&m.msg);
    file.write_all(bytes_of(&(m.samples.len() as u16)));
    for x in 0..m.samples.len() {
        file.write_all(bytes_of(&m.samples[x]));
    }
    file.write_all(bytes_of(&m.ndx));
    file.write_all(bytes_of(&m.snr));
    file.write_all(bytes_of(&m.theta));
    file.write_all(bytes_of(&m.amplitude_a));
    file.write_all(bytes_of(&m.amplitude_b));
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
            let bit_error_table = modes_init_error_info();
            loop {
                let buffer = brx.recv().unwrap();
                btx.send(process_buffer(&buffer, &bit_error_table, cycle_count)).unwrap();
            }
        });
    }

    let mut file = match args.file_output {
        Some(v) => {
            Some(File::create(v).unwrap())
        },
        None => None,
    };

    //for rx in &rxs {
    //    rx.recv().unwrap();
    //}
    let bit_error_table = modes_init_error_info();

    let mut df11total_a: usize = 0;
    let mut df17total_a: usize = 0;
    let mut df18total_a: usize = 0;
    let mut dfothertotal_a: usize = 0;
    let mut dftotal_a: usize = 0;

    let mut df11total_b: usize = 0;
    let mut df17total_b: usize = 0;
    let mut df18total_b: usize = 0;
    let mut dfothertotal_b: usize = 0;
    let mut dftotal_b: usize = 0;

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

                        // While we wait for the threads to finished process the buffer and only
                        // turn on antenna A.
                        for message in process_buffer_single(&buffer, &bit_error_table, 0.0, 1.0, 0.0) {
                            dftotal_a += 1;
                            match message.specific {
                                MessageSpecific::Df11 => { df11total_a += 1; },
                                MessageSpecific::Df17 => { df17total_a += 1; },
                                MessageSpecific::Df18 => { df18total_a += 1; },
                                MessageSpecific::Other => { dfothertotal_a += 1; },
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
                        
                        // Only turn on antenna B.
                        for message in process_buffer_single(&buffer, &bit_error_table, 0.0, 0.0, 1.0) {
                            dftotal_b += 1;
                            match message.specific {
                                MessageSpecific::Df11 => { df11total_b += 1; },
                                MessageSpecific::Df17 => { df17total_b += 1; },
                                MessageSpecific::Df18 => { df18total_b += 1; },
                                MessageSpecific::Other => { dfothertotal_b += 1; },
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

                        // Now, `hm` contains a list of final messages that are potentially
                        // unordered. Do some statistical output to validate proof of concept.
                        for (_, message) in hm {
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

                            let df11ps_a = df11total_a as f64 / elapsed;
                            let df17ps_a = df17total_a as f64 / elapsed;
                            let df18ps_a = df18total_a as f64 / elapsed;
                            let dfotherps_a = dfothertotal_a as f64 / elapsed;
                            let dftotalps_a = dftotal_a as f64 / elapsed;

                            let df11ps_b = df11total_b as f64 / elapsed;
                            let df17ps_b = df17total_b as f64 / elapsed;
                            let df18ps_b = df18total_b as f64 / elapsed;
                            let dfotherps_b = dfothertotal_b as f64 / elapsed;
                            let dftotalps_b = dftotal_b as f64 / elapsed;                            

                            println!("=========   ANTENNA A   =============");
                            println!(" DF11: {:.1} {}", df11ps_a, df11total_a);
                            println!(" DF17: {:.1} {}", df17ps_a, df17total_a);
                            println!(" DF18: {:.1} {}", df18ps_a, df18total_a);
                            println!("OTHER: {:.1} {}", dfotherps_a, dfothertotal_a);
                            println!("ALL  : {:.1} {}", dftotalps_a, dftotal_a);
                            println!("=========   ANTENNA B   =============");
                            println!(" DF11: {:.1} {}", df11ps_b, df11total_b);
                            println!(" DF17: {:.1} {}", df17ps_b, df17total_b);
                            println!(" DF18: {:.1} {}", df18ps_b, df18total_b);
                            println!("OTHER: {:.1} {}", dfotherps_b, dfothertotal_b);
                            println!("ALL  : {:.1} {}", dftotalps_b, dftotal_b);
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
