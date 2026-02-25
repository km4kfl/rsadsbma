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

    /// Number of cycles per thread. If not specified uses LMS beamformer instead of random.
    #[arg(short, long)]
    cycle_count: Option<u32>,

    /// Either a file path or a TCP/IP "address:port". Defaults to "localhost:7878".
    #[arg(long)]
    input_source: Option<String>,

    // If using file input this specifies the number of streams, otherwise the count is read from the source.
    #[arg(short, long)]
    stream_count: Option<usize>,

    /// A file to write messages.
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
    #[clap(default_value_t = 1.0)]
    snr_scaler: f32,

    /// The depth or count of the items used in the rolling average calculation for the tracking steering vector.
    #[arg(short, long)]
    #[clap(default_value_t = 1)]
    weighted_avg_depth: usize,

    /// If set this will cause the amplitudes to be randomized helping to change the beam pattern.
    #[arg(short, long)]
    #[clap(default_value_t = false)]
    randomize_amplitudes: bool,

    /// The `mu` for the LMS beamformer.
    #[arg(short, long)]
    #[clap(default_value_t = 0.002)]
    mu: f32,

    /// By default don't check the preamble for the LMS beamformer.
    #[arg(short, long)]
    #[clap(default_value_t = false)]
    check_preamble: bool,
}

/// Supports `InputMultiplexor`.
enum InputMultiplexorCore {
    TcpStream(TcpStream),
    File(File),
}

/// Treats a file or TCP socket as the same.
///
/// It only supports the operation `read` at the moment.
struct InputMultiplexor {
    core: InputMultiplexorCore,
}

impl InputMultiplexor {
    fn from_open(path: &str) -> Result<InputMultiplexor, String> {
        match File::open(path) {
            Ok(file) => Ok(InputMultiplexor {
                core: InputMultiplexorCore::File(file),
            }),
            Err(error) => Err(error.to_string()),
        }
    }

    fn from_socket(server_addr: &str) -> Result<InputMultiplexor, String> {
        match TcpStream::connect(server_addr) {
            Ok(stream) => Ok(InputMultiplexor {
                core: InputMultiplexorCore::TcpStream(stream),
            }),
            Err(error) => Err(error.to_string()),
        }
    }

    fn from_file(&self) -> bool {
        match &self.core {
            InputMultiplexorCore::TcpStream(_) => false,
            InputMultiplexorCore::File(_) => true,
        }
    }

    fn read(&mut self, buffer: &mut [u8]) -> Result<usize, String> {
        match &mut self.core {
            InputMultiplexorCore::TcpStream(stream) => {
                match stream.read(buffer) {
                    Ok(bytes_read) => Ok(bytes_read),
                    Err(error) => Err(error.to_string()),
                }
            },
            InputMultiplexorCore::File(file) => {
                match file.read(buffer) {
                    Ok(bytes_read) => Ok(bytes_read),
                    Err(error) => Err(error.to_string()),
                }
            },
        }
    }
}

use stream::ProcessStreamResult;
use num::complex::Complex;
use bytemuck::cast_slice;

fn process_stream_lms(
    u8_buffer: &[u8],
    streams: usize,
    bit_error_table: &HashMap<u32, u16>,
    seen: &Arc<Mutex<HashMap<u32, Instant>>>,
    mu: f32,
    check_preamble: bool
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

    // The preamble in pulses.
    let soi_pulses = vec![1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0];
    // The preamble as a sequence of complex numbers centered on DC with a phase of zero.
    let mut soi: Vec<Complex<f32>> = Vec::new();
    for x in 0..soi_pulses.len() {
        soi.push(Complex::new(soi_pulses[x] as f32, 0.0));
    }

    // The samples for the message content. No preamble.
    let mut samples: Vec<f32> = vec![0.0f32; MODES_LONG_MSG_SAMPLES];
    // The preamble.
    let mut p: Vec<f32> = vec![0.0f32; MODES_PREAMBLE_SAMPLES];

    for x in 0..buffer.len() / mul - MODES_PREAMBLE_SAMPLES - MODES_LONG_MSG_SAMPLES {
        let mut w_lms = vec![Complex::new(0.0f32, 0.0f32); streams];

        for i in 0..constants::MODES_PREAMBLE_SAMPLES {
            let soi_sample = soi[i];
            let mut sum = Complex::new(0.0f32, 0.0f32);
            
            // Apply the weights and get the result. Do the beamforming operation.
            for y in 0..streams {
                sum += w_lms[y].conj() * iq[y][x + i];
            }

            // Take the difference between what we were supposed to get and what we got.
            let error = soi_sample - sum;
            
            // Apply the error to the original samples and apply part of that to the weights
            // in an incremental fashion.
            for y in 0..streams {
                w_lms[y] += mu * error.conj() * iq[y][x + i];
            }
        }

        let snr: f32;

        if check_preamble {
            for i in 0..constants::MODES_PREAMBLE_SAMPLES {
                let mut sum = Complex::new(0.0f32, 0.0f32);
                
                // Apply the final weights we got to the sample stream one sample at a time.
                for y in 0..streams {
                    sum += w_lms[y].conj() * iq[y][x + i];
                }

                p[i] = sum.norm();
            }

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
        } else {
            snr = 0.0;
        }

        for i in 0..constants::MODES_LONG_MSG_SAMPLES {
            let mut sum = Complex::new(0.0f32, 0.0f32);
            
            // Apply the final weights we got to the sample stream one sample at a time.
            for y in 0..streams {
                sum += w_lms[y].conj() * iq[y][x + constants::MODES_PREAMBLE_SAMPLES + i];
            }

            // The output of the beamformer equation.
            samples[i] = sum.norm();
        }

        // Process the samples as if they contain a message.

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
                snr: snr,
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
    println!("Hello, world!");

    let args = Args::parse();

    let use_lms;
    let thread_count: u32 = args.thread_count;
    let cycle_count: u32 = match args.cycle_count {
        Some(v) => {
            use_lms = false;
            v
        },
        None => {
            use_lms = true;
            1
        },
    };

    if use_lms {
        println!("Using LMS beamformer.");
        println!("Using {} threads.", thread_count);
    } else {
        match args.ula_spacing_wavelength {
            None => println!("Using random beamformer."),
            Some(_) => println!("Using ULA scanned traditional sum and delay."),
        }
        
        println!("Using {} threads and {} cycles for a total of {} beams/antenna-patterns.", thread_count, cycle_count, thread_count * cycle_count);
    }

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
            let bit_error_table = crc::modes_init_error_info();
            let mut pipe_theta: Vec<Option<Vec<f32>>> = vec![None; cycle_count as usize];
            let mut pipe_amps: Vec<Option<Vec<f32>>> = vec![None; cycle_count as usize];

            loop {
                match brx.recv().unwrap() {
                    ThreadTxMessage::Buffer(buffer, streams) => {
                        // This calls into the `stream` module. See that
                        // module to follow the code.
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
                    ThreadTxMessage::LMSWork(buffer, streams) => {
                        btx.send(process_stream_lms(
                            &buffer,
                            streams,
                            &bit_error_table,
                            &seen_thread,
                            args.mu,
                            args.check_preamble
                        )).unwrap();
                    },
                    ThreadTxMessage::Exit => break,
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
    let mut stat_mes: u64 = 0;
    let mut stat_mu: u64 = 0;
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

    let mut im = match args.input_source {
        None => match InputMultiplexor::from_socket("localhost:7878") {
            Ok(v) => v,
            Err(msg) => panic!("error opening socket: {}", msg),
        },
        Some(input_source) => match input_source.find(":") {
            Some(_) => match InputMultiplexor::from_socket(&input_source) {
                Ok(v) => v,
                Err(msg) => panic!("error opening socket: {}", msg),
            },
            None => match InputMultiplexor::from_open(&input_source) {
                Ok(v) => v,
                Err(msg) => panic!("error opening file: {}", msg),
            }
        },
    };

    println!("connected to source!");

    // We are expecting TWO interleaved streams from TWO antennas.
    let mut read: usize = 0;
    
    let mut short_buffer = vec![0; 1];

    // Read the number of streams.
    let streams = match args.stream_count {
        Some(v) => v,
        None => match im.read(&mut short_buffer[0..1]) {
            Ok(bytes_read) if bytes_read > 0 => {
                short_buffer[0] as usize
            },
            Ok(_) => {
                panic!("source returned zero bytes!");
            },
            Err(e) => {
                panic!("error when reading stream count: {}", e);
            }
        },
    };

    let mut buffer: Vec<u8> = vec![0; MODES_LONG_MSG_SAMPLES * 1024 * (streams * 4)];

    // This sets up the pipes for uniform linear array (ULA) mode. It consumes all of the pipes
    // currently so there won't be any left to try to track airplanes.
    match args.ula_spacing_wavelength {
        None => (),
        Some(spacing) => {
            if use_lms {
                panic!("--ula-spacing-wavelength is incompatible with LMS beamformer. LMS is toggled by not specifying --cycle-count.");
            }

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
    while match im.read(&mut buffer[read..]) {
        Ok(bytes_read) if bytes_read > 0 => {
            read += bytes_read;
            //println!("read bytes {}", bytes_read);
            if read == buffer.len() {
                //println!("sending buffer to threads");
                let start = Instant::now();

                let mut hm: HashMap<u64, Message> = HashMap::new();

                let mut items: Vec<(u64, Message)>;

                if !use_lms {
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

                    items = hm.into_iter().collect();
                    // They might be out of order so sort them to ensure they are ordered.
                    items.sort_by(|a, b| (&a.0).cmp(&b.0));
                } else {
                    items = Vec::new();

                    let bytes_per_strip = 4 * streams;
                    let sample_count = buffer.len() / bytes_per_strip;
                    let chunk_size: usize = sample_count / thread_count as usize;
                    let rem: usize = sample_count % thread_count as usize;
                    
                    for i in 0..thread_count as usize - 1 {
                        let chunk_slice: &[u8] = &buffer[i * chunk_size * bytes_per_strip..(i * chunk_size + chunk_size + MODES_LONG_MSG_SAMPLES) * bytes_per_strip];
                        let chunk: Vec<u8> = chunk_slice.to_vec();
                        pipe_mgmt.send_lms_work_to_thread(i, chunk, streams);
                    }

                    let i = thread_count as usize - 1;
                    let chunk_slice = &buffer[i * chunk_size * bytes_per_strip..(i * chunk_size + chunk_size + rem) * bytes_per_strip];
                    let chunk: Vec<u8> = chunk_slice.to_vec();
                    pipe_mgmt.send_lms_work_to_thread(i, chunk, streams);

                    for i in 0..thread_count as usize {
                        let msgs = rxs[i].recv().unwrap();
                        for mut msg in msgs {
                            items.push((msg.common.ndx, msg));
                        }
                    }
                }

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
                        MessageSpecific::MilitaryExtendedSquitter { .. } => stat_mes += 1,
                        MessageSpecific::MilitaryUse { .. } => stat_mu += 1,
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
                    //println!("MilitaryExtendedSquitter      {}/{:.1}", stat_mes, stat_mes as f64 / elapsed);
                    //println!("MilitaryUse                   {}/{:.1}", stat_mu, stat_mu as f64 / elapsed);
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
                        if !im.from_file() {
                            println!("elapsed:{} buffer_time:{} TOO SLOW!!! REDUCE CYCLES!!!", cur_elapsed, buffer_time);
                        }
                    }
                }

                sample_index += buffer.len() as u64 / (streams * 4) as u64;
                // Copy tail end of the buffer back to the start to catch messages at end or crossing the boundary between buffers.
                {
                    let sz = (constants::MODES_PREAMBLE_SAMPLES + constants::MODES_LONG_MSG_SAMPLES - 1) * 4 * streams;
                    for x in 0..sz {
                        buffer[x] = buffer[buffer.len() - sz + x];
                    }
                    read = sz;
                }
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

    pipe_mgmt.tell_threads_to_exit();

    println!("exiting");
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_getbits() {
    }
}