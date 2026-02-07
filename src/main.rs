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
use std::sync::mpsc::{channel, Sender, Receiver};
use std::collections::{HashMap, VecDeque};
use std::result::Result;
use clap::Parser;
use std::fs::File;
use std::io::prelude::*;
use std::fmt;
use std::f32::consts::PI;

mod crc;
mod constants;
mod stream;
mod pipemgmt;
mod cpr;

use pipemgmt::{ThreadTxMessage, PipeManagement};
use cpr::decode_cpr;

use constants::*;

/// Members that are common to all messages.
///
/// This contains a lot of data useful for debugging and experimentation
/// with the beamforming operation. For example, it contains the raw
/// message bytes, raw samples from the card, theta used to process the
/// samples, amplitudes of each antenna, and if the CRC was okay for the
/// message.
#[allow(dead_code)]
struct MessageCommon {
    /// The bytes that comprise the message after demodulation.
    msg: Vec<u8>,
    /// The raw I/Q samples.
    samples: Vec<i16>,
    /// The sample index the message was found at.
    ndx: u64,
    /// The computed signal to noise ratio.
    snr: f32,
    /// The thetas used during beamforming.
    thetas: Vec<f32>,
    /// The amplitudes used during beamforming.
    amplitudes: Vec<f32>,
    /// Was the CRC OK?
    crc_ok: bool,
    /// The global pipe index.
    pipe_ndx: usize,
}

impl fmt::Debug for MessageCommon {
    /// This prevents the fields of this structure from being debug printed.
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

/// Elements that are common to a few different specific message types.
#[derive(Debug)]
#[allow(dead_code)]
struct DfHeader1 {
    capability: u8,
    /// The transponder address.
    addr: u32,
    metype: u8,
    mesub: u8,
    fs: u8,
    /// The squawk code if any.
    identity: u32,
}

/// This is a good place to put anything specific if any
/// decoding was done on the message. You could put fields
/// specific to each message type here.
#[derive(Debug)]
enum MessageSpecific {
    #[allow(dead_code)]
    AircraftIdenAndCat {
        hdr: DfHeader1,
        aircraft_type: u8,
        flight: Vec<char>,
    },
    #[allow(dead_code)]
    SurfacePositionMessage {
        hdr: DfHeader1,
        movement: u8,
        ground_track: u8,
        f_flag: bool,
        t_flag: bool,
        raw_lat: u32,
        raw_lon: u32,
    },
    #[allow(dead_code)]
    AirbornePositionMessage {
        hdr: DfHeader1,
        f_flag: bool,
        t_flag: bool,
        altitude: f32,
        raw_lat: u32,
        raw_lon: u32,
    },
    #[allow(dead_code)]
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
    #[allow(dead_code)]
    AirborneVelocityMessageShort {
        hdr: DfHeader1,
        heading: f32,
    },
    Other,
}

/// A reason for not being able to decode a message.
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

fn was_addr_recently_seen(addr: &u32, seen: &Arc<Mutex<HashMap<u32, Instant>>>) -> bool {
    match seen.lock().unwrap().get(addr) {
        Some(time_seen) => {
            let dur = Instant::now() - *time_seen;
            if dur.as_secs() < 60 {
                true
            } else {
                false
            }
        },
        None => false
    }
}

fn brute_force_ap(msg: &[u8], seen: &Arc<Mutex<HashMap<u32, Instant>>>) -> bool {
    let msgtype = msg[0] >> 3;

    if
        msgtype == 0 || msgtype == 4 || msgtype == 5 || 
        msgtype == 16 || msgtype == 20 || msgtype == 21 || 
        msgtype == 24
    {
        let crc = crc::modes_compute_crc(msg);
        let last_byte = msg.len() - 1;
        let aux0 = msg[last_byte - 0] as u32 ^ (crc & 0xff);
        let aux1 = msg[last_byte - 1] as u32 ^ ((crc >> 8) & 0xff);
        let aux2 = msg[last_byte - 2] as u32 ^ ((crc >> 16) & 0xff);
        let addr = aux0 | (aux1 << 8) | (aux2 << 16);
        was_addr_recently_seen(&addr, seen)
    } else {
        false
    }
}

/// Process the stream result and do any decoding that is needed.
fn process_result(
    result: stream::ProcessStreamResult,
    bit_error_table: &HashMap<u32, u16>,
    seen: &Arc<Mutex<HashMap<u32, Instant>>>
) -> Result<Message, MessageErrorReason> {
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
    let mut nfixed = 0;

    if !crc_ok && (msgtype == 11 || msgtype == 17 || msgtype == 18) {
        nfixed = crc::fix_bit_errors(&mut msg, bit_error_table);
        
        if nfixed == 0 {
            return Err(MessageErrorReason::BitErrors);
        }

        crc_syndrome = crc::modes_checksum(&msg);
        crc_ok = crc_syndrome == 0;
    }

    let aa1 = msg[1];
    let aa2 = msg[2];
    let aa3 = msg[3];
    let addr = ((aa1 as u32) << 16) | ((aa2 as u32) << 8) | aa3 as u32;    

    if msgtype != 11 && msgtype != 17 && msgtype != 18 {
        if brute_force_ap(&msg, seen) {
            crc_ok = true;
        } else {
            crc_ok = false;
        }
    } else {
        if crc_ok && nfixed == 0 {
            seen.lock().unwrap().insert(addr, Instant::now());
        }

        if msgtype == 11 && !crc_ok && crc_syndrome < 80 {
            if was_addr_recently_seen(&addr, seen) {
                crc_ok = true;
            }
        }
    }

    if !crc_ok {
        return Err(MessageErrorReason::BitErrors);
    }

    // Here is where you want to do your decoding. I just simply
    // copied the parameters over and set a marker for each message
    // type.

    let ca = msg[0] & 7;
    let metype = msg[4] >> 3;
    let mesub = msg[4] & 7;
    let fs = msg[0] & 7;
    let _dr = msg[1] >> 3 & 31;
    let _um = ((msg[1] & 7) << 3) | (msg[2] >> 5);

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
        ndx: result.ndx as u64,
        snr: result.snr,
        thetas: result.thetas,
        samples: result.samples,
        amplitudes: result.amplitudes,
        crc_ok: crc_ok,
        pipe_ndx: result.pipe_ndx,
    };

    match msgtype {
        17 | 18 => {
            let hdr = DfHeader1 {
                capability: ca,
                addr: addr,
                metype: metype,
                mesub: mesub,
                fs: fs,
                identity: identity,
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
                        f_flag: (msg[6] >> 2 & 1) == 1,
                        t_flag: (msg[6] >> 3 & 1) == 1,
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

/// Serialize the common elements of a message to a file.
fn write_message_to_file(file: &mut File, m: &Message) {
    file.write_all(bytes_of(&(m.common.msg.len() as u16))).unwrap();
    file.write_all(&m.common.msg).unwrap();
    file.write_all(bytes_of(&(m.common.samples.len() as u16))).unwrap();
    for x in 0..m.common.samples.len() {
        file.write_all(bytes_of(&m.common.samples[x])).unwrap();
    }
    file.write_all(bytes_of(&m.common.ndx)).unwrap();
    file.write_all(bytes_of(&m.common.snr)).unwrap();
    let thetas = &m.common.thetas;
    file.write_all(bytes_of(&(thetas.len() as u8))).unwrap();
    for theta in thetas {
        file.write_all(bytes_of(theta)).unwrap();
    }
    let amplitudes = &m.common.amplitudes;
    file.write_all(bytes_of(&(m.common.amplitudes.len() as u8))).unwrap();
    for amp in amplitudes {
        file.write_all(bytes_of(amp)).unwrap();
    }
}

/// Anything with a transponder such as an aircraft.
struct Entity {
    addr: u32,
    /// The odd raw lat and lon from a message.
    ///
    /// The `odd_cpr` and `even_cpr` are used to compute a
    /// latitude and longitude.
    odd_cpr: Option<(u32, u32, u64)>,
    /// The even raw lat and lon from a message.
    ///
    /// The `odd_cpr` and `even_cpr` are used to compute a
    /// latitude and longitude.
    even_cpr: Option<(u32, u32, u64)>,
    /// The last known computed latitude from the CPR format.
    lat: Option<f32>,
    /// The last known computed longitude from the CPR format.
    lon: Option<f32>,
    /// The last known altitude.
    alt: Option<f32>,
    /// The last known flight identifier transmitted.
    flight: Option<Vec<char>>,
    /// The last known aircraft type.
    aircraft_type: Option<u8>,
    /// The last update. Uses sample stream time relative to the beginning of the stream.
    ///
    /// This is an actual count of samples since the program started where the message was
    /// found. The convert it to seconds you need the sampling rate.
    last_update: u64,
    /// Total messages to this `addr`.
    message_count: u64,
    /// A list used to compute a rolling weighted average over the weight thetas.
    thetas: VecDeque<Vec<f32>>,
    /// A list used to compute a rolling weighted average.
    snrs: VecDeque<f32>,
    /// A list used to compute a rolling weighted average over the weight amplitudes.
    amps: VecDeque<Vec<f32>>,
    /// The number of messages that matched the set steering vector. 
    ///
    /// This shows how effective the steering vector calculated is at
    /// capturing messages.
    inbeam: u64,
}

impl Entity {
    /// Push new theta, pop any over `num` size, and return the average of the remaining theta elements.
    fn push_theta_cap_avg(&mut self, snr: f32, thetas: Vec<f32>, amps: Vec<f32>, num: usize, snr_scaler: f32) -> (Vec<f32>, Vec<f32>) {
        self.thetas.push_front(thetas);
        self.snrs.push_front(snr);
        self.amps.push_front(amps);

        while self.thetas.len() > num {
            self.thetas.pop_back();
            self.snrs.pop_back();
            self.amps.pop_back();
        }

        self.theta_avg(snr_scaler)
    }

    /// Return the weighted average of the elements of the `thetas` array.
    ///
    /// The SNR weights the average.
    fn theta_avg(&self, snr_scaler: f32) -> (Vec<f32>, Vec<f32>) {
        let mut sum: Vec<f32> = Vec::with_capacity(self.thetas[0].len());
        let mut amp_sum: Vec<f32> = Vec::with_capacity(self.amps[0].len());

        let mut total = 0.0f32;
        let mut amp_total = 0.0f32;

        for _ in 0..self.thetas[0].len() {
            sum.push(0.0);
        }

        for _ in 0..self.amps[0].len() {
            amp_sum.push(0.0);
        }

        for y in 0..self.thetas.len() {
            for x in 0..sum.len() {
                sum[x] += self.thetas[y][x] * self.snrs[y] * snr_scaler;
            }

            total += self.snrs[y] * snr_scaler;
        }

        for y in 0..self.amps.len() {
            for x in 0..amp_sum.len() {
                amp_sum[x] += self.amps[y][x] * self.snrs[y] * snr_scaler;
            }
        }
        
        for x in 0..sum.len() {
            sum[x] /= total;
        }

        for x in 0..amp_sum.len() {
            amp_sum[x] /= total;
        }
        
        (sum, amp_sum)
    }

    /// Check if the pipe_ndx, likely from a message, is currently the target for the address.
    ///
    /// When we trying to track a transponder we try to compute a steering vector. If there are
    /// any free pipes we set that pipe to the steering vector calculated. This checks if the
    /// `pipe_ndx` was the pipe being used and if so we consider the message to have arrived
    /// from this pipe and increment the `inbeam` count.
    ///
    /// In a shorter explanation, we keep track of how effective the computed steering vector
    /// is by counting how many hits it gets.
    fn check_if_in_beam(&mut self, pipe_mgmt: &mut PipeManagement, pipe_ndx: usize) {
        match pipe_mgmt.get_addr_pipe_ndx(self.addr) {
            Some(stored_pipe_ndx) => {
                if stored_pipe_ndx == pipe_ndx {
                    self.inbeam += 1;
                }
            },
            None => (),
        }
    }
}

/// Initialize a new entity/aircraft is none is found.
fn init_entity_if_not(addr: u32, entities: &mut HashMap<u32, Entity>) {
    match entities.get_mut(&addr) {
        Some(_) => (),
        None => {
            entities.insert(addr, Entity {
                addr: addr,
                odd_cpr: None,
                even_cpr: None,
                lat: None,
                lon: None,
                alt: None,
                flight: None,
                last_update: 0u64,
                aircraft_type: None,
                thetas: VecDeque::new(),
                snrs: VecDeque::new(),
                amps: VecDeque::new(),
                message_count: 0,
                inbeam: 0,
            });
        },
    }
}

/// Process the messages computing coordinates and sending output to SBS clients.
///
/// Here we iterate the messages, collecting odd and even raw coordinates, and computing
/// the actual coordinates. There is still a lot more work that can be done here. All of
/// this was copied from the `dump1090` implementation by antirez.
fn process_messages(
    messages: Vec<(u64, Message)>,
    entities: &mut HashMap<u32, Entity>,
    buffer_start_sample_index: u64,
    pipe_mgmt: &mut PipeManagement,
    snr_scaler: f32,
    weighted_avg_depth: usize
) {
    for (buffer_sample_index, m) in messages {
        let sample_index = buffer_sample_index as u64 + buffer_start_sample_index;

        match m.specific {
            MessageSpecific::AirborneVelocityMessageShort {
                hdr,
                heading: _
            } => {
                init_entity_if_not(hdr.addr, entities);
                let ent = entities.get_mut(&hdr.addr).unwrap();
                ent.message_count += 1;

                ent.check_if_in_beam(pipe_mgmt, m.common.pipe_ndx);

                let (thetas, amps) = ent.push_theta_cap_avg(
                    m.common.snr, m.common.thetas, m.common.amplitudes, weighted_avg_depth, snr_scaler
                );
                // Update get average set a pipe or existing pipe.
                pipe_mgmt.set_addr_to_theta(
                    hdr.addr,
                    thetas,
                    Some(amps)
                );
            },
            MessageSpecific::AirborneVelocityMessage {
                hdr, 
                ew_dir: _,
                ew_velocity: _,
                ns_dir: _,
                ns_velocity: _,
                vert_rate_source: _,
                vert_rate_sign: _,
                vert_rate: _,
                velocity: _,
                heading: _
            } => {
                init_entity_if_not(hdr.addr, entities);
                let ent = entities.get_mut(&hdr.addr).unwrap();
                ent.message_count += 1;

                ent.check_if_in_beam(pipe_mgmt, m.common.pipe_ndx);
                
                // Update get average set a pipe or existing pipe.
                let (thetas, amps) = ent.push_theta_cap_avg(
                    m.common.snr, m.common.thetas, m.common.amplitudes, weighted_avg_depth, snr_scaler
                );
                pipe_mgmt.set_addr_to_theta(
                    hdr.addr,
                    thetas,
                    Some(amps)
                );
            },
            MessageSpecific::AircraftIdenAndCat {
                hdr,
                aircraft_type,
                flight,
            } => {
                init_entity_if_not(hdr.addr, entities);
                let ent = entities.get_mut(&hdr.addr).unwrap();
                ent.last_update = sample_index;
                ent.flight = Some(flight);
                ent.aircraft_type = Some(aircraft_type);
                ent.message_count += 1;

                ent.check_if_in_beam(pipe_mgmt, m.common.pipe_ndx);

                // Update get average set a pipe or existing pipe.
                let (thetas, amps) = ent.push_theta_cap_avg(
                    m.common.snr, m.common.thetas, m.common.amplitudes, weighted_avg_depth, snr_scaler
                );
                pipe_mgmt.set_addr_to_theta(
                    hdr.addr,
                    thetas,
                    Some(amps)
                );
            },
            MessageSpecific::AirbornePositionMessage {
                hdr,
                f_flag,
                t_flag: _,
                altitude,
                raw_lat,
                raw_lon,
            } => {
                init_entity_if_not(hdr.addr, entities);
                let ent = entities.get_mut(&hdr.addr).unwrap();
                ent.last_update = sample_index;
                ent.alt = Some(altitude);
                ent.message_count += 1;

                ent.check_if_in_beam(pipe_mgmt, m.common.pipe_ndx);

                // Update get average set a pipe or existing pipe.
                let (thetas, amps) = ent.push_theta_cap_avg(
                    m.common.snr, m.common.thetas, m.common.amplitudes, weighted_avg_depth, snr_scaler
                );
                pipe_mgmt.set_addr_to_theta(
                    hdr.addr,
                    thetas,
                    Some(amps)
                );

                if f_flag {
                    ent.odd_cpr = Some((raw_lat, raw_lon, sample_index));
                } else {
                    ent.even_cpr = Some((raw_lat, raw_lon, sample_index));
                }

                match ent.even_cpr {
                    Some(a) => match ent.odd_cpr {
                        Some(b) => {
                            let delta = if a.2 > b.2 {
                                a.2 - b.2
                            } else {
                                b.2 - a.2
                            };
                            
                            // Divide delta by the sample rate for the number of seconds
                            // between each pair of raw coordinates.
                            if delta / 2000000u64 <= 10 {
                                match decode_cpr(a, b) {
                                    Some((lat, lon)) => {
                                        ent.lat = Some(lat);
                                        ent.lon = Some(lon);
                                    },
                                    None => (),
                                }
                            }
                        },
                        None => (),
                    },
                    None => (),
                }
            },
            _ => (),
        }
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
    #[clap(default_value_t = true)]
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

            match args.ula_spacing_wavelength {
                None => (),
                Some(spacing) => {
                    let total_pipes = thread_count * cycle_count;

                    let slice = PI / (total_pipes as f32 - 1.0);

                    for n in 0..total_pipes {
                        let theta = slice * n as f32 - PI * 0.5f32;
                        
                        let mut thetas: Vec<f32> = Vec::with_capacity(streams - 1);

                        for element_index in 1..streams {
                            let shift = -spacing * PI * 2.0f32 * theta.sin() * element_index as f32;
                            thetas.push(shift);
                        }

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
                        process_messages(
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