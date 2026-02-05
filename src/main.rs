//! Proof of concept for ADS-B reception using beamforming of a two antenna system.
//!
//! This program was designed for experimentation and testing. It provides limited
//! value as is. However, it might serve as a solid foundation or at the least an
//! idea for a future project.
//!
//! Thanks to Salvatore Sanfilippo <antirez@gmail.com> and https://github.com/antirez/dump1090/
//!
//! Thanks to Malcolm Robb <support@attavionics.com> and https://github.com/MalcolmRobb/dump1090/
//!
//! Thanks to https://github.com/flightaware/dump1090

use std::ascii::AsciiExt;
use std::iter::Map;
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

mod crc;
mod constants;
mod stream;

use constants::*;

/// Members that are common to all messages.
///
/// This contains a lot of data useful for debugging and experimentation
/// with the beamforming operation. For example, it contains the raw
/// message bytes, raw samples from the card, theta used to process the
/// samples, amplitudes of each antenna, and if the CRC was okay for the
/// message.
struct MessageCommon {
    /// The bytes that comprise the message after demodulation.
    msg: Vec<u8>,
    /// The raw I/Q samples.
    samples: Vec<i16>,
    /// The sample index the message was found at.
    ndx: usize,
    /// The computed signal to noise ratio.
    snr: f32,
    /// The thetas used during beamforming.
    thetas: Vec<f32>,
    /// The amplitudes used during beamforming.
    amplitudes: Vec<f32>,
    /// Was the CRC OK?
    crc_ok: bool,
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
    let dr = msg[1] >> 3 & 31;
    let um = ((msg[1] & 7) << 3) | (msg[2] >> 5);

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
        thetas: result.thetas,
        samples: result.samples,
        amplitudes: result.amplitudes,
        crc_ok: crc_ok,
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

fn cpr_nl_function(mut lat: f32) -> f32 {
    if lat < 0.0 {
        lat = -lat;
    }

    // Table is symmetric about the equator.
    if lat < 10.47047130 { return 59.0; }
    if lat < 14.82817437 { return 58.0; }
    if lat < 18.18626357 { return 57.0; }
    if lat < 21.02939493 { return 56.0; }
    if lat < 23.54504487 { return 55.0; }
    if lat < 25.82924707 { return 54.0; }
    if lat < 27.93898710 { return 53.0; }
    if lat < 29.91135686 { return 52.0; }
    if lat < 31.77209708 { return 51.0; }
    if lat < 33.53993436 { return 50.0; }
    if lat < 35.22899598 { return 49.0; }
    if lat < 36.85025108 { return 48.0; }
    if lat < 38.41241892 { return 47.0; }
    if lat < 39.92256684 { return 46.0; }
    if lat < 41.38651832 { return 45.0; }
    if lat < 42.80914012 { return 44.0; }
    if lat < 44.19454951 { return 43.0; }
    if lat < 45.54626723 { return 42.0; }
    if lat < 46.86733252 { return 41.0; }
    if lat < 48.16039128 { return 40.0; }
    if lat < 49.42776439 { return 39.0; }
    if lat < 50.67150166 { return 38.0; }
    if lat < 51.89342469 { return 37.0; }
    if lat < 53.09516153 { return 36.0; }
    if lat < 54.27817472 { return 35.0; }
    if lat < 55.44378444 { return 34.0; }
    if lat < 56.59318756 { return 33.0; }
    if lat < 57.72747354 { return 32.0; }
    if lat < 58.84763776 { return 31.0; }
    if lat < 59.95459277 { return 30.0; }
    if lat < 61.04917774 { return 29.0; }
    if lat < 62.13216659 { return 28.0; }
    if lat < 63.20427479 { return 27.0; }
    if lat < 64.26616523 { return 26.0; }
    if lat < 65.31845310 { return 25.0; }
    if lat < 66.36171008 { return 24.0; }
    if lat < 67.39646774 { return 23.0; }
    if lat < 68.42322022 { return 22.0; }
    if lat < 69.44242631 { return 21.0; }
    if lat < 70.45451075 { return 20.0; }
    if lat < 71.45986473 { return 19.0; }
    if lat < 72.45884545 { return 18.0; }
    if lat < 73.45177442 { return 17.0; }
    if lat < 74.43893416 { return 16.0; }
    if lat < 75.42056257 { return 15.0; }
    if lat < 76.39684391 { return 14.0; }
    if lat < 77.36789461 { return 13.0; }
    if lat < 78.33374083 { return 12.0; }
    if lat < 79.29428225 { return 11.0; }
    if lat < 80.24923213 { return 10.0; }
    if lat < 81.19801349 { return 9.0; }
    if lat < 82.13956981 { return 8.0; }
    if lat < 83.07199445 { return 7.0; }
    if lat < 83.99173563 { return 6.0; }
    if lat < 84.89166191 { return 5.0; }
    if lat < 85.75541621 { return 4.0; }
    if lat < 86.53536998 { return 3.0; }
    if lat < 87.00000000 { return 2.0; }
    1.0
}

fn cpr_mod_function(a: f32, b: f32) -> f32 {
    let res = a % b;
    if res < 0.0f32 {
        res + b
    } else {
        res
    }
}

fn cpr_n_function(lat: f32, isodd: f32) -> f32 {
    let nl = cpr_nl_function(lat) as f32 - isodd;
    if nl < 1.0 {
        1.0
    } else {
        nl
    }
}

fn cpr_dlon_function(lat: f32, isodd: f32) -> f32 {
    360.0 / cpr_n_function(lat, isodd)
}

/// Returns latitude and longitude or None if computation is not possible.
fn decode_cpr(even: (u32, u32, u64), odd: (u32, u32, u64)) -> Option<(f32, f32)> {
    let air_dlat0: f32 = 360.0 / 60.0;
    let air_dlat1: f32 = 360.0 / 59.0;
    let lat0 = even.0 as f32;
    let lat1 = odd.0 as f32;
    let lon0 = even.1 as f32;
    let lon1 = odd.1 as f32;

    let j = (((59.0 * lat0 - 60.0 * lat1) / 131072.0) + 0.5).floor();
    let mut rlat0 = air_dlat0 * (cpr_mod_function(j, 60.0) + lat0 / 131072.0);
    let mut rlat1 = air_dlat1 * (cpr_mod_function(j, 59.0) + lat1 / 131072.0);  

    if rlat0 >= 270.0 {
        rlat0 -= 360.0;
    }

    if rlat1 >= 270.0 {
        rlat1 -= 360.0;
    }

    if cpr_nl_function(rlat0) != cpr_nl_function(rlat1) {
        return None;
    }

    if even.2 > odd.2 {
        let ni = cpr_n_function(rlat0, 0.0);
        let m = ((((lon0 * (cpr_nl_function(rlat0) - 1.0)) - (lon1 * cpr_nl_function(rlat0))) / 131072.0) + 0.5).floor();
        let mut lon = cpr_dlon_function(rlat0, 0.0) * (cpr_mod_function(m, ni) + lon0 / 131072.0);
        let lat = rlat0;
        if lon > 180.0 {
            lon -= 360.0;
        }
        Some((lat, lon))
    } else {
        let ni = cpr_n_function(rlat1, 1.0);
        let m = ((((lon0 * (cpr_nl_function(rlat1) - 1.0)) - (lon1 * cpr_nl_function(rlat1))) / 131072.0) + 0.5).floor();
        let mut lon = cpr_dlon_function(rlat1, 1.0) * (cpr_mod_function(m, ni) + lon1 / 131072.0);
        let lat = rlat1;
        if lon > 180.0 {
            lon -= 360.0;
        }        
        Some((lat, lon))
    }
}

/// Anything with a transponder such as an aircraft.
struct Entity {
    odd_cpr: Option<(u32, u32, u64)>,
    even_cpr: Option<(u32, u32, u64)>,
    lat: Option<f32>,
    lon: Option<f32>,
    alt: Option<f32>,
    flight: Option<Vec<char>>,
    aircraft_type: Option<u8>,
    last_update: u64,
    message_count: u64,
    thetas: VecDeque<f32>,
}

impl Entity {
    /// Push new theta, pop any over `num` size, and return the average of the remaining theta elements.
    fn push_theta_cap_avg(&mut self, theta: f32, num: usize) -> f32 {
        self.thetas.push_front(theta);
        while self.thetas.len() > num {
            self.thetas.pop_back();
        }

        self.theta_avg()
    }

    /// Return the average of the elements of the `thetas` array.
    fn theta_avg(&self) -> f32 {
        let mut sum = 0.0f32;
        for t in self.thetas.iter() {
            sum += t;
        }
        
        sum / self.thetas.len() as f32        
    }
}

/// Initialize a new entity/aircraft is none is found.
fn init_entity_if_not(addr: u32, entities: &mut HashMap<u32, Entity>) {
    match entities.get_mut(&addr) {
        Some(_) => (),
        None => {
            entities.insert(addr, Entity {
                odd_cpr: None,
                even_cpr: None,
                lat: None,
                lon: None,
                alt: None,
                flight: None,
                last_update: 0u64,
                aircraft_type: None,
                thetas: VecDeque::new(),
                message_count: 0,
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
    messages: Vec<(usize, Message)>,
    entities: &mut HashMap<u32, Entity>,
    buffer_start_sample_index: u64,
    pipe_mgmt: &mut PipeManagement
) {
    for (buffer_sample_index, m) in messages {
        let sample_index = buffer_sample_index as u64 + buffer_start_sample_index;

        match m.specific {
            MessageSpecific::AirborneVelocityMessageShort {
                hdr, heading
            } => {
                init_entity_if_not(hdr.addr, entities);
                let ent = entities.get_mut(&hdr.addr).unwrap();
                ent.message_count += 1;
                // Update get average set a pipe or existing pipe.
                //pipe_mgmt.set_addr_to_theta(hdr.addr, ent.push_theta_cap_avg(m.common.theta, 10));
            },
            MessageSpecific::AirborneVelocityMessage {
                hdr, ew_dir, ew_velocity, ns_dir, ns_velocity, vert_rate_source, vert_rate_sign,
                vert_rate, velocity, heading
            } => {
                init_entity_if_not(hdr.addr, entities);
                let ent = entities.get_mut(&hdr.addr).unwrap();
                ent.message_count += 1;
                // Update get average set a pipe or existing pipe.
                //pipe_mgmt.set_addr_to_theta(hdr.addr, ent.push_theta_cap_avg(m.common.theta, 10));
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

                // Update get average set a pipe or existing pipe.
                //pipe_mgmt.set_addr_to_theta(hdr.addr, ent.push_theta_cap_avg(m.common.theta, 10));
            },
            MessageSpecific::AirbornePositionMessage {
                hdr,
                f_flag,
                t_flag,
                altitude,
                raw_lat,
                raw_lon,
            } => {
                init_entity_if_not(hdr.addr, entities);
                let ent = entities.get_mut(&hdr.addr).unwrap();
                ent.last_update = sample_index;
                ent.alt = Some(altitude);
                ent.message_count += 1;

                // Update get average set a pipe or existing pipe.
                //pipe_mgmt.set_addr_to_theta(hdr.addr, ent.push_theta_cap_avg(m.common.theta, 10));

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

/// Provides easy to use functions to manage the cycle/pipes across multiple threads.
///
/// This provides the ability to easily find free pipes/cycles to associate with a transponder
/// aircraft and assign a specific theta value for use in beamforming. It also allows one to
/// disassociate a pipe when it is no longer needed causing it to run the standard algorithm
/// which at this moment is a random search.
struct PipeManagement {
    txs: Vec<Sender<ThreadTxMessage>>,
    /// Maps transponder address or u32 to a pipe.
    addr_to_pipe: HashMap<u32, (usize, usize)>,
    /// Maps each pipe to a transponder address or u32.
    pipe_to_addr: Vec<Option<u32>>,
    /// The total number of threads.
    thread_count: usize,
    /// The total number of pipes per thread.
    pipe_count: usize,
}

impl PipeManagement {
    /// Create a new management structure.
    ///
    /// This is only called once during program initialization.
    fn new(thread_count: usize, pipe_count: usize) -> PipeManagement {
        PipeManagement {
            txs: Vec::new(),
            addr_to_pipe: HashMap::new(),
            pipe_to_addr: vec![None; thread_count * pipe_count],
            thread_count: thread_count,
            pipe_count: pipe_count,
        }
    }

    /// Unassigns a pipe assigned to addr if any.
    fn unset_addr(&mut self, addr: u32) {
        let mut args: Option<(usize, usize)> = None;

        match self.addr_to_pipe.get(&addr) {
            None => (),
            Some((thread_ndx, pipe_ndx)) => {
                args = Some((*thread_ndx,  *pipe_ndx));
            },
        }

        match args {
            None => (),
            Some((thread_ndx, pipe_ndx)) => {
                self.addr_to_pipe.remove(&addr);
                self.pipe_to_addr[thread_ndx * self.pipe_count + pipe_ndx] = None;
                self.txs[thread_ndx].send(ThreadTxMessage::UnsetTheta(pipe_ndx)).unwrap();
            }
        }
    }

    /// Finds an unused pipe and sets its theta and assigns it to the specified `addr` or updates an existing.
    fn set_addr_to_theta(
        &mut self,
        addr: u32,
        thetas: Vec<f32>,
    ) -> bool {
        match self.addr_to_pipe.get(&addr) {
            Some((thread_ndx, pipe_ndx)) => {
                self.txs[*thread_ndx].send(ThreadTxMessage::SetTheta(*pipe_ndx, thetas)).unwrap();
                true
            },
            None => {
                for x in 0..self.pipe_to_addr.len() {
                    if self.pipe_to_addr[x].is_none() {
                        self.pipe_to_addr[x] = Some(addr);
                        let thread_ndx = x / self.pipe_count;
                        let pipe_ndx = x - thread_ndx * self.pipe_count;
                        self.addr_to_pipe.insert(addr, (thread_ndx, pipe_ndx));
                        self.txs[thread_ndx].send(ThreadTxMessage::SetTheta(pipe_ndx, thetas)).unwrap();
                        return true;
                    }
                }
                false
            },
        }
    }

    /// Sends a buffer to all threads to be processed.
    fn send_buffer_to_all(&self, buffer: &Vec<u8>, streams: usize) {
        for tx in &self.txs {
            tx.send(ThreadTxMessage::Buffer(buffer.clone(), streams)).unwrap();
        }
    }
    
    /// Used when this structure is first created.
    fn push_tx(&mut self, sender: Sender<ThreadTxMessage>) {
        self.txs.push(sender);
    }
}

/// A collection of messages each thread understands.
enum ThreadTxMessage {
    /// Used to send a buffer to be processed to a thread.
    ///
    /// The first argument is the buffer. The second is the number
    /// of streams contained in the buffer.
    Buffer(Vec<u8>, usize),
    /// Used to set a theta to a constant value for a single pipe.
    SetTheta(usize, Vec<f32>),
    /// Used to revert a pipe back to a value that is randomly choosen per buffer process operation.
    UnsetTheta(usize),
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

    let seen_local = seen.clone();

    for _ in 0..thread_count {
        let (atx, brx) = channel();
        let (btx, arx) = channel();
        pipe_mgmt.push_tx(atx);
        rxs.push(arx);

        let seen_thread = seen.clone();

        thread::spawn(move || {
            println!("spawned");
            let bit_error_table = crc::modes_init_error_info();
            let mut pipe_theta: Vec<Option<Vec<f32>>> = vec![None; cycle_count as usize];

            loop {
                match brx.recv().unwrap() {
                    ThreadTxMessage::Buffer(buffer, streams) => {
                        btx.send(stream::process_buffer(
                            &buffer,
                            &bit_error_table,
                            &pipe_theta,
                            streams,
                            &seen_thread
                        )).unwrap();
                    },
                    ThreadTxMessage::SetTheta(pipe_ndx, thetas) => {
                        pipe_theta[pipe_ndx] = Some(thetas);
                    },
                    ThreadTxMessage::UnsetTheta(pipe_ndx) => {
                        pipe_theta[pipe_ndx] = None;
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

    let bit_error_table = crc::modes_init_error_info();

    let mut entities: HashMap<u32, Entity> = HashMap::new();

    let mut sample_index: u64 = 0;

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
            Ok(mut stream) => {
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
            let mut buffer: Vec<u8> = vec![0; MODES_LONG_MSG_SAMPLES * 1024 * 16];
            let mut read: usize = 0;

            // Read the number of streams.
            let streams = match stream.read(&mut buffer[0..1]) {
                Ok(bytes_read) if bytes_read > 0 => {
                    buffer[0] as usize
                },
                Ok(bytes_read) => {
                    panic!("Sample stream TCP connection returned zero bytes.");
                },
                Err(e) => {
                    panic!("Error: {}", e);
                }
            };

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

                        let mut hm: HashMap<usize, Message> = HashMap::new();
                        
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

                        let mut items: Vec<(usize, Message)> = hm.into_iter().collect();
                        items.sort_by(|a, b| (&a.0).cmp(&b.0));

                        for (_, message) in &items {
                            // Other generates too much because it isn't error checked.
                            match message.specific {
                                MessageSpecific::AircraftIdenAndCat { .. } => stat_aiac += 1,
                                MessageSpecific::SurfacePositionMessage { .. } => stat_spm += 1,
                                MessageSpecific::AirbornePositionMessage { .. } => stat_apm += 1,
                                MessageSpecific::AirborneVelocityMessage { .. } => stat_avm += 1,
                                MessageSpecific::AirborneVelocityMessageShort { .. } => stat_avms += 1,
                                _ => (),
                            }

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

                        process_messages(items, &mut entities, sample_index, &mut pipe_mgmt);

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
                            for (addr, ent) in entities.iter() {
                                println!(
                                    "{:6x} {:.1} {:.2} {:.2} {} {:.2}",
                                    addr,
                                    ent.alt.unwrap_or(0.0),
                                    ent.lat.unwrap_or(0.0),
                                    ent.lon.unwrap_or(0.0),
                                    ent.message_count,
                                    ent.theta_avg()
                                );
                            }                                                    
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