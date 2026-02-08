use crate::crc;
use crate::constants;
use std::time::Instant;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use crate::stream;
use std::fmt;

/// Members that are common to all messages.
///
/// This contains a lot of data useful for debugging and experimentation
/// with the beamforming operation. For example, it contains the raw
/// message bytes, raw samples from the card, theta used to process the
/// samples, amplitudes of each antenna, and if the CRC was okay for the
/// message.
#[allow(dead_code)]
pub struct MessageCommon {
    /// The bytes that comprise the message after demodulation.
    pub msg: Vec<u8>,
    /// The raw I/Q samples.
    pub samples: Vec<i16>,
    /// The sample index the message was found at.
    pub ndx: u64,
    /// The computed signal to noise ratio.
    pub snr: f32,
    /// The thetas used during beamforming.
    pub thetas: Vec<f32>,
    /// The amplitudes used during beamforming.
    pub amplitudes: Vec<f32>,
    /// Was the CRC OK?
    pub crc_ok: bool,
    /// The global pipe index.
    pub pipe_ndx: usize,
}

impl fmt::Debug for MessageCommon {
    /// This prevents the fields of this structure from being debug printed.
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        f.debug_struct("MessageCommon").finish()
    }
}

/// Represents a message after demodulation and decoding.
#[derive(Debug)]
pub struct Message {
    pub common: MessageCommon,
    /// Any data specific to this message. For example, this
    /// might contain fields specific to a message type.
    pub specific: MessageSpecific,
}

/// Elements that are common to a few different specific message types.
#[derive(Debug)]
#[allow(dead_code)]
pub struct DfHeader1 {
    pub capability: u8,
    /// The transponder address.
    pub addr: u32,
    pub metype: u8,
    pub mesub: u8,
    pub fs: u8,
    /// The squawk code if any.
    pub identity: u32,
}

/// This is a good place to put anything specific if any
/// decoding was done on the message. You could put fields
/// specific to each message type here.
#[derive(Debug)]
pub enum MessageSpecific {
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
pub enum MessageErrorReason {
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
pub fn process_result(
    result: stream::ProcessStreamResult,
    bit_error_table: &HashMap<u32, u16>,
    seen: &Arc<Mutex<HashMap<u32, Instant>>>
) -> Result<Message, MessageErrorReason> {
    let mut msg = result.msg;

    let is_long: bool = ((msg[0] >> 3) & 0x10) == 0x10;

    match is_long {
        true => (),
        false => {
            while msg.len() > constants::MODES_SHORT_MSG_BYTES {
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