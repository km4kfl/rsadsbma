use crate::MessageSpecific;
use crate::cpr::decode_cpr;
use crate::PipeManagement;
use crate::Message;
use std::collections::HashMap;
use std::collections::VecDeque;

/// Anything with a transponder such as an aircraft.
pub struct Entity {
    pub addr: u32,
    /// The odd raw lat and lon from a message.
    ///
    /// The `odd_cpr` and `even_cpr` are used to compute a
    /// latitude and longitude.
    pub odd_cpr: Option<(u32, u32, u64)>,
    /// The even raw lat and lon from a message.
    ///
    /// The `odd_cpr` and `even_cpr` are used to compute a
    /// latitude and longitude.
    pub even_cpr: Option<(u32, u32, u64)>,
    /// The last known computed latitude from the CPR format.
    pub lat: Option<f32>,
    /// The last known computed longitude from the CPR format.
    pub lon: Option<f32>,
    /// The last known altitude.
    pub alt: Option<f32>,
    /// The last known flight identifier transmitted.
    pub flight: Option<Vec<char>>,
    /// The last known aircraft type.
    pub aircraft_type: Option<u8>,
    /// The last update. Uses sample stream time relative to the beginning of the stream.
    ///
    /// This is an actual count of samples since the program started where the message was
    /// found. The convert it to seconds you need the sampling rate.
    pub last_update: u64,
    /// Total messages to this `addr`.
    pub message_count: u64,
    /// A list used to compute a rolling weighted average over the weight thetas.
    pub thetas: VecDeque<Vec<f32>>,
    /// A list used to compute a rolling weighted average.
    pub snrs: VecDeque<f32>,
    /// A list used to compute a rolling weighted average over the weight amplitudes.
    pub amps: VecDeque<Vec<f32>>,
    /// The number of messages that matched the set steering vector. 
    ///
    /// This shows how effective the steering vector calculated is at
    /// capturing messages.
    pub inbeam: u64,
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
    pub fn theta_avg(&self, snr_scaler: f32) -> (Vec<f32>, Vec<f32>) {
        let mut sum: Vec<f32> = Vec::with_capacity(self.thetas[0].len());
        let mut amp_sum: Vec<f32> = Vec::with_capacity(self.amps[0].len());

        let mut total = 0.0f32;

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
pub fn process_messages(
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