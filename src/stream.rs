use std::sync::{Arc, Mutex};
use crate::constants;
use crate::Message;
use crate::process_result;
use bytemuck::cast_slice;
use rand::Rng;
use std::collections::HashMap;
use std::time::{Duration, Instant};

/// Represents a message after demodulation but before decoding.
pub struct ProcessStreamResult {
    pub snr: f32,
    pub msg: Vec<u8>,
    pub samples: Vec<i16>,
    pub ndx: usize,
    pub theta: f32,
    pub amplitude_a: f32,
    pub amplitude_b: f32,
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
pub fn process_stream_mfloat32(
    stream: &[f32],
    i16stream: &[i16],
    theta: f32,
    amplitude_a: f32,
    amplitude_b: f32
) -> Vec<ProcessStreamResult> {
    let mut results: Vec<ProcessStreamResult> = Vec::new();

    for x in 0..stream.len() - constants::MODES_PREAMBLE_SAMPLES - constants::MODES_LONG_MSG_SAMPLES - 1 {
        let snr: f32;

        {
            let p = &stream[x..x + constants::MODES_PREAMBLE_SAMPLES];
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

        let payload = &stream[x + constants::MODES_PREAMBLE_SAMPLES..x + constants::MODES_PREAMBLE_SAMPLES + constants::MODES_LONG_MSG_SAMPLES];

        let mut thebyte: u8 = 0;
        let mut msg: Vec<u8> = Vec::new();

        for y in 0..constants::MODES_LONG_MSG_SAMPLES / 2 {
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
            samples: (&i16stream[x * 4..x * 4 + (constants::MODES_PREAMBLE_SAMPLES + constants::MODES_LONG_MSG_SAMPLES) * 4]).to_vec(),
            ndx: x,
            theta: theta,
            amplitude_a: amplitude_a,
            amplitude_b: amplitude_b,
        });
    }

    results
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
pub fn process_buffer_single(
    u8_buffer: &[u8],
    bit_error_table: &HashMap<u32, u16>,
    theta: f32,
    amplitude_a: f32,
    amplitude_b: f32,
    seen: &Arc<Mutex<HashMap<u32, Instant>>>
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
        match process_result(result, bit_error_table, seen) {
            Ok(message) => out.push(message),
            Err(_) => (),
        }        
    }

    out
}

pub fn process_buffer(
    u8_buffer: &[u8],
    bit_error_table: &HashMap<u32, u16>,
    pipe_theta: &Vec<Option<f32>>,
    seen: &Arc<Mutex<HashMap<u32, Instant>>>
) -> Vec<Message> {
    let buffer: &[i16] = cast_slice(u8_buffer);
    let mut mbuffer: Vec<f32> = Vec::with_capacity(buffer.len() / 4);
    let mut rng = rand::thread_rng();
    let mut hm: HashMap<usize, ProcessStreamResult> = HashMap::new();

    for pipe_ndx in 0..pipe_theta.len() {
        let theta = match &pipe_theta[pipe_ndx] {
            None => rng.r#gen::<f32>() * std::f32::consts::PI * 2.0f32 - std::f32::consts::PI,
            Some(theta) => *theta,
        };

        //let amplitude: f32 = rng.r#gen::<f32>() * 2.0;
        let amplitude_a: f32 = 1.0;
        let amplitude_b: f32 = 1.0;

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
        match process_result(result, bit_error_table, seen) {
            Ok(message) => out.push(message),
            Err(_) => (),
        }
    }

    out
}
