use std::sync::{Arc, Mutex};
use crate::constants;
use crate::Message;
use crate::process_result;
use bytemuck::cast_slice;
use rand::Rng;
use std::collections::HashMap;
use std::time::Instant;

/// Represents a message after demodulation but before decoding.
pub struct ProcessStreamResult {
    pub snr: f32,
    pub msg: Vec<u8>,
    pub samples: Vec<i16>,
    pub ndx: usize,
    pub thetas: Vec<f32>,
    pub amplitudes: Vec<f32>,
    pub pipe_ndx: usize,
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
    thetas: &[f32],
    amplitudes: &[f32],
    streams: usize,
    pipe_ndx: usize
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
            samples: (&i16stream[x * (streams * 2)..x * (streams * 2) + (constants::MODES_PREAMBLE_SAMPLES + constants::MODES_LONG_MSG_SAMPLES) * (streams * 2)]).to_vec(),
            ndx: x,
            thetas: thetas.to_vec(),
            amplitudes: amplitudes.to_vec(),
            pipe_ndx: pipe_ndx,
        });
    }

    results
}

/// Does a single beamforming operation. Expects 2 streams of int16 IQ pairs.
///
/// This is a loop unrolled version of `process_buffer_single`. This was done
/// to optimize for performance. The looped version was taking too much CPU
/// time.
pub fn process_buffer_single_x2(
    u8_buffer: &[u8],
    thetas_b: f32,
    amplitude_a: f32,
    amplitude_b: f32,
    pipe_ndx: usize
) -> Vec<ProcessStreamResult> {
    let buffer: &[i16] = cast_slice(u8_buffer);
    let mut mbuffer: Vec<f32> = Vec::with_capacity(buffer.len() / (2 * 2));

    let bri = thetas_b.cos();
    let brq = thetas_b.sin();

    for x in 0..buffer.len() / 4 {
        let chunk = &buffer[x * 4..x * 4 + 4];
        let ai: f32 =     chunk[0] as f32 / 2049.0 * amplitude_a;
        let aq: f32 =     chunk[1] as f32 / 2049.0 * amplitude_a;
        let mut bi: f32 = chunk[2] as f32 / 2049.0 * amplitude_b;
        let mut bq: f32 = chunk[3] as f32 / 2049.0 * amplitude_b;
        
        // Rotate the vectors by the thetas provided.
        bi = bi * bri - bq * brq;
        bq = bi * brq + bq * bri;
        // Sum the vectors.
        let ei = ai + bi;
        let eq = aq + bq;
        // Take the magnitude and store result.
        mbuffer.push((ei * ei + eq * eq).sqrt());
    }

    process_stream_mfloat32(
        &mbuffer,
        &buffer,
        &vec![thetas_b],
        &vec![amplitude_a, amplitude_b],
        2,
        pipe_ndx
    )
}

/// Does a single beamforming operation. Expects 4 streams of int16 IQ pairs.
///
/// This is a loop unrolled version of `process_buffer_single`. This was done
/// to optimize for performance. The looped version was taking too much CPU
/// time.
pub fn process_buffer_single_x4(
    u8_buffer: &[u8],
    thetas_b: f32,
    thetas_c: f32,
    thetas_d: f32,
    amplitude_a: f32,
    amplitude_b: f32,
    amplitude_c: f32,
    amplitude_d: f32,
    pipe_ndx: usize
) -> Vec<ProcessStreamResult> {
    let buffer: &[i16] = cast_slice(u8_buffer);
    let mut mbuffer: Vec<f32> = Vec::with_capacity(buffer.len() / (4 * 2));

    let bri = thetas_b.cos();
    let brq = thetas_b.sin();
    let cri = thetas_c.cos();
    let crq = thetas_c.sin();
    let dri = thetas_d.cos();
    let drq = thetas_d.sin();

    for x in 0..buffer.len() / 8 {
        let chunk = &buffer[x * 8..x * 8 + 8];
        let ai: f32 =     chunk[0] as f32 / 2049.0 * amplitude_a;
        let aq: f32 =     chunk[1] as f32 / 2049.0 * amplitude_a;
        let mut bi: f32 = chunk[2] as f32 / 2049.0 * amplitude_b;
        let mut bq: f32 = chunk[3] as f32 / 2049.0 * amplitude_b;
        let mut ci: f32 = chunk[4] as f32 / 2049.0 * amplitude_c;
        let mut cq: f32 = chunk[5] as f32 / 2049.0 * amplitude_c;
        let mut di: f32 = chunk[6] as f32 / 2049.0 * amplitude_d;
        let mut dq: f32 = chunk[7] as f32 / 2049.0 * amplitude_d;
        
        // Rotate the vectors by the thetas provided.
        bi = bi * bri - bq * brq;
        bq = bi * brq + bq * bri;
        ci = ci * cri - cq * crq;
        cq = ci * crq + cq * cri;
        di = di * dri - dq * drq;
        dq = di * drq + dq * dri;
        // Sum the vectors.
        let ei = ai + bi + ci + di;
        let eq = aq + bq + cq + dq;
        // Take the magnitude and store result.
        mbuffer.push((ei * ei + eq * eq).sqrt());
    }

    process_stream_mfloat32(
        &mbuffer,
        &buffer,
        &vec![thetas_b, thetas_c, thetas_d],
        &vec![amplitude_a, amplitude_b, amplitude_c, amplitude_d],
        4,
        pipe_ndx
    )
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
    thetas: &[f32],
    amplitudes: &[f32],
    streams: usize,
    pipe_ndx: usize
) -> Vec<ProcessStreamResult> {    
    if amplitudes.len() != streams {
        panic!("The number of amplitudes passed as part of Vec<f32> should be equal to the number of streams.")
    }

    if thetas.len() != streams - 1 {
        panic!("The number of theta passed as part of Vec<f32> should be minus 1 the number of streams.")
    }

    if streams == 2 {
        return process_buffer_single_x2(
            u8_buffer,
            thetas[0],
            amplitudes[0],
            amplitudes[1],
            pipe_ndx
        );
    } else if streams == 4 {
        return process_buffer_single_x4(
            u8_buffer,
            thetas[0],
            thetas[1],
            thetas[2],
            amplitudes[0],
            amplitudes[1],
            amplitudes[2],
            amplitudes[3],
            pipe_ndx
        );
    }

    let buffer: &[i16] = cast_slice(u8_buffer);
    let mut mbuffer: Vec<f32> = Vec::with_capacity(buffer.len() / (streams * 2));

    let mut riq: Vec<(f32, f32)> = Vec::with_capacity(streams - 1);

    for theta in thetas {
        riq.push((theta.cos(), theta.sin()));
    }

    let mul = streams * 2;

    for x in 0..buffer.len() / mul {
        let mut ai: f32 = buffer[x * mul + 0] as f32 / 2049.0 * amplitudes[0];
        let mut aq: f32 = buffer[x * mul + 1] as f32 / 2049.0 * amplitudes[0];
        
        for si in 1..streams {
            let (ri, rq) = riq[si - 1];
            let bi = buffer[x * mul + si * 2 + 0] as f32 / 2049.0 * amplitudes[si];
            let bq = buffer[x * mul + si * 2 + 1] as f32 / 2049.0 * amplitudes[si];
            // multiply two complex numbers
            // a = bi
            // b = bq
            // c = ri
            // d = rq
            // (a + ib)(c + id) = (ac - bd) + i(ad + bc)
            // rotates b then adds b to a
            ai = bi * ri - bq * rq + ai;
            aq = bi * rq + bq * ri + aq;
        }

        mbuffer.push((ai * ai + aq * aq).sqrt());
    }

    process_stream_mfloat32(&mbuffer, &buffer, thetas, amplitudes, streams, pipe_ndx)
}

/// Process buffer using many iterations and translate results into messages.
///
/// This function uses many randomized iterations to process the buffer to
/// look for messages. It also supports setting a custom set of thetas for
/// each pipe/cycle. At the end, it translate the raw bytes into a message
/// format.
pub fn process_buffer(
    u8_buffer: &[u8],
    bit_error_table: &HashMap<u32, u16>,
    pipe_theta: &Vec<Option<Vec<f32>>>,
    pipe_amps: &Vec<Option<Vec<f32>>>,
    streams: usize,
    seen: &Arc<Mutex<HashMap<u32, Instant>>>,
    base_pipe_ndx: usize
) -> Vec<Message> {
    let buffer: &[i16] = cast_slice(u8_buffer);
    let mut mbuffer: Vec<f32> = Vec::with_capacity(buffer.len() / 4);
    let mut rng = rand::thread_rng();
    let mut hm: HashMap<usize, ProcessStreamResult> = HashMap::new();

    let mut thetas: Vec<f32> = Vec::with_capacity(streams - 1);
    let mut amplitudes: Vec<f32> = Vec::with_capacity(streams);

    // Default to 1.0 for amplitudes.
    for _ in 0..streams {
        amplitudes.push(1.0f32);
    }

    // This gets filled out later.
    for _ in 1..streams {
        thetas.push(0.0f32);
    }

    for pipe_ndx in 0..pipe_theta.len() {
        match &pipe_theta[pipe_ndx] {
            None => {
                // Assign random thetas for each element/stream.
                for i in 0..streams - 1 {
                    thetas[i] = rng.r#gen::<f32>() * std::f32::consts::PI * 2.0f32 - std::f32::consts::PI;
                }

                for i in 0..streams {
                    amplitudes[i] = 1.0; //rng.r#gen::<f32>();
                }
            },
            Some(thetas_other) => {
                if thetas_other.len() != streams - 1 {
                    panic!("Expected the count of thetas to be minus one the streams.")
                }

                for i in 0..streams - 1 {
                    thetas[i] = thetas_other[i];
                }

                match &pipe_amps[pipe_ndx] {
                    None => {
                        for i in 0..streams {
                            amplitudes[i] = 1.0;
                        }                        
                    },
                    Some(amps) => {
                        if amps.len() != streams {
                            println!("amps.len(): {} {}", amps.len(), streams);
                            panic!("The provided amplitude array must match the number of streams.");
                        }

                        for i in 0..streams {
                            amplitudes[i] = amps[i];
                        }
                    },
                }           
            },
        };

        let results = process_buffer_single(u8_buffer, &thetas, &amplitudes, streams, base_pipe_ndx + pipe_ndx);

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
