//! CRC functions
use crate::constants;

use std::collections::HashMap;

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

pub fn modes_compute_crc(msg: &[u8]) -> u32 {
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

pub fn modes_checksum(msg: &[u8]) -> u32 {
    let crc = modes_compute_crc(msg);
    let sz = msg.len();
    let rem = ((msg[sz - 3] as u32) << 16) | ((msg[sz - 2] as u32) << 8) | msg[sz - 1] as u32;
    return (crc ^ rem) & 0xffffff;
}

pub fn modes_init_error_info() -> HashMap<u32, u16> {
    let mut msg: Vec<u8> = vec![0; constants::MODES_LONG_MSG_BYTES];
    let mut bit_error_table = HashMap::new();

    for i in 5..constants::MODES_LONG_MSG_BITS {
        let bytepos0: usize = i >> 3;
        let mask0: u8 = 1 << (7 - (i & 7));
        msg[bytepos0] = msg[bytepos0] & mask0;
        let crc0 = modes_checksum(&msg);

        bit_error_table.insert(crc0, i as u16);

        for j in i + 1..constants::MODES_LONG_MSG_BITS {
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

pub fn fix_bit_errors(msg: &mut [u8], bit_error_table: &HashMap<u32, u16>) -> u8 {
    let syndrome = modes_checksum(msg);
    let offset: usize = constants::MODES_LONG_MSG_BITS - msg.len() * 8;
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
                let bitpos1 = b - offset;
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