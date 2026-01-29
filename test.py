'''
This reads the output by the `main.rs` program.
'''
import struct
import numpy as np

MODES_PREAMBLE_US = 8
MODES_PREAMBLE_SAMPLES = MODES_PREAMBLE_US * 2
MODES_LONG_MSG_BYTES = 14
MODES_SHORT_MSG_BYTES = 7
MODES_LONG_MSG_BITS = MODES_LONG_MSG_BYTES * 8
MODES_SHORT_MSG_BITS = MODES_SHORT_MSG_BYTES * 8
MODES_LONG_MSG_SAMPLES = MODES_LONG_MSG_BITS * 2
MODES_SHORT_MSG_SAMPLES = MODES_SHORT_MSG_BITS * 2
AIS_CHARSET = "?ABCDEFGHIJKLMNOPQRSTUVWXYZ????? ???????????????0123456789??????"

MODES_CHECKSUM_TABLE = [
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
]

def modes_compute_crc(msg):
    # len(msg) is either long or short (only two lengths possible)

    bits = len(msg) * 8

    assert len(msg) == MODES_LONG_MSG_BYTES or len(msg) == MODES_SHORT_MSG_BYTES

    if bits == 112:
        offset = 0
    else:
        offset = 112 - 56

    crc = 0    

    # subtract 24 so we don't process the CRC field
    for j in range(bits - 24):
        bytendx = int(j // 8)
        bitndx = j % 8
        bitmask = 1 << (7 - bitndx)

        if msg[bytendx] & bitmask != 0:
            crc = crc ^ MODES_CHECKSUM_TABLE[offset + j]

    return crc & 0xffffff

def modes_checksum(msg):
    crc = modes_compute_crc(msg)
    rem = (msg[-3] << 16) | (msg[-2] << 8) | msg[-1]
    return (crc ^ rem) & 0xffffff

def modes_init_error_info():
    msg = [0] * MODES_LONG_MSG_BYTES

    bit_error_table = {}

    for i in range(5, MODES_LONG_MSG_BITS):
        bytepos0 = i >> 3
        mask0 = 1 << (7 - (i & 7))
        msg[bytepos0] = msg[bytepos0] ^ mask0
        crc = modes_checksum(msg)

        bit_error_table[crc] =  (i,)

        for j in range(i + 1, MODES_LONG_MSG_BITS):
            bytepos1 = j >> 3
            mask1 = 1 << (7 - (j & 7))
            msg[bytepos1] = msg[bytepos1] ^ mask1
            crc = modes_checksum(msg)

            bit_error_table[crc] = (i, j)

            msg[bytepos1] = msg[bytepos1] ^ mask1
        msg[bytepos0] = msg[bytepos0] ^ mask0

    return bit_error_table

def fix_bit_errors(msg, bit_error_table):
    syndrome = modes_checksum(msg)
    pei = bit_error_table.get(syndrome, None)
    
    if pei is None:
        return 0
    
    offset = MODES_LONG_MSG_BITS - len(msg) * 8

    for _bitpos in pei:
        bitpos = _bitpos - offset
        if bitpos < 0:
            return 0

    for _bitpos in pei:
        bitpos = _bitpos - offset
        msg[bitpos >> 3] = msg[bitpos >> 3] ^ (1 << (7 - (bitpos & 7)))
    
    return len(pei)

class Message:
    def __init__(self, msg, samples, ndx, snr, theta, aa, ab):
        self.msg = msg
        self.samples = samples
        self.ndx = ndx
        self.snr = snr
        self.theta = theta  # theta for antenna B for phase shift
        self.aa = aa        # amplitude of antenna A
        self.ab = ab        # amplitude of antenna B

def bits_to_bytes(bits):
    thebyte = 0
    out = []
    for x in range(len(bits)):
        if bits[x]:
            thebyte = thebyte | 1
        if x & 7 == 7:
            out.append(thebyte)
            thebyte = 0
        thebyte = thebyte << 1
    return out

def demod(sam):
    p = sam[:MODES_PREAMBLE_SAMPLES]
    payload = sam[MODES_PREAMBLE_SAMPLES:MODES_PREAMBLE_SAMPLES + MODES_LONG_MSG_SAMPLES]
    valid = p[0] > p[1] and p[1] < p[2] and p[2] > p[3] and p[3] < p[0] and p[4] < p[0] and p[5] < p[0] and p[6] < p[0] and p[7] > p[8] and p[8] < p[9] and p[9] > p[6]

    if not valid:
        return

    high = (p[0] + p[2] + p[7] + p[9]) / 6
    if p[4] >= high or p[5] >= high:
        return

    if p[11] > high or p[12] > high or p[13] > high or p[14] > high:
        return

    snr = (p[0] - p[1]) + (p[2] - p[3]) + (p[7] - p[6]) + (p[9] - p[8])

    msg = []

    for y in range(int(len(payload) / 2)):
        a = payload[y * 2 + 0]
        b = payload[y * 2 + 1]
        if a > b:
            msg.append(1)
        else:
            msg.append(0)

    return snr, bits_to_bytes(msg)

def process_message(message):
    pass

def main():
    q = 0
    lq = None
    with open('output.bin', 'rb') as fd:
        while True:
            msg_len_raw = fd.read(2)
            if len(msg_len_raw) == 0:
                break
            msg_len = struct.unpack('<H', msg_len_raw)[0]
            msg = list(fd.read(msg_len))
            samples_len = struct.unpack('<H', fd.read(2))[0]
            samples = fd.read(samples_len * 2)
            samples = np.ndarray(samples_len, np.int16, samples)
            fmt = '<Qffff'
            hdrsz = struct.calcsize(fmt)
            ndx, snr, theta, aa, ab = struct.unpack(fmt, fd.read(hdrsz))
            if aa != 1.0 or ab == 0.0:
                # skip only antenna A
                # skip only antenna B
                continue
            m = Message(msg, samples, ndx, snr, theta, aa, ab)
            msgtype = msg[0] >> 3

            if lq is not None and lq == msg:
                print('dup', ndx, lqndx)

            lq = msg
            lqndx = ndx 

            #print(ndx)
            '''
            ai = samples[0::4] / 2049.0
            aq = samples[1::4] / 2049.0
            bi = samples[2::4] / 2049.0
            bq = samples[3::4] / 2049.0
            a = ai + 1j * aq
            b = bi + 1j * bq
            c = a * aa + b * np.exp(1j * theta) * ab
            cm = np.abs(c)
            res = demod(cm)
            if res is None:
                print('failed')
            '''

            if modes_checksum(msg) != 0:
                # skip anything without a good CRC
                continue

            addr = (msg[1] << 16) | (msg[2] << 8) | msg[3]
            
            q += 1

if __name__ == '__main__':
    main()