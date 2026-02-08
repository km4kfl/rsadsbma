'''
This reads the output by the `main.rs` program.
'''
import struct
import numpy as np
import math
import matplotlib.pyplot as plt
import pickle

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

def process_msg(msg, snr, bit_error_table):
    is_long = ((msg[0] >> 3) & 0x10) == 0x10

    if is_long and len(msg) == MODES_SHORT_MSG_BYTES:
        # The message is long but it's a short message. Nothing can be done.
        return

    if not is_long:
        msg = msg[0:MODES_SHORT_MSG_BYTES]

    msgtype = msg[0] >> 3

    crc_syndrome = modes_checksum(msg)
    crc_ok = crc_syndrome == 0

    ca = msg[0] & 7
    addr = (msg[1] << 16) | (msg[2] << 8) | msg[3]
    metype = msg[4] >> 3
    mesub = msg[4] & 7
    fs = msg[0] & 7
    dr = (msg[1] >> 3) & 31
    um = ((msg[1] & 7) << 3) | (msg[2] >> 5)
    _a = ((msg[3] & 0x80) >> 5) | ((msg[2] & 0x02) >> 0) | ((msg[2] & 0x08) >> 3)
    _b = ((msg[3] & 0x02) << 1) | ((msg[3] & 0x08) >> 2) | ((msg[3] & 0x20) >> 5)
    _c = ((msg[2] & 0x01) << 2) | ((msg[2] & 0x04) >> 1) | ((msg[2] & 0x10) >> 4)
    _d = ((msg[3] & 0x01) << 2) | ((msg[3] & 0x04) >> 1) | ((msg[3] & 0x10) >> 4)
    identity = _a * 1000 + _b * 100 + _c * 10 + _d

    data = {
        'msgtype': msgtype,
        'msg': msg,
        'log': [],
        'addr': addr,
        'snr': float(snr),
    }

    if not crc_ok and (msgtype == 11 or msgtype == 17 or msgtype == 18):
        nfixed = fix_bit_errors(msg, bit_error_table)
        if nfixed == 0:
            # The number of bit errors exceeded 1 and 2 or CRC corrupted.
            return    
        print('fixed bit errors', nfixed)
        crc_syndrome = modes_checksum(msg)
        if crc_syndrome != 0:
            print(msgtype, 'didnt fix')
            return
        crc_ok = True
    
    if not crc_ok:
        return

    # https://github.com/antirez/dump1090/
    if msgtype == 0 or msgtype == 4 or msgtype == 16 or msgtype == 20:
        data['altitude'] = decode_ac13_field(msg)
    
    if msgtype == 17 or msgtype == 18:
        if metype >= 1 and metype <= 4:
            aircraft_type = f'{chr(ord('A') + 4 - metype)}{mesub:d}'
            chars = (msg[5] << 16) | (msg[6] << 8) | msg[7]
            flight = []
            for y in range(4):
                flight.append(AIS_CHARSET[chars & 0x3f])
                chars = chars >> 6
            chars = (msg[8] << 16) | (msg[9] << 8) | msg[10]
            for y in range(4):
                flight.append(AIS_CHARSET[chars & 0x3f])
                chars = chars >> 6
            flight = ''.join(flight)
            data['aircraft_type'] = aircraft_type
            data['flight'] = flight
        elif metype >= 5 and metype <= 8:
            # surface position message
            movement = ((msg[4] & 0x7) << 4) | (msg[5] >> 4)
            if movement != 0:
                data['movement'] = movement
            if (msg[5] >> 3) & 1:
                data['ground_track'] = ((msg[5] & 0x07) << 4) | (msg[6] >> 4)
                data['ground_track'] = data['ground_track'] * 360 / 128
            data['f_flag'] =  (msg[6] >> 2) & 1
            data['t_flag'] =  (msg[6] >> 3) & 1
            data['raw_latitude'] = ((msg[6] & 3) << 15) | (msg[7] << 7) | (msg[8] >> 1)
            data['raw_longitude'] = ((msg[8] & 1) << 16) | (msg[9] << 8) | (msg[10])
        elif metype >= 9 and metype <= 18:
            data['f_flag'] =  (msg[6] >> 2) & 1
            data['t_flag'] =  (msg[6] >> 3) & 1
            ac12field = ((msg[5] << 4) | (msg[6] >> 4)) & 0x0FFF
            if ac12field > 0:
                if ac12field & 0x10:
                    n = ((ac12field & 0xfe0) >> 1) | (ac12field & 0x000f)
                    altitude = n * 25 - 1000
                    data['altitude'] = altitude
                else:
                    n = ((ac12field & 0xfc0) << 1) | (ac12field & 0x3f)
                    data['log'].append('altitude not implemented for ac12field in metype >= 9')
                    # This is in the original code but I have not
                    # finished the translation here for the function
                    # ModeAToModeC found in `mode_ac.c`.

                    # ModeAToModeC(decode_id13_field(n))
                    #if n < -12:
                    #    n = 0
                    #altitude = n * 100           
            data['raw_latitude'] = ((msg[6] & 3) << 15) | (msg[7] << 7) | (msg[8] >> 1)
            data['raw_longitude'] = ((msg[8] & 1) << 16) | (msg[9] << 8) | (msg[10])
        elif metype == 19 and mesub >= 1 and mesub <= 4:
            if mesub == 1 or mesub == 2:
                ew_dir = (msg[5] & 4) >> 2
                ew_velocity = ((msg[5] & 3) << 8) | msg[6]
                ns_dir = (msg[7] & 0x80) >> 7
                ns_velocity = ((msg[7] & 0x7f) << 3) | ((msg[8] & 0xe0) >> 5)
                vert_rate_source = (msg[8] & 0x10) >> 4
                vert_rate_sign = (msg[8] & 0x8) >> 3
                vert_rate = ((msg[8] & 7) << 6) | ((msg[9] & 0xfc) >> 2)
                # Compute velocity and angle from the two speed components.
                velocity = math.sqrt(ns_velocity * ns_velocity + ew_velocity * ew_velocity)
                if velocity > 0:
                    ewv = ew_velocity
                    nsv = ns_velocity

                    if ew_dir:
                        ewv *= -1
                    if ns_dir:
                        nsv *= -1
                    heading = math.atan2(ewv,nsv)

                    # Convert to degrees.
                    heading = heading * 360 / (math.pi * 2)
                    # We don't want negative values but a 0-360 scale.
                    if heading < 0: 
                        heading += 360
                else:
                    heading = 0
                data['heading'] = heading
                data['velocity'] = velocity
                data['ew_velocity'] = ewv
                data['ns_velocity'] = nsv
        elif mesub == 3 or mesub == 4:
            if msg[5] & (1 << 2):
                data['heading'] = (360.0 / 128) * (((msg[5] & 3) << 5) | (msg[6] >> 3))
    return data

def cpr_nl_function(lat):
    if lat < 0: lat = -lat              # Table is symmetric about the equator.
    if lat < 10.47047130: return 59
    if lat < 14.82817437: return 58
    if lat < 18.18626357: return 57
    if lat < 21.02939493: return 56
    if lat < 23.54504487: return 55
    if lat < 25.82924707: return 54
    if lat < 27.93898710: return 53
    if lat < 29.91135686: return 52
    if lat < 31.77209708: return 51
    if lat < 33.53993436: return 50
    if lat < 35.22899598: return 49
    if lat < 36.85025108: return 48
    if lat < 38.41241892: return 47
    if lat < 39.92256684: return 46
    if lat < 41.38651832: return 45
    if lat < 42.80914012: return 44
    if lat < 44.19454951: return 43
    if lat < 45.54626723: return 42
    if lat < 46.86733252: return 41
    if lat < 48.16039128: return 40
    if lat < 49.42776439: return 39
    if lat < 50.67150166: return 38
    if lat < 51.89342469: return 37
    if lat < 53.09516153: return 36
    if lat < 54.27817472: return 35
    if lat < 55.44378444: return 34
    if lat < 56.59318756: return 33
    if lat < 57.72747354: return 32
    if lat < 58.84763776: return 31
    if lat < 59.95459277: return 30
    if lat < 61.04917774: return 29
    if lat < 62.13216659: return 28
    if lat < 63.20427479: return 27
    if lat < 64.26616523: return 26
    if lat < 65.31845310: return 25
    if lat < 66.36171008: return 24
    if lat < 67.39646774: return 23
    if lat < 68.42322022: return 22
    if lat < 69.44242631: return 21
    if lat < 70.45451075: return 20
    if lat < 71.45986473: return 19
    if lat < 72.45884545: return 18
    if lat < 73.45177442: return 17
    if lat < 74.43893416: return 16
    if lat < 75.42056257: return 15
    if lat < 76.39684391: return 14
    if lat < 77.36789461: return 13
    if lat < 78.33374083: return 12
    if lat < 79.29428225: return 11
    if lat < 80.24923213: return 10
    if lat < 81.19801349: return 9
    if lat < 82.13956981: return 8
    if lat < 83.07199445: return 7
    if lat < 83.99173563: return 6
    if lat < 84.89166191: return 5
    if lat < 85.75541621: return 4
    if lat < 86.53536998: return 3
    if lat < 87.00000000: return 2
    else:
        return 1

def cpr_mod_function(a, b):
    res = a % b
    if res < 0:
        res += b
    return res

def cpr_n_function(lat, isodd):
    nl = cpr_nl_function(lat) - isodd
    if nl < 1:
        nl = 1
    return nl

def cpr_dlon_function(lat, isodd):
    return 360.0 / cpr_n_function(lat, isodd)

def decode_cpr(even_cprlat, even_cprlon, even_cprts, odd_cprlat, odd_cprlon, odd_cprts):
    air_dlat0 = 360.0 / 60.0
    air_dlat1 = 360.0 / 59.0
    lat0 = even_cprlat
    lat1 = odd_cprlat
    lon0 = even_cprlon
    lon1 = odd_cprlon

    j = math.floor(((59.0 * lat0 - 60.0 * lat1) / 131072.0) + 0.5)
    rlat0 = air_dlat0 * (cpr_mod_function(j, 60.0) + lat0 / 131072.0)
    rlat1 = air_dlat1 * (cpr_mod_function(j, 59.0) + lat1 / 131072.0)

    if rlat0 >= 270.0:
        rlat0 -= 360.0
    if rlat1 >= 270.0:
        rlat1 -= 360.0

    if cpr_nl_function(rlat0) != cpr_nl_function(rlat1):
        return
    
    if even_cprts > odd_cprts:
        ni = cpr_n_function(rlat0, 0)
        m = math.floor((((lon0 * (cpr_nl_function(rlat0) - 1)) - (lon1 * cpr_nl_function(rlat0))) / 131072.0) + 0.5)
        lon = cpr_dlon_function(rlat0, 0) * (cpr_mod_function(m, ni) + lon0 / 131072.0)
        lat = rlat0
    else:
        ni = cpr_n_function(rlat1, 1);
        m = math.floor((((lon0 * (cpr_nl_function(rlat1) - 1)) - (lon1 * cpr_nl_function(rlat1))) / 131072.0) + 0.5)
        lon = cpr_dlon_function(rlat1, 1) * (cpr_mod_function(m, ni) + lon1 / 131072.0)
        lat = rlat1
    if lon > 180:
        lon -= 360.0
    
    return lat, lon

def main():
    q = 0
    byaddr = {}
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
            fmt = '<Qf'
            hdrsz = struct.calcsize(fmt)
            ndx, snr = struct.unpack(fmt, fd.read(hdrsz))
            cnt = fd.read(1)[0]
            thetas = []
            for _ in range(cnt):
                thetas.append(struct.unpack('<f', fd.read(4))[0])
            cnt = fd.read(1)[0]
            amps = []
            for _ in range(cnt):
                amps.append(struct.unpack('<f', fd.read(4))[0])

            msgtype = msg[0] >> 3
            addr = (msg[1] << 16) | (msg[2] << 8) | msg[3]

            if addr not in byaddr:
                byaddr[addr] = []
            
            byaddr[addr].append((ndx, msg, thetas))

            ai = samples[0::8] / 2049.0
            aq = samples[1::8] / 2049.0
            bi = samples[2::8] / 2049.0
            bq = samples[3::8] / 2049.0
            ci = samples[4::8] / 2049.0
            cq = samples[5::8] / 2049.0
            di = samples[6::8] / 2049.0
            dq = samples[7::8] / 2049.0
            a = ai + 1j * aq
            b = bi + 1j * bq
            c = ci + 1j * cq
            d = di + 1j * dq
            e = a + b * np.exp(1j * thetas[0]) + c * np.exp(1j * thetas[1]) + d * np.exp(1j * thetas[2])

            e = np.abs(e)

            print(e[0:16])

            snr, msg = demod(e)
            print(snr, msg)
            exit()

            best_snr = snr

            if modes_checksum(msg) != 0:
                # skip anything without a good CRC
                continue
            
            q += 1

    bit_error_table = modes_init_error_info()
    
    sps = 2e6

    fd = open('trainingdata.pickle', 'ab')

    x = []
    y = []

    for addr in byaddr:
        out = []
        msgs = byaddr[addr]
        even_cprlat = None
        even_cprlon = None
        even_cprts = None
        odd_cprlat = None
        odd_cprlon = None
        odd_cprts = None
        alt = None
        heading = None
        velocity = None
        for ndx, msg, thetas in msgs:
            data = process_msg(msg, 0.0, bit_error_table)
            if 'raw_latitude' in data:
                raw_lat = data['raw_latitude']
                raw_lon = data['raw_longitude']
                if data['f_flag']:
                    odd_cprlat = raw_lat
                    odd_cprlon = raw_lon
                    odd_cprts = ndx / sps
                    ts = ndx / sps 
                else:
                    even_cprlat = raw_lat
                    even_cprlon = raw_lon
                    even_cprts = ndx / sps
                    ts = ndx / sps 
                if odd_cprts is not None and even_cprts is not None:
                    delta = abs(even_cprts - odd_cprts)
                    if delta < 10.0:
                        res = decode_cpr(even_cprlat, even_cprlon, even_cprts, odd_cprlat, odd_cprlon, odd_cprts)
                        if res is not None:
                            lat, lon = res
                            out.append((3, lat, lon, ts))
                            if 'altitude' in data:
                                alt = data['altitude']
                                out.append((1, alt, ts))
                            out.append((4, thetas, ts))
            else:
                ts = ndx / sps
                # heading, velocity, altitude
                if 'heading' in data:
                    heading = data['heading']
                    out.append((0, heading, ts))
                    out.append((4, thetas, ts))
                if 'altitude' in data:
                    altitude = data['altitude']
                    out.append((1, altitude, ts))
                    out.append((4, thetas, ts))
                if 'velocity' in data:
                    velocity = data['velocity']
                    out.append((2, velocity, ts))
                    out.append((4, thetas, ts))

        print(f'==={addr}===')
        head = None
        head_ts = 0
        vel = None
        vel_ts = 0
        alt = None
        alt_ts = 0
        lat = None
        lon = None
        lat_ts = 0
        for item in out:
            if item[0] == 0:
                head = item[1]
                head_ts = item[2]
            elif item[0] == 1:
                alt = item[1]
                alt_ts = item[2]
            elif item[0] == 2:
                vel = item[1]
                vel_ts = item[2]
            elif item[0] == 3:
                lat = item[1]
                lon = item[2]
                lat_ts = item[3]
            
            if head is not None and alt is not None and vel is not None and lat is not None:
                ts_min = min(head_ts, alt_ts, vel_ts, lat_ts)
                ts_max = max(head_ts, alt_ts, vel_ts, lat_ts)
                delta = ts_max - ts_min
                if delta < 10:
                    #print(head, alt, vel, lat, lon, ts_min)
                    if item[0] == 4:
                        thetas = item[1]
                        ts = item[2]
                        # lat, lon, thetas
                        # 32.59318 -86.07717
                        hlat = 32.59318
                        hlon = -86.07717
                        print(lat, lon, thetas)
                        x.append(lat - hlat)
                        y.append(lon - hlon)
                        #pickle.dump([lat, lon, thetas], fd)

    plt.scatter(x, y)
    plt.show()

if __name__ == '__main__':
    main()