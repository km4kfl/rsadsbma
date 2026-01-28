'''
This reads the output by the `main.rs` program.
'''
import struct
import numpy as np

def main():
    with open('output.bin', 'rb') as fd:
        while True:
            msg_len = struct.unpack('<H', fd.read(2))[0]
            msg = fd.read(msg_len)
            samples_len = struct.unpack('<H', fd.read(2))[0]
            samples = fd.read(samples_len * 2)
            samples = np.ndarray(samples_len, np.int16, samples)
            fmt = '<Qffff'
            hdrsz = struct.calcsize(fmt)
            ndx, snr, theta, aa, ab = struct.unpack(fmt, fd.read(hdrsz))
            print(ndx, snr, theta, aa, ab)

if __name__ == '__main__':
    main()