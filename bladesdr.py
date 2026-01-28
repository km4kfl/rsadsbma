import socket
import argparse
import bladerf
from bladeandnumpy import BladeRFAndNumpy

def main(args):
    sps = 2000000

    dev = BladeRFAndNumpy(f'libusb:serial={args.serial}')

    dev.sync_config(
        bladerf._bladerf.ChannelLayout.RX_X2,
        bladerf._bladerf.Format.SC16_Q11,
        num_buffers=16,
        buffer_size=1024 * 32,
        num_transfers=8,
        stream_timeout=20000
    )

    buffer_samps = 16 * 1024 * 32

    for ch in [0]:
        dev.set_gain_mode(ch, bladerf._bladerf.GainMode.Manual)
        dev.set_bias_tee(ch, False)
        dev.set_bandwidth(ch, sps)
        dev.set_sample_rate(ch, sps)
        dev.set_gain(ch, 60)
        dev.enable_module(ch, True)

    dev.set_frequency(0, 1090000000 + args.freq_offset)

    tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_socket.bind(('localhost', 7878))
    tcp_socket.listen(1)

    while True:
        connection, client = tcp_socket.accept()
        print('got client', client)
        # Clear the buffer incase it is dirty.
        dev.sample_as_bytes(buffer_samps, 2, 4, 0)
        try:
            while True:
                connection.sendall(dev.sample_as_bytes(buffer_samps, 2, 4, 0))
        finally:
            connection.close()

if __name__ == '__main__':
    ap = argparse.ArgumentParser(
        description='Listens on 1090MHZ. Writes samples to connected client.'
    )
    ap.add_argument('--serial', type=str, required=True, help='The first few unambigious letters of the serial for the card to use.')
    ap.add_argument('--freq-offset', type=float, required=True, help='Any offset for correction otherwise zero.')
    main(ap.parse_args())