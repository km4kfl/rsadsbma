'''
This program configures two BladeSDR boards to operate in tandem to provide 
4 RX streams.
'''
import queue
import time
import socket
import argparse
import bladerf
import numpy as np
import threading
from bladeandnumpy import BladeRFAndNumpy

def main(args):
    sps = 2000000

    print('opening master')
    dev_master = BladeRFAndNumpy(f'libusb:serial=9da')
    print('opening slave')
    dev_slave = BladeRFAndNumpy(f'libusb:serial=e85')

    buffer_samps = 16 * 1024 * 32

    print('connecting clocks of master and slave')
    dev_master.set_clock_output(True)
    time.sleep(0.5)
    dev_slave.set_clock_select(bladerf._bladerf.ClockSelect.External)

    print('creating master trigger')
    trig_master = dev_master.trigger_init(0, bladerf._bladerf.TriggerSignal.Trigger_J51_1)
    trig_master.role = bladerf._bladerf.TRIGGER_ROLE_MASTER
    print('creating slave trigger')
    trig_slave = dev_slave.trigger_init(0, bladerf._bladerf.TriggerSignal.Trigger_J51_1)
    trig_slave.role = bladerf._bladerf.TRIGGER_ROLE_SLAVE

    dev_master.trigger_arm(trig_master, 1, 0, 0)
    dev_slave.trigger_arm(trig_slave, 1, 0, 0)

    for dev in [dev_master, dev_slave]:
        print('configuring', dev)
        dev.sync_config(
            bladerf._bladerf.ChannelLayout.RX_X2,
            bladerf._bladerf.Format.SC16_Q11,
            num_buffers=16,
            buffer_size=1024 * 32,
            num_transfers=8,
            stream_timeout=40000
        )

        for ch in [0]:
            dev.set_gain_mode(ch, bladerf._bladerf.GainMode.Manual)
            dev.set_bias_tee(ch, False)
            dev.set_bandwidth(ch, sps)
            dev.set_sample_rate(ch, sps)
            dev.set_gain(ch, 60)
            dev.enable_module(ch, True)
        print('setting frequency')
        dev.set_frequency(0, 1090000000)

    def reader_thread(dev, q):
        print('thread running', dev)
        while True:
            chunk = dev.sample_as_bytes(buffer_samps, 2, 4, 0)
            q.put(chunk)
    
    qm = queue.Queue()
    qs = queue.Queue()

    master_thread = threading.Thread(target = reader_thread, args=(dev_master, qm))
    slave_thread = threading.Thread(target = reader_thread, args=(dev_slave, qs))

    print('...about to fire the trigger')
    time.sleep(1)

    tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_socket.bind(('localhost', 7878))
    tcp_socket.listen(1)

    connection, client = tcp_socket.accept()

    master_thread.start()
    slave_thread.start()    

    dev_master.trigger_fire(trig_master)
    print('trigger fired')    

    # Send the number of streams as a byte.
    connection.sendall(bytes([4]))

    while True:
        st = time.time()
        # I think we should have enough time to service both buffers
        # from the same thread, reinterleave, and then grab the next
        # chunks before the internal buffer overruns.
        #chunk0 = dev_master.sample_as_bytes(buffer_samps, 2, 4, 0)
        #chunk1 = dev_slave.sample_as_bytes(buffer_samps, 2, 4, 0)
        chunk0 = qm.get()
        chunk1 = qs.get()
        #
        # Reinterleave the samples so we have a format like:
        #
        # AABBCCDDAABBCCDD...
        # IQIQIQIQIQIQIQIQ...
        #
        # That will be 4 channels in total from two streams of 2 channels
        # each.
        #
        chunk0 = np.ndarray(int(len(chunk0) // 2), np.int16, chunk0)
        chunk1 = np.ndarray(int(len(chunk1) // 2), np.int16, chunk1)
        chunk2 = np.zeros(chunk0.shape[0] * 2, np.int16)
        chunk2[0::8] = chunk0[0::4]
        chunk2[1::8] = chunk0[1::4]
        chunk2[2::8] = chunk0[2::4]
        chunk2[3::8] = chunk0[3::4]
        chunk2[4::8] = chunk1[0::4]
        chunk2[5::8] = chunk1[1::4]
        chunk2[6::8] = chunk1[2::4]
        chunk2[7::8] = chunk1[3::4]

        et = time.time() - st
        print(et * sps, buffer_samps)
        connection.sendall(chunk2.tobytes())

if __name__ == '__main__':
    ap = argparse.ArgumentParser(
        description='Listens on 1090MHZ. Writes samples to connected client.'
    )
    #ap.add_argument('--serial', type=str, required=True, help='The first few unambigious letters of the serial for the card to use.')
    #ap.add_argument('--freq-offset', type=float, required=True, help='Any offset for correction otherwise zero.')
    main(ap.parse_args())