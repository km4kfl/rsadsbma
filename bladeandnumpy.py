import bladerf
import numpy as np
import numpy.typing as npt

class BladeRFAndNumpy(bladerf.BladeRF):
    """A helper class that assists in converting the raw samples from the BladeRF
    card into a Numpy array of floating point numbers. *It currently does not support
    8-bit samples but could easily be modified to do so.*
    """
    def sample_as_null(
            self,
            samps: int,
            chan_cnt: int,
            samp_size: int,
            trash_samps: int = 1000
            ):
        samps = int(samps)
        samp_size = int(samp_size)
        chan_cnt = int(chan_cnt)
        trash_samps = int(trash_samps)

        debug_tail = 8
        total_bytes = \
            (trash_samps + samps) * (samp_size * chan_cnt) + debug_tail
        buf = bytes(total_bytes)

        self.sync_rx(
            buf,
            (trash_samps + samps) * chan_cnt,
            timeout_ms=5000
        )

        if chan_cnt == 2:
            return
        elif chan_cnt == 1:
            return
        else:
            raise Exception('unexpected channel count')

    def sample_as_bytes(
            self,
            samps: int,
            chan_cnt: int,
            samp_size: int,
            trash_samps: int = 1000
            ):
        samps = int(samps)
        samp_size = int(samp_size)
        chan_cnt = int(chan_cnt)
        trash_samps = int(trash_samps)

        debug_tail = 8
        total_bytes = \
            (trash_samps + samps) * (samp_size * chan_cnt) + debug_tail
        buf = bytes(total_bytes)

        self.sync_rx(
            buf, 
            (trash_samps + samps) * chan_cnt,
            timeout_ms=5000
        )

        assert (buf[-debug_tail:] == b'\x00' * debug_tail)

        buf = buf[trash_samps * (samp_size * chan_cnt):-debug_tail]

        return buf

    def sample_as_f64(
            self,
            samps: int,
            chan_cnt: int,
            samp_size: int,
            trash_samps: int = 1000
            ):
        samps = int(samps)
        samp_size = int(samp_size)
        chan_cnt = int(chan_cnt)
        trash_samps = int(trash_samps)

        debug_tail = 8
        total_bytes = \
            (trash_samps + samps) * (samp_size * chan_cnt) + debug_tail
        buf = bytes(total_bytes)

        self.sync_rx(
            buf, 
            (trash_samps + samps) * chan_cnt,
            timeout_ms=5000
        )

        assert (buf[-debug_tail:] == b'\x00' * debug_tail)

        buf = buf[trash_samps * (samp_size * chan_cnt):-debug_tail]

        sbuf = np.ndarray((len(buf) // 2,), buffer=buf, dtype=np.int16)
        del buf

        sbuf = sbuf.astype(np.float64) / 2049

        if chan_cnt == 2:
            b0 = sbuf[0::2][0::2] + 1j * sbuf[1::2][0::2]
            b1 = sbuf[0::2][1::2] + 1j * sbuf[1::2][1::2]
            return b0, b1
        elif chan_cnt == 1:
            b0 = sbuf[0::2] + 1j * sbuf[1::2]
            assert len(b0) == samps
            return b0
        else:
            raise Exception('unexpected channel count')

    def sample_as_f32(
            self,
            samps: int,
            chan_cnt: int,
            samp_size: int,
            trash_samps: int = 1000
            ):
        samps = int(samps)
        samp_size = int(samp_size)
        chan_cnt = int(chan_cnt)
        trash_samps = int(trash_samps)

        debug_tail = 8
        total_bytes = \
            (trash_samps + samps) * (samp_size * chan_cnt) + debug_tail
        buf = bytes(total_bytes)

        self.sync_rx(
            buf, 
            (trash_samps + samps) * chan_cnt,
            timeout_ms=5000
        )

        assert (buf[-debug_tail:] == b'\x00' * debug_tail)

        buf = buf[trash_samps * (samp_size * chan_cnt):-debug_tail]

        sbuf = np.ndarray((len(buf) // 2,), buffer=buf, dtype=np.int16)
        del buf

        sbuf = sbuf.astype(np.float32) / 2049

        if chan_cnt == 2:
            b0 = sbuf[0::2][0::2] + 1j * sbuf[1::2][0::2]
            b1 = sbuf[0::2][1::2] + 1j * sbuf[1::2][1::2]
            return b0, b1
        elif chan_cnt == 1:
            b0 = sbuf[0::2] + 1j * sbuf[1::2]
            return b0
        else:
            raise Exception('unexpected channel count')

    def sample_as_f64_with_meta(
            self,
            samps: int,
            chan_cnt: int,
            samp_size: int,
            trash_samps: int,
            when_timestamp: int,
            ):
        samps = int(samps)
        samp_size = int(samp_size)
        chan_cnt = int(chan_cnt)
        trash_samps = int(trash_samps)

        debug_tail = 8
        total_bytes = \
            (trash_samps + samps) * (samp_size * chan_cnt) + debug_tail
        buf = bytes(total_bytes)

        print('meta_timestamp', when_timestamp, self.get_timestamp(bladerf.Direction.RX))
        print('samps', samps, 'chan_cnt', chan_cnt, 'trash_samps', trash_samps)
        print('(trash_samps + samps) * chan_cnt', (trash_samps + samps) * chan_cnt)

        status, actual_count, timestamp = self.sync_rx_with_metadata(
            buf, 
            (trash_samps + samps) * chan_cnt,
            meta_flags = 1 << 31,
            meta_timestamp = when_timestamp,
            timeout_ms=20000
        )

        assert (buf[-debug_tail:] == b'\x00' * debug_tail)

        buf = buf[trash_samps * (samp_size * chan_cnt):]

        sbuf = np.ndarray((len(buf) // 2,), buffer=buf, dtype=np.int16)
        del buf

        sbuf = sbuf.astype(np.float64) / 2049

        if chan_cnt == 2:
            b0 = sbuf[0::2][0::2] + 1j * sbuf[1::2][0::2]
            b1 = sbuf[0::2][1::2] + 1j * sbuf[1::2][1::2]
            return b0, b1
        elif chan_cnt == 1:
            b0 = sbuf[0::2] + 1j * sbuf[1::2]
            return b0
        else:
            raise Exception('unexpected channel count')
