import types
import sys
import struct
import time
import random


class FakeBus:
    def __init__(self, bus_id=0, err_rate=0.05, corrupt_rate=0.03, wrap_rate=0.02):
        self._cnt = 0
        self._calls = 0
        self.err_rate = err_rate
        self.corrupt_rate = corrupt_rate
        self.wrap_rate = wrap_rate

    def write_byte_data(self, addr, reg, val):
        return 0

    def write_i2c_block_data(self, addr, reg, data):
        return 0

    def read_i2c_block_data(self, addr, reg, length):
        # emulate realistic I2C latency
        time.sleep(0.02 + random.random() * 0.03)
        self._calls += 1

        # intermittent OSError to exercise retry/fallback logic
        if random.random() < self.err_rate:
            raise OSError("Simulated I2C bus error")

        # sometimes simulate a wrap by jumping a large amount
        if random.random() < self.wrap_rate:
            self._cnt = (self._cnt + 0xF0000000) & 0xFFFFFFFF
        else:
            self._cnt = (self._cnt + 100) & 0xFFFFFFFF

        vals = [self._cnt, (self._cnt + 2) & 0xFFFFFFFF, (self._cnt + 4) & 0xFFFFFFFF, (self._cnt + 6) & 0xFFFFFFFF]

        packed = struct.pack('<IIII', *[v & 0xFFFFFFFF for v in vals])

        # occasionally return corrupted/short reads
        if random.random() < self.corrupt_rate:
            # return only part of the bytes to force unpack errors
            n = max(1, int(len(packed) * random.random()))
            return list(packed[:n])

        # convert to signed 32-bit ints in byte stream as controller expects
        signed_vals = [int(v if v < 0x80000000 else v - 0x100000000) for v in vals]
        return list(struct.pack('<iiii', *signed_vals))


def main():
    # Seed randomness for reproducible stress runs
    random.seed(12345)

    fake = types.SimpleNamespace(SMBus=lambda bus_id=None: FakeBus(bus_id))
    sys.modules['smbus'] = fake

    # import and run the controller (module runs its test on import)
    import nav2_compatible_velocity_controller  # noqa: E402


if __name__ == '__main__':
    main()
