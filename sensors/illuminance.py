import smbus
import time

# Constants
SENSOR_ADDRESS = 0x23
CHECK_NUM      = 10
READ_MODE      = 0x10
SCALE_FACTOR   = 1.2
SLEEP_INTERVAL = 1  # in seconds
BYTE_COUNT     = 2  # Number of bytes to read from the sensor

class IlluminanceSensor:

    def __init__(self) -> None:
        self.bus = smbus.SMBus(1)  # Open I2C bus

    # Read data from illuminance sensor
    def _read_lux(self) -> float:
        data = self.bus.read_i2c_block_data(SENSOR_ADDRESS, READ_MODE, BYTE_COUNT)
        lux = (data[0] << 8) + data[1]
        print(f'Illuminance: {lux} lx')
        return lux / SCALE_FACTOR

    # Calculate average illuminance over CHECK_NUM readings
    def _get_average_illuminance(self) -> float:
        total_lux = 0
        for _ in range(CHECK_NUM):
            total_lux += self._read_lux()
            time.sleep(SLEEP_INTERVAL)
        average_lux = total_lux / CHECK_NUM
        return average_lux

    # Check if the average illuminance is above a given threshold
    def is_above_threshold(self, threshold: float) -> bool:
        print('Checking illuminance sensor...')
        average_lux = self._get_average_illuminance()
        return average_lux > threshold

# for test
if __name__ == '__main__':
    illuminancesensor = IlluminanceSensor()
    print(illuminancesensor.is_above_threshold(threshold=20))
    