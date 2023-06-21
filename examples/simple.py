import board
from OPT4001 import OPT4001

i2c = board.I2C()

opt = OPT4001(i2c, conversion_time=10, operating_mode=3)

while True:
    print(f"{opt.lux} \t {opt.read_lux_FIFO(0)} \t {opt.read_lux_FIFO(1)} \t {opt.read_lux_FIFO(2)}")
