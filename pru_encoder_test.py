import attr
from aenum import Enum

from machinekit import hal
from machinekit import rtapi as rt
from machinekit import config as c

from pinconfig import io_map, pru0_input_pins
from servo import setup_servo

c.load_ini('hardware.ini')

PinDirection = Enum('PinDirection', 'IN OUT LOW HIGH')


@attr.s
class IOPin(object):
    number = attr.ib(default='')
    direction = attr.ib(default=PinDirection.IN)


def get_io_pin(id_, direction=PinDirection.IN):
    port, pin = str(id_)[0], str(id_)[1:]
    dir_ = 'in' if direction is PinDirection.IN else 'out'
    return hal.pins['bb_gpio.p{}.{}-{}'.format(port, dir_, pin)]


def setup_storage():
    hal.loadusr('hal_storage', name='storage', file='storage.ini',
                autosave=True, wait_name='storage')


def read_storage():
    hal.Pin('storage.read-trigger').set(True)  # trigger read


IS1 = io_map[17]
ENC_B = io_map[18]
IS2 = io_map[19]
ENC_A = io_map[20]
EN1 = io_map[21]
EN2 = io_map[22]
IN1 = io_map[23]
IN2 = io_map[24]

output_pins = [EN1, EN2]
input_pins = [IS1, IS2]

output_pins_joined = ','.join(str(x) for x in output_pins)
input_pins_joined = ','.join(str(x) for x in input_pins)

rt.loadrt('hal_bb_gpio', output_pins=output_pins_joined, input_pins=input_pins_joined)
rt.loadrt(c.find('PRUCONF', 'DRIVER'), 'prucode={}/{}'
          .format(c.Config().EMC2_RTLIB_DIR, c.find('PRUCONF', 'PRUBIN')),
          pru=0, num_stepgens=0, num_pwmgens=2, num_encoders=1, halname='hpg')

hpg = hal.Component('hpg')
hpg.pin('pwmgen.00.out.00.pin').set(IN1)
hpg.pin('pwmgen.00.out.01.pin').set(IN2)

hpg.pin('encoder.00.chan.00.A-pin').set(pru0_input_pins[ENC_A])
hpg.pin('encoder.00.chan.00.B-pin').set(pru0_input_pins[ENC_B])
hpg.pin('encoder.00.chan.00.index-pin').set(17)  # ignore
hpg.pin('encoder.00.chan.00.counter-mode').set(0)  # Quadrature Mode

gpio = hal.Component('bb_gpio')


setup_storage()

# we need a thread to execute the component functions
rt.newthread('main-thread', 1000000, fp=True)

hal.addf('bb_gpio.read', 'main-thread')
hal.addf('hpg.capture-position', 'main-thread')

# motor = setup_servo(name='m1',
#                     thread='main-thread',
#                     encoder='hpg.encoder.00.chan.00',
#                     pwmDown='hpg.pwmgen.00.out.00',
#                     pwmUp='hpg.pwmgen.00.out.01',
#                     enableDown=get_io_pin(EN1, PinDirection.OUT).name,
#                     enableUp=get_io_pin(EN2, PinDirection.OUT).name)

hal.addf('hpg.update', 'main-thread')
hal.addf('bb_gpio.write', 'main-thread')

read_storage()

# ready to start the threads
hal.start_threads()

# start haltalk server after everything is initialized
# else binding the remote components on the UI might fail
hal.loadusr('haltalk')
