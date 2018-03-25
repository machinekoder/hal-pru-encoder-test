from machinekit import hal
from machinekit import rtapi as rt


def setup_servo(name='servo', thread='base_thread', encoder='eQEP0',
                encoderScale=2797.0,
                pwmDown='hpg.pwmgen.00.out.00',
                pwmUp='hpg.pwmgen.00.out.01',
                enableDown='bb_gpio.p9.out-15',
                enableUp='bb_gpio.p9.out-17'):

    sigPgain = hal.newsig('%s-pgain' % name, hal.HAL_FLOAT)
    sigIgain = hal.newsig('%s-igain' % name, hal.HAL_FLOAT)
    sigDgain = hal.newsig('%s-dgain' % name, hal.HAL_FLOAT)
    sigCmdVel = hal.newsig('%s-cmd-vel' % name, hal.HAL_FLOAT)
    sigPwmIn = hal.newsig('%s-pwm-in' % name, hal.HAL_FLOAT)
    sigPos = hal.newsig('%s-pos' % name, hal.HAL_FLOAT)
    sigVel = hal.newsig('%s-vel' % name, hal.HAL_FLOAT)
    sigAcc = hal.newsig('%s-acc' % name, hal.HAL_FLOAT)
    sigUp = hal.newsig('%s-pwm-up' % name, hal.HAL_FLOAT)
    sigDown = hal.newsig('%s-pwm-down' % name, hal.HAL_FLOAT)
    sigEnable = hal.newsig('%s-enable' % name, hal.HAL_BIT)
    sigPwmEn = hal.newsig('%s-pwm-enable' % name, hal.HAL_BIT)

    sigTuneStart = hal.newsig('%s-tune-start' % name, hal.HAL_BIT)
    sigTuneMode = hal.newsig('%s-tune-mode' % name, hal.HAL_BIT)

    # 43.7:1 gear
    # encoder resolution of 64 counts per revolution of the motor shaft,
    # 2797 counts per revolution of the gearboxs output shaft.

    # for encoder0.position in shaft revs:
    try:
        hal.Pin('%s.position-scale' % encoder).set(encoderScale)
    except RuntimeError:
        hal.Pin('%s.scale' % encoder).set(encoderScale)
    try:
        hal.Pin('%s.min-speed-estimate' % encoder).set(0.001)
        hal.Pin('%s.capture-prescaler' % encoder).set(5)
    except RuntimeError:
        pass

    # feed into PID
    sigPos.link('%s.position' % encoder)
    # for UI feedback
    sigVel.link('%s.velocity' % encoder)

    # ddt for accel
    ddt = rt.newinst('ddt', 'ddt.%s-acc' % name)
    hal.addf(ddt.name, thread)
    ddt.pin('in').link(sigVel)
    ddt.pin('out').link(sigAcc)

    # PID
    pid = rt.newinst('at_pid', 'pid.%s-vel' % name)
    hal.addf('%s.do-pid-calcs' % pid.name, thread)
    pid.pin('maxoutput').set(1.0)  # set maxout to prevent windup effect
    pid.pin('Pgain').link(sigPgain)
    pid.pin('Igain').link(sigIgain)
    pid.pin('Dgain').link(sigDgain)
    pid.pin('command').link(sigCmdVel)
    pid.pin('output').link(sigPwmIn)
    pid.pin('feedback').link(sigVel)
    pid.pin('enable').link(sigEnable)

    # auto tuning
    pid.pin('tuneCycles').set(200)
    pid.pin('tuneEffort').set(0.15)
    pid.pin('tuneMode').link(sigTuneMode)
    pid.pin('tuneStart').link(sigTuneStart)

    # automatically start auto tuning when switched to tune mode
    timedelay = rt.newinst('timedelay', 'timedelay.%s' % sigTuneStart.name)
    hal.addf(timedelay.name, thread)
    timedelay.pin('in').link(sigTuneMode)
    timedelay.pin('on-delay').set(0.1)
    timedelay.pin('off-delay').set(0.0)

    # convert out singnal to IO
    outToIo = rt.newinst('out_to_io', 'out-to-io.%s' % sigTuneStart.name)
    hal.addf(outToIo.name, thread)
    timedelay.pin('out').link(outToIo.pin('in-bit'))
    outToIo.pin('out-bit').link(sigTuneStart)

    # reset the tune mode to false once tuning is finished
    reset = rt.newinst('reset', 'reset.%s' % sigTuneMode.name)
    hal.addf(reset.name, thread)
    reset.pin('out-bit').link(sigTuneMode)
    reset.pin('reset-bit').set(False)
    reset.pin('trigger').link(sigTuneStart)
    reset.pin('rising').set(False)
    reset.pin('falling').set(True)

    # hbridge
    hbridge = rt.newinst('hbridge', 'hbridge.%s' % name)
    hal.addf(hbridge.name, thread)
    hbridge.pin('up').link(sigUp)
    hbridge.pin('down').link(sigDown)
    hbridge.pin('enable').link(sigEnable)
    hbridge.pin('enable-out').link(sigPwmEn)
    hbridge.pin('command').link(sigPwmIn)

    # PWM signals
    hal.Pin('%s.value' % pwmUp).link(sigUp)
    hal.Pin('%s.value' % pwmDown).link(sigDown)

    # Enable
    hal.Pin(enableUp).link(sigPwmEn)
    hal.Pin(enableDown).link(sigPwmEn)
    hal.Pin('%s.enable' % pwmUp).link(sigPwmEn)
    hal.Pin('%s.enable' % pwmDown).link(sigPwmEn)

    # prevent pid runup if disabled
    sigEnable.set(True)

    # storage
    hal.Pin('storage.%s.pgain' % name).link(sigPgain)
    hal.Pin('storage.%s.igain' % name).link(sigIgain)
    hal.Pin('storage.%s.dgain' % name).link(sigDgain)
