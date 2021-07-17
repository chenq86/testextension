enum PingUnit {
    //% block="μs"
    MicroSeconds,
    //% block="cm"
    Centimeters,
    //% block="inches"
    Inches
}

enum RGB {
    //% block="红"
    RED,
    //% block="绿"
    GREEN,
    //% block="蓝"
    BLUE,
    //% block="全部"
    CLEAR
}


namespace ICBit {
    /**
     * Send a ping and get the echo time (in microseconds) as a result
     * @param trig tigger pin
     * @param echo echo pin
     * @param unit desired conversion unit
     * @param maxCmDistance maximum distance in centimeters (default is 500)
     */
    //% blockId=sonar_ping block="ping trig %trig|echo %echo|unit %unit"
    export function ping(trig: DigitalPin, echo: DigitalPin, unit: PingUnit, maxCmDistance = 500): number {
        // send pulse
        pins.setPull(trig, PinPullMode.PullNone);
        pins.digitalWritePin(trig, 0);
        control.waitMicros(2);
        pins.digitalWritePin(trig, 1);
        control.waitMicros(10);
        pins.digitalWritePin(trig, 0);

        // read pulse
        const d = pins.pulseIn(echo, PulseValue.High, maxCmDistance * 58);

        switch (unit) {
            case PingUnit.Centimeters: return Math.idiv(d, 58);
            case PingUnit.Inches: return Math.idiv(d, 148);
            default: return d;
        }
    }

    enum LCS_Constants {
        // Constants
        ADDRESS = 0x29,
        ID = 0x12, // Register should be equal to 0x44 for the TCS34721 or TCS34725, or 0x4D for the TCS34723 or TCS34727.

        COMMAND_BIT = 0x80,

        ENABLE = 0x00,
        ENABLE_AIEN = 0x10, // RGBC Interrupt Enable
        ENABLE_WEN = 0x08, // Wait enable - Writing 1 activates the wait timer
        ENABLE_AEN = 0x02, // RGBC Enable - Writing 1 actives the ADC, 0 disables it
        ENABLE_PON = 0x01, // Power on - Writing 1 activates the internal oscillator, 0 disables it
        ATIME = 0x01, // Integration time
        WTIME = 0x03, // Wait time (if ENABLE_WEN is asserted)
        AILTL = 0x04, // Clear channel lower interrupt threshold
        AILTH = 0x05,
        AIHTL = 0x06, // Clear channel upper interrupt threshold
        AIHTH = 0x07,
        PERS = 0x0C, // Persistence register - basic SW filtering mechanism for interrupts
        PERS_NONE = 0x00, // Every RGBC cycle generates an interrupt
        PERS_1_CYCLE = 0x01, // 1 clean channel value outside threshold range generates an interrupt
        PERS_2_CYCLE = 0x02, // 2 clean channel values outside threshold range generates an interrupt
        PERS_3_CYCLE = 0x03, // 3 clean channel values outside threshold range generates an interrupt
        PERS_5_CYCLE = 0x04, // 5 clean channel values outside threshold range generates an interrupt
        PERS_10_CYCLE = 0x05, // 10 clean channel values outside threshold range generates an interrupt
        PERS_15_CYCLE = 0x06, // 15 clean channel values outside threshold range generates an interrupt
        PERS_20_CYCLE = 0x07, // 20 clean channel values outside threshold range generates an interrupt
        PERS_25_CYCLE = 0x08, // 25 clean channel values outside threshold range generates an interrupt
        PERS_30_CYCLE = 0x09, // 30 clean channel values outside threshold range generates an interrupt
        PERS_35_CYCLE = 0x0A, // 35 clean channel values outside threshold range generates an interrupt
        PERS_40_CYCLE = 0x0B, // 40 clean channel values outside threshold range generates an interrupt
        PERS_45_CYCLE = 0x0C, // 45 clean channel values outside threshold range generates an interrupt
        PERS_50_CYCLE = 0x0D, // 50 clean channel values outside threshold range generates an interrupt
        PERS_55_CYCLE = 0x0E, // 55 clean channel values outside threshold range generates an interrupt
        PERS_60_CYCLE = 0x0F, // 60 clean channel values outside threshold range generates an interrupt
        CONFIG = 0x0D,
        CONFIG_WLONG = 0x02, // Choose between short and long (12x) wait times via WTIME
        CONTROL = 0x0F, // Set the gain level for the sensor
        STATUS = 0x13,
        STATUS_AINT = 0x10, // RGBC Clean channel interrupt
        STATUS_AVALID = 0x01, // Indicates that the RGBC channels have completed an integration cycle

        CDATAL = 0x14, // Clear channel data
        CDATAH = 0x15,
        RDATAL = 0x16, // Red channel data
        RDATAH = 0x17,
        GDATAL = 0x18, // Green channel data
        GDATAH = 0x19,
        BDATAL = 0x1A, // Blue channel data
        BDATAH = 0x1B,

        GAIN_1X = 0x00, //  1x gain
        GAIN_4X = 0x01, //  4x gain
        GAIN_16X = 0x02, // 16x gain
        GAIN_60X = 0x03  // 60x gain
    }

    let LCS_integration_time_val = 0

    // I2C functions

    function I2C_WriteReg8(addr: number, reg: number, val: number) {
        let buf = pins.createBuffer(2)
        buf.setNumber(NumberFormat.UInt8BE, 0, reg)
        buf.setNumber(NumberFormat.UInt8BE, 1, val)
        pins.i2cWriteBuffer(addr, buf)
    }

    function I2C_ReadReg8(addr: number, reg: number): number {
        let buf = pins.createBuffer(1)
        buf.setNumber(NumberFormat.UInt8BE, 0, reg)
        pins.i2cWriteBuffer(addr, buf)
        buf = pins.i2cReadBuffer(addr, 1)
        return buf.getNumber(NumberFormat.UInt8BE, 0);
    }

    function I2C_ReadReg16(addr: number, reg: number): number {
        let buf = pins.createBuffer(1)
        buf.setNumber(NumberFormat.UInt8BE, 0, reg)
        pins.i2cWriteBuffer(addr, buf)
        buf = pins.i2cReadBuffer(addr, 2)
        // Little endian
        return ((buf.getNumber(NumberFormat.UInt8BE, 1) << 8) | buf.getNumber(NumberFormat.UInt8BE, 0));
    }

    //% blockId="initialize_sensor" block="初始化颜色传感器"
    export function LCS_initialize() {
        // Make sure we're connected to the right sensor.
        let chip_id = I2C_ReadReg8(LCS_Constants.ADDRESS, (LCS_Constants.COMMAND_BIT | LCS_Constants.ID))

        if (chip_id != 0x44) {
            return // Incorrect chip ID
        }

        // Set default integration time and gain.
        LCS_set_integration_time(0.0048)
        LCS_set_gain(LCS_Constants.GAIN_16X)

        // Enable the device (by default, the device is in power down mode on bootup).
        LCS_enable()
    }

    function LCS_enable() {
        // Set the power and enable bits.
        I2C_WriteReg8(LCS_Constants.ADDRESS, (LCS_Constants.COMMAND_BIT | LCS_Constants.ENABLE), LCS_Constants.ENABLE_PON)
        basic.pause(10) // not sure if this is right    time.sleep(0.01) // FIXME delay for 10ms

        I2C_WriteReg8(LCS_Constants.ADDRESS, (LCS_Constants.COMMAND_BIT | LCS_Constants.ENABLE), (LCS_Constants.ENABLE_PON | LCS_Constants.ENABLE_AEN))
    }

    function LCS_set_integration_time(time: number) {
        let val = 0x100 - (time / 0.0024) // FIXME was cast to int type
        if (val > 255) {
            val = 255
        } else if (val < 0) {
            val = 0
        }
        I2C_WriteReg8(LCS_Constants.ADDRESS, (LCS_Constants.COMMAND_BIT | LCS_Constants.ATIME), val)
        LCS_integration_time_val = val
    }

    function LCS_set_gain(gain: number) {
        I2C_WriteReg8(LCS_Constants.ADDRESS, (LCS_Constants.COMMAND_BIT | LCS_Constants.CONTROL), gain)
    }


    function LCS_set_led_state(state: boolean) {
        I2C_WriteReg8(LCS_Constants.ADDRESS, (LCS_Constants.COMMAND_BIT | LCS_Constants.PERS), LCS_Constants.PERS_NONE)
        let val = I2C_ReadReg8(LCS_Constants.ADDRESS, (LCS_Constants.COMMAND_BIT | LCS_Constants.ENABLE))
        if (state) {
            val |= LCS_Constants.ENABLE_AIEN
        } else {
            val &= ~LCS_Constants.ENABLE_AIEN
        }
        I2C_WriteReg8(LCS_Constants.ADDRESS, (LCS_Constants.COMMAND_BIT | LCS_Constants.ENABLE), val)

        basic.pause(2 * (256 - LCS_integration_time_val) * 2.4) // delay for long enough for there to be new (post-change) complete values available
    }

    //% blockId="getSensorData" block="读取颜色值 %colorId"
    export function getColorData(color: RGB): number {
        basic.pause((256 - LCS_integration_time_val) * 2.4);
        let sum = I2C_ReadReg16(LCS_Constants.ADDRESS, (LCS_Constants.COMMAND_BIT | LCS_Constants.CDATAL));
        let vue = 0;
        switch (color) {
            case RGB.RED:
                vue = I2C_ReadReg16(LCS_Constants.ADDRESS, (LCS_Constants.COMMAND_BIT | LCS_Constants.RDATAL));

                break;
            case RGB.GREEN:
                vue = I2C_ReadReg16(LCS_Constants.ADDRESS, (LCS_Constants.COMMAND_BIT | LCS_Constants.GDATAL));

                break;
            case RGB.BLUE:
                vue = I2C_ReadReg16(LCS_Constants.ADDRESS, (LCS_Constants.COMMAND_BIT | LCS_Constants.BDATAL));

                break;
            case RGB.CLEAR:
                return sum;
                break;

        }
        vue = Math.floor(vue / sum * 255);

        serial.writeLine("val: " + vue);
        return vue;
    }


    function LCS_get_raw_data(delay: boolean = false): number[] {
        if (delay) {
            // Delay for the integration time to allow reading immediately after the previous read.
            basic.pause((256 - LCS_integration_time_val) * 2.4)
        }

        let div = (256 - LCS_integration_time_val) * 1024
        let rgbc = [0, 0, 0, 0]
        rgbc[0] = I2C_ReadReg16(LCS_Constants.ADDRESS, (LCS_Constants.COMMAND_BIT | LCS_Constants.RDATAL)) / div
        rgbc[1] = I2C_ReadReg16(LCS_Constants.ADDRESS, (LCS_Constants.COMMAND_BIT | LCS_Constants.GDATAL)) / div
        rgbc[2] = I2C_ReadReg16(LCS_Constants.ADDRESS, (LCS_Constants.COMMAND_BIT | LCS_Constants.BDATAL)) / div
        rgbc[3] = I2C_ReadReg16(LCS_Constants.ADDRESS, (LCS_Constants.COMMAND_BIT | LCS_Constants.CDATAL)) / div
        if (rgbc[0] > 1) {
            rgbc[0] = 1
        }
        if (rgbc[1] > 1) {
            rgbc[1] = 1
        }
        if (rgbc[2] > 1) {
            rgbc[2] = 1
        }
        if (rgbc[3] > 1) {
            rgbc[3] = 1
        }
        return rgbc
    }


    const PCA9685_ADD = 0x40
    const MODE1 = 0x00
    const MODE2 = 0x01
    const SUBADR1 = 0x02
    const SUBADR2 = 0x03
    const SUBADR3 = 0x04

    const LED0_ON_L = 0x06
    const LED0_ON_H = 0x07
    const LED0_OFF_L = 0x08
    const LED0_OFF_H = 0x09

    const ALL_LED_ON_L = 0xFA
    const ALL_LED_ON_H = 0xFB
    const ALL_LED_OFF_L = 0xFC
    const ALL_LED_OFF_H = 0xFD

    const PRESCALE = 0xFE

    const STP_CHA_L = 2047
    const STP_CHA_H = 4095

    const STP_CHB_L = 1
    const STP_CHB_H = 2047

    const STP_CHC_L = 1023
    const STP_CHC_H = 3071

    const STP_CHD_L = 3071
    const STP_CHD_H = 1023

    let initialized = false




    export enum enPos {
        //% blockId="forward" block="forward"
        forward = 1,
        //% blockId="reverse" block="reverse"
        reverse = 2,
        //% blockId="stop" block="stop"
        stop = 3
    }



    export enum enServo {

        S1 = 0,
        S2,
        S3,
        S4,
        S5,
        S6,
        S7,
        S8
    }
    export enum enMotors {
        M1 = 8,
        M2 = 10,
        M3 = 12,
        M4 = 14
    }

    function i2cwrite(addr: number, reg: number, value: number) {
        let buf = pins.createBuffer(2)
        buf[0] = reg
        buf[1] = value
        pins.i2cWriteBuffer(addr, buf)
    }

    function i2ccmd(addr: number, value: number) {
        let buf = pins.createBuffer(1)
        buf[0] = value
        pins.i2cWriteBuffer(addr, buf)
    }

    function i2cread(addr: number, reg: number) {
        pins.i2cWriteNumber(addr, reg, NumberFormat.UInt8BE);
        let val = pins.i2cReadNumber(addr, NumberFormat.UInt8BE);
        return val;
    }

    function initPCA9685(): void {
        i2cwrite(PCA9685_ADD, MODE1, 0x00)
        setFreq(50);
        initialized = true
    }

    function setFreq(freq: number): void {
        // Constrain the frequency
        let prescaleval = 25000000;
        prescaleval /= 4096;
        prescaleval /= freq;
        prescaleval -= 1;
        let prescale = prescaleval; //Math.Floor(prescaleval + 0.5);
        let oldmode = i2cread(PCA9685_ADD, MODE1);
        let newmode = (oldmode & 0x7F) | 0x10; // sleep
        i2cwrite(PCA9685_ADD, MODE1, newmode); // go to sleep
        i2cwrite(PCA9685_ADD, PRESCALE, prescale); // set the prescaler
        i2cwrite(PCA9685_ADD, MODE1, oldmode);
        control.waitMicros(5000);
        i2cwrite(PCA9685_ADD, MODE1, oldmode | 0xa1);
    }

    function setPwm(channel: number, on: number, off: number): void {
        if (channel < 0 || channel > 15)
            return;
        if (!initialized) {
            initPCA9685();
        }
        let buf = pins.createBuffer(5);
        buf[0] = LED0_ON_L + 4 * channel;
        buf[1] = on & 0xff;
        buf[2] = (on >> 8) & 0xff;
        buf[3] = off & 0xff;
        buf[4] = (off >> 8) & 0xff;
        pins.i2cWriteBuffer(PCA9685_ADD, buf);
    }



    //% blockId=SuperBit_Servo block="Servo(180°)|num %num|value %value"
    //% weight=97
    //% blockGap=10
    //% num.min=1 num.max=4 value.min=0 value.max=180
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=20
    export function Servo(num: enServo, value: number): void {

        // 50hz: 20,000 us
        let us = (value * 1800 / 180 + 600); // 0.6 ~ 2.4
        let pwm = us * 4096 / 20000;
        setPwm(num, 0, pwm);

    }

    //% blockId=SuperBit_Servo3 block="Servo(360°)|num %num|pos %pos|value %value"
    //% weight=96
    //% blockGap=10
    //% num.min=1 num.max=4 value.min=0 value.max=90
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=20
    export function Servo3(num: enServo, pos: enPos, value: number): void {

        // 50hz: 20,000 us

        if (pos == enPos.stop) {
            let us = (86 * 1800 / 180 + 600); // 0.6 ~ 2.4
            let pwm = us * 4096 / 20000;
            setPwm(num, 0, pwm);
        }
        else if (pos == enPos.forward) { //0-90 -> 90 - 0
            let us = ((90 - value) * 1800 / 180 + 600); // 0.6 ~ 2.4
            let pwm = us * 4096 / 20000;
            setPwm(num, 0, pwm);
        }
        else if (pos == enPos.reverse) { //0-90 -> 90 -180
            let us = ((90 + value) * 1800 / 180 + 600); // 0.6 ~ 2.4
            let pwm = us * 4096 / 20000;
            setPwm(num, 0, pwm);
        }



    }
    //% blockId=SuperBit_MotorRun block="Motor|%index|speed(-255~255) %speed"
    //% weight=93
    //% speed.min=-255 speed.max=255
    //% name.fieldEditor="gridpicker" name.fieldOptions.columns=4
    export function MotorRun(index: enMotors, speed: number): void {
        if (!initialized) {
            initPCA9685()
        }
        speed = speed * 16; // map 255 to 4096
        if (speed >= 4096) {
            speed = 4095
        }
        if (speed <= -4096) {
            speed = -4095
        }

        let a = index
        let b = index + 1

        if (a > 10) {
            if (speed >= 0) {
                setPwm(a, 0, speed)
                setPwm(b, 0, 0)
            } else {
                setPwm(a, 0, 0)
                setPwm(b, 0, -speed)
            }
        }
        else {
            if (speed >= 0) {
                setPwm(b, 0, speed)
                setPwm(a, 0, 0)
            } else {
                setPwm(b, 0, 0)
                setPwm(a, 0, -speed)
            }
        }

    }


}
