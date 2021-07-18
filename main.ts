enum PingUnit {
    //% block="μs"
    MicroSeconds,
    //% block="cm"
    Centimeters,
    //% block="英寸"
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

enum NeoPixelColors {
    //% block=红
    Red = 0xFF0000,
    //% block=橙
    Orange = 0xFFA500,
    //% block=黄
    Yellow = 0xFFFF00,
    //% block=绿
    Green = 0x00FF00,
    //% block=蓝
    Blue = 0x0000FF,
    //% block=靛蓝
    Indigo = 0x4b0082,
    //% block=紫罗兰
    Violet = 0x8a2be2,
    //% block=紫
    Purple = 0xFF00FF,
    //% block=白
    White = 0xFFFFFF,
    //% block=黑
    Black = 0x000000
}

/**
 * Different modes for RGB or RGB+W NeoPixel strips
 */
enum NeoPixelMode {
    //% block="RGB (GRB 格式)"
    RGB = 1,
    //% block="RGB+W"
    RGBW = 2,
    //% block="RGB (RGB 格式)"
    RGB_RGB = 3
}

//% color="#eb834b" icon="\uf085" block="IC:bit"
namespace ICBit {
    /**
     * Send a ping and get the echo time (in microseconds) as a result
     * @param trig tigger pin
     * @param echo echo pin
     * @param unit desired conversion unit
     * @param maxCmDistance maximum distance in centimeters (default is 500)
     */
    //% blockId=sonar_ping block="ping trig %trig|echo %echo|单位 %unit" weight=29
    //% group="超声波传感器" color="#2c3e50"
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

    //% blockId="initialize_sensor" block="初始化颜色传感器" weight=31
    //% group="颜色传感器" color="#3CB371"
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

    //% blockId="getSensorData" block="读取颜色值 %colorId" weight=30
    //% group="颜色传感器" color="#3CB371"
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

    export enum enSteppers {
        B1 = 0x1,
        B2 = 0x2
    }


    export enum enPos {
        //% blockId="forward" block="前进"
        forward = 1,
        // //% blockId="reverse" block="reverse"
        // reverse = 2,
        //% blockId="stop" block="后退"
        stop = 2
    }

    export enum enTurns {
        //% blockId="T1B4" block="1/4"
        T1B4 = 90,
        //% blockId="T1B2" block="1/2"
        T1B2 = 180,
        //% blockId="T1B0" block="1"
        T1B0 = 360,
        //% blockId="T2B0" block="2"
        T2B0 = 720,
        //% blockId="T3B0" block="3"
        T3B0 = 1080,
        //% blockId="T4B0" block="4"
        T4B0 = 1440,
        //% blockId="T5B0" block="5"
        T5B0 = 1800
    }


    export enum enServo {

        S1 = 0,
        S2,
        S3,
        S4
    }
    export enum enMotors {
        M1 = 8,
        M2 = 10,
        M3 = 13,
        M4 = 15
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

    function setStepper(index: number, dir: boolean): void {
        if (index == enSteppers.B1) {
            if (dir) {
                setPwm(11, STP_CHA_L, STP_CHA_H);
                setPwm(9, STP_CHB_L, STP_CHB_H);
                setPwm(10, STP_CHC_L, STP_CHC_H);
                setPwm(8, STP_CHD_L, STP_CHD_H);
            } else {
                setPwm(8, STP_CHA_L, STP_CHA_H);
                setPwm(10, STP_CHB_L, STP_CHB_H);
                setPwm(9, STP_CHC_L, STP_CHC_H);
                setPwm(11, STP_CHD_L, STP_CHD_H);
            }
        } else {
            if (dir) {
                setPwm(13, STP_CHA_L, STP_CHA_H);
                setPwm(15, STP_CHB_L, STP_CHB_H);
                setPwm(14, STP_CHC_L, STP_CHC_H);
                setPwm(16, STP_CHD_L, STP_CHD_H);
            } else {
                setPwm(16, STP_CHA_L, STP_CHA_H);
                setPwm(14, STP_CHB_L, STP_CHB_H);
                setPwm(15, STP_CHC_L, STP_CHC_H);
                setPwm(13, STP_CHD_L, STP_CHD_H);
            }
        }
    }

    function stopMotor(index: number) {
        setPwm(index, 0, 0);
        setPwm(index + 1, 0, 0);
    }


    //% blockId=SuperBit_Servo block="舵机(180°)| %num|角度 %value"
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





    //% blockId=SuperBit_Servo3 block="舵机(360°)| %num|姿态 %pos|角度 %value"
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
        // else if (pos == enPos.reverse) { //0-90 -> 90 -180
        //     let us = ((90 + value) * 1800 / 180 + 600); // 0.6 ~ 2.4
        //     let pwm = us * 4096 / 20000;
        //     setPwm(num, 0, pwm);
        // }



    }
    //% blockId=SuperBit_MotorRun block="电机|%index|速度(-255~255) %speed"
    //% weight=95
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

    let font: number[] = [];
    font[0] = 0x0022d422;
    font[1] = 0x0022d422;
    font[2] = 0x0022d422;
    font[3] = 0x0022d422;
    font[4] = 0x0022d422;
    font[5] = 0x0022d422;
    font[6] = 0x0022d422;
    font[7] = 0x0022d422;
    font[8] = 0x0022d422;
    font[9] = 0x0022d422;
    font[10] = 0x0022d422;
    font[11] = 0x0022d422;
    font[12] = 0x0022d422;
    font[13] = 0x0022d422;
    font[14] = 0x0022d422;
    font[15] = 0x0022d422;
    font[16] = 0x0022d422;
    font[17] = 0x0022d422;
    font[18] = 0x0022d422;
    font[19] = 0x0022d422;
    font[20] = 0x0022d422;
    font[21] = 0x0022d422;
    font[22] = 0x0022d422;
    font[23] = 0x0022d422;
    font[24] = 0x0022d422;
    font[25] = 0x0022d422;
    font[26] = 0x0022d422;
    font[27] = 0x0022d422;
    font[28] = 0x0022d422;
    font[29] = 0x0022d422;
    font[30] = 0x0022d422;
    font[31] = 0x0022d422;
    font[32] = 0x00000000;
    font[33] = 0x000002e0;
    font[34] = 0x00018060;
    font[35] = 0x00afabea;
    font[36] = 0x00aed6ea;
    font[37] = 0x01991133;
    font[38] = 0x010556aa;
    font[39] = 0x00000060;
    font[40] = 0x000045c0;
    font[41] = 0x00003a20;
    font[42] = 0x00051140;
    font[43] = 0x00023880;
    font[44] = 0x00002200;
    font[45] = 0x00021080;
    font[46] = 0x00000100;
    font[47] = 0x00111110;
    font[48] = 0x0007462e;
    font[49] = 0x00087e40;
    font[50] = 0x000956b9;
    font[51] = 0x0005d629;
    font[52] = 0x008fa54c;
    font[53] = 0x009ad6b7;
    font[54] = 0x008ada88;
    font[55] = 0x00119531;
    font[56] = 0x00aad6aa;
    font[57] = 0x0022b6a2;
    font[58] = 0x00000140;
    font[59] = 0x00002a00;
    font[60] = 0x0008a880;
    font[61] = 0x00052940;
    font[62] = 0x00022a20;
    font[63] = 0x0022d422;
    font[64] = 0x00e4d62e;
    font[65] = 0x000f14be;
    font[66] = 0x000556bf;
    font[67] = 0x0008c62e;
    font[68] = 0x0007463f;
    font[69] = 0x0008d6bf;
    font[70] = 0x000094bf;
    font[71] = 0x00cac62e;
    font[72] = 0x000f909f;
    font[73] = 0x000047f1;
    font[74] = 0x0017c629;
    font[75] = 0x0008a89f;
    font[76] = 0x0008421f;
    font[77] = 0x01f1105f;
    font[78] = 0x01f4105f;
    font[79] = 0x0007462e;
    font[80] = 0x000114bf;
    font[81] = 0x000b6526;
    font[82] = 0x010514bf;
    font[83] = 0x0004d6b2;
    font[84] = 0x0010fc21;
    font[85] = 0x0007c20f;
    font[86] = 0x00744107;
    font[87] = 0x01f4111f;
    font[88] = 0x000d909b;
    font[89] = 0x00117041;
    font[90] = 0x0008ceb9;
    font[91] = 0x0008c7e0;
    font[92] = 0x01041041;
    font[93] = 0x000fc620;
    font[94] = 0x00010440;
    font[95] = 0x01084210;
    font[96] = 0x00000820;
    font[97] = 0x010f4a4c;
    font[98] = 0x0004529f;
    font[99] = 0x00094a4c;
    font[100] = 0x000fd288;
    font[101] = 0x000956ae;
    font[102] = 0x000097c4;
    font[103] = 0x0007d6a2;
    font[104] = 0x000c109f;
    font[105] = 0x000003a0;
    font[106] = 0x0006c200;
    font[107] = 0x0008289f;
    font[108] = 0x000841e0;
    font[109] = 0x01e1105e;
    font[110] = 0x000e085e;
    font[111] = 0x00064a4c;
    font[112] = 0x0002295e;
    font[113] = 0x000f2944;
    font[114] = 0x0001085c;
    font[115] = 0x00012a90;
    font[116] = 0x010a51e0;
    font[117] = 0x010f420e;
    font[118] = 0x00644106;
    font[119] = 0x01e8221e;
    font[120] = 0x00093192;
    font[121] = 0x00222292;
    font[122] = 0x00095b52;
    font[123] = 0x0008fc80;
    font[124] = 0x000003e0;
    font[125] = 0x000013f1;
    font[126] = 0x00841080;
    font[127] = 0x0022d422;

    let _I2CAddr = 0;
    let _screen = pins.createBuffer(1025);
    let _buf2 = pins.createBuffer(2);
    let _buf3 = pins.createBuffer(3);
    let _buf4 = pins.createBuffer(4);
    let _ZOOM = 1;

    function cmd1(d: number) {
        let n = d % 256;
        pins.i2cWriteNumber(_I2CAddr, n, NumberFormat.UInt16BE);
    }

    function cmd2(d1: number, d2: number) {
        _buf3[0] = 0;
        _buf3[1] = d1;
        _buf3[2] = d2;
        pins.i2cWriteBuffer(_I2CAddr, _buf3);
    }

    function cmd3(d1: number, d2: number, d3: number) {
        _buf4[0] = 0;
        _buf4[1] = d1;
        _buf4[2] = d2;
        _buf4[3] = d3;
        pins.i2cWriteBuffer(_I2CAddr, _buf4);
    }

    function set_pos(col: number = 0, page: number = 0) {
        cmd1(0xb0 | page) // page number
        let c = col * (_ZOOM + 1)
        cmd1(0x00 | (c % 16)) // lower start column address
        cmd1(0x10 | (c >> 4)) // upper start column address    
    }

    // clear bit
    function clrbit(d: number, b: number): number {
        if (d & (1 << b))
            d -= (1 << b)
        return d
    }

    /**
     * set pixel in OLED
     * @param x is X alis, eg: 0
     * @param y is Y alis, eg: 0
     * @param color is dot color, eg: 1
     */
    //% blockId="OLED12864_I2C_PIXEL" block="设置 pixel 在 x %x|y %y|颜色 %color"
    //% parts=OLED12864_I2C trackArgs=0
    //% weight=80
    export function pixel(x: number, y: number, color: number = 1) {
        let page = y >> 3
        let shift_page = y % 8
        let ind = x * (_ZOOM + 1) + page * 128 + 1
        let b = (color) ? (_screen[ind] | (1 << shift_page)) : clrbit(_screen[ind], shift_page)
        _screen[ind] = b
        set_pos(x, page)
        if (_ZOOM) {
            _screen[ind + 1] = b
            _buf3[0] = 0x40
            _buf3[1] = _buf3[2] = b
            pins.i2cWriteBuffer(_I2CAddr, _buf3)
        }
        else {
            _buf2[0] = 0x40
            _buf2[1] = b
            pins.i2cWriteBuffer(_I2CAddr, _buf2)
        }
    }

    /**
     * show text in OLED
     * @param x is X alis, eg: 0
     * @param y is Y alis, eg: 0
     * @param s is the text will be show, eg: 'Hello!'
     * @param color is string color, eg: 1
     */
    //% blockId="OLED12864_I2C_SHOWSTRING" block="显示 string 在 x %x|y %y|文本 %s|颜色 %color"
    //% weight=80 blockGap=8
    //% parts=OLED12864_I2C trackArgs=0
    export function showString(x: number, y: number, s: string, color: number = 1) {
        let col = 0
        let p = 0
        let ind = 0
        for (let n = 0; n < s.length; n++) {
            p = font[s.charCodeAt(n)]
            for (let i = 0; i < 5; i++) {
                col = 0
                for (let j = 0; j < 5; j++) {
                    if (p & (1 << (5 * i + j)))
                        col |= (1 << (j + 1))
                }
                ind = (x + n) * 5 * (_ZOOM + 1) + y * 128 + i * (_ZOOM + 1) + 1
                if (color == 0)
                    col = 255 - col
                _screen[ind] = col
                if (_ZOOM)
                    _screen[ind + 1] = col
            }
        }
        set_pos(x * 5, y)
        let ind0 = x * 5 * (_ZOOM + 1) + y * 128
        let buf = _screen.slice(ind0, ind + 1)
        buf[0] = 0x40
        pins.i2cWriteBuffer(_I2CAddr, buf)
    }

    /**
     * show a number in OLED
     * @param x is X alis, eg: 0
     * @param y is Y alis, eg: 0
     * @param num is the number will be show, eg: 12
     * @param color is number color, eg: 1
     */
    //% blockId="OLED12864_I2C_NUMBER" block="显示 数字 在 x %x|y %y|数字 %num|颜色 %color"
    //% weight=80 blockGap=8
    //% parts=OLED12864_I2C trackArgs=0
    export function showNumber(x: number, y: number, num: number, color: number = 1) {
        showString(x, y, num.toString(), color)
    }


    /**
     * draw / redraw screen
     */
    //% blockId="OLED12864_I2C_DRAW" block="画"
    //% weight=64 blockGap=8
    //% parts=OLED12864_I2C trackArgs=0
    export function draw() {
        set_pos()
        pins.i2cWriteBuffer(_I2CAddr, _screen)
    }

    /**
     * clear screen
     */
    //% blockId="OLED12864_I2C_CLEAR" block="清除"
    //% weight=63 blockGap=8
    //% parts=OLED12864_I2C trackArgs=0
    export function clear() {
        _screen.fill(0)
        _screen[0] = 0x40
        draw()
    }

    /**
     * OLED initialize
     * @param addr is i2c addr, eg: 60
     */
    //% blockId="OLED12864_I2C_init" block="初始化 OLED 地址为 %addr"
    //% weight=85 blockGap=8
    //% parts=OLED12864_I2C trackArgs=0
    export function init(addr: number) {
        _I2CAddr = addr;
        cmd1(0xAE)       // SSD1306_DISPLAYOFF
        cmd1(0xA4)       // SSD1306_DISPLAYALLON_RESUME
        cmd2(0xD5, 0xF0) // SSD1306_SETDISPLAYCLOCKDIV
        cmd2(0xA8, 0x3F) // SSD1306_SETMULTIPLEX
        cmd2(0xD3, 0x00) // SSD1306_SETDISPLAYOFFSET
        cmd1(0 | 0x0)    // line #SSD1306_SETSTARTLINE
        cmd2(0x8D, 0x14) // SSD1306_CHARGEPUMP
        cmd2(0x20, 0x00) // SSD1306_MEMORYMODE
        cmd3(0x21, 0, 127) // SSD1306_COLUMNADDR
        cmd3(0x22, 0, 63)  // SSD1306_PAGEADDR
        cmd1(0xa0 | 0x1) // SSD1306_SEGREMAP
        cmd1(0xc8)       // SSD1306_COMSCANDEC
        cmd2(0xDA, 0x12) // SSD1306_SETCOMPINS
        cmd2(0x81, 0xCF) // SSD1306_SETCONTRAST
        cmd2(0xd9, 0xF1) // SSD1306_SETPRECHARGE
        cmd2(0xDB, 0x40) // SSD1306_SETVCOMDETECT
        cmd1(0xA6)       // SSD1306_NORMALDISPLAY
        cmd2(0xD6, 1)    // zoom on
        cmd1(0xAF)       // SSD1306_DISPLAYON
        clear()
        _ZOOM = 1
    }







    /**
     * A NeoPixel strip
     */
    export class Strip {
        buf: Buffer;
        pin: DigitalPin;
        // TODO: encode as bytes instead of 32bit
        brightness: number;
        start: number; // start offset in LED strip
        _length: number; // number of LEDs
        _mode: NeoPixelMode;
        _matrixWidth: number; // number of leds in a matrix - if any

        /**
         * Shows all LEDs to a given color (range 0-255 for r, g, b).
         * @param rgb RGB color of the LED
         */
        //% blockId="neopixel_set_strip_color" block="%strip|显示 颜色 %rgb=neopixel_colors"
        //% strip.defl=strip
        //% parts="neopixel"
        showColor(rgb: number) {
            rgb = rgb >> 0;
            this.setAllRGB(rgb);
            this.show();
        }

        /**
         * Shows a rainbow pattern on all LEDs.
         * @param startHue the start hue value for the rainbow, eg: 1
         * @param endHue the end hue value for the rainbow, eg: 360
         */
        //% blockId="neopixel_set_strip_rainbow" block="%strip|显示 彩虹 从 %startHue|到 %endHue"
        //% strip.defl=strip
        //% parts="neopixel"
        //% advanced= true
        showRainbow(startHue: number = 1, endHue: number = 360) {
            if (this._length <= 0) return;

            startHue = startHue >> 0;
            endHue = endHue >> 0;
            const saturation = 100;
            const luminance = 50;
            const steps = this._length;
            const direction = HueInterpolationDirection.Clockwise;

            //hue
            const h1 = startHue;
            const h2 = endHue;
            const hDistCW = ((h2 + 360) - h1) % 360;
            const hStepCW = Math.idiv((hDistCW * 100), steps);
            const hDistCCW = ((h1 + 360) - h2) % 360;
            const hStepCCW = Math.idiv(-(hDistCCW * 100), steps);
            let hStep: number;
            if (direction === HueInterpolationDirection.Clockwise) {
                hStep = hStepCW;
            } else if (direction === HueInterpolationDirection.CounterClockwise) {
                hStep = hStepCCW;
            } else {
                hStep = hDistCW < hDistCCW ? hStepCW : hStepCCW;
            }
            const h1_100 = h1 * 100; //we multiply by 100 so we keep more accurate results while doing interpolation

            //sat
            const s1 = saturation;
            const s2 = saturation;
            const sDist = s2 - s1;
            const sStep = Math.idiv(sDist, steps);
            const s1_100 = s1 * 100;

            //lum
            const l1 = luminance;
            const l2 = luminance;
            const lDist = l2 - l1;
            const lStep = Math.idiv(lDist, steps);
            const l1_100 = l1 * 100

            //interpolate
            if (steps === 1) {
                this.setPixelColor(0, hsl(h1 + hStep, s1 + sStep, l1 + lStep))
            } else {
                this.setPixelColor(0, hsl(startHue, saturation, luminance));
                for (let i = 1; i < steps - 1; i++) {
                    const h = Math.idiv((h1_100 + i * hStep), 100) + 360;
                    const s = Math.idiv((s1_100 + i * sStep), 100);
                    const l = Math.idiv((l1_100 + i * lStep), 100);
                    this.setPixelColor(i, hsl(h, s, l));
                }
                this.setPixelColor(steps - 1, hsl(endHue, saturation, luminance));
            }
            this.show();
        }

        /**
         * Displays a vertical bar graph based on the `value` and `high` value.
         * If `high` is 0, the chart gets adjusted automatically.
         * @param value current value to plot
         * @param high maximum value, eg: 255
         */
        //% blockId=neopixel_show_bar_graph block="%strip|显示 柱状图 从 %value|到 %high"
        //% icon="\uf080"
        //% parts="neopixel"

        showBarGraph(value: number, high: number): void {
            if (high <= 0) {
                this.clear();
                this.setPixelColor(0, NeoPixelColors.Yellow);
                this.show();
                return;
            }

            value = Math.abs(value);
            const n = this._length;
            const n1 = n - 1;
            let v = Math.idiv((value * n), high);
            if (v == 0) {
                this.setPixelColor(0, 0x666600);
                for (let i = 1; i < n; ++i)
                    this.setPixelColor(i, 0);
            } else {
                for (let i = 0; i < n; ++i) {
                    if (i <= v) {
                        const b = Math.idiv(i * 255, n1);
                        this.setPixelColor(i, ICBit.rgb(b, 0, 255 - b));
                    }
                    else this.setPixelColor(i, 0);
                }
            }
            this.show();
        }

        /**
         * Set LED to a given color (range 0-255 for r, g, b).
         * You need to call ``show`` to make the changes visible.
         * @param pixeloffset position of the NeoPixel in the strip
         * @param rgb RGB color of the LED
         */
        //% blockId="neopixel_set_pixel_color" block="%strip|设置 pixel 颜色 在 %pixeloffset|到 %rgb=neopixel_colors"
        //% strip.defl=strip
        //% parts="neopixel"

        setPixelColor(pixeloffset: number, rgb: number): void {
            this.setPixelRGB(pixeloffset >> 0, rgb >> 0);
        }

        /**
         * Sets the number of pixels in a matrix shaped strip
         * @param width number of pixels in a row
         */
        //% blockId=neopixel_set_matrix_width block="%strip|设置 矩阵 宽度 %width"
        //% strip.defl=strip
        //% parts="neopixel"
        setMatrixWidth(width: number) {
            this._matrixWidth = Math.min(this._length, width >> 0);
        }

        /**
         * Set LED to a given color (range 0-255 for r, g, b) in a matrix shaped strip
         * You need to call ``show`` to make the changes visible.
         * @param x horizontal position
         * @param y horizontal position
         * @param rgb RGB color of the LED
         */
        //% blockId="neopixel_set_matrix_color" block="%strip|设置 矩阵 颜色 在 x %x|y %y|到 %rgb=neopixel_colors"
        //% strip.defl=strip
        //% parts="neopixel"
        setMatrixColor(x: number, y: number, rgb: number) {
            if (this._matrixWidth <= 0) return; // not a matrix, ignore
            x = x >> 0;
            y = y >> 0;
            rgb = rgb >> 0;
            const cols = Math.idiv(this._length, this._matrixWidth);
            if (x < 0 || x >= this._matrixWidth || y < 0 || y >= cols) return;
            let i = x + y * this._matrixWidth;
            this.setPixelColor(i, rgb);
        }

        /**
         * For NeoPixels with RGB+W LEDs, set the white LED brightness. This only works for RGB+W NeoPixels.
         * @param pixeloffset position of the LED in the strip
         * @param white brightness of the white LED
         */
        //% blockId="neopixel_set_pixel_white" block="%strip|设置 pixel 白色 LED 在 %pixeloffset|到 %white"
        //% strip.defl=strip
        //% parts="neopixel"
        //% advanced= true
        setPixelWhiteLED(pixeloffset: number, white: number): void {
            if (this._mode === NeoPixelMode.RGBW) {
                this.setPixelW(pixeloffset >> 0, white >> 0);
            }
        }

        /**
         * Send all the changes to the strip.
         */
        //% blockId="neopixel_show" block="%strip|显示"
        //% strip.defl=strip
        //% parts="neopixel"
        show() {
            // only supported in beta
            // ws2812b.setBufferMode(this.pin, this._mode);
            // ws2812b.sendBuffer(this.buf, this.pin);
        }

        /**
         * Turn off all LEDs.
         * You need to call ``show`` to make the changes visible.
         */
        //% blockId="neopixel_clear" block="%strip|清除"
        //% strip.defl=strip
        //% parts="neopixel"
        clear(): void {
            const stride = this._mode === NeoPixelMode.RGBW ? 4 : 3;
            this.buf.fill(0, this.start * stride, this._length * stride);
        }

        /**
         * Gets the number of pixels declared on the strip
         */
        //% blockId="neopixel_length" block="%strip|长度"
        //% strip.defl=strip
        //% weight=32
        //% advanced= true
        length() {
            return this._length;
        }

        /**
         * Set the brightness of the strip. This flag only applies to future operation.
         * @param brightness a measure of LED brightness in 0-255. eg: 255
         */
        //% blockId="neopixel_set_brightness" block="%strip|设置 亮度 %brightness"
        //% strip.defl=strip
        //% parts="neopixel"
        setBrightness(brightness: number): void {
            this.brightness = brightness & 0xff;
        }

        /**
         * Apply brightness to current colors using a quadratic easing function.
         **/
        //% blockId="neopixel_each_brightness" block="%strip|清除 亮度"
        //% strip.defl=strip
        //% parts="neopixel"
        easeBrightness(): void {
            const stride = this._mode === NeoPixelMode.RGBW ? 4 : 3;
            const br = this.brightness;
            const buf = this.buf;
            const end = this.start + this._length;
            const mid = Math.idiv(this._length, 2);
            for (let i = this.start; i < end; ++i) {
                const k = i - this.start;
                const ledoffset = i * stride;
                const br = k > mid
                    ? Math.idiv(255 * (this._length - 1 - k) * (this._length - 1 - k), (mid * mid))
                    : Math.idiv(255 * k * k, (mid * mid));
                const r = (buf[ledoffset + 0] * br) >> 8; buf[ledoffset + 0] = r;
                const g = (buf[ledoffset + 1] * br) >> 8; buf[ledoffset + 1] = g;
                const b = (buf[ledoffset + 2] * br) >> 8; buf[ledoffset + 2] = b;
                if (stride == 4) {
                    const w = (buf[ledoffset + 3] * br) >> 8; buf[ledoffset + 3] = w;
                }
            }
        }

        /**
         * Create a range of LEDs.
         * @param start offset in the LED strip to start the range
         * @param length number of LEDs in the range. eg: 4
         */
        //% blockId="neopixel_range" block="%strip|值域 从 %start|到 %length|leds"
        //% strip.defl=strip
        //% parts="neopixel"
        //% blockSetVariable=range
        //% weight= 52
        range(start: number, length: number): Strip {
            start = start >> 0;
            length = length >> 0;
            let strip = new Strip();
            strip.buf = this.buf;
            strip.pin = this.pin;
            strip.brightness = this.brightness;
            strip.start = this.start + Math.clamp(0, this._length - 1, start);
            strip._length = Math.clamp(0, this._length - (strip.start - this.start), length);
            strip._matrixWidth = 0;
            strip._mode = this._mode;
            return strip;
        }

        /**
         * Shift LEDs forward and clear with zeros.
         * You need to call ``show`` to make the changes visible.
         * @param offset number of pixels to shift forward, eg: 1
         */
        //% blockId="neopixel_shift" block="%strip|移动 pixels %offset"
        //% strip.defl=strip
        //% parts="neopixel"
        //% weight= 50
        shift(offset: number = 1): void {
            offset = offset >> 0;
            const stride = this._mode === NeoPixelMode.RGBW ? 4 : 3;
            this.buf.shift(-offset * stride, this.start * stride, this._length * stride)
        }

        /**
         * Rotate LEDs forward.
         * You need to call ``show`` to make the changes visible.
         * @param offset number of pixels to rotate forward, eg: 1
         */
        //% blockId="neopixel_rotate" block="%strip|旋转 pixels %offset"
        //% strip.defl=strip
        //% parts="neopixel"
        //% weight=50
        rotate(offset: number = 1): void {
            offset = offset >> 0;
            const stride = this._mode === NeoPixelMode.RGBW ? 4 : 3;
            this.buf.rotate(-offset * stride, this.start * stride, this._length * stride)
        }

        /**
         * Set the pin where the neopixel is connected, defaults to P0.
         */
        //% parts="neopixel"
        setPin(pin: DigitalPin): void {
            this.pin = pin;
            pins.digitalWritePin(this.pin, 0);
            // don't yield to avoid races on initialization
        }

        /**
         * Estimates the electrical current (mA) consumed by the current light configuration.
         */
        //% blockId=neopixel_power block="%strip|电力 (mA)"
        //% strip.defl=strip
        //% weight=32
        //% advanced= true
        power(): number {
            const stride = this._mode === NeoPixelMode.RGBW ? 4 : 3;
            const end = this.start + this._length;
            let p = 0;
            for (let i = this.start; i < end; ++i) {
                const ledoffset = i * stride;
                for (let j = 0; j < stride; ++j) {
                    p += this.buf[i + j];
                }
            }
            return Math.idiv(this.length() * 7, 10) /* 0.7mA per neopixel */
                + Math.idiv(p * 480, 10000); /* rought approximation */
        }

        private setBufferRGB(offset: number, red: number, green: number, blue: number): void {
            if (this._mode === NeoPixelMode.RGB_RGB) {
                this.buf[offset + 0] = red;
                this.buf[offset + 1] = green;
            } else {
                this.buf[offset + 0] = green;
                this.buf[offset + 1] = red;
            }
            this.buf[offset + 2] = blue;
        }

        private setAllRGB(rgb: number) {
            let red = unpackR(rgb);
            let green = unpackG(rgb);
            let blue = unpackB(rgb);

            const br = this.brightness;
            if (br < 255) {
                red = (red * br) >> 8;
                green = (green * br) >> 8;
                blue = (blue * br) >> 8;
            }
            const end = this.start + this._length;
            const stride = this._mode === NeoPixelMode.RGBW ? 4 : 3;
            for (let i = this.start; i < end; ++i) {
                this.setBufferRGB(i * stride, red, green, blue)
            }
        }
        private setAllW(white: number) {
            if (this._mode !== NeoPixelMode.RGBW)
                return;

            let br = this.brightness;
            if (br < 255) {
                white = (white * br) >> 8;
            }
            let buf = this.buf;
            let end = this.start + this._length;
            for (let i = this.start; i < end; ++i) {
                let ledoffset = i * 4;
                buf[ledoffset + 3] = white;
            }
        }
        private setPixelRGB(pixeloffset: number, rgb: number): void {
            if (pixeloffset < 0
                || pixeloffset >= this._length)
                return;

            let stride = this._mode === NeoPixelMode.RGBW ? 4 : 3;
            pixeloffset = (pixeloffset + this.start) * stride;

            let red = unpackR(rgb);
            let green = unpackG(rgb);
            let blue = unpackB(rgb);

            let br = this.brightness;
            if (br < 255) {
                red = (red * br) >> 8;
                green = (green * br) >> 8;
                blue = (blue * br) >> 8;
            }
            this.setBufferRGB(pixeloffset, red, green, blue)
        }
        private setPixelW(pixeloffset: number, white: number): void {
            if (this._mode !== NeoPixelMode.RGBW)
                return;

            if (pixeloffset < 0
                || pixeloffset >= this._length)
                return;

            pixeloffset = (pixeloffset + this.start) * 4;

            let br = this.brightness;
            if (br < 255) {
                white = (white * br) >> 8;
            }
            let buf = this.buf;
            buf[pixeloffset + 3] = white;
        }
    }

    /**
     * Create a new NeoPixel driver for `numleds` LEDs.
     * @param pin the pin where the neopixel is connected.
     * @param numleds number of leds in the strip, eg: 24,30,60,64
     */
    //% blockId="neopixel_create" block="NeoPixel 在 端口 %pin|用 %numleds| leds 模式 %mode"
    //% parts="neopixel"
    //% colorSecondary="#FFFFFF"
    //% trackArgs=0,2
    //% blockSetVariable=strip
    //% weight=51
    export function create(pin: DigitalPin, numleds: number, mode: NeoPixelMode): Strip {
        let strip = new Strip();
        let stride = mode === NeoPixelMode.RGBW ? 4 : 3;
        strip.buf = pins.createBuffer(numleds * stride);
        strip.start = 0;
        strip._length = numleds;
        strip._mode = mode || NeoPixelMode.RGB;
        strip._matrixWidth = 0;
        strip.setBrightness(128)
        strip.setPin(pin)
        return strip;
    }

    /**
     * Converts red, green, blue channels into a RGB color
     * @param red value of the red channel between 0 and 255. eg: 255
     * @param green value of the green channel between 0 and 255. eg: 255
     * @param blue value of the blue channel between 0 and 255. eg: 255
     */
    //% blockId="neopixel_rgb" block="红 %red|绿 %green|蓝 %blue"
    //% weight=32
    export function rgb(red: number, green: number, blue: number): number {
        return packRGB(red, green, blue);
    }

    /**
     * Gets the RGB value of a known color
    */
    //% blockId="neopixel_colors" block="%color"
    //% weight=32
    export function colors(color: NeoPixelColors): number {
        return color;
    }

    function packRGB(a: number, b: number, c: number): number {
        return ((a & 0xFF) << 16) | ((b & 0xFF) << 8) | (c & 0xFF);
    }
    function unpackR(rgb: number): number {
        let r = (rgb >> 16) & 0xFF;
        return r;
    }
    function unpackG(rgb: number): number {
        let g = (rgb >> 8) & 0xFF;
        return g;
    }
    function unpackB(rgb: number): number {
        let b = (rgb) & 0xFF;
        return b;
    }

    /**
     * Converts a hue saturation luminosity value into a RGB color
     * @param h hue from 0 to 360
     * @param s saturation from 0 to 99
     * @param l luminosity from 0 to 99
     */
    //% blockId=neopixelHSL block="色度 %h|饱和度 %s|亮度 %l"
    //% weight=32
    export function hsl(h: number, s: number, l: number): number {
        h = Math.round(h);
        s = Math.round(s);
        l = Math.round(l);

        h = h % 360;
        s = Math.clamp(0, 99, s);
        l = Math.clamp(0, 99, l);
        let c = Math.idiv((((100 - Math.abs(2 * l - 100)) * s) << 8), 10000); //chroma, [0,255]
        let h1 = Math.idiv(h, 60);//[0,6]
        let h2 = Math.idiv((h - h1 * 60) * 256, 60);//[0,255]
        let temp = Math.abs((((h1 % 2) << 8) + h2) - 256);
        let x = (c * (256 - (temp))) >> 8;//[0,255], second largest component of this color
        let r$: number;
        let g$: number;
        let b$: number;
        if (h1 == 0) {
            r$ = c; g$ = x; b$ = 0;
        } else if (h1 == 1) {
            r$ = x; g$ = c; b$ = 0;
        } else if (h1 == 2) {
            r$ = 0; g$ = c; b$ = x;
        } else if (h1 == 3) {
            r$ = 0; g$ = x; b$ = c;
        } else if (h1 == 4) {
            r$ = x; g$ = 0; b$ = c;
        } else if (h1 == 5) {
            r$ = c; g$ = 0; b$ = x;
        }
        let m = Math.idiv((Math.idiv((l * 2 << 8), 100) - c), 2);
        let r = r$ + m;
        let g = g$ + m;
        let b = b$ + m;
        return packRGB(r, g, b);
    }

    export enum HueInterpolationDirection {
        Clockwise,
        CounterClockwise,
        Shortest
    }


}
