// tests go here; this will not be compiled when this package is used as an extension.
basic.forever(() => {
    let p = 超声波传感器.ping(DigitalPin.P1, DigitalPin.P2, PingUnit.Inches);
    led.plotBarGraph(p, 0);
})