package org.firstinspires.ftc.teamcode.Sorter;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;

public class TestLEDbench {
    private LED whiteLED;
    private LED greenLED;
    private LED blueLED;

    public void init(HardwareMap hwMap) {
        whiteLED = hwMap.get(LED.class, "LED1");
        greenLED = hwMap.get(LED.class, "LED2");
        blueLED = hwMap.get(LED.class, "LED3");
    }

    public void setWhiteLED(boolean isOn) {
        if (isOn) {
            whiteLED.on();
        }
        else {
            whiteLED.off();
        }
    }
    public void setGreenLED(boolean isOn) {
        if (isOn) {
            greenLED.on();
        }
        else {
            greenLED.off();
        }
    }
    public void setBlueLED(boolean isOn) {
        if (isOn) {
            blueLED.on();
        }
        else {
            blueLED.off();
        }
    }

}

