package org.firstinspires.ftc.teamcode.Sorter;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name="LED Toggle (Fixed Active-Low)", group="Utility")
public class LEDtest extends OpMode {

    private DigitalChannel LED1, LED2, LED3;

    boolean ledsOn = false;
    boolean lastY = false;

    @Override
    public void init() {
        LED1 = hardwareMap.get(DigitalChannel.class, "LED1");
        LED2 = hardwareMap.get(DigitalChannel.class, "LED2");
        LED3 = hardwareMap.get(DigitalChannel.class, "LED3");

        LED1.setMode(DigitalChannel.Mode.OUTPUT);
        LED2.setMode(DigitalChannel.Mode.OUTPUT);
        LED3.setMode(DigitalChannel.Mode.OUTPUT);

        // Start OFF (active-low means TRUE = off)
        LED1.setState(true);
        LED2.setState(true);
        LED3.setState(true);

        telemetry.addLine("Initialized - Press Y to toggle LEDs");
        telemetry.update();
    }

    @Override
    public void loop() {
        boolean yPressed = gamepad1.y;

        if (yPressed && !lastY) {
            ledsOn = !ledsOn;

            // active-low: false = ON, true = OFF
            boolean outputState = !ledsOn;

            LED1.setState(outputState);
            LED2.setState(outputState);
            LED3.setState(outputState);
        }

        lastY = yPressed;

        telemetry.addData("LEDs", ledsOn ? "ON" : "OFF");
        telemetry.update();
    }
}
