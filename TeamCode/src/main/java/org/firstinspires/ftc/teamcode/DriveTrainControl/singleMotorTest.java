package org.firstinspires.ftc.teamcode.DriveTrainControl;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="single motor test", group=".")
public class singleMotorTest extends OpMode {
    DcMotor expMotor2, expMotor3;
    DcMotorEx motor;
    @Override
    public void init() {}

    @Override
    public void loop() {
        if (gamepad1.left_bumper) {
            expMotor2.setPower(1.0); // Forward
        } else if (gamepad1.right_bumper) {
            expMotor2.setPower(-1.0); // Reverse
        }
        if (gamepad1.b) {
            expMotor2.setPower(0.0);
        }
        if (gamepad1.left_bumper) {
            expMotor3.setPower(-1.0); // Forward
        } else if (gamepad1.right_bumper) {
            expMotor3.setPower(1.0); // Reverse
        }
        if (gamepad1.b) {
            expMotor3.setPower(0.0);
        }
    }}