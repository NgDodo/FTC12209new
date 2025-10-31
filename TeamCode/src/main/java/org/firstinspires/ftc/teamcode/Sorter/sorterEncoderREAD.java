package org.firstinspires.ftc.teamcode.Sorter;

import com.arcrobotics.ftclib.controller.PDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.ButtonHelper;

@TeleOp(name="sorter encoder read")
public class sorterEncoderREAD extends LinearOpMode {
    DcMotorEx m0, m2, m3;


    @Override
    public void runOpMode() throws InterruptedException {
        m0 = hardwareMap.get(DcMotorEx.class, "m0");
        m2 = hardwareMap.get(DcMotorEx.class, "m2");
        m3 = hardwareMap.get(DcMotorEx.class, "m3");
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("current pos", m0.getCurrentPosition());
            telemetry.update();
        }
    }
}
