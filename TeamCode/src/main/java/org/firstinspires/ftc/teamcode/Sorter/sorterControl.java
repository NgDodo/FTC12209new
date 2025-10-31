package org.firstinspires.ftc.teamcode.Sorter;

import com.arcrobotics.ftclib.controller.PDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.ButtonHelper;

import java.util.Map;

@TeleOp(name="sorter control")
public class sorterControl extends LinearOpMode {
    DcMotorEx sorterMotor;

    enum SORTER_MODE {
        INTAKING,
        SHOOTING
    }
    SORTER_MODE sorterMode = SORTER_MODE.INTAKING;
    int sorterModeOffset = 0;
    int desiredPosition = 0;

    final int TOTAL_ROTATION_TICKS = 8192;
    final int ACCEPTABLE_ERROR = 20;
    ButtonHelper gamepad1Helper;
    @Override
    public void runOpMode() throws InterruptedException {
        gamepad1Helper = new ButtonHelper(gamepad1);

        sorterMotor = hardwareMap.get(DcMotorEx.class, "m0");
        sorterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sorterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfcoef = sorterMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        while (opModeIsActive()) {
            // always set pid to go to current position goal (based on sorterError from setpoint) + offset
                /// offset equal to 0 in intake, 18500/6 in scoring
            // if button to rotate clicked, increase/decreate curernt position goal by 18500/3

            /// Rotate Sorter
            if (gamepad1.dpadLeftWasPressed()) {
                desiredPosition += TOTAL_ROTATION_TICKS/3;
            }
            else if (gamepad1.dpadRightWasPressed()) {
                desiredPosition -= TOTAL_ROTATION_TICKS/3;
            }

            /// Control Sorter Modes
            if (gamepad1.yWasPressed()) {
                if (sorterMode == SORTER_MODE.SHOOTING){
                    sorterMode = SORTER_MODE.INTAKING;
                }
                else if (sorterMode == SORTER_MODE.INTAKING){
                    sorterMode = SORTER_MODE.SHOOTING;
                }
            }

            if (sorterMode == SORTER_MODE.INTAKING) {
                sorterModeOffset = 0;
            }
            if (sorterMode == SORTER_MODE.SHOOTING) {
                sorterModeOffset = TOTAL_ROTATION_TICKS / 2;
            }

            if (Math.abs(desiredPosition - sorterMotor.getCurrentPosition()) > ACCEPTABLE_ERROR){
                sorterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sorterMotor.setTargetPosition(desiredPosition + sorterModeOffset);
                sorterMotor.setPower(0.03);
            }

            telemetry.addData("PIDF", pidfcoef);
            telemetry.addData("Sorter Power", sorterMotor.getPower());
            telemetry.addData("Current Position", sorterMotor.getCurrentPosition());
            telemetry.addData("Desired Position", desiredPosition);
            telemetry.addData("Offset", sorterModeOffset);
            telemetry.addData("MODE", sorterMode);
            telemetry.update();
        }
    }
}
