package org.firstinspires.ftc.teamcode.Sorter;

import com.arcrobotics.ftclib.controller.PDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.ButtonHelper;

@TeleOp(name="sorter encoder")
public class sorterEncoder extends LinearOpMode {
    CRServo s5;
    DcMotorEx m2;

    int sorterPosition;
    enum SORTER_MODE {
        INTAKING,
        SHOOTING
    }
    SORTER_MODE sorterMode = SORTER_MODE.INTAKING;
    int sorterModeOffset = 0;
    int desiredPosition;

    ///  create pid controller --- and kpid values
    PDController sorterPID;
    private static final double kP = 0.0005;
    private static final double kI = 0.000005;
    private static final double kD = 0.0001;
    final int TOTAL_ROTATION_TICKS = 8192;
    final int ACCEPTABLE_ERROR = 20;
    ButtonHelper gamepad1Helper;
    @Override
    public void runOpMode() throws InterruptedException {
        gamepad1Helper = new ButtonHelper(gamepad1);

        s5 = hardwareMap.get(CRServo.class, "s5");
        s5.setDirection(DcMotorSimple.Direction.REVERSE);
        m2 = hardwareMap.get(DcMotorEx.class, "m2");
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        sorterPosition = m2.getCurrentPosition();
        desiredPosition = 0;

        sorterPID = new PDController(kP, kD);
        waitForStart();

        while (opModeIsActive()) {
            // always set pid to go to current position goal (based on sorterError from setpoint) + offset
                /// offset equal to 0 in intake, 18500/6 in scoring
            // if button to rotate clicked, increase/decreate curernt position goal by 18500/3

            double sorterError = sorterPosition - desiredPosition - sorterModeOffset;


            double sorterPower = sorterPID.calculate(sorterError);
            if (Math.abs(sorterError) > ACCEPTABLE_ERROR) {
                s5.setPower(sorterPower);
            }

            /// Rotate Sorter
            if (gamepad1Helper.isButtonJustPressed("dpad_left")) {
                desiredPosition += TOTAL_ROTATION_TICKS/3;
            }
            else if (gamepad1Helper.isButtonJustPressed("dpad_right")) {
                desiredPosition -= TOTAL_ROTATION_TICKS/3;
            }

            /// Control Sorter Modes
            if (gamepad1Helper.isButtonJustPressed("y")) {
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

            // Update Sorter Position
            sorterPosition = m2.getCurrentPosition();
            gamepad1Helper.update();
            telemetry.addData("Sorter Power", sorterPower);
            telemetry.addData("Sorter Error", sorterError);
            telemetry.addData("Current Position", sorterPosition);
            telemetry.addData("Desired Position", desiredPosition);
            telemetry.addData("MODE", sorterMode);
            telemetry.addData("MODE---OFFSET", sorterModeOffset);
            telemetry.update();
        }
    }
}
