package org.firstinspires.ftc.teamcode.Sorter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Sorter Control (Stable Revolver)")
public class sorterControl extends LinearOpMode {

    DcMotorEx sorterMotor;

    enum SORTER_MODE {
        INTAKING,
        SHOOTING
    }

    SORTER_MODE sorterMode = SORTER_MODE.INTAKING;

    // Constants
    private static final int TOTAL_ROTATION_TICKS = 8192;   // encoder ticks per full revolution
    private static final int ACCEPTABLE_ERROR = 30;         // tolerance band
    private static final double MOVE_POWER = 0.35;          // movement power
    private static final double HOLD_POWER = 0.08;          // hold steady power

    private int desiredPosition = 0;
    private int sorterModeOffset = 0;

    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastY = false;
    private boolean moving = false;

    @Override
    public void runOpMode() throws InterruptedException {
        sorterMotor = hardwareMap.get(DcMotorEx.class, "m0");
        sorterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sorterMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        sorterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sorterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        desiredPosition = sorterMotor.getCurrentPosition();

        telemetry.addLine("Sorter initialized — press Start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            boolean dpadLeft = gamepad1.dpad_left;
            boolean dpadRight = gamepad1.dpad_right;

            // --- Advance sorter 1 slot CW ---
            if (dpadLeft && !lastDpadLeft && !moving) {
                desiredPosition += TOTAL_ROTATION_TICKS / 3;
                sorterMotor.setTargetPosition(desiredPosition + sorterModeOffset);
                sorterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sorterMotor.setPower(MOVE_POWER);
                moving = true;
            }

            // --- Advance sorter 1 slot CCW ---
            if (dpadRight && !lastDpadRight && !moving) {
                desiredPosition -= TOTAL_ROTATION_TICKS / 3;
                sorterMotor.setTargetPosition(desiredPosition + sorterModeOffset);
                sorterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sorterMotor.setPower(MOVE_POWER);
                moving = true;
            }

            // --- Switch between intake / shooting alignment ---
            if (gamepad1.y && !lastY) {
                sorterMode = (sorterMode == SORTER_MODE.SHOOTING)
                        ? SORTER_MODE.INTAKING
                        : SORTER_MODE.SHOOTING;
                sorterModeOffset = (sorterMode == SORTER_MODE.SHOOTING)
                        ? TOTAL_ROTATION_TICKS / 2
                        : 0;
            }

            // --- Check motion completion ---
            int currentPos = sorterMotor.getCurrentPosition();
            int targetPos = desiredPosition + sorterModeOffset;
            int error = targetPos - currentPos;

            if (moving) {
                if (Math.abs(error) <= ACCEPTABLE_ERROR || !sorterMotor.isBusy()) {
                    // stop movement cleanly
                    sorterMotor.setPower(0);
                    sorterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    sorterMotor.setPower(HOLD_POWER); // hold position gently
                    moving = false;
                }
            }

            // --- Telemetry ---
            telemetry.addLine("=== SORTER ===");
            telemetry.addData("Current Pos", currentPos);
            telemetry.addData("Target Pos", targetPos);
            telemetry.addData("Error", error);
            telemetry.addData("Mode", sorterMode);
            telemetry.addData("Moving", moving);
            telemetry.update();

            // Save button states for edge detection
            lastDpadLeft = dpadLeft;
            lastDpadRight = dpadRight;
            lastY = gamepad1.y;

            sleep(20);
        }
    }
}
