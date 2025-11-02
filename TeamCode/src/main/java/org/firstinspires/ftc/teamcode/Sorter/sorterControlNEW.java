package org.firstinspires.ftc.teamcode.Sorter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Sorter with External Encoder (m2) + Offset Toggle", group="Sorter")
public class sorterControlNEW extends LinearOpMode {

    private DcMotorEx sorterMotor;  // motor m0 physically spins sorter
    private DcMotorEx sorterEncoder; // motor m2 port used only for encoder feedback

    private static final int TARGET_POSITION = 2375;  // ticks to reach (for D-pad)
    private static final int OFFSET_TICKS = 1000;     // ticks to offset when Y toggled
    private static final double MOVE_POWER = 1;       // speed

    @Override
    public void runOpMode() throws InterruptedException {
        sorterMotor = hardwareMap.get(DcMotorEx.class, "m0");
        sorterEncoder = hardwareMap.get(DcMotorEx.class, "m2"); // only for reading encoder

        sorterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sorterEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sorterEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Ready — press D-pad right to move sorter, Y to offset by 1187.5 ticks.");
        telemetry.update();
        waitForStart();

        boolean lastDpadRight = false;
        boolean lastY = false;
        boolean offsetActive = false;  // track toggle state

        while (opModeIsActive()) {
            boolean dpadRight = gamepad1.dpad_right;
            boolean yPressed = gamepad1.y;

            // --- D-Pad Right Control (Normal 2375 ticks) ---
            if (dpadRight && !lastDpadRight) {
                sorterEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                sorterEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                sorterMotor.setPower(MOVE_POWER);
                while (opModeIsActive() && Math.abs(sorterEncoder.getCurrentPosition()) < TARGET_POSITION) {
                    telemetry.addData("Sorter Encoder (m2)", sorterEncoder.getCurrentPosition());
                    telemetry.addData("Target", TARGET_POSITION);
                    telemetry.update();
                }

                sorterMotor.setPower(0);
                telemetry.addData("Done", "Reached ~%d ticks", sorterEncoder.getCurrentPosition());
                telemetry.update();
            }

            // --- Y Toggle for Offset Movement (+/-1187 ticks) ---
            if (yPressed && !lastY) {
                offsetActive = !offsetActive;  // toggle state

                sorterEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                sorterEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                // Decide direction: forward when ON, backward when OFF
                double direction = offsetActive ? 1.0 : -1.0;
                sorterMotor.setPower(MOVE_POWER * direction);

                while (opModeIsActive() && Math.abs(sorterEncoder.getCurrentPosition()) < OFFSET_TICKS) {
                    telemetry.addData("Offset Active", offsetActive);
                    telemetry.addData("Sorter Encoder (m2)", sorterEncoder.getCurrentPosition());
                    telemetry.addData("Target Offset", OFFSET_TICKS);
                    telemetry.update();
                }

                sorterMotor.setPower(0);
                telemetry.addData("Offset", offsetActive ? "Activated (+1187)" : "Deactivated (-1187)");
                telemetry.addData("Final Encoder", sorterEncoder.getCurrentPosition());
                telemetry.update();
            }

            lastDpadRight = dpadRight;
            lastY = yPressed;

            telemetry.addData("Current Encoder (m2)", sorterEncoder.getCurrentPosition());
            telemetry.addData("Offset Active", offsetActive);
            telemetry.update();

            sleep(20);
        }
    }
}
