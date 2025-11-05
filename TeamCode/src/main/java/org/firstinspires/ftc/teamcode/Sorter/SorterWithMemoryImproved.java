package org.firstinspires.ftc.teamcode.Sorter;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Sorter Auto-Intake + Memory + Shooting (No Drift)", group="Sorter")
public class SorterWithMemoryImproved extends LinearOpMode {

    private DcMotorEx sorterMotor;
    private DcMotorEx sorterEncoder;
    private RevColorSensorV3 intakeColor;
    private RevColorSensorV3 shooterColor;

    // LED outputs (using LED2 = Green, LED3 = Blue)
    private DigitalChannel LED_Green;
    private DigitalChannel LED_Blue;

    private static final int FULL_ROT = 8200;
    private static final int SLOT = FULL_ROT / 3;
    private static final int OFFSET = (int) (FULL_ROT / 6.3);
    private static final double POWER = 1.0;

    // ABSOLUTE chamber positions (intake aligned)
    private static final int CHAMBER_0_POS = 0;
    private static final int CHAMBER_1_POS = SLOT;
    private static final int CHAMBER_2_POS = 2 * SLOT;

    private boolean[] chamberFull = new boolean[3];
    private int currentChamber = 0;

    private boolean shootingMode = false;
    private boolean lastY = false;
    private boolean lastDpadRight = false;

    // === Timed detection ===
    private long colorStartTime = 0;
    private boolean colorActive = false;
    private static final long DETECT_TIME_MS = 300;

    @Override
    public void runOpMode() throws InterruptedException {

        sorterMotor   = hardwareMap.get(DcMotorEx.class, "m0");
        sorterEncoder = hardwareMap.get(DcMotorEx.class, "m2");
        intakeColor   = hardwareMap.get(RevColorSensorV3.class, "intakeColor");
        shooterColor  = hardwareMap.get(RevColorSensorV3.class, "shooterColor");

        // Initialize LEDs (LED2 = Green, LED3 = Blue)
        LED_Green = hardwareMap.get(DigitalChannel.class, "LED2");
        LED_Blue  = hardwareMap.get(DigitalChannel.class, "LED3");

        LED_Green.setMode(DigitalChannel.Mode.OUTPUT);
        LED_Blue.setMode(DigitalChannel.Mode.OUTPUT);

        // Start with LEDs off (active-low: true = OFF)
        LED_Green.setState(true);
        LED_Blue.setState(true);

        sorterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        sorterEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        sorterEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            int rawPos = sorterEncoder.getCurrentPosition();
            int normPos = normalize(rawPos);

            // ===== Shooting toggle =====
            boolean yPressed = gamepad1.y;
            if (yPressed && !lastY) {
                shootingMode = !shootingMode;

                // Move to absolute position based on current chamber and mode
                int targetPos = getChamberPosition(currentChamber, shootingMode);
                moveToExternal(targetPos);
            }
            lastY = yPressed;

            // ===== Manual chamber advance in shooting mode =====
            if (shootingMode) {
                boolean dpadRightPressed = gamepad1.dpad_right;
                if (dpadRightPressed && !lastDpadRight) {
                    // Advance to next chamber manually
                    currentChamber = nextChamber(currentChamber);
                    int targetPos = getChamberPosition(currentChamber, true);
                    moveToExternal(targetPos);
                }
                lastDpadRight = dpadRightPressed;
            }

            // ===== Auto-rotate only in intake mode =====
            if (!shootingMode) {
                autoIntakeColorCheck();
            }

            // ===== Detect shooter color and update LEDs =====
            String shooterColorDetected = detectShooterColor();
            updateLEDs(shooterColorDetected);

            // ==== TELEMETRY ====
            telemetry.addLine("=== Sorter ===");
            telemetry.addData("Raw Encoder", rawPos);
            telemetry.addData("Normalized", normPos);
            telemetry.addData("Current Chamber", currentChamber + 1);
            telemetry.addData("Target Position", getChamberPosition(currentChamber, shootingMode));
            telemetry.addData("Position Error", calculateShortestError(normPos, getChamberPosition(currentChamber, shootingMode)));
            telemetry.addLine();
            telemetry.addData("Ch1 Full", chamberFull[0]);
            telemetry.addData("Ch2 Full", chamberFull[1]);
            telemetry.addData("Ch3 Full", chamberFull[2]);
            telemetry.addLine();
            telemetry.addData("Shooting Mode", shootingMode);
            telemetry.addData("Color to Shoot", shooterColorDetected);
            telemetry.addData("Intake Color Active", colorActive);
            telemetry.update();
        }
    }

    /**
     * Update LEDs based on shooter color
     * GREEN ball -> Green LED on, Blue LED off
     * PURPLE ball -> Blue LED on, Green LED off
     * NONE -> Both LEDs off
     */
    private void updateLEDs(String color) {
        // Active-low: false = ON, true = OFF

        if (color.equals("GREEN")) {
            LED_Green.setState(true);   // Green ON
            LED_Blue.setState(false);   // Blue OFF
        } else if (color.equals("PURPLE")) {
            LED_Green.setState(false);  // Green OFF
            LED_Blue.setState(true);    // Blue ON
        } else {
            LED_Green.setState(false);  // Green OFF
            LED_Blue.setState(false);   // Blue OFF
        }
    }

    /**
     * Get the absolute position for a chamber in intake or shooting mode
     * This prevents drift by always using fixed positions
     */
    private int getChamberPosition(int chamber, boolean shooting) {
        int basePos;

        switch(chamber) {
            case 0: basePos = CHAMBER_0_POS; break;
            case 1: basePos = CHAMBER_1_POS; break;
            case 2: basePos = CHAMBER_2_POS; break;
            default: basePos = CHAMBER_0_POS;
        }

        // Add offset if in shooting mode
        if (shooting) {
            basePos = normalize(basePos + OFFSET);
        }

        return basePos;
    }

    /** Auto chamber switching with continuous color detection - INTAKE MODE ONLY */
    private void autoIntakeColorCheck() {
        if (allChambersFull()) return;

        String detected = detectIntakeColor();

        if (detected.equals("NONE")) {
            colorActive = false;
            colorStartTime = 0;
            return;
        }

        if (!colorActive) {
            colorActive = true;
            colorStartTime = System.currentTimeMillis();
        }

        if (System.currentTimeMillis() - colorStartTime >= DETECT_TIME_MS) {

            if (!chamberFull[currentChamber]) {
                chamberFull[currentChamber] = true;

                // Switch to next chamber
                currentChamber = nextChamber(currentChamber);

                // Move to absolute position of new chamber (always intake mode here)
                int target = getChamberPosition(currentChamber, false);
                moveToExternal(target);
            }

            colorActive = false;
            colorStartTime = 0;
        }
    }

    /** Detect intake color - GREEN / PURPLE / NONE */
    private String detectIntakeColor() {
        int r = intakeColor.red();
        int g = intakeColor.green();
        int b = intakeColor.blue();

        if (g > r && g > b && g > 80 && g < 600) return "GREEN";
        if (b > r && b > g && b > 80 && b < 600) return "PURPLE";
        return "NONE";
    }

    /** Detect shooter color - GREEN / PURPLE / NONE */
    private String detectShooterColor() {
        int r = shooterColor.red();
        int g = shooterColor.green();
        int b = shooterColor.blue();

        if (g > r && g > b && g > 80 && g < 600) return "GREEN";
        if (b > r && b > g && b > 80 && b < 600) return "PURPLE";
        return "NONE";
    }

    private boolean allChambersFull() {
        return chamberFull[0] && chamberFull[1] && chamberFull[2];
    }

    private int nextChamber(int c) {
        if (c == 1) return 0;
        if (c == 0) return 2;
        return 1;
    }

    private int normalize(int ticks) {
        return ((ticks % FULL_ROT) + FULL_ROT) % FULL_ROT;
    }

    /**
     * IMPROVED movement function with:
     * - Tighter tolerance
     * - Better loop timing
     * - Smarter power ramping
     * - Final positioning correction
     */
    private void moveToExternal(int targetTicks) {
        final int COARSE_TOL = 400;     // Start final approach
        final int FINE_TOL = 145;       // Acceptable final error
        final int PERFECT_TOL = 100;     // Try to get within this
        final double MAX_POWER = 1;
        final double MIN_POWER = 0.08;
        final long TIMEOUT_MS = 2000;
        final long SETTLE_MS = 100;

        ElapsedTime timer = new ElapsedTime();
        ElapsedTime settleTimer = new ElapsedTime();
        boolean settling = false;

        while (opModeIsActive() && timer.milliseconds() < TIMEOUT_MS) {
            int pos = normalize(sorterEncoder.getCurrentPosition());
            int error = calculateShortestError(pos, targetTicks);

            // If we're within perfect tolerance, start settling
            if (Math.abs(error) <= PERFECT_TOL) {
                if (!settling) {
                    settling = true;
                    settleTimer.reset();
                    sorterMotor.setPower(0);
                }

                // If settled for long enough, we're done
                if (settleTimer.milliseconds() >= SETTLE_MS) {
                    break;
                }

                // Check if we drifted too far
                if (Math.abs(error) > FINE_TOL) {
                    settling = false;
                } else {
                    continue;
                }
            } else {
                settling = false;
            }

            // Calculate power using smooth ramping
            double power;
            int absError = Math.abs(error);

            if (absError > COARSE_TOL) {
                // Full speed for large errors
                power = MAX_POWER;
            } else {
                // Proportional control for fine positioning
                // Linear interpolation between MIN and MAX power
                double ratio = (double) absError / COARSE_TOL;
                power = MIN_POWER + (MAX_POWER - MIN_POWER) * ratio;
                power = Math.max(MIN_POWER, Math.min(MAX_POWER, power));
            }

            sorterMotor.setPower(Math.signum(error) * power);

            // Small delay for stable loop timing
            sleep(5);
        }

        // Final stop
        sorterMotor.setPower(0);
        sleep(50);

        // One more correction if needed
        int finalError = calculateShortestError(
                normalize(sorterEncoder.getCurrentPosition()),
                targetTicks
        );

        if (Math.abs(finalError) > FINE_TOL && Math.abs(finalError) < COARSE_TOL) {
            // Small correction move
            double correctionPower = Math.signum(finalError) * MIN_POWER;
            sorterMotor.setPower(correctionPower);

            ElapsedTime correctionTimer = new ElapsedTime();
            while (opModeIsActive() && correctionTimer.milliseconds() < 200) {
                int pos = normalize(sorterEncoder.getCurrentPosition());
                int err = calculateShortestError(pos, targetTicks);

                if (Math.abs(err) <= PERFECT_TOL) {
                    break;
                }

                sleep(5);
            }

            sorterMotor.setPower(0);
        }

        sleep(30);
    }

    /**
     * Calculate the shortest rotational error (handles wraparound)
     */
    private int calculateShortestError(int current, int target) {
        int error = target - current;

        // Handle wraparound - always take shortest path
        if (error > FULL_ROT / 2) {
            error -= FULL_ROT;
        } else if (error < -FULL_ROT / 2) {
            error += FULL_ROT;
        }

        return error;
    }
}