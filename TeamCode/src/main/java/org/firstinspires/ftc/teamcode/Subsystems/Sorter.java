package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Configurable
public class Sorter {
    public enum sorterStateFSM {
        INTAKE_STATIC,       // in intake mode, not moving
        SWITCHING_CHAMBERS,  // rotating chambers while in intake mode
        SHOOTING             // spinning full 360° to shoot all balls
    }

    private RevColorSensorV3 intakeColor;

    private DcMotorEx sorterMotor;
    private DcMotorEx sorterEncoder;

    CRServo s3;
    Servo s2;

    public sorterStateFSM sorterState;
    public String[] chamberColors = {"NONE", "NONE", "NONE"};
    public enum MOTIF {
        GPP,
        PGP,
        PPG
    }

    public MOTIF currentMotif;

    // === Non-blocking sorter movement ===
    private int sorterTargetPosition = 0;
    private ElapsedTime sorterTimer = new ElapsedTime();

    // === Timed color detection ===
    private long colorStartTime = 0;
    private boolean colorActive = false;
    private static final long DETECT_TIME_MS = 10;

    // === Sorter constants ===
    private static final int FULL_ROT = 8192;
    private static final int SLOT = FULL_ROT / 3;
    private static final int OFFSET = (FULL_ROT / 2); // 180 degrees from full rotation
    private static final int CHAMBER_0_POS = 0;
    private static final int CHAMBER_1_POS = SLOT;
    private static final int CHAMBER_2_POS = 2 * SLOT;
    private int currentChamber = 0;

    boolean lastDpadRight = false;
    boolean lastY = false;
    boolean lastA = false;

    private ElapsedTime moveTimer = new ElapsedTime();

    // === PID State ===
    private double integral = 0.0;
    private double lastError = 0.0;
    private long lastTime = 0;

    // === PID GAINS (Editable in FTC Dashboard) ===
    public static double kP = 0.002;
    public static double kI = 0.0;
    public static double kD = 0.000039;

    // === Shoot Spin Sequence ===
    // Tracks encoder position at the start of a shoot so we can measure 360°
    private int shootStartPosition = 0;
    private boolean shootSpinStarted = false;

    // Feedback LEDs
    private Servo chambersFull_LED, currentSorterAColor_LED;
    private double oscillating_LED_color = 0.0;

    private static Gamepad gamepad1;
    public int manualTeleopOffset = 0;

    public Sorter(HardwareMap hardwareMap){
        this.sorterState = sorterStateFSM.INTAKE_STATIC;
        this.sorterMotor = hardwareMap.get(DcMotorEx.class, "m0");
        this.sorterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.sorterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.sorterEncoder = hardwareMap.get(DcMotorEx.class, "bR");
        this.sorterEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.sorterEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.chamberColors[0] = "NONE";
        this.chamberColors[1] = "NONE";
        this.chamberColors[2] = "NONE";

        this.intakeColor = hardwareMap.get(RevColorSensorV3.class, "intakeColor");

        this.currentMotif = MOTIF.GPP;

        this.s2 = hardwareMap.get(Servo.class, "s2");
        this.s3 = hardwareMap.get(CRServo.class, "s3");
        this.s3.setDirection(DcMotorSimple.Direction.REVERSE);
        this.s2.setPosition(.68);

        this.chambersFull_LED = hardwareMap.get(Servo.class, "led1");
        this.currentSorterAColor_LED = hardwareMap.get(Servo.class, "led2");
    }

    public void updateSorter() {
        // If not shooting, check auto intake color
        if (sorterState.equals(sorterStateFSM.INTAKE_STATIC)) {
            autoIntakeColorCheck();
        }

        if (sorterState.equals(sorterStateFSM.SHOOTING)) {
            updateShootSpin();
        } else {
            updateSorterPIDMove();
        }

        updateChambersFullLED();
    }

    public void updateSorter(Gamepad gamepad1) {
        boolean dpadRightPressed = gamepad1.dpad_right;
        boolean yPressed = gamepad1.y;
        boolean aPressed = gamepad1.a;

        // If not shooting, check auto intake color
        if (sorterState.equals(sorterStateFSM.INTAKE_STATIC)) {
            autoIntakeColorCheck();
        }

        if (sorterState.equals(sorterStateFSM.SHOOTING)) {
            updateShootSpin(gamepad1);
        } else {
            updateSorterPIDMove();
        }

        if (dpadRightPressed && !lastDpadRight) {
            sorterState = sorterStateFSM.SWITCHING_CHAMBERS;

            currentChamber = prevChamber(currentChamber);
            rotateChamberColorsCounterClockwise();
            int targetPos = getChamberPosition(currentChamber, false);
            startSorterMove(targetPos);
        }

        if (aPressed && !lastA) {
            startShootingSequence();
        }

        // Update Internal TurretMOTIF
        if (yPressed && !lastY) {
            switch (currentMotif) {
                case GPP:
                    currentMotif = MOTIF.PGP;
                    break;
                case PGP:
                    currentMotif = MOTIF.PPG;
                    break;
                case PPG:
                    currentMotif = MOTIF.GPP;
                    break;
            }
        }

        updateChambersFullLED();

        lastDpadRight = dpadRightPressed;
        lastY = yPressed;
        lastA = aPressed;
    }

    // ========================================================================
    // SHOOT SPIN — replaces the old multi-step sequence
    // ========================================================================

    /**
     * Spins the sorter one full rotation (360°) at max power to fire all balls.
     * When complete, clears all chamber colors and returns to INTAKE_STATIC.
     * TeleOp variant: also rumbles the gamepad on completion.
     */
    // === Shoot Spin constants ===
    public static double SHOOT_POWER_SLOW = -1;   // gentle start to avoid jam
    public static double SHOOT_POWER_FAST = -1;   // full power once ball is clear
    public static int SHOOT_RAMP_TICKS = FULL_ROT / 7; // ~120° before kicking to full
    // === Unjam constants ===
    public static double UNJAM_STALL_SECONDS = 0.25;    // how long with no movement before unjamming
    public static double UNJAM_REVERSE_SECONDS = 0.25;  // how long to reverse
    public static int UNJAM_MOVEMENT_THRESHOLD = 100;   // ticks — below this counts as "not moving"
    public static double UNJAM_REVERSE_POWER = 1;    // reverse power during unjam

    // === Unjam State ===
    private int lastEncoderSnapshot = 0;
    private ElapsedTime stallTimer = new ElapsedTime();
    private ElapsedTime unjamTimer = new ElapsedTime();
    private boolean isUnjamming = false;

    private void updateShootSpin(Gamepad gamepad1) {
        if (!shootSpinStarted) {
            shootStartPosition = sorterEncoder.getCurrentPosition();
            lastEncoderSnapshot = shootStartPosition;
            shootSpinStarted = true;
            isUnjamming = false;
            stallTimer.reset();
            s3.setPower(1.0);
        }

        int currentPos = sorterEncoder.getCurrentPosition();
        int ticksTraveled = Math.abs(currentPos - shootStartPosition);

        // === Unjam check ===
        if (isUnjamming) {
            // Reversing — wait out the unjam duration then resume
            sorterMotor.setPower(UNJAM_REVERSE_POWER);
            if (unjamTimer.seconds() >= UNJAM_REVERSE_SECONDS) {
                isUnjamming = false;
                // Recalculate shoot start so ticksTraveled doesn't jump backward
                shootStartPosition = currentPos - ticksTraveled;
                lastEncoderSnapshot = currentPos;
                stallTimer.reset();
            }
            return;
        }

        // Check if encoder has moved enough since last snapshot
        if (Math.abs(currentPos - lastEncoderSnapshot) > UNJAM_MOVEMENT_THRESHOLD) {
            // Moving fine — update snapshot and reset stall timer
            lastEncoderSnapshot = currentPos;
            stallTimer.reset();
        } else if (stallTimer.seconds() >= UNJAM_STALL_SECONDS) {
            // Stalled — trigger unjam
            isUnjamming = true;
            unjamTimer.reset();
            return;
        }

        // === Normal shoot spin ===
        if (ticksTraveled < FULL_ROT * 2) {
            double power = (ticksTraveled < SHOOT_RAMP_TICKS) ? SHOOT_POWER_SLOW : SHOOT_POWER_FAST;
            sorterMotor.setPower(power);
        } else {
            sorterMotor.setPower(0.0);
            s3.setPower(0.0);
            shootSpinStarted = false;
            isUnjamming = false;

            chamberColors[0] = "NONE";
            chamberColors[1] = "NONE";
            chamberColors[2] = "NONE";

            sorterState = sorterStateFSM.INTAKE_STATIC;
            int targetPos = getChamberPosition(currentChamber, false);
            startSorterMove(targetPos);

            if (gamepad1 != null) gamepad1.rumble(500);
        }
    }
    private void updateShootSpin() {
        updateShootSpin(null);
    }

    // ========================================================================
    // PID MOVE (used for chamber switching and intake alignment)
    // ========================================================================

    private void updateSorterPIDMove() {
        int rawPos = sorterEncoder.getCurrentPosition();
        int pos = _normalize(rawPos);
        int error = _calculateShortestError(pos, sorterTargetPosition);

        long currentTime = System.nanoTime();
        double dt = (currentTime - lastTime) / 1e9;
        lastTime = currentTime;

        if (dt <= 0 || dt > 0.1) dt = 0.02;

        double pTerm = kP * error;

        integral += error * dt;
        integral = Math.max(-5000, Math.min(5000, integral));
        double iTerm = kI * integral;

        double derivative = (error - lastError) / dt;
        double dTerm = kD * derivative;

        lastError = error;

        double power = Math.max(-1.0, Math.min(1.0, pTerm + iTerm + dTerm));
        sorterMotor.setPower(power);

        if (sorterState == sorterStateFSM.SWITCHING_CHAMBERS) {
            if (Math.abs(error) < 80) {
                sorterState = sorterStateFSM.INTAKE_STATIC;
            }
        }
    }

    private void updateChambersFullLED() {
        if (allChambersFull()) {
            this.oscillating_LED_color += 0.005;
            if (this.oscillating_LED_color > 0.722) {
                this.oscillating_LED_color = 0.277;
            }
            this.chambersFull_LED.setPosition(oscillating_LED_color);
        } else {
            this.chambersFull_LED.setPosition(0.0);
        }
    }

    private void updateCurrentSorterALED(String color) {
        if (color.equals("GREEN")) {
            this.currentSorterAColor_LED.setPosition(0.5);
        } else if (color.equals("PURPLE")) {
            this.currentSorterAColor_LED.setPosition(0.722);
        } else {
            this.currentSorterAColor_LED.setPosition(0);
        }
    }

    // ========================================================================
    // CHAMBER POSITION CALCULATOR
    // ========================================================================

    private int getChamberPosition(int chamber, boolean shooting) {
        int basePos;
        switch(chamber) {
            case 0: basePos = CHAMBER_0_POS; break;
            case 1: basePos = CHAMBER_1_POS; break;
            case 2: basePos = CHAMBER_2_POS; break;
            default: basePos = CHAMBER_0_POS;
        }
        if (shooting) basePos = _normalize(basePos + OFFSET);
        return basePos;
    }

    // ========================================================================
    // ENCODER UTILITIES
    // ========================================================================

    private int _normalize(int ticks) {
        return ((ticks % FULL_ROT) + FULL_ROT) % FULL_ROT;
    }

    private int _calculateShortestError(int current, int target) {
        int error = target + manualTeleopOffset - current;
        if (error > FULL_ROT / 2)  error -= FULL_ROT;
        else if (error < -FULL_ROT / 2) error += FULL_ROT;
        return error;
    }

    // ========================================================================
    // AUTOMATIC INTAKE COLOR DETECTION
    // ========================================================================

    private void autoIntakeColorCheck() {
        String detected = detectIntakeColor();
        updateCurrentSorterALED(detected);

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
            if (chamberColors[0].equals("NONE")) {
                chamberColors[0] = detected;
                sorterState = sorterStateFSM.SWITCHING_CHAMBERS;

                currentChamber = prevChamber(currentChamber);
                rotateChamberColorsCounterClockwise();
                int targetPos = getChamberPosition(currentChamber, false);
                startSorterMove(targetPos);
            }
            colorActive = false;
            colorStartTime = 0;
        }
    }

    public boolean allChambersFull() {
        return !chamberColors[0].equals("NONE")
                && !chamberColors[1].equals("NONE")
                && !chamberColors[2].equals("NONE");
    }

    private String detectIntakeColor() {
        int r = intakeColor.red();
        int g = intakeColor.green();
        int b = intakeColor.blue();

        if (g > r && g > b && g > 80 && g < 600) return "GREEN";
        if (b > r && b > g && b > 80 && b < 600) return "PURPLE";
        return "NONE";
    }

    // ========================================================================
    // CHAMBER SORTING OPERATIONS
    // ========================================================================

    private void rotateChamberColorsClockwise() {
        String[] out = {chamberColors[0], chamberColors[1], chamberColors[2]};
        chamberColors[0] = out[2];
        chamberColors[1] = out[0];
        chamberColors[2] = out[1];
    }

    private void rotateChamberColorsCounterClockwise() {
        String[] out = {chamberColors[0], chamberColors[1], chamberColors[2]};
        chamberColors[0] = out[1];
        chamberColors[1] = out[2];
        chamberColors[2] = out[0];
    }

    private int indexOfColor(String[] chamberColors, String desiredColor, boolean returnNextBest) {
        for (int i = 0; i <= 2; i++) {
            if (chamberColors[i].equals(desiredColor)) return i;
        }
        if (returnNextBest) {
            for (int i = 0; i <= 2; i++) {
                if (!chamberColors[i].equals("NONE")) return i;
            }
        }
        return -1;
    }

    // ========================================================================
    // CHAMBER ROTATION SEQUENCE
    // ========================================================================

    private int nextChamber(int c) {
        if (c == 1) return 0;
        if (c == 0) return 2;
        return 1;
    }

    private int prevChamber(int c) {
        if (c == 0) return 1;
        if (c == 1) return 2;
        return 0;
    }

    // ========================================================================
    // SORTER MOVEMENT INITIATION
    // ========================================================================

    private void startSorterMove(int targetPosition) {
        sorterTargetPosition = targetPosition;
        sorterTimer.reset();
    }

    public void startShootingSequence() {
        shootSpinStarted = false; // ensure fresh start
        sorterState = sorterStateFSM.SHOOTING;
    }

    public void autoAlignChamberColors() {
        int greenIndex = indexOfColor(chamberColors, "GREEN", true);

        int rotationCompensateForMotif = 0;
        if (currentMotif.equals(MOTIF.GPP)) rotationCompensateForMotif = -1;
        if (currentMotif.equals(MOTIF.PGP)) rotationCompensateForMotif =  1;
        if (currentMotif.equals(MOTIF.PPG)) rotationCompensateForMotif =  0;

        if (greenIndex != -1) {
            int rotationsNeeded = 0 + rotationCompensateForMotif;

            if (greenIndex == 0) {
                rotationsNeeded = 0 + rotationCompensateForMotif;
            } else if (greenIndex == 1) {
                rotationsNeeded = -1 + rotationCompensateForMotif;
            } else if (greenIndex == 2) {
                rotationsNeeded = -2 + rotationCompensateForMotif;
            }

            for (int i = 0; i < Math.abs(rotationsNeeded); i++) {
                if (rotationsNeeded > 0) {
                    rotateChamberColorsClockwise();
                    currentChamber = nextChamber(currentChamber);
                } else if (rotationsNeeded < 0) {
                    rotateChamberColorsCounterClockwise();
                    currentChamber = prevChamber(currentChamber);
                }
            }

            int targetPos = getChamberPosition(currentChamber, false);
            startSorterMove(targetPos);
        }
    }

    public void updateManualOffset(int amount_to_change) {
        manualTeleopOffset += amount_to_change;
    }

    public void resetSorterAtEndOfAuton(int trigger) {
        if (trigger != -101) {
            sorterState = sorterStateFSM.INTAKE_STATIC;
            int targetPos = getChamberPosition(currentChamber, false);
            startSorterMove(targetPos);
            deactivateShooter();
        }
    }

    private void activateShooter() {
        s2.setPosition(0);
        s3.setPower(1.0);
    }

    private void deactivateShooter() {
        s2.setPosition(0.68);
        s3.setPower(0.0);
    }

    public void setShootingConstants(double shoot_duration, double servo_retract_delay, double sorter_wait_time, double mode_toggle_wait_time) {
        // No longer used — shooting is now a single full spin.
        // Kept for API compatibility.
    }

    public void postTelemetry(Telemetry telemetry) {
        int rawPos = sorterEncoder.getCurrentPosition();
        int normPos = _normalize(rawPos);

        telemetry.addLine("=== Sorter ===");
        telemetry.addData("Current Detected Color: ", detectIntakeColor());
        telemetry.addData("Current Sorter State", sorterState);
        telemetry.addData("Pos", normPos);
        telemetry.addData("Target Position", sorterTargetPosition);
        telemetry.addData("Chamber", currentChamber + 1);
        telemetry.addData("Moving", sorterState.equals(sorterStateFSM.SWITCHING_CHAMBERS));
        telemetry.addData("color active", colorActive);
        telemetry.addData("color start time", colorStartTime);
        telemetry.addData("current chamber", currentChamber);

        String ch1 = !chamberColors[0].equals("NONE") ? chamberColors[0] : "X";
        String ch2 = !chamberColors[1].equals("NONE") ? chamberColors[1] : "X";
        String ch3 = !chamberColors[2].equals("NONE") ? chamberColors[2] : "X";
        telemetry.addData("Ch1/2/3", ch1 + "/" + ch2 + "/" + ch3);

        if (sorterState == sorterStateFSM.SHOOTING) {
            int ticksTraveled = Math.abs(sorterEncoder.getCurrentPosition() - shootStartPosition);
            telemetry.addData("Shoot progress (ticks)", ticksTraveled + " / " + FULL_ROT);
        }

        telemetry.addData("Current Motif State: ", currentMotif);
        telemetry.addLine();
    }
}