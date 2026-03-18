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
        SORTING,             // auto-sorting after all 3 balls intaked
        SHOOT_ALIGNING,      // PID-moving to aligned position before shooting
        SHOOTING             // spinning full 360° to shoot all balls
    }

    private RevColorSensorV3 intakeColor;

    private DcMotorEx sorterMotor;
    private DcMotorEx sorterEncoder;

    CRServo s3;
    Servo s2;

    public sorterStateFSM sorterState;
    public String[] chamberColors = {"NONE", "NONE", "NONE"};

    // =========================================================================
    // MOTIF ENUM
    // Slot layout (top-down, CW = shoot direction):
    //   [0] = BOTTOM — intake position, fires 2nd
    //   [1] = LEFT   — fires 1st  (CW spin hits this first)
    //   [2] = RIGHT  — fires 3rd
    //
    // Motif = firing order. First letter fires first (LEFT slot).
    //   GPP -> green must be in LEFT   (index 1)
    //   PGP -> green must be in BOTTOM (index 0)
    //   PPG -> green must be in RIGHT  (index 2)
    // =========================================================================
    public enum MOTIF {
        GPP,   // Green fires 1st -> green goes to LEFT
        PGP,   // Green fires 2nd -> green goes to BOTTOM
        PPG    // Green fires 3rd -> green goes to RIGHT
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
    private static final int FULL_ROT      = 8192;
    private static final int SLOT          = FULL_ROT / 3;
    private static final int OFFSET        = FULL_ROT / 2;
    private static final int CHAMBER_0_POS = 0;
    private static final int CHAMBER_1_POS = SLOT;
    private static final int CHAMBER_2_POS = 2 * SLOT;
    private int currentChamber = 0;

    boolean lastDpadRight = false;
    boolean lastY = false;
    boolean lastA = false;

    private ElapsedTime moveTimer = new ElapsedTime();

    // === PID State ===
    private double integral  = 0.0;
    private double lastError = 0.0;
    private long   lastTime  = 0;

    // === PID GAINS (Editable in FTC Dashboard) ===
    public static double kP = 0.002;
    public static double kI = 0.0;
    public static double kD = 0.000039;

    // === Shoot Spin Sequence ===
    private int     shootStartPosition = 0;
    private boolean shootSpinStarted   = false;

    // Feedback LEDs
    private Servo  chambersFull_LED, currentSorterAColor_LED;
    private double oscillating_LED_color = 0.0;

    private static Gamepad gamepad1;
    public int manualTeleopOffset = 0;

    // === CCW-only PID flag ===
    // When true, the PID uses _calculateCCWOnlyError() so it never shortcuts
    // CW through the transfer wedge during sorting or shoot-alignment moves.
    private boolean sortingCCWOnly = false;

    // =========================================================================
    // CONSTRUCTOR
    // =========================================================================
    public Sorter(HardwareMap hardwareMap) {
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

        this.chambersFull_LED        = hardwareMap.get(Servo.class, "led1");
        this.currentSorterAColor_LED = hardwareMap.get(Servo.class, "led2");
    }

    // =========================================================================
    // UPDATE - autonomous (no gamepad)
    // =========================================================================
    public void updateSorter() {
        if (sorterState == sorterStateFSM.INTAKE_STATIC) {
            autoIntakeColorCheck();
        }

        if (sorterState == sorterStateFSM.SHOOTING) {
            updateShootSpin();
        } else {
            updateSorterPIDMove();
        }

        if (sorterState == sorterStateFSM.SHOOT_ALIGNING) {
            int pos   = _normalize(sorterEncoder.getCurrentPosition());
            int error = _calculateCCWOnlyError(pos, sorterTargetPosition);
            if (Math.abs(error) < 80) {
                sortingCCWOnly = false;
                sorterState    = sorterStateFSM.SHOOTING;
            }
        }

        if (sorterState == sorterStateFSM.SORTING) {
            int pos   = _normalize(sorterEncoder.getCurrentPosition());
            int error = _calculateCCWOnlyError(pos, sorterTargetPosition);
            if (Math.abs(error) < 80) {
                sortingCCWOnly = false;
                sorterState    = sorterStateFSM.INTAKE_STATIC;
            }
        }

        updateChambersFullLED();
    }

    // =========================================================================
    // UPDATE - teleop (with gamepad)
    // =========================================================================
    public void updateSorter(Gamepad gamepad1) {
        boolean dpadRightPressed = gamepad1.dpad_right;
        boolean yPressed         = gamepad1.y;
        boolean aPressed         = gamepad1.a;

        if (sorterState == sorterStateFSM.INTAKE_STATIC) {
            autoIntakeColorCheck();
        }

        if (sorterState == sorterStateFSM.SHOOTING) {
            updateShootSpin(gamepad1);
        } else {
            updateSorterPIDMove();
        }

        if (sorterState == sorterStateFSM.SHOOT_ALIGNING) {
            int pos   = _normalize(sorterEncoder.getCurrentPosition());
            int error = _calculateCCWOnlyError(pos, sorterTargetPosition);
            if (Math.abs(error) < 80) {
                sortingCCWOnly = false;
                sorterState    = sorterStateFSM.SHOOTING;
            }
        }

        if (sorterState == sorterStateFSM.SORTING) {
            int pos   = _normalize(sorterEncoder.getCurrentPosition());
            int error = _calculateCCWOnlyError(pos, sorterTargetPosition);
            if (Math.abs(error) < 80) {
                sortingCCWOnly = false;
                sorterState    = sorterStateFSM.INTAKE_STATIC;
            }
        }

        // Manual chamber advance (CCW one slot)
        if (dpadRightPressed && !lastDpadRight) {
            currentChamber = prevChamber(currentChamber);
            rotateChamberColorsCounterClockwise();
            int targetPos = getChamberPosition(currentChamber, false);
            startSorterMove(targetPos);
            sorterState = sorterStateFSM.SWITCHING_CHAMBERS;
        }

        // Manual shoot trigger
        if (aPressed && !lastA) {
            startShootingSequence();
        }

        // Cycle motif
        if (yPressed && !lastY) {
            switch (currentMotif) {
                case GPP: currentMotif = MOTIF.PGP; break;
                case PGP: currentMotif = MOTIF.PPG; break;
                case PPG: currentMotif = MOTIF.GPP; break;
            }
        }

        updateChambersFullLED();

        lastDpadRight = dpadRightPressed;
        lastY         = yPressed;
        lastA         = aPressed;
    }

    // =========================================================================
    // SHOOT SPIN
    // =========================================================================
    public static double SHOOT_POWER_SLOW         = -0.8;
    public static double SHOOT_POWER_FAST         = -0.8;
    public static int    SHOOT_RAMP_TICKS         = FULL_ROT / 7;
    public static double UNJAM_STALL_SECONDS      = 0.25;
    public static double UNJAM_REVERSE_SECONDS    = 0.25;
    public static int    UNJAM_MOVEMENT_THRESHOLD = 100;
    public static double UNJAM_REVERSE_POWER      = 1;

    private int         lastEncoderSnapshot = 0;
    private ElapsedTime stallTimer          = new ElapsedTime();
    private ElapsedTime unjamTimer          = new ElapsedTime();
    private boolean     isUnjamming         = false;

    private void updateShootSpin(Gamepad gamepad1) {
        if (!shootSpinStarted) {
            shootStartPosition  = sorterEncoder.getCurrentPosition();
            lastEncoderSnapshot = shootStartPosition;
            shootSpinStarted    = true;
            isUnjamming         = false;
            stallTimer.reset();
            s3.setPower(1.0);
        }

        int currentPos    = sorterEncoder.getCurrentPosition();
        int ticksTraveled = Math.abs(currentPos - shootStartPosition);

        if (isUnjamming) {
            sorterMotor.setPower(UNJAM_REVERSE_POWER);
            if (unjamTimer.seconds() >= UNJAM_REVERSE_SECONDS) {
                isUnjamming         = false;
                shootStartPosition  = currentPos - ticksTraveled;
                lastEncoderSnapshot = currentPos;
                stallTimer.reset();
            }
            return;
        }

        if (Math.abs(currentPos - lastEncoderSnapshot) > UNJAM_MOVEMENT_THRESHOLD) {
            lastEncoderSnapshot = currentPos;
            stallTimer.reset();
        } else if (stallTimer.seconds() >= UNJAM_STALL_SECONDS) {
            isUnjamming = true;
            unjamTimer.reset();
            return;
        }

        if (ticksTraveled < FULL_ROT * 2) {
            double power = (ticksTraveled < SHOOT_RAMP_TICKS) ? SHOOT_POWER_SLOW : SHOOT_POWER_FAST;
            sorterMotor.setPower(power);
        } else {
            sorterMotor.setPower(0.0);
            s3.setPower(0.0);
            shootSpinStarted = false;
            isUnjamming      = false;

            chamberColors[0] = "NONE";
            chamberColors[1] = "NONE";
            chamberColors[2] = "NONE";

            sorterState = sorterStateFSM.INTAKE_STATIC;
            startSorterMove(getChamberPosition(currentChamber, false));

            if (gamepad1 != null) gamepad1.rumble(500);
        }
    }

    private void updateShootSpin() {
        updateShootSpin(null);
    }

    // =========================================================================
    // PID MOVE
    // =========================================================================
    private void updateSorterPIDMove() {
        int pos = _normalize(sorterEncoder.getCurrentPosition());

        // Use CCW-only error during sorting/aligning so the PID never takes a
        // shorter CW path that would drive the spindexer through the transfer wedge.
        int error = sortingCCWOnly
                ? _calculateCCWOnlyError(pos, sorterTargetPosition)
                : _calculateShortestError(pos, sorterTargetPosition);

        long   currentTime = System.nanoTime();
        double dt          = (currentTime - lastTime) / 1e9;
        lastTime = currentTime;
        if (dt <= 0 || dt > 0.1) dt = 0.02;

        double pTerm = kP * error;

        integral += error * dt;
        integral  = Math.max(-5000, Math.min(5000, integral));
        double iTerm = kI * integral;

        double dTerm = kD * ((error - lastError) / dt);
        lastError = error;

        double power = Math.max(-1.0, Math.min(1.0, pTerm + iTerm + dTerm));
        sorterMotor.setPower(power);

        if (sorterState == sorterStateFSM.SWITCHING_CHAMBERS && Math.abs(error) < 80) {
            sorterState = sorterStateFSM.INTAKE_STATIC;
        }
    }

    // =========================================================================
    // LED FEEDBACK
    // =========================================================================
    private void updateChambersFullLED() {
        if (allChambersFull()) {
            oscillating_LED_color += 0.005;
            if (oscillating_LED_color > 0.722) oscillating_LED_color = 0.277;
            chambersFull_LED.setPosition(oscillating_LED_color);
        } else {
            chambersFull_LED.setPosition(0.0);
        }
    }

    private void updateCurrentSorterALED(String color) {
        if (color.equals("GREEN"))       currentSorterAColor_LED.setPosition(0.5);
        else if (color.equals("PURPLE")) currentSorterAColor_LED.setPosition(0.722);
        else                             currentSorterAColor_LED.setPosition(0);
    }

    // =========================================================================
    // CHAMBER POSITION
    // =========================================================================
    private int getChamberPosition(int chamber, boolean shooting) {
        int basePos;
        switch (chamber) {
            case 1:  basePos = CHAMBER_1_POS; break;
            case 2:  basePos = CHAMBER_2_POS; break;
            default: basePos = CHAMBER_0_POS; break;
        }
        if (shooting) basePos = _normalize(basePos + OFFSET);
        return basePos;
    }

    // =========================================================================
    // ENCODER UTILITIES
    // =========================================================================
    private int _normalize(int ticks) {
        return ((ticks % FULL_ROT) + FULL_ROT) % FULL_ROT;
    }

    // Standard shortest-path error — used for normal intake chamber switching.
    private int _calculateShortestError(int current, int target) {
        int error = target + manualTeleopOffset - current;
        if (error >  FULL_ROT / 2) error -= FULL_ROT;
        if (error < -FULL_ROT / 2) error += FULL_ROT;
        return error;
    }

    // CCW-only error — always >= 0, so PID always drives CCW.
    // On this robot, SHOOT_POWER is negative meaning negative power = CW,
    // so CCW requires a POSITIVE error to produce positive PID output.
    // Prevents the shortest-path logic from cutting CW through the transfer
    // wedge when a 2-step sort move is needed.
    private int _calculateCCWOnlyError(int current, int target) {
        // First check the shortest-path distance. If we're already within
        // the settle threshold, return 0 so the PID doesn't overshoot by
        // taking the long way around (which would read as ~FULL_ROT ticks).
        int shortest = _calculateShortestError(current, target);
        if (Math.abs(shortest) < 80) return 0;

        // Otherwise force the error positive (CCW direction on this robot).
        int error = (target + manualTeleopOffset - current) % FULL_ROT;
        if (error < 0) error += FULL_ROT;
        return error;
    }

    // =========================================================================
    // AUTOMATIC INTAKE COLOR DETECTION
    // =========================================================================
    private void autoIntakeColorCheck() {
        String detected = detectIntakeColor();
        updateCurrentSorterALED(detected);

        if (detected.equals("NONE")) {
            colorActive    = false;
            colorStartTime = 0;
            return;
        }

        if (!colorActive) {
            colorActive    = true;
            colorStartTime = System.currentTimeMillis();
        }

        if (System.currentTimeMillis() - colorStartTime >= DETECT_TIME_MS) {
            if (chamberColors[0].equals("NONE")) {
                chamberColors[0] = detected;

                if (allChambersFull()) {
                    // All 3 balls loaded — sort immediately for the current motif.
                    // Driver just presses shoot, no extra wait needed.
                    autoAlignChamberColors();
                    sorterState = sorterStateFSM.SORTING;
                } else {
                    // More balls still needed — advance to next intake slot.
                    currentChamber = prevChamber(currentChamber);
                    rotateChamberColorsCounterClockwise();
                    int targetPos = getChamberPosition(currentChamber, false);
                    startSorterMove(targetPos);
                    sorterState = sorterStateFSM.SWITCHING_CHAMBERS;
                }
            }
            colorActive    = false;
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

    // =========================================================================
    // CHAMBER ARRAY ROTATION
    // =========================================================================

    // CW physical spin
    private void rotateChamberColorsClockwise() {
        String tmp       = chamberColors[0];
        chamberColors[0] = chamberColors[2];  // RIGHT  -> BOTTOM
        chamberColors[2] = chamberColors[1];  // LEFT   -> RIGHT
        chamberColors[1] = tmp;               // BOTTOM -> LEFT
    }

    // CCW physical spin: LEFT->BOTTOM, BOTTOM->RIGHT, RIGHT->LEFT
    // [0]=BOTTOM, [1]=LEFT, [2]=RIGHT
    private void rotateChamberColorsCounterClockwise() {
        String tmp       = chamberColors[0];
        chamberColors[0] = chamberColors[1];  // LEFT   -> BOTTOM
        chamberColors[1] = chamberColors[2];  // RIGHT  -> LEFT
        chamberColors[2] = tmp;               // BOTTOM -> RIGHT
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

    // =========================================================================
    // CHAMBER INDEX HELPERS
    // prevChamber = one CCW step of the physical spindexer
    // =========================================================================
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

    // =========================================================================
    // SORTER MOVEMENT
    // =========================================================================
    private void startSorterMove(int targetPosition) {
        sorterTargetPosition = targetPosition;
        sorterTimer.reset();
    }

    // =========================================================================
    // SHOOTING SEQUENCE
    // =========================================================================
    public void startShootingSequence() {
        shootSpinStarted = false;
        // If already sorted on intake, 0 steps needed and returns immediately.
        // If motif changed after intake, re-sorts correctly here.
        autoAlignChamberColors();
        sorterState = sorterStateFSM.SHOOT_ALIGNING;
    }

    // =========================================================================
    // MOTIF ALIGNMENT
    //
    // Slot mapping:  [0]=BOTTOM (fires 2nd), [1]=LEFT (fires 1st), [2]=RIGHT (fires 3rd)
    // CCW step formula: newSlotIndex = (currentSlotIndex + 2) % 3
    //
    // sortingCCWOnly is set here so the PID is locked to CCW for this move.
    // Without it, a 2-step CCW move (2/3 rotation) looks shorter going 1/3 CW,
    // which drives the spindexer through the transfer wedge and fires a ball.
    // =========================================================================
    public void autoAlignChamberColors() {
        int targetSlot = greenTargetSlotForMotif(currentMotif);
        int greenSlot  = findGreenSlot();

        if (greenSlot == -1) return;  // no balls loaded

        // Count CCW steps needed
        int simSlot = greenSlot;
        int steps   = 0;
        while (simSlot != targetSlot && steps < 3) {
            simSlot = (simSlot + 2) % 3;
            steps++;
        }

        if (steps == 0) {
            // Already aligned — no movement, no CCW lock needed
            sortingCCWOnly = false;
            return;
        }

        // Rotate array and chamber tracker together, one CCW step at a time
        for (int i = 0; i < steps; i++) {
            rotateChamberColorsCounterClockwise();
            currentChamber = prevChamber(currentChamber);
        }

        // Lock PID to CCW-only for this move
        sortingCCWOnly = true;
        startSorterMove(getChamberPosition(currentChamber, false));
    }

    // Which chamberColors[] index must hold green for the chosen motif?
    private int greenTargetSlotForMotif(MOTIF motif) {
        switch (motif) {
            case GPP: return 1;   // LEFT   - fires 1st
            case PGP: return 0;   // BOTTOM - fires 2nd
            case PPG: return 2;   // RIGHT  - fires 3rd
            default:  return 1;
        }
    }

    // Find the slot index holding green; fall back to any ball; -1 if empty.
    private int findGreenSlot() {
        for (int i = 0; i < 3; i++) {
            if ("GREEN".equals(chamberColors[i])) return i;
        }
        for (int i = 0; i < 3; i++) {
            if (!"NONE".equals(chamberColors[i])) return i;
        }
        return -1;
    }

    // =========================================================================
    // MISC PUBLIC API
    // =========================================================================
    public void updateManualOffset(int amount_to_change) {
        manualTeleopOffset += amount_to_change;
    }

    public void resetSorterAtEndOfAuton(int trigger) {
        if (trigger != -101) {
            sorterState = sorterStateFSM.INTAKE_STATIC;
            startSorterMove(getChamberPosition(currentChamber, false));
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

    public void setShootingConstants(double shoot_duration, double servo_retract_delay,
                                     double sorter_wait_time, double mode_toggle_wait_time) {
        // No longer used - kept for API compatibility.
    }

    // =========================================================================
    // TELEMETRY
    // =========================================================================

    public void setShooterSpeed(double speed) {
        SHOOT_POWER_FAST = -speed;
        SHOOT_POWER_SLOW = -speed;
    }
    public void postTelemetry(Telemetry telemetry) {
        int normPos = _normalize(sorterEncoder.getCurrentPosition());

        telemetry.addLine("=== Sorter ===");
        telemetry.addData("Detected color",    detectIntakeColor());
        telemetry.addData("State",             sorterState);
        telemetry.addData("Position (norm)",   normPos);
        telemetry.addData("Target position",   sorterTargetPosition);
        telemetry.addData("Current chamber",   currentChamber);
        telemetry.addData("Color active",      colorActive);
        telemetry.addData("Motif",             currentMotif);
        telemetry.addData("CCW-only lock",     sortingCCWOnly);

        String ch0 = !"NONE".equals(chamberColors[0]) ? chamberColors[0] : "X";
        String ch1 = !"NONE".equals(chamberColors[1]) ? chamberColors[1] : "X";
        String ch2 = !"NONE".equals(chamberColors[2]) ? chamberColors[2] : "X";
        telemetry.addData("BOTTOM/LEFT/RIGHT", ch0 + " / " + ch1 + " / " + ch2);

        if (sorterState == sorterStateFSM.SHOOTING) {
            int traveled = Math.abs(sorterEncoder.getCurrentPosition() - shootStartPosition);
            telemetry.addData("Shoot progress", traveled + " / " + FULL_ROT);
        }

        telemetry.addLine();
    }
}