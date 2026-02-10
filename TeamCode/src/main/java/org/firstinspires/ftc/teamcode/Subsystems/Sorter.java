package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.DriveTrainControl.ChamberTracking.AutoShootMotifPreset_BLUE;

@Configurable
public class Sorter {
    public enum sorterStateFSM {
        INTAKE_STATIC, // in intake mode, not moving
        SWITCHING_CHAMBERS, // rotating chambers while in intake mode
        SHOOTING
    }
    private RevColorSensorV3 intakeColor;

    private DcMotorEx sorterMotor;
    private DcMotorEx sorterEncoder;

    public sorterStateFSM sorterState;
    private String[] chamberColors = {"NONE", "NONE", "NONE"};
    private enum MOTIF {
        GPP,
        PGP,
        PPG
    }

    private MOTIF currentMotif;

    // === Non-blocking sorter movement ===
    private int sorterTargetPosition = 0;
    private ElapsedTime sorterTimer = new ElapsedTime();
    private ElapsedTime sorterSettleTimer = new ElapsedTime();
    private boolean sorterSettling = false;
    private static final int COARSE_TOL = 1000;
    private static final double MAX_POWER = 0.55;
    private static final double MIN_POWER = 0.08;


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

    private ElapsedTime moveTimer = new ElapsedTime();

    // === PID State ===
    private double integral = 0.0;
    private double lastError = 0.0;
    private long lastTime = 0;

    // === PID GAINS (Editable in FTC Dashboard) ===
    public static double kP = 0.002;
    public static double kI = 0.0;
    public static double kD = 0.00006;

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
    }

    public void updateSorter(Gamepad gamepad1) {
        // If not shooting, check auto intake color
        if (sorterState.equals(sorterStateFSM.INTAKE_STATIC)) {
            autoIntakeColorCheck();
        }

        ///// ===== If moving: ===== /////
        boolean dpadRightPressed = gamepad1.dpad_right;

        if (sorterState.equals(sorterStateFSM.SWITCHING_CHAMBERS)) {
            updateSorterPIDMove();
        }


        if (dpadRightPressed && !lastDpadRight) {
            sorterState = sorterStateFSM.SWITCHING_CHAMBERS;

            currentChamber = nextChamber(currentChamber);
            rotateChamberColorsClockwise();
            int targetPos = getChamberPosition(currentChamber, sorterState.equals(sorterStateFSM.SHOOTING));
            startSorterMove(targetPos);
        }

        boolean yPressed = gamepad1.y;

        ///// ===== Update Internal MOTIF  ====== /////
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

        lastDpadRight = dpadRightPressed;
        lastY = yPressed;
    }

    private int updateSorterPIDMove() {
        // Get current position and calculate error to target
        int pos = _normalize(sorterEncoder.getCurrentPosition());
        int error = _calculateShortestError(pos, sorterTargetPosition);

        // Calculate time delta
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastTime) / 1e9;
        lastTime = currentTime;

        if (dt <= 0 || dt > 0.1) {
            dt = 0.02;
        }

        // PID terms
        double pTerm = kP * error;

        integral += error * dt;
        integral = Math.max(-5000, Math.min(5000, integral)); // Anti-windup
        double iTerm = kI * integral;

        double derivative = (error - lastError) / dt;
        double dTerm = kD * derivative;

        lastError = error;

        // Total output
        double power = pTerm + iTerm + dTerm;

        // Clamp output
        power = Math.max(-1.0, Math.min(1.0, power));

        sorterMotor.setPower(power);

        return error;
    }

    // ========================================================================
    // SORTER MOVEMENT INITIATION
    // ========================================================================

    /**
     * Starts a new sorter movement to target position
     * Resets all movement state variables
     */
    private void _startSorterMove(int targetPosition) {
        sorterTargetPosition = targetPosition;
        sorterSettling = false;
        sorterState = sorterStateFSM.SWITCHING_CHAMBERS;
        sorterTimer.reset();
    }

    // ========================================================================
    // CHAMBER POSITION CALCULATOR
    // ========================================================================

    /**
     * Calculates the encoder position for a given chamber
     * In shooting mode, adds 180-degree offset to align chamber A with shooter
     *
     * @param chamber Which chamber (0, 1, or 2)
     * @param shooting Whether in shooting mode (true) or intake mode (false)
     * @return Target encoder position in ticks
     */
    /**
     * Calculates the encoder position for a given chamber
     * In shooting mode, adds 180-degree offset to align chamber A with shooter
     *
     * @param chamber Which chamber (0, 1, or 2)
     * @param shooting Whether in shooting mode (true) or intake mode (false)
     * @return Target encoder position in ticks
     */
    private int getChamberPosition(int chamber, boolean shooting) {
        int basePos;
        switch(chamber) {
            case 0: basePos = CHAMBER_0_POS; break;  // 0 degrees
            case 1: basePos = CHAMBER_1_POS; break;  // 120 degrees
            case 2: basePos = CHAMBER_2_POS; break;  // 240 degrees
            default: basePos = CHAMBER_0_POS;
        }

        // In shooting mode, rotate entire sorter by 60 degrees
        if (shooting) basePos = _normalize(basePos + OFFSET);

        return basePos;
    }

    // ========================================================================
    // ENCODER POSITION NORMALIZATION
    // ========================================================================

    /**
     * Normalizes encoder ticks to 0-8192 range (one full rotation)
     * Handles negative values and values beyond one rotation
     * Example: -100 becomes 8092, 8300 becomes 108
     *
     * @param ticks Raw encoder ticks
     * @return Normalized ticks in range [0, FULL_ROT)
     */
    private int _normalize(int ticks) {
        return ((ticks % FULL_ROT) + FULL_ROT) % FULL_ROT;
    }
    // ========================================================================
    // SHORTEST PATH CALCULATION
    // ========================================================================

    /**
     * Calculates the shortest rotational error between current and target positions
     * Since the sorter is circular, it can rotate either direction
     * This function chooses the shorter path
     *
     * Example: Current=100, Target=8000
     * - Clockwise: 8000-100 = 7900 ticks
     * - Counter-clockwise: 100+192-8000 = 292 ticks (SHORTER!)
     * - Returns: -292 (negative = counter-clockwise)
     *
     * @param current Current encoder position (normalized)
     * @param target Target encoder position (normalized)
     * @return Shortest error (positive = clockwise, negative = counter-clockwise)
     */
    private int _calculateShortestError(int current, int target) {
        int error = target - current;

        // If error is more than half a rotation, go the other way
        if (error > FULL_ROT / 2) {
            error -= FULL_ROT;  // Subtract full rotation to get shorter path
        } else if (error < -FULL_ROT / 2) {
            error += FULL_ROT;  // Add full rotation to get shorter path
        }

        return error;
    }
    // ========================================================================
    // AUTOMATIC INTAKE COLOR DETECTION
    // ========================================================================

    /**
     * Continuously checks for ball at intake during intake mode
     * When ball detected for DETECT_TIME_MS, fills current chamber and rotates to next
     * Uses timed detection to avoid false positives from brief color flashes
     */
    private void autoIntakeColorCheck() {
        // Don't check while sorter is moving (wait for it to settle)
        if (sorterState.equals(sorterStateFSM.SWITCHING_CHAMBERS)) return;

        String detected = detectIntakeColor();

        // No ball detected - reset timer
        if (detected.equals("NONE")) {
            colorActive = false;
            colorStartTime = 0;
            return;
        }

        // Ball detected - start/continue timer
        if (!colorActive) {
            colorActive = true;
            colorStartTime = System.currentTimeMillis();
        }

        // Ball has been detected continuously for required time
        if (System.currentTimeMillis() - colorStartTime >= DETECT_TIME_MS) {
            // Only fill chamber if it's not already full
            if (!chamberColors[currentChamber].equals("NONE")) {
                chamberColors[0] = detected;                  // Mark chamber A color (at intake position)
                rotateChamberColorsClockwise();
                currentChamber = nextChamber(currentChamber); // Move to next chamber

                // Rotate sorter to position next empty chamber at intake
                int target = getChamberPosition(currentChamber, false);
                startSorterMove(target);
            }

            // Reset detection timer
            colorActive = false;
            colorStartTime = 0;
        }
    }
    /**
     * Detects ball color at intake sensor
     * Analyzes RGB values to determine if ball is green, purple, or not present
     *
     * @return "GREEN", "PURPLE", or "NONE"
     */
    private String detectIntakeColor() {
        int r = intakeColor.red();
        int g = intakeColor.green();
        int b = intakeColor.blue();

        // Green ball: green channel dominant and in valid range
        if (g > r && g > b && g > 80 && g < 600) return "GREEN";

        // Purple ball: blue channel dominant and in valid range
        if (b > r && b > g && b > 80 && b < 600) return "PURPLE";

        return "NONE";  // No ball or unrecognized color
    }

    // ========================================================================
    // CHAMBER SORTING OPERATIONS
    // ========================================================================

    /**
     * Helper class for rotating chamber color arrays
     * Currently unused - was intended for tracking ball colors as chambers rotate
     * Kept for potential future implementation
     */
    /**
     * Rotates chamber array clockwise
     * Example: [A, B, C] → [C, A, B]
     */
    private void rotateChamberColorsClockwise () {
        String[] out = {chamberColors[0], chamberColors[1], chamberColors[2]};
        chamberColors[0] = out[2];
        chamberColors[1] = out[0];
        chamberColors[2] = out[1];
    }

    /**
     * Rotates chamber array counter-clockwise
     * Example: [A, B, C] → [B, C, A]
     */
    private void rotateChamberColorsCounterClockwise () {
        String[] out = {chamberColors[0], chamberColors[1], chamberColors[2]};
        chamberColors[0] = out[1];
        chamberColors[1] = out[2];
        chamberColors[2] = out[0];
    }

    /**
     * Finds the index of chamber with ball of desired color, based on A (0), B (1), or C (2)
     * If no chamber has the desired ball color, returns -1
     * bool returnNextBest: returns the next best chamber if desired color is not found
     */
    private int indexOfColor (String[] chamberColors, String desiredColor, boolean returnNextBest) {
        for (int i = 0; i <= 2; i++) {
            if (chamberColors[i].equals(desiredColor)){
                return i;
            }
        }
        if (returnNextBest) {
            for (int i = 0; i<= 2; i++) {
                if (!chamberColors[i].equals("NONE")) {
                    return i;
                }
            }
        }
        return -1;
    }

    // ========================================================================
    // CHAMBER ROTATION SEQUENCE
    // ========================================================================

    /**
     * Gets the next chamber in rotation sequence
     * Rotation order: 0 → 2 → 1 → 0 (counter-clockwise from above)
     * This matches the physical counter-clockwise rotation of the sorter
     *
     * @param c Current chamber (0, 1, or 2)
     * @return Next chamber in sequence
     */
    private int nextChamber(int c) {
        if (c == 1) return 0;
        if (c == 0) return 2;
        return 1;  // c == 2, return 1
    }
    /**
     * Gets the previous chamber in the nextChamber sequence
     * Since nextChamber goes: 0→2→1→0
     * prevChamber goes backwards: 0→1→2→0
     *
     * @param c Current chamber (0, 1, or 2)
     * @return Previous chamber
     */
    private int prevChamber(int c) {
        if (c == 0) return 1;  // Backwards from 0 is 1
        if (c == 1) return 2;  // Backwards from 1 is 2
        return 0;              // c == 2, backwards is 0
    }

    // ========================================================================
    // SORTER MOVEMENT INITIATION
    // ========================================================================

    /**
     * Starts a new sorter movement to target position
     * Resets all movement state variables
     */
    private void startSorterMove(int targetPosition) {
        sorterTargetPosition = targetPosition;
        sorterState = sorterStateFSM.SWITCHING_CHAMBERS;
        sorterSettling = false;
        sorterTimer.reset();
    }
    public void postTelemetry(Telemetry telemetry) {
        int rawPos = sorterEncoder.getCurrentPosition();
        int normPos = _normalize(rawPos);

        telemetry.addLine("=== Sorter ===");
        telemetry.addData("Current Detected Color: ", detectIntakeColor());
        telemetry.addData("Current Sorter State", sorterState);
        telemetry.addData("Pos", normPos);                                                  // Encoder position
        telemetry.addData("Target Position", sorterTargetPosition);
        telemetry.addData("Chamber", currentChamber + 1);                                // Current chamber (1-3 for display)
        telemetry.addData("Moving", sorterState.equals(sorterStateFSM.SWITCHING_CHAMBERS)); // Is sorter moving?

        // Chamber status: O = full, X = empty

        String ch1 = !chamberColors[0].equals("NONE") ? chamberColors[0] : "X";
        String ch2 = !chamberColors[1].equals("NONE") ? chamberColors[1] : "X";
        String ch3 = !chamberColors[2].equals("NONE") ? chamberColors[2] : "X";
        telemetry.addData("Ch1/2/3", ch1 + "/" + ch2 + "/" + ch3);

        telemetry.addData("Current Motif State: ", currentMotif);
        telemetry.addLine();
    }
}
