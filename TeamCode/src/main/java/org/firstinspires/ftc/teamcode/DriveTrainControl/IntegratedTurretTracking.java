package org.firstinspires.ftc.teamcode.DriveTrainControl;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

@TeleOp(name = "Integrated Turret Tracking", group = "DriveTrainControl")
public class IntegratedTurretTracking extends OpMode {

    // === Drive Train & Mechanisms ===
    DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    DcMotor m1;
    DcMotorEx m2; // Turret motor
    DcMotorEx m3, m0;
    CRServo s3;
    Servo s2;

    // === Color Sensors & RGB LEDs ===
    private RevColorSensorV3 intakeColor;
    private RevColorSensorV3 shooterColor;
    private Servo led1;
    private Servo led2;

    // === Vision & IMU ===
    private Limelight3A limelight;
    private IMU imu;

    // === Limelight Configuration ===
    private static final String LIMELIGHT_NAME = "Webcam 2";
    private static final int APRILTAG_PIPELINE = 1;

    // === Turret return to center ===
    private boolean turretReturning = false;
    private boolean turretWrapping = false;
    private int turretWrapTarget = 0;
    private static final double TURRET_RETURN_POWER = 0.3;
    private static final double TURRET_WRAP_POWER = 0.7;
    private static final int TURRET_CENTER_TOLERANCE = 50;
    private static final int TURRET_WRAP_TOLERANCE = 100;
    private static final int TURRET_WRAP_THRESHOLD = 100;

    // === Tracking Mode ===
    private enum TrackingMode {
        OFF,
        BODY_TRACK,
        TURRET_TRACK
    }
    private TrackingMode currentTrackingMode = TrackingMode.OFF;
    private boolean lastBButton = false;

    // === Turret Configuration ===
    private static final double TURRET_RANGE_DEG = 330.0;
    private static final double TICKS_PER_REV = 1393.1;
    private static final double TICKS_PER_DEGREE = TICKS_PER_REV / 360.0;
    private static final int CENTER_POSITION = 0;
    private static final int MAX_POSITION = (int)((TURRET_RANGE_DEG / 2.0) * TICKS_PER_DEGREE);
    private static final int MIN_POSITION = -MAX_POSITION;

    // === Turret PID ===
    private double turretKp = 0.014;
    private double turretKi = 0.002;
    private double turretKd = 0.0005;
    private double turretKFF = 0.4;
    private double turretIntegral = 0;
    private double turretLastError = 0;
    private long turretLastTime = 0;
    private static final double TURRET_MAX_POWER = 0.85;
    private static final double TURRET_MIN_POWER = 0.06;

    // === Body Tracking PID ===
    private double bodyKp = 0.012;
    private double bodyKi = 0.001;
    private double bodyKd = 0.003;
    private double bodyIntegral = 0.0;
    private double bodyLastError = 0.0;
    private double bodyLastTime = 0.0;
    private static final double BODY_MAX_TURN_POWER = 0.28;
    private static final double BODY_MIN_TURN_POWER = 0.02;
    private static final double BODY_TOLERANCE_DEG = 0.6;

    // === Smoothing & Prediction ===
    private static final double SMOOTHING_ALPHA = 0.65;
    private double smoothedBearing = 0.0;
    private boolean bearingInitialized = false;
    private boolean hasEverSeenTag = false;

    private static final boolean USE_PREDICTION = true;
    private double lastValidBearing = 0.0;
    private double bearingVelocity = 0.0;
    private long lastValidTime = 0;
    private static final long MAX_PREDICTION_MS = 500;

    // === Sorter constants ===
    private static final int FULL_ROT = 8192;
    private static final int SLOT = FULL_ROT / 3;
    private static final int OFFSET = (int) (FULL_ROT / 6);
    private static final int CHAMBER_0_POS = 0;
    private static final int CHAMBER_1_POS = SLOT;
    private static final int CHAMBER_2_POS = 2 * SLOT;

    private boolean[] chamberFull = new boolean[3];
    private int currentChamber = 0;
    private boolean shootingMode = false;
    private boolean lastY = false;
    private boolean lastDpadRight = false;

    // === Non-blocking sorter movement ===
    private boolean sorterMoving = false;
    private int sorterTargetPosition = 0;
    private ElapsedTime sorterTimer = new ElapsedTime();
    private ElapsedTime sorterSettleTimer = new ElapsedTime();
    private boolean sorterSettling = false;
    private static final int COARSE_TOL = 1000;
    private static final int FINE_TOL = 60;
    private static final int PERFECT_TOL = 30;
    private static final double MAX_POWER = 0.55;
    private static final double MIN_POWER = 0.08;
    private static final long SORTER_TIMEOUT_MS = 2000;
    private static final long SETTLE_MS = 100;

    private boolean lastXButton = false;
    private boolean distanceBasedRPM = false;
    private double lastValidDistance = 110.0;

    // === Timed color detection ===
    private long colorStartTime = 0;
    private boolean colorActive = false;
    private static final long DETECT_TIME_MS = 25;

    // === Empty chamber detection ===
    private long emptyStartTime = 0;
    private boolean emptyDetectionActive = false;
    private static final long EMPTY_DETECT_TIME_MS = 200;

    // === Shooter presets ===
    private final int[] rpmPresets = {2600, 3200};
    private int presetIndex = -1;
    private double targetRPM = 0;
    private boolean lastRightBumper = false;
    private boolean lastLeftBumper = false;
    private boolean lastDpadLeft = false;

    private static final double TICKS_PER_REV_FLYWHEEL = 28.0;
    private static final double RPM_TOLERANCE = 100.0;
    private boolean lastDpadDown = false;

    // === Flywheel PID ===
    private double flywheelKp = 0.0012;
    private double flywheelKi = 0.00001;
    private double flywheelKd = 0.0;
    private double flywheelKF = 0.00025;
    private double flywheelIntegral = 0;
    private double flywheelLastError = 0;
    private long flywheelLastTime = 0;

    @Override
    public void init() {
        // === DriveTrain ===
        frontLeftMotor  = hardwareMap.get(DcMotorEx.class, "fL");
        backLeftMotor   = hardwareMap.get(DcMotorEx.class, "bL");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "fR");
        backRightMotor  = hardwareMap.get(DcMotorEx.class, "bR");

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // === Mechanisms ===
        m1 = hardwareMap.get(DcMotor.class, "m1");
        m2 = hardwareMap.get(DcMotorEx.class, "m2");
        m3 = hardwareMap.get(DcMotorEx.class, "m3");
        m0 = hardwareMap.get(DcMotorEx.class, "m0");

        s2 = hardwareMap.get(Servo.class, "s2");
        s3 = hardwareMap.get(CRServo.class, "s3");
        s3.setDirection(DcMotorSimple.Direction.REVERSE);
        s2.setPosition(.68);

        for (DcMotor motor : new DcMotor[]{m1, m3, m0}) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // === Turret Setup ===
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        m0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m3.setDirection(DcMotorSimple.Direction.REVERSE);

        m1.setDirection(DcMotorSimple.Direction.REVERSE);

        // === Color Sensors ===
        intakeColor  = hardwareMap.get(RevColorSensorV3.class, "intakeColor");
        shooterColor = hardwareMap.get(RevColorSensorV3.class, "shooterColor");

        // === RGB LEDs ===
        led1 = hardwareMap.get(Servo.class, "led1");
        led2 = hardwareMap.get(Servo.class, "led2");
        led1.setPosition(1.0);
        led2.setPosition(1.0);

        // === IMU Setup ===
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(imuParams);
        imu.resetYaw();

        // === Limelight Setup ===
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();

        bodyLastTime = getTimeSeconds();
        turretLastTime = System.nanoTime();
        flywheelLastTime = System.nanoTime();

        telemetry.addLine("Integrated Turret Tracking Initialized");
        telemetry.addLine("B: Cycle Tracking Mode (OFF/Body/Turret)");
        telemetry.update();
    }

    @Override
    public void loop() {
        // === Tracking Mode Toggle (B Button) ===
        boolean bPressed = gamepad1.b;
        if (bPressed && !lastBButton) {
            switch (currentTrackingMode) {
                case OFF:
                    currentTrackingMode = TrackingMode.BODY_TRACK;
                    resetTracking();
                    hasEverSeenTag = false;
                    break;
                case BODY_TRACK:
                    currentTrackingMode = TrackingMode.TURRET_TRACK;
                    resetTracking();
                    hasEverSeenTag = false;
                    break;
                case TURRET_TRACK:
                    currentTrackingMode = TrackingMode.OFF;
                    resetTracking();
                    hasEverSeenTag = false;
                    startTurretReturn();
                    break;
            }
        }
        lastBButton = bPressed;

        // === Drive Train ===
        double y = applyDeadzone(-gamepad1.left_stick_y);
        double x = applyDeadzone(gamepad1.left_stick_x);
        double rx = applyDeadzone(gamepad1.right_stick_x);

        // === Apply tracking corrections ===
        if (currentTrackingMode == TrackingMode.BODY_TRACK) {
            double bodyCorrection = getBodyTrackingCorrection();
            rx -= bodyCorrection;
        } else if (currentTrackingMode == TrackingMode.TURRET_TRACK) {
            updateTurretTracking(rx);
        } else if (currentTrackingMode == TrackingMode.OFF) {
            updateTurretReturn();
        }

        double fl = y + x + rx, bl = y - x + rx, fr = y - x - rx, br = y + x - rx;
        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(bl),
                Math.max(Math.abs(fr), Math.abs(br)))));
        frontLeftMotor.setPower(clipLowPower(fl / max));
        backLeftMotor.setPower(clipLowPower(bl / max));
        frontRightMotor.setPower(clipLowPower(fr / max));
        backRightMotor.setPower(clipLowPower(br / max));

        // === Sorter ===
        int rawPos = backRightMotor.getCurrentPosition();
        int normPos = normalize(rawPos);

        boolean yPressed = gamepad1.y;
        if (yPressed && !lastY) {
            shootingMode = !shootingMode;
            int targetPos = getChamberPosition(currentChamber, shootingMode);
            startSorterMove(targetPos);
        }
        lastY = yPressed;

        boolean dpadRightPressed = gamepad1.dpad_right;
        if (dpadRightPressed && !lastDpadRight) {
            currentChamber = nextChamber(currentChamber);
            int targetPos = getChamberPosition(currentChamber, shootingMode);
            startSorterMove(targetPos);
        }
        lastDpadRight = dpadRightPressed;

        updateSorterMovement();

        if (!shootingMode) {
            autoIntakeColorCheck();
        } else {
            checkChamberEmpty();
        }

        String shooterColorDetected = detectShooterColor();
        updateColorLEDs(shooterColorDetected);

        // === Intake ===
        if (!shootingMode) {
            double triggerPower = gamepad1.right_trigger - gamepad1.left_trigger;
            m1.setPower(triggerPower);
        } else {
            m1.setPower(0);
        }

        // === Shooter ===
        if (gamepad1.a) {
            s2.setPosition(0);
            s3.setPower(1.0);
        } else {
            s2.setPosition(.68);
            s3.setPower(0.0);
        }

        // === Flywheel RPM ===
        boolean xPressed = gamepad1.x;
        if (xPressed && !lastXButton) {
            distanceBasedRPM = !distanceBasedRPM;
        }
        lastXButton = xPressed;

        if (gamepad1.right_bumper && !lastRightBumper) {
            if (distanceBasedRPM) {
                double distance = getAprilTagDistance();
                if (distance > 0) {
                    lastValidDistance = distance;
                    targetRPM = (distance < 110.0) ? 2500 : 3000;
                } else {
                    targetRPM = (lastValidDistance < 110.0) ? 2500 : 3000;
                }
            } else {
                presetIndex = (presetIndex + 1) % rpmPresets.length;
                targetRPM = rpmPresets[presetIndex];
            }
        } else if (gamepad1.dpad_down && !lastDpadDown) {
            targetRPM = 0;
        } else if (gamepad1.dpad_left && !lastDpadLeft) {
            targetRPM = -4000;
        }

        boolean leftBumperPressed = gamepad1.left_bumper;
        if (leftBumperPressed && !lastLeftBumper) {
            targetRPM = 1500;
        }

        lastRightBumper = gamepad1.right_bumper;
        lastLeftBumper = leftBumperPressed;
        lastDpadLeft = gamepad1.dpad_left;
        lastDpadDown = gamepad1.dpad_down;

        // === Flywheel PID Control ===
        double currentVelocity = m3.getVelocity();
        double currentRPM = (currentVelocity / TICKS_PER_REV_FLYWHEEL) * 60.0;

        long currentTime = System.nanoTime();
        double dt = (currentTime - flywheelLastTime) / 1e9;

        double error = targetRPM - currentRPM;

        flywheelIntegral += error * dt;
        flywheelIntegral = Math.max(-10000, Math.min(10000, flywheelIntegral));

        double derivative = (error - flywheelLastError) / dt;
        double feedforward = flywheelKF * targetRPM;
        double pidOutput = (flywheelKp * error) + (flywheelKi * flywheelIntegral) +
                (flywheelKd * derivative) + feedforward;

        pidOutput = Math.max(-1.0, Math.min(1.0, pidOutput));
        m3.setPower(pidOutput);

        flywheelLastError = error;
        flywheelLastTime = currentTime;

        updateRPMLED();
        updateTelemetry(normPos, shooterColorDetected);
    }

    private void updateTurretTracking(double robotRotation) {
        if (turretWrapping) {
            updateTurretWrapping();
            return;
        }

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            LLResultTypes.FiducialResult validTarget = null;
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                int id = fiducial.getFiducialId();
                if (id == 20 || id == 24) {
                    validTarget = fiducial;
                    break;
                }
            }

            if (validTarget != null) {
                hasEverSeenTag = true;
                double bearing = validTarget.getTargetXDegrees();

                if (bearingInitialized) {
                    double timeDiff = (System.currentTimeMillis() - lastValidTime) / 1000.0;
                    if (timeDiff > 0 && timeDiff < 0.5) {
                        bearingVelocity = (bearing - lastValidBearing) / timeDiff;
                        bearingVelocity = Math.max(-200, Math.min(200, bearingVelocity));
                    }
                }

                lastValidBearing = bearing;
                lastValidTime = System.currentTimeMillis();

                if (!bearingInitialized) {
                    smoothedBearing = bearing;
                    bearingInitialized = true;
                } else {
                    smoothedBearing = SMOOTHING_ALPHA * bearing + (1.0 - SMOOTHING_ALPHA) * smoothedBearing;
                }

                long currentTime = System.nanoTime();
                double dt = (currentTime - turretLastTime) / 1e9;
                if (dt <= 0) dt = 1e-6;

                double error = smoothedBearing;
                turretIntegral += error * dt;
                turretIntegral = Math.max(-1000.0, Math.min(1000.0, turretIntegral));

                double derivative = (error - turretLastError) / dt;

                double pidOutput = (turretKp * error) + (turretKi * turretIntegral) + (turretKd * derivative);
                double feedforward = -robotRotation * turretKFF;
                double totalOutput = pidOutput + feedforward;
                totalOutput = Math.max(-TURRET_MAX_POWER, Math.min(TURRET_MAX_POWER, totalOutput));

                if (Math.abs(totalOutput) > 0 && Math.abs(totalOutput) < TURRET_MIN_POWER) {
                    totalOutput = Math.copySign(TURRET_MIN_POWER, totalOutput);
                }

                if (Math.abs(error) < 0.8 && Math.abs(robotRotation) < 0.1) {
                    totalOutput = feedforward;
                    turretIntegral = 0;
                }

                int currentPosition = m2.getCurrentPosition();

                if (shouldWrapTurret(currentPosition, totalOutput)) {
                    startTurretWrapping(currentPosition);
                    return;
                }

                if (currentPosition >= MAX_POSITION && totalOutput > 0) {
                    totalOutput = 0;
                } else if (currentPosition <= MIN_POSITION && totalOutput < 0) {
                    totalOutput = 0;
                }

                m2.setPower(totalOutput);
                turretLastError = error;
                turretLastTime = currentTime;
            } else {
                handleNoTag(robotRotation);
            }
        } else {
            handleNoTag(robotRotation);
        }
    }

    private void handleNoTag(double robotRotation) {
        long timeSinceValid = System.currentTimeMillis() - lastValidTime;

        if (USE_PREDICTION && timeSinceValid < MAX_PREDICTION_MS && bearingInitialized) {
            double timeSec = timeSinceValid / 1000.0;
            double predictedBearing = lastValidBearing + (bearingVelocity * timeSec);

            double decayFactor = 1.0 - (timeSec / (MAX_PREDICTION_MS / 1000.0));
            decayFactor = Math.max(0.3, decayFactor);

            double error = predictedBearing * decayFactor;

            long currentTime = System.nanoTime();
            double dt = (currentTime - turretLastTime) / 1e9;
            if (dt <= 0) dt = 1e-6;

            double pidOutput = (turretKp * error * 0.8) + (turretKd * (error - turretLastError) / dt * 0.5);
            double feedforward = -robotRotation * turretKFF;
            double totalOutput = pidOutput + feedforward;
            totalOutput = Math.max(-TURRET_MAX_POWER, Math.min(TURRET_MAX_POWER, totalOutput));

            if (Math.abs(totalOutput) > 0 && Math.abs(totalOutput) < TURRET_MIN_POWER) {
                totalOutput = Math.copySign(TURRET_MIN_POWER, totalOutput);
            }

            int currentPosition = m2.getCurrentPosition();
            if (currentPosition >= MAX_POSITION && totalOutput > 0) {
                totalOutput = 0;
            } else if (currentPosition <= MIN_POSITION && totalOutput < 0) {
                totalOutput = 0;
            }

            m2.setPower(totalOutput);
            turretLastError = error;
            turretLastTime = currentTime;
        } else {
            double feedforward = -robotRotation * turretKFF;

            int currentPosition = m2.getCurrentPosition();
            if (currentPosition >= MAX_POSITION && feedforward > 0) {
                feedforward = 0;
            } else if (currentPosition <= MIN_POSITION && feedforward < 0) {
                feedforward = 0;
            }

            m2.setPower(feedforward);
            turretIntegral = 0;
        }
    }

    private double getBodyTrackingCorrection() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            LLResultTypes.FiducialResult validTarget = null;
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                if (fiducial.getFiducialId() == 20 || fiducial.getFiducialId() == 24) {
                    validTarget = fiducial;
                    break;
                }
            }

            if (validTarget != null) {
                hasEverSeenTag = true;
                double bearing = validTarget.getTargetXDegrees();

                if (!bearingInitialized) {
                    smoothedBearing = bearing;
                    bearingInitialized = true;
                } else {
                    smoothedBearing = SMOOTHING_ALPHA * bearing + (1.0 - SMOOTHING_ALPHA) * smoothedBearing;
                }

                double error = smoothedBearing;
                double now = getTimeSeconds();
                double dt = now - bodyLastTime;
                if (dt <= 0) dt = 1e-6;

                bodyIntegral += error * dt;
                bodyIntegral = clamp(bodyIntegral, -100.0, 100.0);

                double derivative = (error - bodyLastError) / dt;
                double pidOut = bodyKp * error + bodyKi * bodyIntegral + bodyKd * derivative;
                pidOut = clamp(pidOut, -BODY_MAX_TURN_POWER, BODY_MAX_TURN_POWER);

                if (Math.abs(error) < BODY_TOLERANCE_DEG) {
                    pidOut = 0.0;
                    bodyIntegral = 0.0;
                } else if (Math.abs(pidOut) < BODY_MIN_TURN_POWER) {
                    pidOut = Math.copySign(BODY_MIN_TURN_POWER, pidOut);
                }

                bodyLastError = error;
                bodyLastTime = now;
                return -pidOut;
            } else {
                resetTracking();
                return 0;
            }
        } else {
            resetTracking();
            return 0;
        }
    }

    private void resetTracking() {
        bearingInitialized = false;
        turretIntegral = 0.0;
        turretLastError = 0.0;
        bodyIntegral = 0.0;
        bodyLastError = 0.0;
        bearingVelocity = 0.0;
        smoothedBearing = 0.0;
        turretLastTime = System.nanoTime();
        bodyLastTime = getTimeSeconds();
    }

    private void startTurretReturn() {
        turretReturning = true;
        turretWrapping = false;
    }

    private void updateTurretReturn() {
        if (!turretReturning) return;

        int currentPosition = m2.getCurrentPosition();
        int error = CENTER_POSITION - currentPosition;

        if (Math.abs(error) <= TURRET_CENTER_TOLERANCE) {
            m2.setPower(0);
            turretReturning = false;
            return;
        }

        double power = Math.signum(error) * TURRET_RETURN_POWER;
        m2.setPower(power);
    }

    private boolean shouldWrapTurret(int currentPosition, double desiredOutput) {
        boolean nearMaxLimit = (currentPosition >= MAX_POSITION - TURRET_WRAP_THRESHOLD) && (desiredOutput > 0);
        boolean nearMinLimit = (currentPosition <= MIN_POSITION + TURRET_WRAP_THRESHOLD) && (desiredOutput < 0);
        return nearMaxLimit || nearMinLimit;
    }

    private void startTurretWrapping(int currentPosition) {
        turretWrapping = true;

        if (currentPosition > 0) {
            turretWrapTarget = MIN_POSITION + TURRET_WRAP_THRESHOLD;
        } else {
            turretWrapTarget = MAX_POSITION - TURRET_WRAP_THRESHOLD;
        }

        turretIntegral = 0;
        turretLastError = 0;
    }

    private void updateTurretWrapping() {
        if (!turretWrapping) return;

        int currentPosition = m2.getCurrentPosition();
        int error = turretWrapTarget - currentPosition;

        if (Math.abs(error) <= TURRET_WRAP_TOLERANCE) {
            turretWrapping = false;
            m2.setPower(0);
            return;
        }

        double power = Math.signum(error) * TURRET_WRAP_POWER;
        m2.setPower(power);
    }

    private double getAprilTagDistance() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                if (fiducial.getFiducialId() == 20 || fiducial.getFiducialId() == 24) {
                    double area = fiducial.getTargetArea();
                    if (area > 3.0) {
                        return 80.0;
                    } else {
                        return 120.0;
                    }
                }
            }
        }
        return -1;
    }

    private void updateRPMLED() {
        if (!shootingMode) {
            String intakeDetected = detectIntakeColor();
            if (!intakeDetected.equals("NONE")) {
                led2.setPosition(1.0);
                return;
            }
        }

        if (targetRPM == 0) {
            led2.setPosition(0);
            return;
        }
        double currentRPM = (m3.getVelocity() / TICKS_PER_REV_FLYWHEEL) * 60.0;
        double rpmError = Math.abs(targetRPM - currentRPM);
        if (rpmError <= RPM_TOLERANCE) {
            led2.setPosition(0.3);
        } else {
            led2.setPosition(0);
        }
    }

    private void updateSorterMovement() {
        if (!sorterMoving) return;

        int pos = normalize(backRightMotor.getCurrentPosition());  // or m2 depending on your motor
        int error = calculateShortestError(pos, sorterTargetPosition);

        if (sorterTimer.milliseconds() > SORTER_TIMEOUT_MS) {
            m0.setPower(0);
            sorterMoving = false;
            sorterSettling = false;
            return;
        }

        // TIGHTER TOLERANCES for faster completion
        int FAST_PERFECT_TOL = 50;   // Reduced from 30-160
        int FAST_FINE_TOL = 100;     // Reduced from 60-180

        if (Math.abs(error) <= FAST_PERFECT_TOL) {
            if (!sorterSettling) {
                sorterSettling = true;
                sorterSettleTimer.reset();
                m0.setPower(0);
            }

            // SHORTER SETTLE TIME for faster response
            long FAST_SETTLE_MS = 30;  // Reduced from 100ms
            if (sorterSettleTimer.milliseconds() >= FAST_SETTLE_MS) {
                m0.setPower(0);
                sorterMoving = false;
                sorterSettling = false;
                return;
            }

            if (Math.abs(error) > FAST_FINE_TOL) {
                sorterSettling = false;
            } else {
                return;
            }
        } else {
            sorterSettling = false;
        }

        double power;
        int absError = Math.abs(error);

        if (absError > COARSE_TOL) {
            power = MAX_POWER;
        } else {
            double ratio = (double) absError / COARSE_TOL;
            power = MIN_POWER + (MAX_POWER - MIN_POWER) * ratio;
            power = Math.max(MIN_POWER, Math.min(MAX_POWER, power));
        }

        m0.setPower(Math.signum(error) * power);
    }


    private void startSorterMove(int targetPosition) {
        sorterTargetPosition = targetPosition;
        sorterMoving = true;
        sorterSettling = false;
        sorterTimer.reset();
    }

    private void updateColorLEDs(String color) {
        if (color.equals("GREEN")) {
            led1.setPosition(0.5);
        } else if (color.equals("PURPLE")) {
            led1.setPosition(0.722);
        } else {
            led1.setPosition(0);
        }
    }

    private int getChamberPosition(int chamber, boolean shooting) {
        int basePos;
        switch(chamber) {
            case 0: basePos = CHAMBER_0_POS; break;
            case 1: basePos = CHAMBER_1_POS; break;
            case 2: basePos = CHAMBER_2_POS; break;
            default: basePos = CHAMBER_0_POS;
        }
        if (shooting) basePos = normalize(basePos + OFFSET);
        return basePos;
    }

    private void autoIntakeColorCheck() {
        if (sorterMoving) return;

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
                currentChamber = nextChamber(currentChamber);
                int target = getChamberPosition(currentChamber, false);
                startSorterMove(target);
            }
            colorActive = false;
            colorStartTime = 0;
        }
    }

    private void checkChamberEmpty() {
        if (!shootingMode || !chamberFull[currentChamber]) {
            emptyDetectionActive = false;
            emptyStartTime = 0;
            return;
        }
        String detected = detectShooterColor();
        if (!detected.equals("NONE")) {
            emptyDetectionActive = false;
            emptyStartTime = 0;
            return;
        }
        if (!emptyDetectionActive) {
            emptyDetectionActive = true;
            emptyStartTime = System.currentTimeMillis();
        }
        if (System.currentTimeMillis() - emptyStartTime >= EMPTY_DETECT_TIME_MS) {
            chamberFull[currentChamber] = false;
            emptyDetectionActive = false;
            emptyStartTime = 0;
        }
    }

    private String detectIntakeColor() {
        int r = intakeColor.red();
        int g = intakeColor.green();
        int b = intakeColor.blue();
        if (g > r && g > b && g > 80 && g < 600) return "GREEN";
        if (b > r && b > g && b > 80 && b < 600) return "PURPLE";
        return "NONE";
    }

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

    private int calculateShortestError(int current, int target) {
        int error = target - current;
        if (error > FULL_ROT / 2) {
            error -= FULL_ROT;
        } else if (error < -FULL_ROT / 2) {
            error += FULL_ROT;
        }
        return error;
    }

    private void updateTelemetry(int normPos, String shooterColorDetected) {
        double currentRPM = (m3.getVelocity() / TICKS_PER_REV_FLYWHEEL) * 60.0;
        double rpmError = Math.abs(targetRPM - currentRPM);
        boolean rpmReady = (targetRPM > 0) && (rpmError <= RPM_TOLERANCE);

        telemetry.addLine("=== Tracking Mode (B) ===");
        String modeStr = "";
        switch (currentTrackingMode) {
            case OFF:
                modeStr = "OFF";
                break;
            case BODY_TRACK:
                modeStr = "BODY TRACKING";
                break;
            case TURRET_TRACK:
                modeStr = "TURRET TRACKING";
                break;
        }
        telemetry.addData("Mode", modeStr);

        if (currentTrackingMode != TrackingMode.OFF) {
            if (hasEverSeenTag) {
                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                    LLResultTypes.FiducialResult validTarget = null;
                    for (LLResultTypes.FiducialResult fiducial : fiducials) {
                        if (fiducial.getFiducialId() == 20 || fiducial.getFiducialId() == 24) {
                            validTarget = fiducial;
                            break;
                        }
                    }

                    if (validTarget != null) {
                        telemetry.addData("Tag ID", validTarget.getFiducialId());
                        telemetry.addData("Bearing", String.format("%.2f°", smoothedBearing));

                        if (currentTrackingMode == TrackingMode.TURRET_TRACK) {
                            int turretPos = m2.getCurrentPosition();
                            double turretDeg = turretPos / TICKS_PER_DEGREE;
                            telemetry.addData("Turret Pos", String.format("%.1f°", turretDeg));
                            telemetry.addData("Error", String.format("%.2f°", turretLastError));
                            if (turretWrapping) {
                                telemetry.addData("Status", "WRAPPING");
                            }
                        } else {
                            telemetry.addData("Error", String.format("%.2f°", bodyLastError));
                        }

                        boolean aligned = (currentTrackingMode == TrackingMode.BODY_TRACK) ?
                                Math.abs(bodyLastError) < BODY_TOLERANCE_DEG :
                                Math.abs(turretLastError) < 0.8;
                        telemetry.addData("Aligned", aligned ? "YES" : "NO");
                    } else {
                        telemetry.addData("Status", "No Valid Tag (20/24)");
                    }
                } else {
                    telemetry.addData("Status", "No Limelight Data");
                }
            } else {
                telemetry.addData("Status", "Waiting for AprilTag");
            }
        }
        telemetry.addLine();

        telemetry.addLine("=== Sorter ===");
        telemetry.addData("Pos", normPos);
        telemetry.addData("Chamber", currentChamber + 1);
        telemetry.addData("Moving", sorterMoving);
        String ch1 = chamberFull[0] ? "O" : "X";
        String ch2 = chamberFull[1] ? "O" : "X";
        String ch3 = chamberFull[2] ? "O" : "X";
        telemetry.addData("Ch1/2/3", ch1 + "/" + ch2 + "/" + ch3);
        telemetry.addData("Mode", shootingMode ? "SHOOT (Y)" : "INTAKE (Y)");
        telemetry.addData("Color", shooterColorDetected);
        telemetry.addLine();

        telemetry.addLine("=== Shooter ===");
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Actual RPM", String.format("%.0f", currentRPM));
        telemetry.addData("Ready", rpmReady ? "YES" : "NO");
        telemetry.addData("Shooter", gamepad1.a ? "FIRING" : "Ready");
        telemetry.addData("Distance Mode", distanceBasedRPM ? "AUTO (X)" : "MANUAL (X)");
        telemetry.addLine();

        telemetry.addLine("=== Controls ===");
        telemetry.addLine("B: Cycle Track Mode");
        telemetry.addLine("Y: Mode | DpadRight: Chamber");
        telemetry.addLine("A: Shoot");
        telemetry.addLine("RB: RPM | LB: 1500 | DpadDown: Off");

        telemetry.update();
    }

    private double getTimeSeconds() {
        return System.nanoTime() / 1e9;
    }
//
    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private double applyDeadzone(double v) {
        return Math.abs(v) < 0.05 ? 0 : v;
    }

    private double clipLowPower(double p) {
        return Math.abs(p) < 0.04 ? 0 : p;
    }

    @Override
    public void stop() {
        if (limelight != null) limelight.stop();
        m2.setPower(0);
    }
}