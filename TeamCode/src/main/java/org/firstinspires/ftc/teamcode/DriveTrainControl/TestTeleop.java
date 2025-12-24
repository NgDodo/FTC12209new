    package org.firstinspires.ftc.teamcode.DriveTrainControl;

    import com.qualcomm.hardware.limelightvision.Limelight3A;
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

    import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
    import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
    import org.firstinspires.ftc.vision.VisionPortal;
    import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
    import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

    import java.util.List;

    @TeleOp(name = "TestTeleop", group = "DriveTrainControl")
    public class TestTeleop extends OpMode {

        // === Drive Train & Mechanisms ===
        DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
        DcMotor m1, m2;
        DcMotorEx m3, m0;
        CRServo s3;
        Servo s2;

        // === Color Sensors & RGB LEDs ===
        private RevColorSensorV3 intakeColor;
        private RevColorSensorV3 shooterColor;
        private Servo led1; // Ball color indicator
        private Servo led2; // RPM ready indicator

        // === Vision & IMU ===
        private VisionPortal visionPortal;
        private AprilTagProcessor aprilTag;
        private Limelight3A limelight;
        private IMU imu;

        // === AprilTag Body Tracking ===
        private boolean aprilTagTrackingEnabled = false;
        private boolean lastBButton = false;

        private double tagKp = 0.012;
        private double tagKi = 0.001;
        private double tagKd = 0.003;
        private double tagIntegral = 0.0;
        private double tagLastError = 0.0;
        private double tagLastTime = 0.0;
        private final double TAG_MAX_TURN_POWER = 0.28;
        private final double TAG_MIN_TURN_POWER = 0.02;
        private final double TAG_TOLERANCE_DEG = 0.6;
        private final double TAG_SMOOTHING_ALPHA = 0.2;
        private double smoothedTagBearing = 0.0;
        private boolean tagBearingInitialized = false;

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

        // === Non-blocking sorter movement with jam detection ===
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
        private double lastValidDistance = 110.0; // Store last valid AprilTag distance

        // === Jam detection ===
        private int lastSorterPosition = 0;
        private long lastSorterMoveTime = 0;
        private static final long JAM_CHECK_INTERVAL_MS = 000;
        private static final int MIN_MOVEMENT_TICKS = 0;
        private boolean sorterJammed = false;

        // === Timed color detection ===
        private long colorStartTime = 0;
        private boolean colorActive = false;
        private static final long DETECT_TIME_MS = 1;

        // === Empty chamber detection ===
        private long emptyStartTime = 0;
        private boolean emptyDetectionActive = false;
        private static final long EMPTY_DETECT_TIME_MS = 12;

        // === Shooter presets ===
        private final int[] rpmPresets = {2600, 3200};
        private int presetIndex = -1;
        private double targetRPM = 0;
        private boolean lastRightBumper = false;
        private boolean lastLeftBumper = false;
        private boolean lastDpadLeft = false;

        private static final double TICKS_PER_REV = 28.0;
        private static final double RPM_TOLERANCE = 100.0;
        private boolean lastDpadDown = false;

        // === PID Coefficients for Flywheel ===
        private double kP = 0.0012;
        private double kI = 0.00001;
        private double kD = 0.0;
        private double kF = 0.00025; // Feedforward

        // === PID variables for Flywheel ===
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
            m2 = hardwareMap.get(DcMotor.class, "m2");
            m3 = hardwareMap.get(DcMotorEx.class, "m3");
            m0 = hardwareMap.get(DcMotorEx.class, "m0");

            s2 = hardwareMap.get(Servo.class, "s2");
            s3 = hardwareMap.get(CRServo.class, "s3");
            s3.setDirection(DcMotorSimple.Direction.REVERSE);
            s2.setPosition(.68);

            for (DcMotor motor : new DcMotor[]{m1, m2, m3, m0}) {
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            m0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // === Flywheel motor setup ===
            m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m3.setDirection(DcMotorSimple.Direction.REVERSE);

            // === Intake motor setup ===
            m1.setDirection(DcMotorSimple.Direction.REVERSE);

            // === Color Sensors ===
            intakeColor  = hardwareMap.get(RevColorSensorV3.class, "intakeColor");
            shooterColor = hardwareMap.get(RevColorSensorV3.class, "shooterColor");

            // === RGB LEDs ===
            led1 = hardwareMap.get(Servo.class, "led1");
            led2 = hardwareMap.get(Servo.class, "led2");
            led1.setPosition(1.0); // Start with white
            led2.setPosition(1.0); // Start with white

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
            limelight = hardwareMap.get(Limelight3A.class, "Webcam 2");
            limelight.start();

            // === AprilTag Vision setup ===
            aprilTag = new AprilTagProcessor.Builder().build();
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();

            tagLastTime = getTimeSeconds();
            flywheelLastTime = System.nanoTime();

            telemetry.addLine("LM2 TeleOp Initialized");
            telemetry.addLine("B Button: AprilTag Body Tracking");
            telemetry.addLine("Dpad Up: Clear Sorter Jam");
            telemetry.update();
        }

        @Override
        public void loop() {
            // === AprilTag Body Tracking Toggle (B Button) ===
            boolean bPressed = gamepad1.b;
            if (bPressed && !lastBButton) {
                aprilTagTrackingEnabled = !aprilTagTrackingEnabled;
                if (!aprilTagTrackingEnabled) {
                    resetAprilTagTracking();
                }
            }
            lastBButton = bPressed;


            // === Drive Train ===
            double y = applyDeadzone(-gamepad1.left_stick_y);
            double x = applyDeadzone(gamepad1.left_stick_x);
            double rx = applyDeadzone(gamepad1.right_stick_x);

            // === FIXED: Apply AprilTag correction to rotation ===
            if (aprilTagTrackingEnabled) {
                double tagCorrection = getAprilTagBodyCorrection();
                rx -= tagCorrection;
            }

            double fl = y + x + rx, bl = y - x + rx, fr = y - x - rx, br = y + x - rx;
            double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(bl),
                    Math.max(Math.abs(fr), Math.abs(br)))));
            frontLeftMotor.setPower(clipLowPower(fl / max));
            backLeftMotor.setPower(clipLowPower(bl / max));
            frontRightMotor.setPower(clipLowPower(fr / max));
            backRightMotor.setPower(clipLowPower(br / max));

            // === Sorter ===
            updateSorterMovement();
            int rawPos = backRightMotor.getCurrentPosition();
            int normPos = normalize(rawPos);

            boolean yPressed = gamepad1.y;
            if (yPressed && !lastY) {
                shootingMode = !shootingMode;
                int targetPos = getChamberPosition(currentChamber, shootingMode);
                startSorterMove(targetPos);
            }
            lastY = yPressed;

            // === Chamber switching (works in both modes) ===
            boolean dpadRightPressed = gamepad1.dpad_right;
            if (dpadRightPressed && !lastDpadRight) {
                currentChamber = nextChamber(currentChamber);
                int targetPos = getChamberPosition(currentChamber, shootingMode);
                startSorterMove(targetPos);
            }
            lastDpadRight = dpadRightPressed;

            // === Manual jam clear ===
            if (gamepad1.dpad_up && sorterJammed) {
                sorterJammed = false;
            }

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
            // Toggle distance-based RPM mode with X button
            boolean xPressed = gamepad1.x;
            if (xPressed && !lastXButton) {
                distanceBasedRPM = !distanceBasedRPM;
            }
            lastXButton = xPressed;

            if (gamepad1.right_bumper && !lastRightBumper) {
                if (distanceBasedRPM) {
                    // Distance-based mode: check AprilTag distance
                    List<AprilTagDetection> detections = aprilTag.getDetections();
                    if (!detections.isEmpty() && detections.get(0).ftcPose != null) {
                        double distance = detections.get(0).ftcPose.range;
                        lastValidDistance = distance; // Update last valid distance
                        targetRPM = (distance < 110.0) ? 2500 : 3000;
                    } else {
                        // Use last valid distance if no tag visible
                        targetRPM = (lastValidDistance < 110.0) ? 2500 : 3000;
                    }
                } else {
                    // Manual cycle mode
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

            // === PID Control for Flywheel ===
            double currentVelocity = m3.getVelocity();
            double currentRPM = (currentVelocity / TICKS_PER_REV) * 60.0;

            long currentTime = System.nanoTime();
            double dt = (currentTime - flywheelLastTime) / 1e9; // Convert to seconds

            double error = targetRPM - currentRPM;

            flywheelIntegral += error * dt;
            // Anti-windup
            flywheelIntegral = Math.max(-10000, Math.min(10000, flywheelIntegral));

            double derivative = (error - flywheelLastError) / dt;

            // Feedforward term (estimated power needed for target RPM)
            double feedforward = kF * targetRPM;

            // PID output
            double pidOutput = (kP * error) + (kI * flywheelIntegral) + (kD * derivative) + feedforward;

            // Clamp output to [-1, 1]
            pidOutput = Math.max(-1.0, Math.min(1.0, pidOutput));

            // Apply power to motor
            m3.setPower(pidOutput);

            flywheelLastError = error;
            flywheelLastTime = currentTime;

            updateRPMLED();
            updateTelemetry(normPos, shooterColorDetected);
        }


        private double getAprilTagBodyCorrection() {
            List<AprilTagDetection> detections = aprilTag.getDetections();

            if (!detections.isEmpty() && detections.get(0).ftcPose != null) {
                AprilTagDetection det = detections.get(0);

                // Only track AprilTags with ID 20 or 24
                if (det.id != 20 && det.id != 24) {
                    resetAprilTagTracking();
                    return 0;
                }

                double bearing = det.ftcPose.bearing;

                if (!tagBearingInitialized) {
                    smoothedTagBearing = bearing;
                    tagBearingInitialized = true;
                } else {
                    smoothedTagBearing = TAG_SMOOTHING_ALPHA * bearing +
                            (1.0 - TAG_SMOOTHING_ALPHA) * smoothedTagBearing;
                }

                double imuHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                double desiredHeading = wrapAngle(imuHeading + smoothedTagBearing);
                double error = wrapAngle(desiredHeading - imuHeading);

                double now = getTimeSeconds();
                double dt = now - tagLastTime;
                if (dt <= 0) dt = 1e-6;

                tagIntegral += error * dt;
                tagIntegral = clamp(tagIntegral, -100.0, 100.0);

                double derivative = (error - tagLastError) / dt;
                double pidOut = tagKp * error + tagKi * tagIntegral + tagKd * derivative;
                pidOut = clamp(pidOut, -TAG_MAX_TURN_POWER, TAG_MAX_TURN_POWER);

                if (Math.abs(error) < TAG_TOLERANCE_DEG) {
                    pidOut = 0.0;
                    tagIntegral = 0.0;
                } else if (Math.abs(pidOut) < TAG_MIN_TURN_POWER) {
                    pidOut = Math.copySign(TAG_MIN_TURN_POWER, pidOut);
                }

                tagLastError = error;
                tagLastTime = now;
                return pidOut;
            } else {
                resetAprilTagTracking();
                return 0;
            }
        }

        private void resetAprilTagTracking() {
            tagBearingInitialized = false;
            tagIntegral = 0.0;
            tagLastError = 0.0;
            tagLastTime = getTimeSeconds();
        }

        private void updateRPMLED() {
            // In intake mode, show white if ball detected
            if (!shootingMode) {
                String intakeDetected = detectIntakeColor();
                if (!intakeDetected.equals("NONE")) {
                    led2.setPosition(1.0); // White when ball detected in intake
                    return;
                }
            }

            // In shooting mode, show RPM status
            if (targetRPM == 0) {
                led2.setPosition(0); // Green when not spinning
                return;
            }
            double currentRPM = (m3.getVelocity() / TICKS_PER_REV) * 60.0;
            double rpmError = Math.abs(targetRPM - currentRPM);
            if (rpmError <= RPM_TOLERANCE) {
                led2.setPosition(0.3); // Red when RPM is ready
            } else {
                led2.setPosition(0); // Green when spinning up
            }
        }

        private void updateSorterMovement() {
            if (!sorterMoving) return;

            int pos = normalize(backRightMotor.getCurrentPosition());
            int error = calculateShortestError(pos, sorterTargetPosition);

            long currentTime = System.currentTimeMillis();
            if (currentTime - lastSorterMoveTime >= JAM_CHECK_INTERVAL_MS) {
                int positionChange = Math.abs(pos - lastSorterPosition);
                if (positionChange < MIN_MOVEMENT_TICKS && Math.abs(error) > PERFECT_TOL) {
                    sorterJammed = true;
                    m0.setPower(0);
                    sorterMoving = false;
                    sorterSettling = false;
                    return;
                }
                lastSorterPosition = pos;
                lastSorterMoveTime = currentTime;
            }

            if (sorterTimer.milliseconds() > SORTER_TIMEOUT_MS) {
                m0.setPower(0);
                sorterMoving = false;
                sorterSettling = false;
                return;
            }

            if (Math.abs(error) <= PERFECT_TOL) {
                if (!sorterSettling) {
                    sorterSettling = true;
                    sorterSettleTimer.reset();
                    m0.setPower(0);
                }

                if (sorterSettleTimer.milliseconds() >= SETTLE_MS) {
                    m0.setPower(0);
                    sorterMoving = false;
                    sorterSettling = false;
                    return;
                }

                if (Math.abs(error) > FINE_TOL) {
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
            if (sorterJammed) return;
            sorterTargetPosition = targetPosition;
            sorterMoving = true;
            sorterSettling = false;
            sorterTimer.reset();
            lastSorterPosition = normalize(backRightMotor.getCurrentPosition());
            lastSorterMoveTime = System.currentTimeMillis();
        }

        private void updateColorLEDs(String color) {
            if (color.equals("GREEN")) {
                led1.setPosition(0.5); // Green
            } else if (color.equals("PURPLE")) {
                led1.setPosition(0.722); // Purple
            } else {
                led1.setPosition(0); // Default to green when no ball
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

                    if (!allChambersFull()) {
                        // Find next empty chamber
                        int nextEmpty = findNextEmptyChamber(currentChamber);
                        if (nextEmpty != -1) {
                            currentChamber = nextEmpty;
                            int target = getChamberPosition(currentChamber, false);
                            startSorterMove(target);
                        }
                    }
                }
                colorActive = false;
                colorStartTime = 0;
            }
        }

        private int findNextEmptyChamber(int startChamber) {
            // Try next chamber first
            int next = nextChamber(startChamber);
            if (!chamberFull[next]) return next;

            // Try the one after that
            next = nextChamber(next);
            if (!chamberFull[next]) return next;

            // All full
            return -1;
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
            double currentRPM = (m3.getVelocity() / TICKS_PER_REV) * 60.0;
            double rpmError = Math.abs(targetRPM - currentRPM);
            boolean rpmReady = (targetRPM > 0) && (rpmError <= RPM_TOLERANCE);

            List<AprilTagDetection> detections = aprilTag.getDetections();
            boolean hasAprilTag = !detections.isEmpty() && detections.get(0).ftcPose != null;

            // === AprilTag Body Tracking Status ===
            telemetry.addLine("=== AprilTag Body Tracking ===");
            telemetry.addData("Active", aprilTagTrackingEnabled ? "YES (B)" : "NO");

            if (aprilTagTrackingEnabled) {
                if (hasAprilTag) {
                    AprilTagDetection det = detections.get(0);
                    double imuHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                    telemetry.addData("Tag ID", det.id);
                    telemetry.addData("Bearing", String.format("%.2f°", smoothedTagBearing));
                    telemetry.addData("IMU Heading", String.format("%.2f°", imuHeading));
                    telemetry.addData("Error", String.format("%.2f°", tagLastError));
                    telemetry.addData("Aligned", Math.abs(tagLastError) < TAG_TOLERANCE_DEG ? "YES ✓" : "NO");
                } else {
                    telemetry.addData("Status", "No AprilTag Visible");
                }
            }
            telemetry.addLine();

            // === Sorter Status ===
            telemetry.addLine("=== Sorter ===");
            telemetry.addData("Pos", normPos);
            telemetry.addData("Chamber", currentChamber + 1);
            telemetry.addData("Jammed", sorterJammed ? "⚠️ DpadUp" : "NO");
            telemetry.addData("Moving", sorterMoving);
            telemetry.addData("Ch1/2/3", String.format("%s/%s/%s",
                    chamberFull[0] ? "●" : "○",
                    chamberFull[1] ? "●" : "○",
                    chamberFull[2] ? "●" : "○"));
            telemetry.addData("Mode", shootingMode ? "SHOOT (Y)" : "INTAKE (Y)");
            telemetry.addData("Color", shooterColorDetected);
            telemetry.addLine();

            // === Shooter Status ===
            telemetry.addLine("=== Shooter ===");
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Actual RPM", String.format("%.0f", currentRPM));
            telemetry.addData("Ready", rpmReady ? "YES ✓" : "NO");
            telemetry.addData("Shooter", gamepad1.a ? "FIRING" : "Ready");
            telemetry.addLine();

            // === Controls Summary ===
            telemetry.addLine("=== Controls ===");
            telemetry.addLine("B: Tag Track");
            telemetry.addLine("Y: Mode | DpadRight: Switch Chamber");
            telemetry.addLine("A: Shoot | DpadUp: Clear Jam");
            telemetry.addLine("RB: RPM+ | LB: 1500 RPM | DpadDown: RPM Off");
            telemetry.addLine("DpadLeft: Reverse");

            telemetry.update();
        }

        private double getTimeSeconds() {
            return System.nanoTime() / 1e9;
        }

        private static double clamp(double v, double lo, double hi) {
            return Math.max(lo, Math.min(hi, v));
        }

        private static double wrapAngle(double angle) {
            angle %= 360.0;
            if (angle <= -180.0) angle += 360.0;
            if (angle > 180.0) angle -= 360.0;
            return angle;
        }

        private double applyDeadzone(double v) {
            return Math.abs(v) < 0.05 ? 0 : v;
        }

        private double clipLowPower(double p) {
            return Math.abs(p) < 0.04 ? 0 : p;
        }

        @Override
        public void stop() {
            if (visionPortal != null) visionPortal.close();
            if (limelight != null) limelight.stop();
        }
    }