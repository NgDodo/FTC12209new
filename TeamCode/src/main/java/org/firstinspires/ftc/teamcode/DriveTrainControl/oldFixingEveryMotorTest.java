package org.firstinspires.ftc.teamcode.DriveTrainControl;

import com.arcrobotics.ftclib.controller.PDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

import MiniBot.ButtonHelper;

@TeleOp(name = "OLD DriveTrain + Stable Tracking + s5 Position", group = "DriveTrainControl")
public class oldFixingEveryMotorTest extends LinearOpMode {

    DcMotorEx frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    DcMotor m1, m2;
    DcMotorEx m3;
    CRServo s1, s3, s5;
    Servo s2;

    // Camera PD control
    private boolean cameraTrackingMode = true;

    // Pulse state (non-blocking)
    private boolean pulsing = false;
    private double pulseEndTime = 0.0;
    private double pulsePower = 0.0;

    // RPM presets for m3
    private final int[] rpmPresets = {4000, 4500, 5500};
    private int presetIndex = -1;
    private double targetRPM = 0;
    private boolean lastRightBumper = false;
    private boolean lastLeftBumper = false;
    private static final double TICKS_PER_REV = 28.0;

    private boolean lastDpadRight = false;
    private boolean lastDpadLeft = false;

    private int myExposure;
    private int minExposure;
    private int maxExposure;
    private int myGain;
    private int minGain;
    private int maxGain;

    boolean thisExpUp = false;
    boolean thisExpDn = false;
    boolean thisGainUp = false;
    boolean thisGainDn = false;

    boolean lastExpUp = false;
    boolean lastExpDn = false;
    boolean lastGainUp = false;
    boolean lastGainDn = false;

    private VisionPortal visionPortal = null;
    private AprilTagProcessor aprilTag;

    private PDController pid;
    private static final double kP = 0.0005;
    private static final double kI = 0.000005;
    private static final double kD = 0.0001;

    private double acceptableTurretError = 10.0;

    // SORTER VARIABLES
    int sorterPosition;
    enum SORTER_MODE { INTAKING, SHOOTING }
    SORTER_MODE sorterMode = SORTER_MODE.INTAKING;
    int sorterModeOffset = 0;
    int desiredPosition;

    // PID controller for sorter
    PDController sorterPID;
    private static final double kPSorter = 0.0005;
    private static final double kISorter = 0.000005;
    private static final double kDSorter = 0.0001;
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

        sorterPID = new PDController(kPSorter, kDSorter);

        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "fL");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "bL");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "fR");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "bR");

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        m1 = hardwareMap.get(DcMotor.class, "m1");
        m2 = hardwareMap.get(DcMotor.class, "m2");
        m3 = hardwareMap.get(DcMotorEx.class, "m3");

        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        s1 = hardwareMap.get(CRServo.class, "s1");
        s2 = hardwareMap.get(Servo.class, "s2");
        s3 = hardwareMap.get(CRServo.class, "s3");
        s5 = hardwareMap.get(CRServo.class, "s5");

        s3.setDirection(DcMotorSimple.Direction.REVERSE);
        s2.setPosition(1.0);

        pid = new PDController(kP, kD);
        initAprilTag();

        getCameraSetting();
        myExposure = Math.min(5, minExposure);
        myGain = maxGain;
        setManualExposure(myExposure, myGain);

        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // === Drive Train ===
            double y = applyDeadzone(gamepad1.left_stick_y);
            double x = applyDeadzone(-gamepad1.left_stick_x);
            double rx = applyDeadzone(gamepad1.right_stick_x);

            double fl = y + x + rx;
            double bl = y - x + rx;
            double fr = y - x - rx;
            double br = y + x - rx;

            double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(bl),
                    Math.max(Math.abs(fr), Math.abs(br)))));

            frontLeftMotor.setPower(clipLowPower(fl / max));
            backLeftMotor.setPower(clipLowPower(bl / max));
            frontRightMotor.setPower(clipLowPower(fr / max));
            backRightMotor.setPower(clipLowPower(br / max));

            // === m1 + m2 trigger control ===
            double triggerPower = gamepad1.right_trigger - gamepad1.left_trigger;
            m1.setPower(-triggerPower);
            m2.setPower(triggerPower);

            // === m3 preset RPM ===
            if (gamepad1.right_bumper && !lastRightBumper) {
                presetIndex = (presetIndex + 1) % rpmPresets.length;
                targetRPM = rpmPresets[presetIndex];
            } else if (gamepad1.left_bumper && !lastLeftBumper) {
                targetRPM = 0;
            }
            lastRightBumper = gamepad1.right_bumper;
            lastLeftBumper = gamepad1.left_bumper;

            double targetTicksPerSec = (targetRPM / 60.0) * TICKS_PER_REV;
            m3.setVelocity(targetTicksPerSec);

            lastDpadRight = gamepad1.dpad_right;
            lastDpadLeft = gamepad1.dpad_left;

            // === Turret (s1) Tracking ===
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            int numTags = currentDetections.size();
            if (numTags > 0 && currentDetections.get(0).id == 24) {
                AprilTagDetection det = currentDetections.get(0);
                double tagX = det.center.x;
                double frameCenter = 640.0 / 2.0;
                double errorX = tagX - frameCenter;

                telemetry.addData("Tag", "####### %d Detected ######", numTags);
                if (Math.abs(errorX) > acceptableTurretError) {
                    pid.setSetPoint(0);
                    double turretPow = pid.calculate(errorX);
                    s1.setPower(turretPow);
                    telemetry.addData("Turret Power", turretPow);
                }
            } else {
                telemetry.addData("Tag", "----------- none -----------");
            }

            telemetry.addData("Exposure", "%d  (%d - %d)", myExposure, minExposure, maxExposure);
            telemetry.addData("Gain", "%d  (%d - %d)", myGain, minGain, maxGain);
            telemetry.update();

            // === Exposure & Gain Controls ===
            thisExpUp = gamepad1.left_bumper;
            thisExpDn = gamepad1.left_trigger > 0.25;
            thisGainUp = gamepad1.right_bumper;
            thisGainDn = gamepad1.right_trigger > 0.25;

            if (thisExpUp && !lastExpUp) {
                myExposure = Range.clip(myExposure + 1, minExposure, maxExposure);
                setManualExposure(myExposure, myGain);
            } else if (thisExpDn && !lastExpDn) {
                myExposure = Range.clip(myExposure - 1, minExposure, maxExposure);
                setManualExposure(myExposure, myGain);
            }

            if (thisGainUp && !lastGainUp) {
                myGain = Range.clip(myGain + 1, minGain, maxGain);
                setManualExposure(myExposure, myGain);
            } else if (thisGainDn && !lastGainDn) {
                myGain = Range.clip(myGain - 1, minGain, maxGain);
                setManualExposure(myExposure, myGain);
            }

            lastExpUp = thisExpUp;
            lastExpDn = thisExpDn;
            lastGainUp = thisGainUp;
            lastGainDn = thisGainDn;

            sleep(20);

            // === Sorter Control ===
            double sorterError = sorterPosition - desiredPosition - sorterModeOffset;
            double sorterPower = sorterPID.calculate(sorterError);
            if (Math.abs(sorterError) > ACCEPTABLE_ERROR) {
                s5.setPower(sorterPower);
            }

            if (gamepad1Helper.isButtonJustPressed("dpad_left")) {
                desiredPosition += TOTAL_ROTATION_TICKS / 3;
            } else if (gamepad1Helper.isButtonJustPressed("dpad_right")) {
                desiredPosition -= TOTAL_ROTATION_TICKS / 3;
            }

            if (gamepad1Helper.isButtonJustPressed("y")) {
                if (sorterMode == SORTER_MODE.SHOOTING) {
                    sorterMode = SORTER_MODE.INTAKING;
                } else if (sorterMode == SORTER_MODE.INTAKING) {
                    sorterMode = SORTER_MODE.SHOOTING;
                }
            }

            if (sorterMode == SORTER_MODE.INTAKING) {
                sorterModeOffset = 0;
            } else if (sorterMode == SORTER_MODE.SHOOTING) {
                sorterModeOffset = TOTAL_ROTATION_TICKS / 2;
            }

            sorterPosition = m2.getCurrentPosition();
            gamepad1Helper.update();
        }
    }

    // === helpers ===
    private double applyDeadzone(double value) {
        return (Math.abs(value) < 0.05) ? 0.0 : value;
    }

    private double clipLowPower(double power) {
        return (Math.abs(power) < 0.04) ? 0.0 : power;
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    private boolean setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) return false;
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            while (!isStopRequested() && visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                sleep(20);
            }
        }

        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);

            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
            return true;
        }
        return false;
    }

    private void getCameraSetting() {
        if (visionPortal == null) return;
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            while (!isStopRequested() && visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                sleep(20);
            }
        }

        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            minExposure = (int) exposureControl.getMinExposure(TimeUnit.MILLISECONDS) + 1;
            maxExposure = (int) exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);

            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            minGain = gainControl.getMinGain();
            maxGain = gainControl.getMaxGain();
        }
    }
}
