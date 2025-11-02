package org.firstinspires.ftc.teamcode.TurretTest;

import com.arcrobotics.ftclib.controller.PDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name="Optimize turret (Stable)", group="Concept")
public class turretDetectionOptimization extends LinearOpMode {

    private VisionPortal visionPortal = null;
    private AprilTagProcessor aprilTag;
    private int myExposure, minExposure, maxExposure;
    private int myGain, minGain, maxGain;

    private boolean thisExpUp, thisExpDn, thisGainUp, thisGainDn;
    private boolean lastExpUp, lastExpDn, lastGainUp, lastGainDn;

    private CRServo s1;
    private PDController pid;

    private static final double kP = 0.0005;
    private static final double kD = 0.0001;
    private static final double acceptableTurretError = 10.0;

    @Override
    public void runOpMode() {
        s1 = hardwareMap.get(CRServo.class, "s1");
        pid = new PDController(kP, kD);

        initAprilTag();
        getCameraSetting();
        sleep(1000); // Allow camera to warm up

        // Better default exposure/gain
        myExposure = Range.clip(15, minExposure, maxExposure);
        myGain = (minGain + maxGain) / 2;
        setManualExposure(myExposure, myGain);

        telemetry.addData("Camera", "Ready — press START");
        telemetry.update();
        waitForStart();

        try {
            while (opModeIsActive()) {
                List<AprilTagDetection> detections = aprilTag.getDetections();

                if (!detections.isEmpty()) {
                    AprilTagDetection det = detections.get(0);
                    double tagX = det.center.x;
                    double frameCenter = 640.0 / 2.0;
                    double errorX = tagX - frameCenter;

                    telemetry.addData("Tag", "Detected ID %d", det.id);
                    telemetry.addData("X Error", "%.1f", errorX);

                    if (Math.abs(errorX) > acceptableTurretError) {
                        pid.setSetPoint(0);
                        double turretPow = pid.calculate(errorX);
                        s1.setPower(turretPow);
                        telemetry.addData("Turret Power", turretPow);
                    } else {
                        s1.setPower(0);
                    }
                } else {
                    telemetry.addData("Tag", "None detected");
                    s1.setPower(0);
                }

                telemetry.addData("Exposure", "%d  (%d - %d)", myExposure, minExposure, maxExposure);
                telemetry.addData("Gain", "%d  (%d - %d)", myGain, minGain, maxGain);
                telemetry.update();

                sleep(20);
            }
        } finally {
            // Prevent disconnect on stop
            if (visionPortal != null) {
                visionPortal.close();
            }
        }
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

        // Wait until camera is ready
        while (!isStopRequested() && visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting for stream...");
            telemetry.update();
            sleep(20);
        }

        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            return true;
        }
        return false;
    }

    private void getCameraSetting() {
        if (visionPortal == null) return;
        while (!isStopRequested() && visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting to open...");
            telemetry.update();
            sleep(20);
        }

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        minExposure = (int) exposureControl.getMinExposure(TimeUnit.MILLISECONDS) + 1;
        maxExposure = (int) exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        minGain = gainControl.getMinGain();
        maxGain = gainControl.getMaxGain();
    }
}
