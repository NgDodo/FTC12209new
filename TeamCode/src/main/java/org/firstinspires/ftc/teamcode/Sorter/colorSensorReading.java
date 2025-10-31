package org.firstinspires.ftc.teamcode.Sorter;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ButtonHelper;

@TeleOp(name="color sensor reading")
public class colorSensorReading extends LinearOpMode {
    RevColorSensorV3 intakeColorSensor, shooterColorSensor;
    ButtonHelper gamepad1Helper;
    @Override
    public void runOpMode() throws InterruptedException {
        gamepad1Helper = new ButtonHelper(gamepad1);

        intakeColorSensor = hardwareMap.get(RevColorSensorV3.class, "intakeColor");
        shooterColorSensor = hardwareMap.get(RevColorSensorV3.class, "shooterColor");
        waitForStart();

        while (opModeIsActive()) {
            gamepad1Helper.update();
            telemetry.addData("Intake Color:", "(" + intakeColorSensor.red() + ", " + intakeColorSensor.green() + ", " + intakeColorSensor.blue() + ")");
            telemetry.addData("Shooter Color:", "(" + shooterColorSensor.red() + ", " + shooterColorSensor.green() + ", " + shooterColorSensor.blue() + ")");

            String intakeColor;
            if (intakeColorSensor.green() > intakeColorSensor.blue() && intakeColorSensor.green() > intakeColorSensor.red() && intakeColorSensor.green() > 80 && intakeColorSensor.green() < 500) {
                intakeColor = "GREEN";
            }
            else if(intakeColorSensor.blue() > intakeColorSensor.green() && intakeColorSensor.blue() > intakeColorSensor.red() && intakeColorSensor.blue() > 80 && intakeColorSensor.green() < 500){
                intakeColor = "PURPLE";
            }
            else {
                intakeColor = "NONE";
            }

            String shooterColor;
            if (shooterColorSensor.green() > shooterColorSensor.blue() && shooterColorSensor.green() > shooterColorSensor.red() && shooterColorSensor.green() > 100 && shooterColorSensor.green() < 500) {
                shooterColor = "GREEN";
            }
            else if(shooterColorSensor.blue() > shooterColorSensor.green() && shooterColorSensor.blue() > shooterColorSensor.red() && shooterColorSensor.blue() > 100 && shooterColorSensor.green() < 500){
                shooterColor = "PURPLE";
            }
            else {
                shooterColor = "NONE";
            }

            telemetry.addData("Intake Color: ", intakeColor);
            telemetry.addData("Shooter Color: ", shooterColor);
            telemetry.update();
        }
    }
}
