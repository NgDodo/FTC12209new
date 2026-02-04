package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Intake {
    public enum intakeStateFSM {
        INTAKE_IN,
        INTAKE_OUT,
        INTAKE_STOP
    }
    private DcMotorEx intakeMotor;
    public intakeStateFSM intakeState;

    public double intakePower;

    public Intake(){
        this.intakeState = intakeStateFSM.INTAKE_STOP;
        this.intakeMotor = hardwareMap.get(DcMotorEx.class, "m1");
        this.intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.intakePower = 0;

    }
    public void updateIntake() {
        switch (intakeState) {
            case INTAKE_IN:
                this.intakeMotor.setPower(intakePower);
                break;
            case INTAKE_OUT:
                this.intakeMotor.setPower(-intakePower);
                break;
            case INTAKE_STOP:
                this.intakeMotor.setPower(0);
                break;
        }
    }
    public void updateIntake(Gamepad gamepad1) {
        /// Overrided for teleop control

        if (gamepad1.right_trigger - gamepad1.left_trigger > 0) {
            intakeState = intakeStateFSM.INTAKE_IN;
        }
        if (gamepad1.right_trigger - gamepad1.left_trigger < 0) {
            intakeState = intakeStateFSM.INTAKE_OUT;
        }
        if (gamepad1.right_trigger - gamepad1.left_trigger == 0) {
            intakeState = intakeStateFSM.INTAKE_STOP;
        }

        switch (intakeState) {
            case INTAKE_IN:
                this.intakeMotor.setPower(gamepad1.right_trigger);
                break;
            case INTAKE_OUT:
                this.intakeMotor.setPower(-gamepad1.right_trigger);
                break;
            case INTAKE_STOP:
                this.intakeMotor.setPower(0);
                break;
        }
    }
    public void postTelemetry() {
        /// lowk there's absolutely no telemetry for the intake to post
    }
}
