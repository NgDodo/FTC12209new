package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.DriveTrainControl.SubsystemTeleop.subsystemTeleop;

public class Intake {
    public enum intakeStateFSM {
        INTAKE_IN,
        INTAKE_OUT,
        INTAKE_STOP
    }
    public DcMotorEx intakeMotor;
    public intakeStateFSM intakeState;

    public double intakePower;

    public Intake(HardwareMap hardwareMap){
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
    public void updateIntake (Gamepad gamepad1) {
        /// Overrided for teleop control
        this.intakePower = gamepad1.right_trigger - gamepad1.left_trigger;
        this.intakeMotor.setPower(intakePower);
    }
    public void postTelemetry(Telemetry telemetry) {
        telemetry.addData("Intake STATE: ", intakeState);
        telemetry.addLine();
    }
}
