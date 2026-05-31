package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem {

    private final DcMotor intakeFront;
    private final DcMotor intakeRear;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeFront = hardwareMap.get(DcMotor.class, "intakeMotorFront");
        intakeRear  = hardwareMap.get(DcMotor.class, "intakeMotorBack");
        intakeRear.setDirection(DcMotor.Direction.REVERSE);
    }

    public void intake() {
        intakeFront.setPower(1.0);
        intakeRear.setPower(1.0);
    }

    public void rearOff() {
        intakeFront.setPower(1.0);
        intakeRear.setPower(0);
    }

    public void eject() {
        intakeFront.setPower(-1.0);
        intakeRear.setPower(-1.0);
    }

    public void stop() {
        intakeFront.setPower(0);
        intakeRear.setPower(0);
    }

    public void setPower(double power) {
        intakeFront.setPower(power);
        intakeRear.setPower(power);
    }
}