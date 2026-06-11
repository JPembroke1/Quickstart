package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Shoooter Test")
public class test extends LinearOpMode {

    private DcMotor motor1;
    private DcMotor motor2;

    @Override
    public void runOpMode() {

        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");

        // Reverse one motor if needed
        motor2.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            double power = 1.0;

            motor1.setPower(power);
            motor2.setPower(power);

            telemetry.addData("Power", power);
            telemetry.update();
        }

        motor1.setPower(0);
        motor2.setPower(0);
    }
}