package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Intake;

@TeleOp(name = "Intake Test", group = "Tuning")
public class IntakeToggleTeleOp extends LinearOpMode {

    private Intake intake;

    @Override
    public void runOpMode() {
        intake = new Intake(hardwareMap);

        boolean intakeEnabled = false;
        boolean previousRightBumper = false;

        telemetry.addLine("Intake Toggle TeleOp");
        telemetry.addLine("Right bumper toggles intake on/off");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {
            boolean rightBumperPressed = gamepad1.right_bumper;

            // Rising edge detect
            if (rightBumperPressed && !previousRightBumper) {
                intakeEnabled = !intakeEnabled;

                if (intakeEnabled) {
                    intake.enable();
                } else {
                    intake.disable();
                }
            }

            intake.update();

            telemetry.addLine("=== Intake ===");
            telemetry.addData("Enabled", intakeEnabled);
            telemetry.addData("State", intake.getState());
            telemetry.update();

            previousRightBumper = rightBumperPressed;

        }

        intake.disable();
    }
}
