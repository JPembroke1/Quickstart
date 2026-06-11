package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystem.intake.IntakeLed;
import org.firstinspires.ftc.teamcode.subsystem.intake.IntakeLed.LedState;

@TeleOp(name = "Intake LED Tuner", group = "Tuning")
public class IntakeLedTunerOpMode extends LinearOpMode {

    private Servo led;

    private LedState selectedState = LedState.OFF;

    private boolean previousDpadLeft = false;
    private boolean previousDpadRight = false;
    private boolean previousA = false;
    private boolean previousB = false;

    @Override
    public void runOpMode() {
        led = hardwareMap.get(Servo.class, "led");

        applySelectedState();

        telemetry.addLine("Intake LED Tuner");
        telemetry.addLine("Use D-pad LEFT/RIGHT to change LED state");
        telemetry.addLine("Edit state values in Panels Configurables");
        telemetry.addLine("Press A to reapply current state");
        telemetry.addLine("Press B to set OFF");
        addTelemetry();
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean dpadLeftPressed = gamepad1.dpad_left && !previousDpadLeft;
            boolean dpadRightPressed = gamepad1.dpad_right && !previousDpadRight;
            boolean aPressed = gamepad1.a && !previousA;
            boolean bPressed = gamepad1.b && !previousB;

            if (dpadRightPressed) {
                selectedState = nextState(selectedState);
                applySelectedState();
            }

            if (dpadLeftPressed) {
                selectedState = previousState(selectedState);
                applySelectedState();
            }

            if (aPressed) {
                applySelectedState();
            }

            if (bPressed) {
                selectedState = LedState.OFF;
                applySelectedState();
            }

            addTelemetry();
            telemetry.update();

            previousDpadLeft = gamepad1.dpad_left;
            previousDpadRight = gamepad1.dpad_right;
            previousA = gamepad1.a;
            previousB = gamepad1.b;

            sleep(20);
        }
    }

    private void applySelectedState() {
        led.setPosition(IntakeLed.getPosition(selectedState));
    }

    private LedState nextState(LedState current) {
        LedState[] states = LedState.values();
        int nextIndex = (current.ordinal() + 1) % states.length;
        return states[nextIndex];
    }

    private LedState previousState(LedState current) {
        LedState[] states = LedState.values();
        int previousIndex = (current.ordinal() - 1 + states.length) % states.length;
        return states[previousIndex];
    }

    private void addTelemetry() {
        telemetry.addLine("=== Intake LED Tuner ===");
        telemetry.addData("Selected State", selectedState);
        telemetry.addData("Servo Position", "%.3f", IntakeLed.getPosition(selectedState));

        telemetry.addLine();
        telemetry.addLine("=== Tunable Values ===");
        telemetry.addData("OFF", "%.3f", IntakeLed.OFF);
        telemetry.addData("DISABLED", "%.3f", IntakeLed.DISABLED);
        telemetry.addData("BALL_1", "%.3f", IntakeLed.BALL_1);
        telemetry.addData("BALL_2", "%.3f", IntakeLed.BALL_2);
        telemetry.addData("BALL_3", "%.3f", IntakeLed.BALL_3);
        telemetry.addData("JAMMED", "%.3f", IntakeLed.JAMMED);
        telemetry.addData("FULL", "%.3f", IntakeLed.FULL);

        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("D-pad LEFT/RIGHT = change state");
        telemetry.addLine("A = reapply current state");
        telemetry.addLine("B = OFF");
    }
}