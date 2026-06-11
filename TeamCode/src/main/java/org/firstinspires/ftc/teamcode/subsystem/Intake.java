package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystem.intake.IntakeLed;
import org.firstinspires.ftc.teamcode.subsystem.intake.IntakePowerConfig;
import org.firstinspires.ftc.teamcode.subsystem.intake.IntakeSensorConfig;
import org.firstinspires.ftc.teamcode.subsystem.intake.IntakeTimeoutConfig;

public class Intake {

    private final DcMotorEx frontIntake;
    private final DcMotorEx rearIntake;


    private final DistanceSensor sensor1; // sensor1 = front
    private final DistanceSensor sensor2; // sensor2 = middle
    private final DistanceSensor sensor3; // sensor3 = back

    private final Servo led;

    private int s1Count = 0;
    private int s2Count = 0;
    private int s3Count = 0;

    // Filtered occupancy state
    private boolean ball1Occupied = false; // deepest (sensor3)
    private boolean ball2Occupied = false; // middle  (sensor2)
    private boolean ball3Occupied = false; // entry   (sensor1)

    private boolean prevBall1Occupied = false;
    private boolean prevBall2Occupied = false;
    private boolean prevBall3Occupied = false;

    private boolean ball1Rising = false;
    private boolean ball2Rising = false;
    private boolean ball3Rising = false;

    private double sensor1Mm = 9999;
    private double sensor2Mm = 9999;
    private double sensor3Mm = 9999;

    public enum IndexState {
        DISABLED,
        SEEK_BALL1,
        HOLDING_1,
        SEEK_BALL2,
        HOLDING_2,
        SEEK_BALL3,
        FULL,
        JAMMED,
        MANUAL_OVERRIDE
    }

    private IndexState state = IndexState.DISABLED;
    private final ElapsedTime stateTimer = new ElapsedTime();

    private boolean enabled = false;
    private boolean manualOverride = false;

    private double manualFrontPower = 0.0;
    private double manualRearPower = 0.0;

    public Intake(HardwareMap hardwareMap) {
        frontIntake = hardwareMap.get(DcMotorEx.class, "frontIntake");
        rearIntake = hardwareMap.get(DcMotorEx.class, "rearIntake");

        sensor1 = hardwareMap.get(DistanceSensor.class, "sensor1");
        sensor2 = hardwareMap.get(DistanceSensor.class, "sensor2");
        sensor3 = hardwareMap.get(DistanceSensor.class, "sensor3");

        led = hardwareMap.get(Servo.class, "led");
    }

    public void enable() {
        enabled = true;
        manualOverride = false;

        sampleSensors();
        setState(determineStateFromBalls());
    }

    public void disable() {
        enabled = false;
        manualOverride = false;
        setState(IndexState.DISABLED);
        stopMotors();
        updateLedForState();
    }

    public void clearJam() {
        if (state == IndexState.JAMMED) {
            sampleSensors();
            setState(determineStateFromBalls());
        }
    }

    public void setManualOverride(double frontPower, double rearPower) {
        enabled = true;
        manualOverride = true;
        manualFrontPower = clip(frontPower);
        manualRearPower = clip(rearPower);
        setState(IndexState.MANUAL_OVERRIDE);
    }

    public void clearManualOverride() {
        manualOverride = false;

        if (!enabled) {
            setState(IndexState.DISABLED);
            stopMotors();
            return;
        }

        sampleSensors();
        setState(determineStateFromBalls());
    }

    public void reverse() {
        setManualOverride(
                IntakePowerConfig.REVERSE_POWER_FRONT,
                IntakePowerConfig.REVERSE_POWER_REAR
        );
    }

    public void stopManual() {
        setManualOverride(0.0, 0.0);
    }

    public void update() {
        sampleSensors();

        if (!enabled) {
            setState(IndexState.DISABLED);
            stopMotors();
            updateLedForState();
            return;
        }

        if (manualOverride) {
            setState(IndexState.MANUAL_OVERRIDE);
            setMotorPowers(manualFrontPower, manualRearPower);
            updateLedForState();
            return;
        }

        switch (state) {
            case DISABLED:
                stopMotors();
                setState(determineStateFromBalls());
                break;

            case SEEK_BALL1:
                runSeek();

                if (ball1Occupied) {
                    setState(IndexState.HOLDING_1);
                } else if (stateTimer.seconds() > IntakeTimeoutConfig.SEEK_BALL1_TIMEOUT_S) {
                    setState(IndexState.JAMMED);
                }
                break;

            case HOLDING_1:
                runHold();

                if (!ball1Occupied) {
                    setState(IndexState.SEEK_BALL1);
                } else if (ball1Occupied && ball2Occupied) {
                    setState(IndexState.HOLDING_2);
                } else if (ball3Rising || ball3Occupied) {
                    setState(IndexState.SEEK_BALL2);
                }
                break;

            case SEEK_BALL2:
                runSeek();

                if (ball1Occupied && ball2Occupied) {
                    setState(IndexState.HOLDING_2);
                } else if (!ball1Occupied) {
                    setState(IndexState.SEEK_BALL1);
                } else if (stateTimer.seconds() > IntakeTimeoutConfig.SEEK_BALL2_TIMEOUT_S) {
                    setState(IndexState.JAMMED);
                }
                break;

            case HOLDING_2:
                runHold();

                if (ball1Occupied && ball2Occupied && ball3Occupied) {
                    setState(IndexState.FULL);
                } else if (!ball1Occupied && !ball2Occupied) {
                    setState(IndexState.SEEK_BALL1);
                } else if (ball1Occupied && !ball2Occupied) {
                    setState(IndexState.HOLDING_1);
                } else if (!ball1Occupied && ball2Occupied) {
                    setState(IndexState.SEEK_BALL1);
                } else if (ball3Rising || ball3Occupied) {
                    setState(IndexState.SEEK_BALL3);
                }
                break;

            case SEEK_BALL3:
                runSeek();

                if (ball1Occupied && ball2Occupied && ball3Occupied) {
                    setState(IndexState.FULL);
                } else if (!ball1Occupied) {
                    setState(IndexState.SEEK_BALL1);
                } else if (!ball2Occupied) {
                    setState(IndexState.HOLDING_1);
                } else if (stateTimer.seconds() > IntakeTimeoutConfig.SEEK_BALL3_TIMEOUT_S) {
                    setState(IndexState.JAMMED);
                }
                break;

            case FULL:
                stopMotors();

                if (!(ball1Occupied && ball2Occupied && ball3Occupied)) {
                    setState(determineStateFromBalls());
                }
                break;

            case JAMMED:
                stopMotors();
                break;

            case MANUAL_OVERRIDE:
                setMotorPowers(manualFrontPower, manualRearPower);
                break;
        }

        updateLedForState();
    }

    private void sampleSensors() {
        prevBall1Occupied = ball1Occupied;
        prevBall2Occupied = ball2Occupied;
        prevBall3Occupied = ball3Occupied;

        sensor1Mm = sensor1.getDistance(DistanceUnit.MM); // entry
        sensor2Mm = sensor2.getDistance(DistanceUnit.MM); // middle
        sensor3Mm = sensor3.getDistance(DistanceUnit.MM); // deepest

        ball3Occupied = updateFilter(sensor1Mm < IntakeSensorConfig.SENSOR_THRESHOLD_MM, 1);
        ball2Occupied = updateFilter(sensor2Mm < IntakeSensorConfig.SENSOR_THRESHOLD_MM, 2);
        ball1Occupied = updateFilter(sensor3Mm < IntakeSensorConfig.SENSOR_THRESHOLD_MM, 3);

        ball1Rising = ball1Occupied && !prevBall1Occupied;
        ball2Rising = ball2Occupied && !prevBall2Occupied;
        ball3Rising = ball3Occupied && !prevBall3Occupied;
    }

    private boolean updateFilter(boolean detected, int sensorIndex) {
        int filterCount = Math.max(1, IntakeSensorConfig.FILTER_COUNT);

        switch (sensorIndex) {
            case 1:
                if (detected) {
                    s1Count = Math.min(filterCount, s1Count + 1);
                } else {
                    s1Count = Math.max(0, s1Count - 1);
                }
                return s1Count >= filterCount;

            case 2:
                if (detected) {
                    s2Count = Math.min(filterCount, s2Count + 1);
                } else {
                    s2Count = Math.max(0, s2Count - 1);
                }
                return s2Count >= filterCount;

            case 3:
                if (detected) {
                    s3Count = Math.min(filterCount, s3Count + 1);
                } else {
                    s3Count = Math.max(0, s3Count - 1);
                }
                return s3Count >= filterCount;

            default:
                return false;
        }
    }

    private IndexState determineStateFromBalls() {
        if (ball1Occupied && ball2Occupied && ball3Occupied) {
            return IndexState.FULL;
        }

        if (ball1Occupied && ball2Occupied) {
            return IndexState.HOLDING_2;
        }

        if (ball1Occupied) {
            return IndexState.HOLDING_1;
        }

        return IndexState.SEEK_BALL1;
    }

    private void setState(IndexState newState) {
        if (state != newState) {
            state = newState;
            stateTimer.reset();
        }
    }

    private void runSeek() {
        setMotorPowers(
                IntakePowerConfig.SEEK_POWER_FRONT,
                IntakePowerConfig.SEEK_POWER_REAR
        );
    }

    private void runHold() {
        setMotorPowers(
                IntakePowerConfig.HOLD_POWER_FRONT,
                IntakePowerConfig.HOLD_POWER_REAR
        );
    }

    private void stopMotors() {
        setMotorPowers(0.0, 0.0);
    }

    private void setMotorPowers(double front, double rear) {
        frontIntake.setPower(clip(front));
        rearIntake.setPower(clip(rear));
    }

    private double clip(double value) {
        return Math.max(-1.0, Math.min(1.0, value));
    }

    private void updateLedForState() {
        switch (state) {
            case DISABLED:
                led.setPosition(IntakeLed.OFF);
                break;

            case JAMMED:
                led.setPosition(IntakeLed.JAMMED);
                break;

            case FULL:
                led.setPosition(IntakeLed.FULL);
                break;

            default:
                updateLedForCount(getBallCount());
                break;
        }
    }

    private void updateLedForCount(int ballCount) {
        switch (ballCount) {
            case 1:
                led.setPosition(IntakeLed.BALL_1);
                break;
            case 2:
                led.setPosition(IntakeLed.BALL_2);
                break;
            case 3:
                led.setPosition(IntakeLed.BALL_3);
                break;
            default:
                led.setPosition(IntakeLed.OFF);
                break;
        }
    }

    public IndexState getState() {
        return state;
    }

    public int getBallCount() {
        int count = 0;
        if (ball1Occupied) count++;
        if (ball2Occupied) count++;
        if (ball3Occupied) count++;
        return count;
    }

    public boolean isBall1Occupied() {
        return ball1Occupied;
    }

    public boolean isBall2Occupied() {
        return ball2Occupied;
    }

    public boolean isBall3Occupied() {
        return ball3Occupied;
    }

    public boolean isJammed() {
        return state == IndexState.JAMMED;
    }

    public boolean isManualOverride() {
        return manualOverride;
    }

    public double getStateTimeSeconds() {
        return stateTimer.seconds();
    }

    public double getSensor1Mm() {
        return sensor1Mm;
    }

    public double getSensor2Mm() {
        return sensor2Mm;
    }

    public double getSensor3Mm() {
        return sensor3Mm;
    }
}
