package org.firstinspires.ftc.teamcode.subsystem.intake;


import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class IntakePowerConfig {
    public static double SEEK_POWER_FRONT = 1.0;
    public static double SEEK_POWER_REAR = 1.0;

    public static double HOLD_POWER_FRONT = 1.0;
    public static double HOLD_POWER_REAR = 0.0;

    public static double REVERSE_POWER_FRONT = -1.0;
    public static double REVERSE_POWER_REAR = -1.0;
}