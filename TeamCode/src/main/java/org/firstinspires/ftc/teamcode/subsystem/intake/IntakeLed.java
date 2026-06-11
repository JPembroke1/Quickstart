package org.firstinspires.ftc.teamcode.subsystem.intake;

// Let Android Studio auto-import the correct Panels annotation
// import ...Configurable;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class IntakeLed {

    public static double OFF = 0.00;
    public static double DISABLED = 0.00;
    public static double BALL_1 = 0.277;
    public static double BALL_2 = 0.388;
    public static double BALL_3 = 0.500;
    public static double JAMMED = 0.611;
    public static double FULL = 0.500;

    public enum LedState {
        OFF,
        DISABLED,
        BALL_1,
        BALL_2,
        BALL_3,
        JAMMED,
        FULL
    }

    public static double getPosition(LedState state) {
        switch (state) {

            case DISABLED:
                return clip(DISABLED);

            case BALL_1:
                return clip(BALL_1);

            case BALL_2:
                return clip(BALL_2);

            case BALL_3:
                return clip(BALL_3);

            case JAMMED:
                return clip(JAMMED);

            case FULL:
                return clip(FULL);

            default:
                return clip(OFF);
        }
    }

    private static double clip(double value) {
        return Math.max(0.0, Math.min(1.0, value));
    }
}