package org.firstinspires.ftc.teamcode.math;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class TurretLocation {
    public static double turretOffsetInches = -1.496;

    public static Pose getTurretPose(Pose robotPose) {
        return new Pose(
                robotPose.getX() - turretOffsetInches * Math.cos(robotPose.getHeading()),
                robotPose.getY() - turretOffsetInches * Math.sin(robotPose.getHeading()),
                robotPose.getHeading()
        );
    }
}