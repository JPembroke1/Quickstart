package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.IntakeSubsystem;

@Autonomous(name = "Blue Side Close", group = "Blue")
public class bluesideClose extends OpMode {

    private Follower        follower;
    private IntakeSubsystem intake;

    private enum AutoState {
        PATH_1,
        PATH_2,
        DONE
    }

    private AutoState state = AutoState.PATH_1;


    // Where the robot starts on the field
    private final Pose startPose = new Pose(23, 127, Math.toRadians(145));

    // Shooting Position
    private final Pose Shoot     = new Pose(72, 72, Math.toRadians(180));

    // 2nd stack
    private final Pose Stack_2     = new Pose(38, 60, Math.toRadians(180));


    private PathChain pathToPos1;
    private PathChain pathToPos2;

    public void buildPaths() {
        // Path 1: Straight line from start to shooting
        pathToPos1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, Shoot))
                .setLinearHeadingInterpolation(startPose.getHeading(), Shoot.getHeading())
                .build();

        // Path 2: Straight shooting to stack 2
        pathToPos2 = follower.pathBuilder()
                .addPath(new BezierLine(Shoot, Stack_2))
                .setLinearHeadingInterpolation(Shoot.getHeading(), Stack_2.getHeading())
                .build();
    }


    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.createFollower(hardwareMap);
        intake   = new IntakeSubsystem(hardwareMap);

        buildPaths();
        follower.setStartingPose(startPose);

        telemetry.addLine("Initialized. Ready to start.");
        telemetry.update();
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        // Kick off the first path
        follower.followPath(pathToPos1);
        state = AutoState.PATH_1;
    }

    @Override
    public void loop() {
        follower.update();

        switch (state) {
            case PATH_1:
                // Intake runs during path 1
                intake.intake();
                if (!follower.isBusy()) {
                    // Arrived at pose1 — stop intake and start path 2
                    intake.stop();
                    follower.followPath(pathToPos2, true);
                    state = AutoState.PATH_2;
                }
                break;

            case PATH_2:
                // Intake off during path 2
                intake.stop();
                if (!follower.isBusy()) {
                    state = AutoState.DONE;
                }
                break;

            case DONE:
                intake.stop();
                break;
        }

        // Debugging telemetry on Driver Hub
        telemetry.addData("state",   state);
        telemetry.addData("x",       follower.getPose().getX());
        telemetry.addData("y",       follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        intake.stop();
    }
}