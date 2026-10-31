package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem_Motor;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystemFF;
import org.firstinspires.ftc.teamcode.subsystems.StopperSubsystem;
import org.firstinspires.ftc.teamcode.utils.OpModeStorage;

@Autonomous(name = "Blue Upper", group = "Autonomous")
public class BlueUpperAuto extends OpMode {

    private Follower follower;
    private IntakeSubsystem_Motor intake;
    private ShooterSubsystemFF shooter;
    private StopperSubsystem stopper;

    private final ElapsedTime timer = new ElapsedTime();
    private int step = -2;

    private static final double SHOOT_RPM = 3200;
    private static final double FEED_DELAY = 0.3;
    private static final double GATE_TIME = 1.5;
    private static final double COLLECT_WAIT = 2.0;

    private final PoseFactory poses = PoseFactory.degrees();

    private final Pose startPose = poses.of(81.5, 133.5, 270);
    private final Pose point1 = poses.of(81.5, 125.5, 0);
    private final Pose point2 = poses.of(113.5, 133.5, 3.9173);
    private final Pose control2 = poses.of(87.853, 131.7438, 0);
    private final Pose point3 = poses.of(133.5, 133.5, 0);

    private final Pose shootPose = poses.of(81.5, 129.5, 270);

    private final Pose point5 = poses.of(123.5, 93.5, -17.0291);
    private final Pose control5 = poses.of(102.338, 99.9816, 0);
    private final Pose point6 = poses.of(129.5, 93.5, 0);

    private final Pose exitPose = poses.of(127.6048, 47.8152, -90.3614);
    private final Pose exitControl = poses.of(128.0573, 119.5566, 0);

    private Path path1, path2, path3, path4;
    private Path path5, path6, path7, path9;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);

        intake = new IntakeSubsystem_Motor(hardwareMap);
        shooter = new ShooterSubsystemFF(hardwareMap);
        stopper = new StopperSubsystem(hardwareMap);

        shooter.setTargetRpm(SHOOT_RPM);
        intake.stopIntake();
        stopper.off();

        path1 = Paths.line(startPose, point1)
                .linear(startPose, point1);

        path2 = Paths.curve(point1, control2, point2).tangent();

        path3 = Paths.line(point2, point3).tangent();

        path4 = Paths.line(point3, shootPose)
                .linear(point3, shootPose);

        path5 = Paths.curve(shootPose, control5, point5).tangent();

        path6 = Paths.line(point5, point6).tangent();

        // Return directly at the shooting heading.
        path7 = Paths.line(point6, shootPose)
                .linear(point6, shootPose);

        path9 = Paths.curve(shootPose, exitControl, exitPose).tangent();

        OpModeStorage.autonomousIsBlue = false;
        OpModeStorage.autonomousEndPose = null;
    }

    @Override
    public void start() {
        step = -2;
        shooter.update(true);
        follower.hold(startPose);
    }

    @Override
    public void loop() {
        follower.update();
        shooter.update(true);

        switch (step) {

            // Wait for shooter speed before firing the preload.
            case -2:
                if (shooter.isReady()) {
                    stopper.on();
                    timer.reset();
                    step = -1;
                }
                break;

            // Fire preload, then begin Path 1.
            case -1:
                if (timer.seconds() >= GATE_TIME) {
                    stopper.off();
                    intake.stopIntake();
                    follower.follow(path1);
                    step = 0;
                } else if (timer.seconds() >= FEED_DELAY) {
                    intake.startIntake();
                }
                break;

            // Finish Path 1; start intake while stationary.
            case 0:
                if (!follower.isBusy()) {
                    follower.hold(path1.endPose());
                    intake.startIntake();
                    timer.reset();
                    step = 1;
                }
                break;

            // Wait 2 s, then begin first collection run.
            case 1:
                if (timer.seconds() >= COLLECT_WAIT) {
                    follower.follow(path2);
                    step = 2;
                }
                break;

            // Continue collecting along Path 3.
            case 2:
                if (!follower.isBusy()) {
                    follower.follow(path3);
                    step = 3;
                }
                break;

            // Stop intake and return along Path 4.
            case 3:
                if (!follower.isBusy()) {
                    intake.stopIntake();
                    follower.follow(path4);
                    step = 4;
                }
                break;

            // Arrive for the second or third shot.
            case 4:
            case 9:
                if (!follower.isBusy() && shooter.isReady()) {
                    follower.hold(shootPose);
                    intake.stopIntake();
                    stopper.on();
                    timer.reset();
                    step++;
                }
                break;

            // Gate open for 1.5 s; intake starts after 0.3 s.
            case 5:
            case 10:
                if (timer.seconds() >= GATE_TIME) {
                    stopper.off();
                    intake.stopIntake();

                    if (step == 5) {
                        intake.startIntake();
                        timer.reset();
                        step = 6;
                    } else {
                        follower.follow(path9);
                        step = 11;
                    }
                } else if (timer.seconds() >= FEED_DELAY) {
                    intake.startIntake();
                }
                break;

            // Wait stationary for 2 s before Path 5.
            case 6:
                if (timer.seconds() >= COLLECT_WAIT) {
                    follower.follow(path5);
                    step = 7;
                }
                break;

            // Continue collecting along Path 6.
            case 7:
                if (!follower.isBusy()) {
                    follower.follow(path6);
                    step = 8;
                }
                break;

            // Stop intake and return along Path 7.
            case 8:
                if (!follower.isBusy()) {
                    intake.stopIntake();
                    follower.follow(path7);
                    step = 9;
                }
                break;

            // Hold the exit pose until Stop.
            case 11:
                if (!follower.isBusy()) {
                    follower.hold(path9.endPose());
                    step = 12;
                }
                break;
        }

        OpModeStorage.autonomousEndPose = follower.pose();

        telemetry.addData("Step", step);
        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("RPM", shooter.getVelocityRpm());
        telemetry.addData("Ready", shooter.isReady());
        telemetry.update();
    }

    @Override
    public void stop() {
        OpModeStorage.autonomousEndPose = follower.pose();
        shooter.stop();
        intake.stopIntake();
        stopper.off();
        follower.stop();
        follower.update();
    }
}