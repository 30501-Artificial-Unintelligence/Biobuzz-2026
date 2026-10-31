package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem_Motor;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystemFF;
import org.firstinspires.ftc.teamcode.subsystems.StopperSubsystem;
import org.firstinspires.ftc.teamcode.utils.OpModeStorage;

@Autonomous(name = "Blue Lower", group = "Autonomous")
public class BlueLowerAuto extends OpMode {

    private Follower follower;
    private IntakeSubsystem_Motor intake;
    private ShooterSubsystemFF shooter;
    private StopperSubsystem stopper;

    private final ElapsedTime timer = new ElapsedTime();
    private int step = 0;

    private static final double SHOOT_RPM = 3200;
    private static final double FEED_DELAY = 0.3;
    private static final double GATE_TIME = 1.5;
    private static final double COLLECT_WAIT = 2.0;

    private final PoseFactory poses = PoseFactory.degrees();

    private final Pose startPose = poses.of(81.5, 7.5, 270);
    private final Pose shootPose = poses.of(81.5, 11.5, 90);
    private final Pose collectPose = poses.of(93.5, 11.5, 270);
    private final Pose collectControl = poses.of(88.1712, 31.9683, 0);
    private final Pose returnControl = poses.of(88.2398, 32.4502, 0);
    private final Pose exitPose = poses.of(123.9472, 30.9306, 24.5964);

    private Path path1, path2, path3, path5;

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

        path1 = Paths.line(startPose, shootPose)
                .linear(startPose, shootPose);

        path2 = Paths.curve(shootPose, collectControl, collectPose)
                .heading(Interpolator.longLinear(shootPose, collectPose));

        // Return directly to the shooting pose; no tiny Path 4 needed.
        path3 = Paths.curve(collectPose, returnControl, shootPose)
                .linear(collectPose, shootPose);

        path5 = Paths.line(shootPose, exitPose).tangent();

        OpModeStorage.autonomousIsBlue = true;
        OpModeStorage.autonomousEndPose = null;
    }

    @Override
    public void start() {
        step = 0;
        shooter.update(true);
        follower.follow(path1);
    }

    @Override
    public void loop() {
        follower.update();
        shooter.update(true);

        switch (step) {

            // First or second arrival at the shooting position.
            case 0:
            case 4:
                if (!follower.isBusy() && shooter.isReady()) {
                    follower.hold(shootPose);
                    intake.stopIntake();
                    stopper.on();
                    timer.reset();
                    step++;
                }
                break;

            // Gate open for 1.5 s; intake starts after 0.3 s.
            case 1:
            case 5:
                if (timer.seconds() >= GATE_TIME) {
                    stopper.off();
                    intake.stopIntake();

                    if (step == 1) {
                        intake.startIntake();
                        timer.reset();
                        step = 2;
                    } else {
                        follower.follow(path5);
                        step = 6;
                    }
                } else if (timer.seconds() >= FEED_DELAY) {
                    intake.startIntake();
                }
                break;

            // Collect while stationary for 2 s, then start Path 2.
            case 2:
                if (timer.seconds() >= COLLECT_WAIT) {
                    follower.follow(path2);
                    step = 3;
                }
                break;

            // Intake runs throughout Path 2, stops before Path 3.
            case 3:
                if (!follower.isBusy()) {
                    intake.stopIntake();
                    follower.follow(path3);
                    step = 4;
                }
                break;

            // Finish exit path; keep shooter on until OpMode stops.
            case 6:
                if (!follower.isBusy()) {
                    follower.hold(path5.endPose());
                    step = 7;
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