package org.firstinspires.ftc.teamcode.TeleOps;


import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem_Motor;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.utils.OpModeStorage;
import org.firstinspires.ftc.teamcode.utils.TurretTracking;


@TeleOp(name = "blueTeleop")
public class blueTeleop extends OpMode {
    private Follower follower;
    private TurretTracking tracking;
    /**subsystems declaration
     *
     */

    private IntakeSubsystem_Motor intake;
    private ShooterSubsystem shooter;
    private TurretSubsystem turret;

    /**button edges
     *
     */
    private boolean prevA = false;

    /**Subsystem states
     *
     */
    public boolean intakeOn = true;

    /**configs
     *
     */
    TurretSubsystem.Config turretConfig = new TurretSubsystem.Config();

    // Fill these with the two target positions in your Pedro field coordinates.
    // NaN deliberately marks coordinates that have not been configured yet.
    private static final double LOWER_TARGET_X = Double.NaN;
    private static final double LOWER_TARGET_Y = Double.NaN;

    private static final double UPPER_TARGET_X = Double.NaN;
    private static final double UPPER_TARGET_Y = Double.NaN;

    // Assumes the field runs from Y = 0 to Y = 144 inches.
    private static final double FIELD_MIDPOINT_Y = 72.0;


    @Override
    public void init() {
        // Replace with the measured voltage when the turret points forward.
        turretConfig.encoderCenterVolts = 1.5;

        follower = Constants.create(hardwareMap);
        intake = new IntakeSubsystem_Motor(hardwareMap);
        turret = new TurretSubsystem(hardwareMap, turretConfig);
        tracking = new TurretTracking(
                turret,
                new TurretTracking.Target(LOWER_TARGET_X, LOWER_TARGET_Y),
                new TurretTracking.Target(UPPER_TARGET_X, UPPER_TARGET_Y),
                FIELD_MIDPOINT_Y
        );
    }

    public void start() {
        follower.setPose(OpModeStorage.autonomousEndPose);
        follower.update();
        intake.startIntake();
    }

    @Override
    public void loop() {
        /**
         * Drivetrain and pedro
         */
        //drive
        //field centric drive
        /*
        DrivePowers powers = ManualDrive.fieldCentric(
            -gamepad1.left_stick_y,
            gamepad1.left_stick_x,
            gamepad1.right_stick_x,
            follower.pose().heading()
        );
        follower.manual(powers);
         */

        //robot centric drive
        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                follower.pose().heading()
        );
        ManualDrive.driveOrHold(follower, powers);

        // relocalise button
        if (gamepad1.startWasPressed()) {
            Pose cornerPose = new Pose(10.5, 10.5, Math.toRadians(90));
            // On the fly Pose creation, we dont recommend this for Autonomous. Only accepts radians for heading
            follower.setPose(cornerPose); // overrides our pose
        }

        follower.update();
        //localization
        Pose robotPose = follower.pose(); // returns a Pose object
        tracking.update(robotPose.x(), robotPose.y(), robotPose.heading());
        // Math.toDegrees() is a built-in java method

        /*
        need OpModeStorage.autonomousEndPose = follower.pose(); in auto to record pose to transition
         */

        /**
         * Intake Subsystem
         */
        boolean a1 = gamepad1.a;
        if (a1 && !prevA) intakeOn = !intakeOn;
        prevA = a1;

        if (intakeOn) intake.startIntake();
        else intake.stopIntake();

        /**
         * Shooter Subsystem
         */

        /**
         * Telemetry
         */
        //robot pose
        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));

        //turret
        telemetry.addData("Target half", tracking.getSelectedHalf());
        telemetry.addData("Target distance (in)", tracking.getDistanceInches());

        telemetry.addData("Requested aim (deg)",
                Math.toDegrees(tracking.getRequestedAngleRadians()));

        telemetry.addData("Measured turret (deg)",
                Math.toDegrees(turret.getAngleRadians()));

        telemetry.addData("Turret state", turret.getState());
        telemetry.addData("Turret aligned", tracking.isAligned());

        telemetry.update();


    }
}
//lexihere!