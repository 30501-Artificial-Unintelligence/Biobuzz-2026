package org.firstinspires.ftc.teamcode.teleOps;


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
import org.firstinspires.ftc.teamcode.utils.AutoAim;


@TeleOp(name = "blueTeleop")
public class blueTeleop extends OpMode {
    private Follower follower;
    private TurretTracking tracking;
    private AutoAim autoAim;
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
        autoAim = new AutoAim(
                follower,
                84, 46,
                84, 96,
                0 // Forward-facing shooter
        );
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

        // relocalise button
        if (gamepad1.startWasPressed()) {
            autoAim.cancel();
            Pose cornerPose = new Pose(10.5, 10.5, Math.toRadians(90));
            // On the fly Pose creation, we dont recommend this for Autonomous. Only accepts radians for heading
            follower.setPose(cornerPose); // overrides our pose
        }


        /*
        Robot drivetrain aiming
         */

        boolean driverMoving =
                Math.abs(gamepad1.left_stick_y) > 0.1 ||
                        Math.abs(gamepad1.left_stick_x) > 0.1 ||
                        Math.abs(gamepad1.right_stick_x) > 0.1;

        boolean aiming = autoAim.update(gamepad1.right_bumper, driverMoving);

        if (!aiming) {
            ///robot centric drive.
            ManualDrive.driveOrHold(
                    follower,
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );
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
        //auto aiming
        // Aiming telemetry.

        telemetry.addData("Auto aiming", autoAim.isAiming());
        telemetry.addData("Aim aligned", autoAim.isAligned(2.0));

        if (autoAim.isAiming()) {
            telemetry.addData(
                    "Aim error (deg)",
                    Math.toDegrees(autoAim.getHeadingError())
            );
        }

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