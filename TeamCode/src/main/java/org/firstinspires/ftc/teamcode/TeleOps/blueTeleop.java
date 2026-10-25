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
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystemFF;
import org.firstinspires.ftc.teamcode.subsystems.StopperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.utils.OpModeStorage;
import org.firstinspires.ftc.teamcode.utils.TurretTracking;
import org.firstinspires.ftc.teamcode.utils.AutoAim;
import org.firstinspires.ftc.teamcode.utils.AutoAimVision;


@TeleOp(name = "blueTeleop")
public class blueTeleop extends OpMode {
    private Follower follower;
    private TurretTracking tracking;
    private AutoAim autoAim;
    private AutoAimVision visionAim;


    /**subsystems declaration
     *
     */

    private IntakeSubsystem_Motor intake;
    private ShooterSubsystemFF shooter;
    private TurretSubsystem turret;
    private StopperSubsystem stopper;
    private VisionSubsystem vision;

    /**button edges
     *
     */
    private boolean prevA = false;
    private boolean prevX = false;
    private boolean prevUp = false;
    private boolean prevDown = false;
    private boolean prevPoseButton, prevVisionButton;

    /**Subsystem states
     *
     */
    public boolean intakeOn = true;
    private boolean shooterEnabled = false;
    private static final double DEFAULT_RPM = 3200;
    private static final double RPM_STEP = 250;
    private static final double MAX_RPM = 6000;
    private static final double MIN_RPM = 0;

    /**configs
     *
     */
    TurretSubsystem.Config turretConfig = new TurretSubsystem.Config();

    // Fill these with the two target positions in your Pedro field coordinates.
// Goal coordinates in Pedro field coordinates (inches)
    private static final double LOWER_GOAL_X = 84.0;
    private static final double LOWER_GOAL_Y = 46.0;

    private static final double UPPER_GOAL_X = 84.0;
    private static final double UPPER_GOAL_Y = 96.0;

    // 0 = shooter points forward along robot +X
    private static final double SHOOTER_OFFSET = 0.0;

    // Assumes the field runs from Y = 0 to Y = 144 inches.
    private static final double FIELD_MIDPOINT_Y = 72.0;

    private static final double DRIVE_DEADBAND = 0.08;

    private enum AimMode {
        NONE,
        POSE,
        VISION
    }

    private AimMode aimMode = AimMode.NONE;




    @Override
    public void init() {
        // Replace with the measured voltage when the turret points forward.
        turretConfig.encoderCenterVolts = 1.5;

        follower = Constants.create(hardwareMap);
        autoAim = new AutoAim(
                follower,
                LOWER_GOAL_X,
                LOWER_GOAL_Y,
                UPPER_GOAL_X,
                UPPER_GOAL_Y,
                SHOOTER_OFFSET
        );
        intake = new IntakeSubsystem_Motor(hardwareMap);
        shooter = new ShooterSubsystemFF(hardwareMap);
        stopper = new StopperSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap);

        visionAim = new AutoAimVision(follower, vision);
        visionAim.setSelectionMode(
                VisionSubsystem.SelectionMode.MOST_TAGS
        );
        //turret = new TurretSubsystem(hardwareMap, turretConfig);

        shooter.setTargetRpm(DEFAULT_RPM);

        /*
        tracking = new TurretTracking(
                turret,
                new TurretTracking.Target(LOWER_TARGET_X, LOWER_TARGET_Y),
                new TurretTracking.Target(UPPER_TARGET_X, UPPER_TARGET_Y),
                FIELD_MIDPOINT_Y
        );

         */
    }

    @Override
    public void start() {
        Pose startPose;

        if (OpModeStorage.autonomousEndPose != null) {
            // Use the saved Autonomous position
            startPose = OpModeStorage.autonomousEndPose;
        } else {
            // Default to center of field, facing +Y
            startPose = new Pose(72, 72, Math.toRadians(90));
        }

        follower.setPose(startPose);
        follower.update();

        intake.startIntake();
    }

    @Override
    public void loop() {
        /**
         * Drivetrain and pedro
         */

        // All aim transitions occur before the single follower.update().
        double forward = -gamepad1.left_stick_y;
        double strafe  = -gamepad1.left_stick_x;
        double turn    = -gamepad1.right_stick_x;
        boolean driverMoving = Math.abs(forward) > DRIVE_DEADBAND ||
                Math.abs(strafe) > DRIVE_DEADBAND ||
                Math.abs(turn) > DRIVE_DEADBAND;

        boolean poseButton = gamepad1.y;
        boolean visionButton = gamepad1.right_bumper;
        boolean newPosePress = poseButton && !prevPoseButton;
        boolean newVisionPress = visionButton && !prevVisionButton;
        prevPoseButton = poseButton;
        prevVisionButton = visionButton;

        if (gamepad1.startWasPressed()) {
            autoAim.cancel();
            visionAim.cancel();
            aimMode = AimMode.NONE;
            follower.setPose(new Pose(10.5, 10.5, Math.toRadians(90)));
        } else if (driverMoving) {
            autoAim.cancel();
            visionAim.cancel();
            aimMode = AimMode.NONE;
        } else if (newVisionPress) {
            // Right bumper wins when both buttons are pressed together.
            autoAim.cancel();
            aimMode = AimMode.VISION;
        } else if (newPosePress) {
            visionAim.cancel();
            if (poseGoalCoordinatesSet()) {
                aimMode = AimMode.POSE;
            } else {
                autoAim.cancel();
                aimMode = AimMode.NONE;
            }
        }

        // Only the selected controller receives an active button.
        // Non-selected controllers also receive input updates so their
        // rising-edge detectors reset before switching back.
        boolean poseAiming = autoAim.update(
                aimMode == AimMode.POSE && poseButton,
                driverMoving || aimMode != AimMode.POSE);
        boolean visionAiming = visionAim.update(
                aimMode == AimMode.VISION && visionButton,
                driverMoving || aimMode != AimMode.VISION);

        if (aimMode == AimMode.POSE && !poseAiming) aimMode = AimMode.NONE;
        if (aimMode == AimMode.VISION && !visionAiming) aimMode = AimMode.NONE;

        if (aimMode == AimMode.NONE) {
            ManualDrive.driveOrHold(follower, forward, strafe, turn);
        }

        follower.update();
        Pose robotPose = follower.pose();

        //tracking.update(robotPose.x(), robotPose.y(), robotPose.heading());
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
        // Toggle shooter using X
        boolean x1 = gamepad1.x;

        if (x1 && !prevX) {
            shooterEnabled = !shooterEnabled;
        }

        prevX = x1;
        //RPM adjust
        // Increase target RPM
        boolean up = gamepad1.dpad_up;

        if (up && !prevUp) {
            shooter.setTargetRpm(
                    Math.min(
                            MAX_RPM,
                            shooter.getTargetRpm() + RPM_STEP
                    )
            );
        }

        prevUp = up;

        // Decrease target RPM
        boolean down = gamepad1.dpad_down;

        if (down && !prevDown) {
            shooter.setTargetRpm(
                    Math.max(
                            MIN_RPM,
                            shooter.getTargetRpm() - RPM_STEP
                    )
            );
        }

        prevDown = down;

        shooter.update(shooterEnabled);


        /**
         * Stopper Subsystem
         */
        if (gamepad1.b ) { //&& shooterEnabled && shooter.isReady()
            stopper.on();
        } else {
            stopper.off();
        }

        /**
         * Telemetry
         */
        //robot pose
        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Heading deg", Math.toDegrees(robotPose.heading()));
        telemetry.addData("Aim mode", aimMode);
        telemetry.addData("Pose aim configured", poseGoalCoordinatesSet());
        telemetry.addData("Vision status", visionAim.getStatus());
        telemetry.addData("Vision correction deg", visionAim.getCommandedCorrectionDegrees());
        telemetry.addData("Vision error deg", visionAim.getHeadingErrorDegrees());
        telemetry.addData("Vision aligned <=2deg", visionAim.isAligned(2.0));

        VisionSubsystem.AimObservation obs = visionAim.getLastObservation();
        if (obs != null) {
            telemetry.addData("Vision CELL", obs.goal);
            telemetry.addData("Tags audience/far", "%d / %d", obs.audienceCount, obs.farCount);
            telemetry.addData("Selected tag IDs", obs.visibleIds);
            telemetry.addData("Tag vec F/L/U (m)", "%.2f / %.2f / %.2f",
                    obs.forwardM, obs.leftM, obs.upM);
        }

        telemetry.addData("Shooter Enabled", shooterEnabled);
        telemetry.addData("Target RPM", shooter.getTargetRpm());
        telemetry.addData("Current RPM", shooter.getVelocityRpm());
        telemetry.addData("Ready", shooter.isReady());

        telemetry.addData("LIVE kP", ShooterSubsystemFF.kP);
        telemetry.addData("LIVE kI", ShooterSubsystemFF.kI);
        telemetry.addData("LIVE kD", ShooterSubsystemFF.kD);
        telemetry.addData("LIVE kV", ShooterSubsystemFF.kV);
        telemetry.addData("LIVE kS", ShooterSubsystemFF.kS);

        telemetry.addData("Motor Power Command", shooter.getPowerCmd());
        telemetry.addData("Shooter Target RPM", shooter.getTargetRpm());
        telemetry.addData("Shooter Actual RPM", shooter.getVelocityRpm());

        /*
        //turret
        telemetry.addData("Target half", tracking.getSelectedHalf());
        telemetry.addData("Target distance (in)", tracking.getDistanceInches());

        telemetry.addData("Requested aim (deg)",
                Math.toDegrees(tracking.getRequestedAngleRadians()));

        telemetry.addData("Measured turret (deg)",
                Math.toDegrees(turret.getAngleRadians()));

        telemetry.addData("Turret state", turret.getState());
        telemetry.addData("Turret aligned", tracking.isAligned());
        */

        telemetry.update();


    }

    private static boolean poseGoalCoordinatesSet() {
        return Double.isFinite(LOWER_GOAL_X) &&
                Double.isFinite(LOWER_GOAL_Y) &&
                Double.isFinite(UPPER_GOAL_X) &&
                Double.isFinite(UPPER_GOAL_Y);
    }

    @Override
    public void stop() {
        if (autoAim != null) autoAim.cancel();
        if (visionAim != null) visionAim.cancel();
        if (vision != null) vision.stop();
    }

}
//lexihere!