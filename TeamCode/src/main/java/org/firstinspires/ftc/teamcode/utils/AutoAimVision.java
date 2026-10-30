package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;

/**
 * Captures one vision target per button press, then uses Pedro's
 * heading controller to rotate without commanding translation.
 * Manual stick input cancels aiming.
 */
public class AutoAimVision {
    // Positive if camera lens is FORWARD of shooter's exit point.
    // This is along the centerline only. Both have the same firing yaw.
    public static double CAMERA_AHEAD_OF_SHOOTER_M = 0.18;
    public static double MIN_SHOOTER_TARGET_FORWARD_M = 0.10;

    private final Follower follower;
    private final VisionSubsystem vision;
    private VisionSubsystem.SelectionMode selectionMode =
            VisionSubsystem.SelectionMode.MOST_TAGS;

    private boolean aiming = false;
    private boolean previousButton = false;
    private double targetHeadingRad = Double.NaN;
    private double commandedCorrectionRad = Double.NaN;
    private VisionSubsystem.AimObservation lastObservation;
    private String status = "IDLE";

    public AutoAimVision(Follower follower, VisionSubsystem vision) {
        this.follower = follower;
        this.vision = vision;
    }

    public void setSelectionMode(VisionSubsystem.SelectionMode mode) {
        if (mode != null) selectionMode = mode;
    }

    /** Returns true if vision owns the drivetrain. Call every TeleOp loop. */
    public boolean update(boolean aimButton, boolean driverMoving) {
        boolean pressed = aimButton && !previousButton;
        previousButton = aimButton;

        if (driverMoving) {
            cancel();
            return false;
        }

        if (pressed) {
            startAim();
        }

        if (aiming) {
            Foresight algorithm = (Foresight) follower.algorithm();

            DrivePowers powers = ManualDrive.headingLock(
                    follower,
                    algorithm.config.headingFeedback.get(),
                    new DrivePowers(0, 0, 0),
                    targetHeadingRad
            );

            follower.manual(powers);
        }

        return aiming;
    }

    private void startAim() {
        Pose current = follower.pose();
        // current.y() matters ONLY if using the optional FIELD_HALF mode.
        VisionSubsystem.AimObservation observation =
                vision.getBlueGoal(selectionMode, current.y());
        if (observation == null) {
            // If already aiming, retain the previous HOLD rather than
            // silently claiming to have acquired a new target.
            status = "NO_FRESH_TARGET";
            return;
        }

        // Camera and shooter on same lateral centerline. Compensate only
        // their forward separation, not field position or camera height.
        double targetForwardFromShooter = observation.forwardM + CAMERA_AHEAD_OF_SHOOTER_M;
        if (!Double.isFinite(targetForwardFromShooter) ||
                targetForwardFromShooter <= MIN_SHOOTER_TARGET_FORWARD_M) {
            status = "INVALID_GEOMETRY";
            return;
        }

        double correction = Math.atan2(observation.leftM, targetForwardFromShooter);
        double desiredHeading = wrap(current.heading() + correction);
        if (!Double.isFinite(desiredHeading)) {
            status = "INVALID_HEADING";
            return;
        }

        lastObservation = observation;
        targetHeadingRad = desiredHeading;
        commandedCorrectionRad = correction;

        aiming = true;
        status = "AIMING";
    }

    public void cancel() {
        if (aiming) follower.stop();
        aiming = false;
        if (!status.equals("NO_FRESH_TARGET") &&
                !status.equals("INVALID_GEOMETRY") &&
                !status.equals("INVALID_HEADING")) {
            status = "IDLE";
        }
    }

    public boolean isAiming() { return aiming; }
    public String getStatus() { return status; }
    public VisionSubsystem.AimObservation getLastObservation() { return lastObservation; }
    public double getCommandedCorrectionDegrees() {
        return Math.toDegrees(commandedCorrectionRad);
    }
    public double getHeadingErrorDegrees() {
        return aiming ? Math.toDegrees(wrap(targetHeadingRad - follower.pose().heading())) : Double.NaN;
    }
    public boolean isAligned(double toleranceDegrees) {
        return aiming && Math.abs(getHeadingErrorDegrees()) <= toleranceDegrees;
    }

    private static double wrap(double radians) {
        return Math.atan2(Math.sin(radians), Math.cos(radians));
    }
}
