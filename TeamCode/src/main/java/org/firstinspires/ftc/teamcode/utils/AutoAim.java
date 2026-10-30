package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.follower.ManualDrive;

/** Button-triggered, stationary aiming for Pedro 3. */
public class AutoAim {
    private final Follower follower;
    private final double lowerGoalX, lowerGoalY;
    private final double upperGoalX, upperGoalY;
    private final double shooterOffset;

    private boolean aiming = false;
    private boolean previousAimButton = false;
    private double targetHeading = 0;

    /**
     * Goal coordinates are inches in the same field frame as the robot pose.
     * Shooter offset is radians relative to robot forward:
     * 0 for forward-facing, Math.PI for backward-facing.
     */
    public AutoAim(Follower follower,
                   double lowerGoalX, double lowerGoalY,
                   double upperGoalX, double upperGoalY,
                   double shooterOffset) {
        this.follower = follower;
        this.lowerGoalX = lowerGoalX;
        this.lowerGoalY = lowerGoalY;
        this.upperGoalX = upperGoalX;
        this.upperGoalY = upperGoalY;
        this.shooterOffset = shooterOffset;
    }

    /**
     * Call every TeleOp loop. Returns true while aiming owns the drivetrain.
     * Run manual driving only when this returns false, and call
     * follower.update() once per loop in TeleOp after this method.
     * Stick movement or cancelButton cancels aiming.
     */
    public boolean update(boolean aimButton, boolean driverMoving) {
        boolean justPressed = aimButton && !previousAimButton;
        previousAimButton = aimButton;

        if (driverMoving) {
            cancel();
        } else if (justPressed) {
            startAim();
        }

        if (aiming) {
            Foresight algorithm = (Foresight) follower.algorithm();

            DrivePowers powers = ManualDrive.headingLock(
                    follower,
                    algorithm.config.headingFeedback.get(),
                    new DrivePowers(0, 0, 0),
                    targetHeading
            );

            follower.manual(powers);
        }

        return aiming;
    }

    private void startAim() {
        Pose robot = follower.pose();
        // Select once per button press; exactly Y=70 uses the upper goal.
        boolean useLowerGoal = robot.y() < 70.0;
        double goalX = useLowerGoal ? lowerGoalX : upperGoalX;
        double goalY = useLowerGoal ? lowerGoalY : upperGoalY;
        double dx = goalX - robot.x();
        double dy = goalY - robot.y();

        if (Math.hypot(dx, dy) < 0.001) {
            return;
        }

        targetHeading = wrapAngle(Math.atan2(dy, dx) - shooterOffset);
        // Create a fresh target from the current pose on each button press.
        aiming = true;
    }

    public void cancel() {
        if (aiming) {
            follower.stop();
            aiming = false;
        }
    }

    public boolean isAiming() {
        return aiming;
    }

    /** Signed heading error in radians. */
    public double getHeadingError() {
        return wrapAngle(targetHeading - follower.pose().heading());
    }

    /** Heading check only; does not check angular velocity or shooter RPM. */
    public boolean isAligned(double toleranceDegrees) {
        return aiming && Math.abs(getHeadingError())
                <= Math.toRadians(toleranceDegrees);
    }

    private static double wrapAngle(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }
}
