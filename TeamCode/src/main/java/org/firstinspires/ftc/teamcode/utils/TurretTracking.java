package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

/**
 * Points the turret at one of two fixed field targets using the CURRENT pose.
 * This is geometric tracking only: no projectile lead or velocity compensation.
 * The turret is treated as centered on the robot; its 2 mm offset is ignored.
 *
 * Coordinates: x/y in inches, heading in radians; heading zero faces field +X
 * and positive rotation is counterclockwise. All targets and the starting pose
 * must use the same field coordinate system.
 *
 * Place beside Turret.java in TeamCode's subsystems package.
 * Construct once after constructing/calibrating Turret:
 *
 *   tracking = new TurretTracking(turret,
 *           new TurretTracking.Target(LOWER_TARGET_X, LOWER_TARGET_Y),
 *           new TurretTracking.Target(UPPER_TARGET_X, UPPER_TARGET_Y),
 *           FIELD_MIDPOINT_Y);
 *
 * LOWER_TARGET means the target to use while the ROBOT is in the lower half.
 * It does not require that target itself to be located in the lower half.
 * A midpoint of 72 inches assumes a field Y range of 0..144 inches.
 * Supply your actual goal coordinates; no guessed goal locations are included.
 *
 * Pedro 3 active TeleOp loop, AFTER setting manual drive inputs:
 *
 *   follower.update();
 *   Pose pose = follower.pose();  // com.pedropathing.math.Pose
 *   tracking.update(pose.x(), pose.y(), pose.heading());
 *
 * tracking.update() calls turret.update(), so DO NOT call turret.update() again.
 * It never updates Pedro or Pinpoint. TeleOp owns the single follower update.
 * Call tracking.stop() from TeleOp stop(). Do not call tracking.update() during
 * init unless you intentionally want the turret to move before Start.
 */
public final class TurretTracking {
    public enum FieldHalf { LOWER, UPPER }

    /** Immutable target coordinates in the same field frame as the robot pose. */
    public static final class Target {
        public final double xInches;
        public final double yInches;

        public Target(double xInches, double yInches) {
            requireFinite("Target X", xInches);
            requireFinite("Target Y", yInches);
            this.xInches = xInches;
            this.yInches = yInches;
        }
    }

    private final TurretSubsystem turret;
    private final Target lowerHalfTarget;
    private final Target upperHalfTarget;
    private final double midpointYInches;

    // These describe the most recent update and are useful for telemetry.
    private FieldHalf selectedHalf;
    private boolean hasValidAim;
    private double requestedAngleRadians = Double.NaN;
    private double distanceInches = Double.NaN;

    public TurretTracking(TurretSubsystem turret, Target lowerHalfTarget,
                          Target upperHalfTarget, double midpointYInches) {
        if (turret == null) {
            throw new IllegalArgumentException("Turret must be supplied");
        }
        if (lowerHalfTarget == null || upperHalfTarget == null) {
            throw new IllegalArgumentException("Both field targets must be supplied");
        }
        requireFinite("Field midpoint Y", midpointYInches);
        this.turret = turret;
        this.lowerHalfTarget = lowerHalfTarget;
        this.upperHalfTarget = upperHalfTarget;
        this.midpointYInches = midpointYInches;
    }

    /** Call once per active loop with Pedro's freshly updated, cached pose. */
    public void update(double robotXInches, double robotYInches,
                       double robotHeadingRadians) {
        if (!finite(robotXInches) || !finite(robotYInches)
                || !finite(robotHeadingRadians)) {
            // Do not keep reporting readiness for an old aim after invalid input.
            stop();
            turret.update(); // Still sample the encoder, without a new command.
            return;
        }

        // Switch automatically in BOTH directions. Exactly on the dividing line
        // uses UPPER. There is no delay or hysteresis in this initial version.
        // If pose noise causes repeated switches near this line, add a small
        // spatial hysteresis band here later (separate from turret wrap handling).
        selectedHalf = robotYInches < midpointYInches
                ? FieldHalf.LOWER : FieldHalf.UPPER;
        Target target = getSelectedTarget();

        // Field-space vector from the robot center to the selected target.
        double deltaX = target.xInches - robotXInches;
        double deltaY = target.yInches - robotYInches;
        distanceInches = Math.hypot(deltaX, deltaY);

        // At the target's exact XY location there is no meaningful aim direction.
        // The tiny epsilon handles numerical coincidence; it is not a shot range.
        if (!finite(distanceInches) || distanceInches < 1e-6) {
            hasValidAim = false;
            requestedAngleRadians = Double.NaN;
            turret.stop();
            turret.update();
            return;
        }

        // atan2 handles all four quadrants and returns the FIELD aim direction.
        double fieldAimRadians = Math.atan2(deltaY, deltaX);

        // Subtract robot heading to express the aim relative to robot forward.
        // Example: field aim 90 deg, robot heading 30 deg -> turret aim +60 deg.
        requestedAngleRadians = wrapRadians(
                fieldAimRadians - wrapRadians(robotHeadingRadians));
        hasValidAim = true;

        // Turret owns mechanical limits, branch selection/unwinding and feedback.
        // It must process the new request BEFORE anyone checks alignment.
        turret.setTargetAngleRadians(requestedAngleRadians);
        turret.update();
    }

    /** Alignment only; this does not check shooter speed, hood or shot feasibility. */
    public boolean isAligned() {
        return hasValidAim && turret.readyToLaunch();
    }

    public boolean hasValidAim() { return hasValidAim; }
    public FieldHalf getSelectedHalf() { return selectedHalf; }

    /** Null before the first valid pose or after stop(). */
    public Target getSelectedTarget() {
        if (selectedHalf == null) return null;
        return selectedHalf == FieldHalf.LOWER ? lowerHalfTarget : upperHalfTarget;
    }

    /** Robot-relative direction; Turret may choose its equivalent at the wrap seam. */
    public double getRequestedAngleRadians() { return requestedAngleRadians; }
    public double getDistanceInches() { return distanceInches; }

    /**
     * Stops issuing turret commands and clears aim readiness. This does not
     * disable servo PWM; the servo can still finish/hold its last command.
     * A subsequent update() resumes automatic tracking.
     */
    public void stop() {
        hasValidAim = false;
        selectedHalf = null;
        requestedAngleRadians = Double.NaN;
        distanceInches = Double.NaN;
        turret.stop();
    }

    /** Normalize a direction into [-pi, pi). Mechanical wrapping stays in Turret. */
    private static double wrapRadians(double angle) {
        double result = angle % (2.0 * Math.PI);
        if (result >= Math.PI) result -= 2.0 * Math.PI;
        if (result < -Math.PI) result += 2.0 * Math.PI;
        return result;
    }

    private static void requireFinite(String name, double value) {
        if (!finite(value)) {
            throw new IllegalArgumentException(name + " must be a finite number");
        }
    }

    private static boolean finite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }
}
