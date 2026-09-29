package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Single positional servo, absolute analog encoder on the SERVO shaft.
 * All public angles/rates use radians and radians/second, robot-relative.
 * Positive angle is counterclockwise; zero is the robot's forward direction.
 * No Pedro, Pinpoint, command framework, or projectile math dependency.
 *
 * Calibrate encoderCenterVolts with the turret mechanically centered, then
 * verify servo center, direction, travel and limits before running full travel.
 * Servo travel means the measured travel across SDK positions 0..1 with the
 * PWM range you actually use. Do not change Servo direction/scaleRange elsewhere.
 * Analog feedback cannot reliably detect a disconnected wire that reads a
 * plausible voltage, nor backlash/slip downstream of this encoder.
 *
 * Usage: construct once; call setTargetAngleRadians then update once per loop.
 * Feed permission must include atTarget(), alongside shooter/SOTM readiness.
 * stop() stops new commands; it does NOT disable servo PWM or remove holding torque.
 */
public final class TurretSubsystem {
    public enum State {
        DISABLED,       // Read feedback, but do not send new servo commands.
        TRACKING,       // Follow the selected mechanically reachable target.
        BOUNDARY_HOLD,  // Target is just beyond a limit; wait before committing to unwind.
        UNWINDING,      // Moving/holding on the selected opposite side; shooting blocked.
        SENSOR_FAULT   // A reading failed the voltage or mechanical-range checks.
    }

    /** Set these BEFORE constructing Turret. Values are copied, not updated live. */
    public static final class Config {
        // Measured servo-shaft travel across SDK commands 0..1 with your PWM settings.
        public double servoTravelDeg = 270.0; // PROVISIONAL: measure this.
        // One servo revolution produces 1.745 turret revolutions.
        public double turretPerServoRatio = 1.745;
        // Servo command that physically points the turret forward.
        public double servoCenterPosition = 0.5;
        // +1 if increasing the command moves counterclockwise; otherwise -1.
        public int servoSign = 1;
        // +1 if increasing encoder angle means counterclockwise turret motion.
        public int encoderSign = 1;
        // Voltage span representing one complete encoder revolution.
        public double encoderFullScaleVolts = 3.3;
        // Measure this voltage with the turret physically pointing forward.
        public double encoderCenterVolts = Double.NaN; // REQUIRED calibration.
        // Mechanical limits relative to forward; never command beyond these.
        public double minAngleDeg = -180.0;
        public double maxAngleDeg = 180.0;
        // Target must pass this far beyond the boundary to trigger an unwind.
        public double boundaryHysteresisDeg = 3.0;
        // Maximum immediate position error permitted by readyToLaunch().
        public double alignmentToleranceDeg = 2.0;
        // Larger time smooths velocity more but adds delay. Zero means no smoothing.
        public double velocityFilterSeconds = 0.05;
    }

    private final Servo servo;
    private final AnalogInput encoder;
    // Immutable calibration. Internal angles use radians.
    private final double turretPerServoRatio;
    private final double turretRadiansPerServoPosition;  // TURRET radians corresponding to servo positions 0..1.
    private final double servoCenterPosition;
    private final double encoderFullScaleVolts;
    private final double encoderCenterVolts;
    private final double minAngleRadians;
    private final double maxAngleRadians;
    private final double boundaryHysteresisRadians;
    private final double alignmentToleranceRadians;
    private final double velocityFilterSeconds;
    private final int servoSign;
    private final int encoderSign;
    private State state = State.DISABLED;
    private boolean enabled;
    private boolean hasSample;     // At least one valid encoder reading is available.
    private boolean hasTarget;     // A target has been selected since the last stop.
    private boolean branchLocked;  // Keep the selected side during an unwind.
    private double angle;          // Measured physical turret angle.
    private double velocity;       // Filtered measured turret angular velocity.
    private double target;         // Selected physical target inside mechanical limits.
    private double requested;      // Desired direction, normalized to [-pi, pi).
    private double branchTarget;   // Equivalent direction before clamping during unwind.
    private double voltage;
    private double lastTime = Double.NaN; // Unset until the first update.
    private boolean aligned;
    private static final double TAU = 2.0 * Math.PI; // One full revolution in radians.

    public TurretSubsystem(Servo servo, AnalogInput encoder, Config c) {
        this.servo = servo;
        this.encoder = encoder;
        turretPerServoRatio = c.turretPerServoRatio;
        turretRadiansPerServoPosition = Math.toRadians(c.servoTravelDeg) * turretPerServoRatio;
        servoCenterPosition = c.servoCenterPosition;
        encoderFullScaleVolts = c.encoderFullScaleVolts;
        encoderCenterVolts = c.encoderCenterVolts;
        servoSign = c.servoSign;
        encoderSign = c.encoderSign;
        minAngleRadians = Math.toRadians(c.minAngleDeg);
        maxAngleRadians = Math.toRadians(c.maxAngleDeg);
        boundaryHysteresisRadians = Math.toRadians(c.boundaryHysteresisDeg);
        alignmentToleranceRadians = Math.toRadians(c.alignmentToleranceDeg);
        velocityFilterSeconds = c.velocityFilterSeconds;
        // Check calibration once at construction, before any servo command.
        validateConfiguration();
    }

    /**
     * These checks protect assumptions used by the angle conversion and wrapping.
     * A failed check gives a specific explanation on the Driver Station.
     */
    private void validateConfiguration() {
        // NaN means "not a number". We use it for an unset encoder center.
        // Infinity is also invalid: neither can describe a physical calibration.
        requireFinite("turretPerServoRatio", turretPerServoRatio);
        requireFinite("servo travel multiplied by gear ratio", turretRadiansPerServoPosition);
        requireFinite("servoCenterPosition", servoCenterPosition);
        requireFinite("encoderFullScaleVolts", encoderFullScaleVolts);
        requireFinite("encoderCenterVolts (measure with turret centered)", encoderCenterVolts);
        requireFinite("minAngleDeg", minAngleRadians);
        requireFinite("maxAngleDeg", maxAngleRadians);
        requireFinite("boundaryHysteresisDeg", boundaryHysteresisRadians);
        requireFinite("alignmentToleranceDeg", alignmentToleranceRadians);
        requireFinite("velocityFilterSeconds", velocityFilterSeconds);

        validateGearingAndDirections();
        validateEncoderCalibration();
        validateMechanicalLimits();
        validateTuning();
        validateServoCommandRange();
    }

    private void validateGearingAndDirections() {
        // Ratios and travel are positive magnitudes. Direction is handled
        // separately by servoSign and encoderSign, each either +1 or -1.
        if (turretPerServoRatio <= 0) {
            throw new IllegalArgumentException("turretPerServoRatio must be positive");
        }
        if (turretRadiansPerServoPosition <= 0) {
            throw new IllegalArgumentException("servoTravelDeg must be positive");
        }
        if (servoSign != 1 && servoSign != -1) {
            throw new IllegalArgumentException("servoSign must be +1 or -1");
        }
        if (encoderSign != 1 && encoderSign != -1) {
            throw new IllegalArgumentException("encoderSign must be +1 or -1");
        }
    }

    private void validateEncoderCalibration() {
        if (encoderFullScaleVolts <= 0) {
            throw new IllegalArgumentException("encoderFullScaleVolts must be positive");
        }
        if (encoderCenterVolts < 0 || encoderCenterVolts > encoderFullScaleVolts) {
            throw new IllegalArgumentException(
                    "encoderCenterVolts must be between 0 and encoderFullScaleVolts");
        }

        // The analog sensor repeats its reading every encoder revolution.
        // This class interprets readings within +/-180 ENCODER degrees of center.
        // Divide turret limits by the ratio to check that encoder-side range.
        // The endpoints must be strictly inside that interval to avoid ambiguity.
        double minimumEncoderAngle = minAngleRadians / turretPerServoRatio;
        double maximumEncoderAngle = maxAngleRadians / turretPerServoRatio;
        if (minimumEncoderAngle <= -Math.PI || maximumEncoderAngle >= Math.PI) {
            throw new IllegalArgumentException(
                    "Turret limits must stay within +/-180 encoder degrees of center");
        }
    }

    private void validateMechanicalLimits() {
        // Zero represents the centered turret. Both sides must have some travel.
        if (minAngleRadians >= 0 || maxAngleRadians <= 0) {
            throw new IllegalArgumentException(
                    "minAngleDeg must be negative and maxAngleDeg must be positive");
        }

        // The current target selector assumes every aiming direction is reachable.
        // It therefore needs at least 360 degrees of total mechanical travel.
        // Supporting less travel needs explicit handling for unreachable directions.
        double totalTravel = maxAngleRadians - minAngleRadians;
        double roundingAllowance = 1e-9; // Radians; floating-point rounding only.
        if (totalTravel < TAU - roundingAllowance) {
            throw new IllegalArgumentException("Total turret travel must be at least 360 degrees");
        }
        if (totalTravel >= 2 * TAU) {
            throw new IllegalArgumentException("This implementation requires less than 720 degrees of travel");
        }
    }

    private void validateTuning() {
        if (boundaryHysteresisRadians <= 0) {
            throw new IllegalArgumentException("boundaryHysteresisDeg must be positive");
        }
        if (alignmentToleranceRadians <= 0) {
            throw new IllegalArgumentException("alignmentToleranceDeg must be positive");
        }
        // Keep the boundary hold region larger than the allowed aiming error.
        if (boundaryHysteresisRadians <= alignmentToleranceRadians) {
            throw new IllegalArgumentException(
                    "boundaryHysteresisDeg must exceed alignmentToleranceDeg");
        }
        // Zero disables smoothing; negative filter time has no physical meaning.
        if (velocityFilterSeconds < 0) {
            throw new IllegalArgumentException("velocityFilterSeconds cannot be negative");
        }
    }

    private void validateServoCommandRange() {
        // The SDK accepts servo positions from 0 to 1. Check both mechanical
        // endpoints; the linear mapping then keeps every angle between them valid.
        // This also catches a center position that leaves insufficient travel.
        double positionAtMinimum = positionFor(minAngleRadians);
        double positionAtMaximum = positionFor(maxAngleRadians);
        if (positionAtMinimum < 0 || positionAtMinimum > 1) {
            throw new IllegalArgumentException(
                    "Minimum turret angle maps outside servo positions 0..1; check center/travel/ratio");
        }
        if (positionAtMaximum < 0 || positionAtMaximum > 1) {
            throw new IllegalArgumentException(
                    "Maximum turret angle maps outside servo positions 0..1; check center/travel/ratio");
        }
    }

    private static void requireFinite(String name, double value) {
        if (!finite(value)) {
            throw new IllegalArgumentException(name + " must be set to a finite number");
        }
    }

    /** Direction request: equivalent angles differing by 2*pi are allowed. */
    public void setTargetAngleRadians(double radians) {
        if (!finite(radians)) {
            throw new IllegalArgumentException("Nonfinite target");
        }
        requested = wrap(radians);
        enabled = true;
        // A new request must be processed by update before feed is permitted.
        aligned = false;
    }

    public void update() {
        update(System.nanoTime() * 1e-9);
    }

    /** Explicit monotonic time overload for deterministic tests. */
    public void update(double nowSeconds) {
        if (!finite(nowSeconds) || (finite(lastTime) && nowSeconds <= lastTime)) {
            throw new IllegalArgumentException("Time must increase");
        }
        double dt = finite(lastTime) ? nowSeconds - lastTime : 0;
        lastTime = nowSeconds;
        voltage = encoder.getVoltage();
        if (!finite(voltage) || voltage < 0 || voltage > encoderFullScaleVolts + 0.02) {
            fault();
            return;
        }
        // Convert volts -> encoder revolutions -> encoder radians from center.
        // Wrap on the ENCODER side first, then apply the speed-increasing gearing.
        // Example: +10 encoder degrees gives +17.45 turret degrees (sign = +1).
        double encoderRevolutionsFromCenter = (voltage - encoderCenterVolts) / encoderFullScaleVolts;
        double encoderAngleFromCenter = wrap(encoderRevolutionsFromCenter * TAU);
        double measured = encoderSign * turretPerServoRatio * encoderAngleFromCenter;
        if (measured < minAngleRadians - alignmentToleranceRadians || measured > maxAngleRadians + alignmentToleranceRadians) {
            fault();
            return;
        }
        // Ignore velocity across startup or a loop gap longer than 250 ms.
        if (hasSample && dt > 0 && dt <= 0.25) {
            // Do NOT wrap turret displacement: this is a bounded mechanism.
            double rawVelocity = (measured - angle) / dt;
            // A first-order low-pass filter blends the old estimate with the new one.
            double newSampleWeight = dt / (velocityFilterSeconds + dt);
            velocity += newSampleWeight * (rawVelocity - velocity);
        } else {
            velocity = 0;
        }
        angle = measured;
        hasSample = true;
        if (!enabled) {
            state = State.DISABLED;
            aligned = false;
            return;
        }

        // Equivalent directions differ by whole revolutions. Choose the valid one
        // closest to the measured physical angle, not the shortest circular error.
        double candidate = nearestValid(requested, angle);
        if (!hasTarget) {
            // First command after startup/stop: establish which equivalent to use.
            target = candidate;
            branchTarget = candidate;
            branchLocked = Math.abs(target - angle) > Math.PI;
            hasTarget = true;
        } else if (!branchLocked) {
            // Follow the previous command branch until it exceeds a limit by
            // hysteresis; this avoids 179.9/-179.9 noise causing repeated sweeps.
            double continuous = nearestEquivalent(requested, target);
            boolean crossing = Math.abs(candidate - target) > Math.PI;
            if (crossing && continuous >= minAngleRadians - boundaryHysteresisRadians
                    && continuous <= maxAngleRadians + boundaryHysteresisRadians) {
                target = clamp(continuous);
                state = State.BOUNDARY_HOLD;
                resetAlignment();
                servo.setPosition(positionFor(target));
                return;
            }
            target = candidate;
            if (crossing || Math.abs(target - angle) > Math.PI) {
                branchLocked = true;
                branchTarget = target;
            }
        } else {
            // Keep the chosen branch during the sweep. If the requested aim
            // retreats across the seam, hold this side's limit until it returns
            // sufficiently inside. Shooting remains blocked throughout.
            branchTarget = nearestEquivalent(requested, target);
            target = clamp(branchTarget);
        }

        // Physical error is NOT wrapped: +179 to -176 requires a -355 degree move.
        boolean reached = Math.abs(target - angle) <= alignmentToleranceRadians;
        // Release the unwind lock only after reaching a target beyond the hold band.
        // A target that stays in the boundary band can keep UNWINDING active even
        // after motion stops. This is the current conservative boundary behavior.
        boolean interior = target > minAngleRadians + boundaryHysteresisRadians && target < maxAngleRadians - boundaryHysteresisRadians;
        if (branchLocked && reached && interior
                && branchTarget >= minAngleRadians && branchTarget <= maxAngleRadians) {
            branchLocked = false;
        }
        state = branchLocked ? State.UNWINDING : State.TRACKING;
        servo.setPosition(positionFor(target));
        // Immediate readiness: no dwell timer and no requirement to stop moving.
        // A moving turret can be ready while accurately following an SOTM target.
        aligned = state == State.TRACKING && reached;
    }

    /** Stop issuing commands. The positional servo may still move to/hold its last command. */
    public void stop() {
        enabled = false;
        hasTarget = false;
        branchLocked = false;
        state = State.DISABLED;
        resetAlignment();
    }

    /** Immediate measured-angle check after update(); no dwell timer. */
    public boolean readyToLaunch() {
        return aligned && state == State.TRACKING;
    }
    public boolean atTarget() {
        return readyToLaunch();
    }
    public boolean isUnwinding() {
        return state == State.UNWINDING;
    }
    public State getState() {
        return state;
    }
    public double getAngleRadians() {
        return hasSample ? angle : Double.NaN;
    }
    public double getAngularVelocityRadiansPerSecond() {
        return hasSample ? velocity : Double.NaN;
    }
    public double getTargetAngleRadians() {
        return hasTarget ? target : Double.NaN;
    }
    public double getEncoderVoltage() {
        return voltage;
    }
    public double getErrorRadians() {
        return hasTarget && hasSample ? target - angle : Double.NaN;
    }

    // Linear calibration; no external PID correction is applied to this command.
    // With the defaults: position = 0.5 + turretDegrees / 471.15.
    private double positionFor(double radians) {
        return servoCenterPosition + servoSign * radians / turretRadiansPerServoPosition;
    }
    private double clamp(double radians) {
        return Math.max(minAngleRadians, Math.min(maxAngleRadians, radians));
    }
    private void resetAlignment() {
        aligned = false;
    }
    private void fault() {
        // Stop new writes and invalidate feedback. A subsequent setTarget call can
        // re-enable the class; a valid sensor sample is still required to command.
        // This does not disable PWM or detect every possible disconnected sensor.
        stop();
        hasSample = false;
        state = State.SENSOR_FAULT;
    }
    /** Select direction + k * 2*pi within the limits, nearest to the physical angle. */
    private double nearestValid(double direction, double reference) {
        double best = Double.NaN;
        double bestDistance = Double.POSITIVE_INFINITY;
        // Solve min <= direction + k*TAU <= max for integer k.
        // ceil rounds the lower bound up; floor rounds the upper bound down.
        // Tiny allowances preserve endpoints despite floating-point rounding.
        int first = (int) Math.ceil((minAngleRadians - direction) / TAU - 1e-12);
        int last = (int) Math.floor((maxAngleRadians - direction) / TAU + 1e-12);
        for (int k = first; k <= last; k++) {
            double candidate = clamp(direction + k * TAU);
            double distance = Math.abs(candidate - reference);
            if (distance < bestDistance) {
                best = candidate;
                bestDistance = distance;
            }
        }
        if (!finite(best)) {
            throw new IllegalStateException("No reachable equivalent angle");
        }
        return best;
    }
    /** Closest equivalent direction to a reference; it MAY be outside the limits. */
    private static double nearestEquivalent(double angle, double reference) {
        // Round the required revolution count to the nearest integer.
        return angle + TAU * Math.floor((reference - angle) / TAU + 0.5);
    }
    /** Normalize a DIRECTION into [-pi, pi), not a physical movement error. */
    private static double wrap(double angle) {
        double result = angle % TAU;
        if (result >= Math.PI) {
            result -= TAU;
        }
        if (result < -Math.PI) {
            result += TAU;
        }
        return result;
    }
    private static boolean finite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }
}
