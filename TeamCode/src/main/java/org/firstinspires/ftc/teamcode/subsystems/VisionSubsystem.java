package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/**
 * Relative 3D AprilTag targeting for the two blue BIOBUZZ CELLS.
 * No field map or MegaTag2 required. All vectors returned in robot-relative
 * forward/left/up axes, measured in METERS from the CAMERA, not robot center.
 *
 * This first version averages only the visible tag CENTERS, not the full
 * four-tag cluster's geometric center when tags are missing.
 */
public class VisionSubsystem {
    public enum Goal { AUDIENCE, FAR }
    public enum SelectionMode { MOST_TAGS, FIELD_HALF, AUDIENCE_ONLY, FAR_ONLY }

    /** Older Limelight OS camera coordinates are right/down/forward (X/Y/Z).
     * Firmware using NWU uses forward/left/up (X/Y/Z) instead.
     * Check your actual firmware before changing this setting.
     */
    public enum CameraFrame { LEGACY_OPTICAL, NWU }

    public static int PIPELINE = 0;
    public static CameraFrame CAMERA_FRAME = CameraFrame.LEGACY_OPTICAL;

    // Measured physical camera mounting pitch. Positive = optical axis upward.
    // Do not use the Limelight web UI's sign without checking its convention.
    public static double CAMERA_PITCH_UP_DEG = 40.0;

    public static long MAX_STALENESS_MS = 250;
    public static double MIN_RANGE_M = 0.15;
    public static double MAX_RANGE_M = 10.0;

    // Only used in FIELD_HALF mode. Verify which half corresponds to each
    // CELL for your Pedro coordinate orientation before enabling this mode.
    public static double FIELD_SPLIT_Y_IN = 72.0;
    public static Goal LOWER_HALF_GOAL = Goal.AUDIENCE;
    public static Goal UPPER_HALF_GOAL = Goal.FAR;

    public static final class AimObservation {
        public final Goal goal;
        public final int tagCount;
        public final int audienceCount;
        public final int farCount;
        public final String visibleIds;
        public final double forwardM;
        public final double leftM;
        public final double upM;
        public final double totalArea;

        private AimObservation(Goal goal, int tagCount,
                               int audienceCount, int farCount, String visibleIds,
                               double forwardM, double leftM, double upM,
                               double totalArea) {
            this.goal = goal;
            this.tagCount = tagCount;
            this.audienceCount = audienceCount;
            this.farCount = farCount;
            this.visibleIds = visibleIds;
            this.forwardM = forwardM;
            this.leftM = leftM;
            this.upM = upM;
            this.totalArea = totalArea;
        }

        public double horizontalDistanceM() {
            return Math.hypot(forwardM, leftM);
        }
    }

    private static final class Accumulator {
        private final boolean[] seen = new boolean[46];
        int count;
        double forward, left, up, area;
        StringBuilder ids = new StringBuilder();

        void add(int id, double f, double l, double u, double a) {
            // Ignore duplicate detections of an ID, if any.
            if (seen[id]) return;
            seen[id] = true;
            count++;
            forward += f;
            left += l;
            up += u;
            if (Double.isFinite(a) && a > 0) area += a;
            if (ids.length() > 0) ids.append(',');
            ids.append(id);
        }

        AimObservation observation(Goal goal, int audienceCount, int farCount) {
            if (count == 0) return null;
            return new AimObservation(goal, count, audienceCount, farCount,
                    ids.toString(), forward / count, left / count, up / count, area);
        }
    }

    private final Limelight3A limelight;

    public VisionSubsystem(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(PIPELINE);
        limelight.start();
    }

    /**
     * Returns null when there is no fresh usable observation of the chosen
     * goal. FIELD_HALF needs a trustworthy Pedro field Y; other modes don't.
     * Only calls getLatestResult() ONCE per invocation, keeping all tags in
     * the selection from the same camera frame.
     */
    public AimObservation getBlueGoal(SelectionMode mode, double robotYIn) {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;

        long staleness = result.getStaleness(); // FTC SDK reports milliseconds
        if (staleness < 0 || staleness > MAX_STALENESS_MS) return null;

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) return null;

        Accumulator audience = new Accumulator();
        Accumulator far = new Accumulator();
        final double pitch = Math.toRadians(CAMERA_PITCH_UP_DEG);
        final double cos = Math.cos(pitch);
        final double sin = Math.sin(pitch);

        for (LLResultTypes.FiducialResult tag : tags) {
            int id = tag.getFiducialId();
            if (id < 38 || id > 45) continue; // blue only

            Pose3D target = tag.getTargetPoseCameraSpace();
            if (target == null || target.getPosition() == null) continue;

            double x = target.getPosition().x;
            double y = target.getPosition().y;
            double z = target.getPosition().z;
            if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(z)) continue;

            // Camera-space -> horizontal robot-relative vector.
            // No camera yaw or roll; user mounted it straight on the centerline.
            // Rotate out the measured pitch (positive optical axis up).
            double f, l, u;
            if (CAMERA_FRAME == CameraFrame.LEGACY_OPTICAL) {
                // Classic Limelight camera: X right, Y down, Z forward.
                f = z * cos + y * sin;
                l = -x;
                u = z * sin - y * cos;
            } else {
                // NWU camera: X forward, Y left, Z up.
                f = x * cos - z * sin;
                l = y;
                u = x * sin + z * cos;
            }

            double range = Math.sqrt(f*f + l*l + u*u);
            if (!Double.isFinite(range) || range < MIN_RANGE_M || range > MAX_RANGE_M)
                continue;
            if (f <= 0.05) continue; // Reject tags behind/at the camera

            if (id <= 41) audience.add(id, f, l, u, tag.getTargetArea());
            else far.add(id, f, l, u, tag.getTargetArea());
        }

        AimObservation a = audience.observation(Goal.AUDIENCE, audience.count, far.count);
        AimObservation f = far.observation(Goal.FAR, audience.count, far.count);
        if (mode == null) mode = SelectionMode.MOST_TAGS;

        switch (mode) {
            case AUDIENCE_ONLY: return a;
            case FAR_ONLY: return f;
            case FIELD_HALF:
                if (!Double.isFinite(robotYIn)) return null;
                Goal preferred = robotYIn < FIELD_SPLIT_Y_IN ? LOWER_HALF_GOAL : UPPER_HALF_GOAL;
                return preferred == Goal.AUDIENCE ? a : f;
            case MOST_TAGS:
            default:
                if (a == null) return f;
                if (f == null) return a;
                if (a.tagCount != f.tagCount) return a.tagCount > f.tagCount ? a : f;
                return a.totalArea >= f.totalArea ? a : f;
        }
    }

    public void stop() {
        limelight.stop();
    }
}
