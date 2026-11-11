package org.firstinspires.ftc.teamcode.utils;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * Read-only electrical monitoring for Panels 1.x. Requires Panels Telemetry.
 * Construct once per OpMode, then call update() from the normal OpMode loop.
 * Does not change motor direction, mode, power, or bulk caching settings.
 * Sends live traces only; does not save a CSV.
 */
public final class PowerMonitor {
    private static final String[] DRIVE = {
            "frontLeft", "frontRight", "backLeft", "backRight"
    };
    private static final String[] INTAKE = {"intakeMotor", "intakeMotor2"};
    private static final String[] SHOOTER = {"motor_one", "motor_two"};

    private final Map<String, DcMotorEx> motors = new LinkedHashMap<>();
    private final List<LynxModule> hubs;
    private final List<String> hubLabels = new ArrayList<>();
    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    private final long intervalNanos;
    private final long startNanos = System.nanoTime();
    private long lastSampleNanos;
    private boolean sampled;

    /** Defaults to at most 10 samples per second. */
    public PowerMonitor(HardwareMap hardwareMap) {
        this(hardwareMap, 100);
    }

    public PowerMonitor(HardwareMap hardwareMap, long intervalMs) {
        if (intervalMs <= 0 || intervalMs > Long.MAX_VALUE / 1_000_000L) {
            throw new IllegalArgumentException("intervalMs must be positive and in range");
        }
        intervalNanos = intervalMs * 1_000_000L;
        addMotors(hardwareMap, DRIVE);
        addMotors(hardwareMap, INTAKE);
        addMotors(hardwareMap, SHOOTER);

        hubs = hardwareMap.getAll(LynxModule.class);
        for (int i = 0; i < hubs.size(); i++) {
            Set<String> names = hardwareMap.getNamesOf(hubs.get(i));
            String name = names.isEmpty() ? "unnamed" : names.iterator().next();
            hubLabels.add("hub/" + i + "_" + name);
        }
    }

    private void addMotors(HardwareMap hardwareMap, String[] names) {
        for (String name : names) {
            motors.put(name, hardwareMap.get(DcMotorEx.class, name));
        }
    }

    /** Call every loop; actual hardware reads are rate-limited internally. */
    public void update() {
        update(true);
    }

    /**
     * Pass false if your OpMode already calls the shared Panels manager's update().
     * In that case, call this BEFORE that existing telemetry update.
     */
    public void update(boolean sendTelemetry) {
        long now = System.nanoTime();
        if (sampled && now - lastSampleNanos < intervalNanos) return;

        panelsTelemetry.addData("electrical/time_s", (now - startNanos) / 1e9);
        if (sampled) {
            panelsTelemetry.addData("electrical/sampleInterval_ms", (now - lastSampleNanos) / 1e6);
        }
        sampled = true;
        lastSampleNanos = now;

        Map<String, Double> currents = new LinkedHashMap<>();
        for (Map.Entry<String, DcMotorEx> entry : motors.entrySet()) {
            double amps = entry.getValue().getCurrent(CurrentUnit.AMPS);
            currents.put(entry.getKey(), amps);
            panelsTelemetry.addData("motor/" + entry.getKey() + "/current_A", amps);
            // Commanded output (-1 to 1), NOT electrical power in watts.
            panelsTelemetry.addData("motor/" + entry.getKey() + "/command", entry.getValue().getPower());
        }

        panelsTelemetry.addData("subsystem/drive/current_A", sum(currents, DRIVE));
        panelsTelemetry.addData("subsystem/intake/current_A", sum(currents, INTAKE));
        panelsTelemetry.addData("subsystem/shooter/current_A", sum(currents, SHOOTER));

        double totalAmps = 0;
        double totalWatts = 0;
        double minVolts = Double.POSITIVE_INFINITY;
        for (int i = 0; i < hubs.size(); i++) {
            LynxModule hub = hubs.get(i);
            double volts = hub.getInputVoltage(VoltageUnit.VOLTS);
            double amps = hub.getCurrent(CurrentUnit.AMPS);
            double watts = volts * amps;
            String label = hubLabels.get(i);
            panelsTelemetry.addData(label + "/voltage_V", volts);
            panelsTelemetry.addData(label + "/current_A", amps);
            panelsTelemetry.addData(label + "/power_W", watts);
            totalAmps += amps;
            totalWatts += watts;
            minVolts = Math.min(minVolts, volts);
        }

        // Hub-based estimate: separate battery-powered loads may be excluded.
        // Do NOT add motor current again; it is already part of hub current.
        panelsTelemetry.addData("hubs/totalCurrent_A", totalAmps);
        panelsTelemetry.addData("hubs/totalPower_W", totalWatts);
        if (!hubs.isEmpty()) panelsTelemetry.addData("hubs/minVoltage_V", minVolts);
        panelsTelemetry.addData("electrical/readDuration_ms", (System.nanoTime() - now) / 1e6);
        if (sendTelemetry) panelsTelemetry.update();
    }

    private static double sum(Map<String, Double> currents, String[] names) {
        double total = 0;
        for (String name : names) total += currents.get(name);
        return total;
    }
}
