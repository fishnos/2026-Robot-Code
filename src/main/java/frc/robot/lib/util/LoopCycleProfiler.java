package frc.robot.lib.util;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

/**
 * Lightweight loop profiler for timing robot and subsystem periodic sections.
 */
public final class LoopCycleProfiler {
    private static final double NANOS_TO_MILLIS = 1.0e-6;
    private static final double WARNING_MIN_INTERVAL_SECONDS = 0.5;
    private static final int MAX_SLOW_LOOP_SECTIONS_IN_WARNING = 8;

    private static final Map<String, Double> sectionDurationsMs = new LinkedHashMap<>();

    private static long cycleStartNanos = System.nanoTime();
    private static long cycleIndex = 0;
    private static double lastWarningTimestampSeconds = Double.NEGATIVE_INFINITY;
    private static double lastLoggedSlowCycleThresholdMs = Double.NaN;

    private LoopCycleProfiler() {
    }

    public static void beginCycle() {
        sectionDurationsMs.clear();
        cycleStartNanos = System.nanoTime();
        cycleIndex++;
        Logger.recordOutput("LoopProfiler/CycleIndex", cycleIndex);
    }

    public static long markStart() {
        return System.nanoTime();
    }

    public static double endSection(String sectionName, long sectionStartNanos) {
        double durationMs = (System.nanoTime() - sectionStartNanos) * NANOS_TO_MILLIS;
        sectionDurationsMs.merge(sectionName, durationMs, Double::sum);
        Logger.recordOutput("LoopProfiler/" + sectionName + "Ms", durationMs);
        return durationMs;
    }

    public static void finishCycle(double slowCycleThresholdMs) {
        double totalMs = (System.nanoTime() - cycleStartNanos) * NANOS_TO_MILLIS;
        Logger.recordOutput("LoopProfiler/TotalMs", totalMs);
        if (Double.compare(lastLoggedSlowCycleThresholdMs, slowCycleThresholdMs) != 0) {
            Logger.recordOutput("LoopProfiler/SlowCycleThresholdMs", slowCycleThresholdMs);
            lastLoggedSlowCycleThresholdMs = slowCycleThresholdMs;
        }

        boolean isSlowCycle = totalMs >= slowCycleThresholdMs;
        Logger.recordOutput("LoopProfiler/IsSlowCycle", isSlowCycle);

        if (!isSlowCycle) {
            return;
        }

        String warning = buildSlowCycleWarning(totalMs);
        Logger.recordOutput("LoopProfiler/SlowCycleSummary", warning);
        double nowSeconds = Timer.getFPGATimestamp();
        if (nowSeconds - lastWarningTimestampSeconds >= WARNING_MIN_INTERVAL_SECONDS) {
            DriverStation.reportWarning(warning, false);
            lastWarningTimestampSeconds = nowSeconds;
        }
    }

    private static String buildSlowCycleWarning(double totalMs) {
        List<Map.Entry<String, Double>> sortedSections = new ArrayList<>(sectionDurationsMs.entrySet());
        sortedSections.sort((first, second) -> Double.compare(second.getValue(), first.getValue()));

        int sectionLimit = Math.min(MAX_SLOW_LOOP_SECTIONS_IN_WARNING, sortedSections.size());
        StringBuilder warning = new StringBuilder();
        warning.append(String.format(Locale.US, "Slow loop %.2f ms; top sections: ", totalMs));
        for (int index = 0; index < sectionLimit; index++) {
            Map.Entry<String, Double> entry = sortedSections.get(index);
            if (index > 0) {
                warning.append(", ");
            }
            warning.append(entry.getKey());
            warning.append('=');
            warning.append(String.format(Locale.US, "%.2fms", entry.getValue()));
        }
        return warning.toString();
    }
}
