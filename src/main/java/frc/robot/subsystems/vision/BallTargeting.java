package frc.robot.subsystems.vision;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.vision.VisionIO.BallObservation;

/**
 * BallTargeting — reads neural detector output and provides
 * the best ball target for the driver or autonomous systems.
 *
 * Usage:
 * BallTargeting.getInstance().getBestBall() → BallObservation or null
 * BallTargeting.getInstance().hasBall() → boolean
 * BallTargeting.getInstance().getTxToClosestBall() → double (degrees)
 *
 * Wire into Vision.periodic() by calling update() with the latest inputs.
 */
public class BallTargeting {
    // Eagerly initialized singleton instance for thread safety and simplicity
    private static final BallTargeting instance = new BallTargeting();

    // Configuration constants
    private static final double MIN_CONFIDENCE = 0.50; // Ignore detections below this confidence level
    private static final double ACCEPTANCE_CONE_DEG = 25.0; // Degrees for collection cone, influenced by robot heading
    private static final double DRIVE_CORRECTION_KP = 0.01; // Proportional gain for steering toward ball
    private static final double LATERAL_KP = 0.02; // Lateral strafe gain per degree of tx, multiplied by chassis speed

    private BallObservation bestBall = null;
    private Supplier<Double> driverHeadingSupplier = () -> 0.0;
    private boolean assistEnabled = false;

    private BallTargeting() {
        // Private constructor to enforce singleton pattern
    }

    public static BallTargeting getInstance() {
        return instance;
    }

    /**
     * Call this from Vision.periodic() with the latest camera inputs.
     * Pass the index of whichever camera has the neural detector.
     *
     * @param inputs      Array of auto-logged vision inputs.
     * @param cameraIndex The index corresponding to the neural detector camera.
     */
    public void update(VisionIOInputsAutoLogged[] inputs, int cameraIndex) {
        if (inputs == null || cameraIndex < 0 || cameraIndex >= inputs.length) {
            bestBall = null;
            publishToNT();
            return;
        }

        BallObservation[] detections = inputs[cameraIndex].ballObservations;
        bestBall = selectBestBall(detections);
        publishToNT();
    }

    /**
     * Sets the supplier for the driver's intended heading (in degrees relative to
     * the robot).
     * Used to filter balls that the driver is not aiming towards.
     *
     * @param supplier A supplier returning the driver's heading intent in degrees.
     */
    public void setDriverHeadingSupplier(Supplier<Double> supplier) {
        this.driverHeadingSupplier = supplier != null ? supplier : () -> 0.0;
    }

    /**
     * Selects the best ball to target.
     * Strategy: highest area (closest) above confidence threshold, within the
     * driver's intent cone.
     *
     * @param detections The array of current ball observations.
     * @return The best BallObservation, or null if none meet the criteria.
     */
    private BallObservation selectBestBall(BallObservation[] detections) {
        if (detections == null || detections.length == 0) {
            return null;
        }

        double driverHeadingDeg = driverHeadingSupplier.get();
        BallObservation best = null;

        for (BallObservation obs : detections) {
            if (obs.confidence() < MIN_CONFIDENCE) {
                continue;
            }

            // Calculate angle difference properly, accounting for potential wrap-around
            // using WPILib's MathUtil (assuming inputs are in degrees)
            double angleDiff = Math.toDegrees(
                    Math.abs(MathUtil.angleModulus(Math.toRadians(obs.tx() - driverHeadingDeg))));

            if (angleDiff > ACCEPTANCE_CONE_DEG) {
                continue;
            }

            if (best == null || obs.area() > best.area()) {
                best = obs;
            }
        }

        return best;
    }

    // -------------------------------------------------------
    // Public interface for drive/command code
    // -------------------------------------------------------

    /**
     * @return True if a ball is currently detected above confidence threshold and
     *         within the acceptance cone.
     */
    public boolean hasBall() {
        return bestBall != null;
    }

    /**
     * @return The best ball observation, or null if none detected.
     */
    public BallObservation getBestBall() {
        return bestBall;
    }

    /**
     * Horizontal angle to closest valid ball in degrees.
     * Negative = ball is to the left.
     * Returns 0.0 if no ball detected — always check hasBall() first.
     *
     * @return The horizontal angle offset (tx) to the best ball.
     */
    public double getTxToClosestBall() {
        return bestBall != null ? bestBall.tx() : 0.0;
    }

    /**
     * Normalized drive correction value (-1.0 to 1.0) for steering toward the ball.
     * Feed this into your swerve rotation input.
     *
     * @return A clamped rotation control effort.
     */
    public double getDriveCorrection() {
        if (bestBall == null) {
            return 0.0;
        }

        double correction = bestBall.tx() * DRIVE_CORRECTION_KP;
        return MathUtil.clamp(correction, -1.0, 1.0);
    }

    /**
     * 2056-style assisted lateral correction.
     * Returns a robot-centric Y velocity (m/s) to nudge the robot laterally
     * toward the detected ball. The magnitude scales with chassis speed so the
     * correction feels proportional to how fast the driver is driving.
     *
     * @param chassisSpeedMps The current translational speed of the chassis (m/s).
     * @return Lateral velocity correction in m/s. Positive = strafe right, negative
     *         = strafe left.
     *         Returns 0 if no ball detected or assist is disabled.
     */
    public double getLateralCorrectionMps(double chassisSpeedMps) {
        if (!isAssistActive()) {
            return 0.0;
        }
        // tx is positive when the ball is to the right of center
        return bestBall.tx() * LATERAL_KP * chassisSpeedMps;
    }

    /**
     * Enables or disables the vision-assisted lateral strafe.
     * Typically bound to the intake button — assist is active only while held.
     *
     * @param enabled True to enable, false to disable.
     */
    public void setAssistEnabled(boolean enabled) {
        this.assistEnabled = enabled;
    }

    /**
     * @return True if the assist is enabled AND a ball is currently detected.
     */
    public boolean isAssistActive() {
        return assistEnabled && hasBall();
    }

    // -------------------------------------------------------
    // Telemetry publishing for dashboard/debugging
    // -------------------------------------------------------
    private void publishToNT() {
        Logger.recordOutput("Vision/BallTargeting/HasBall", hasBall());
        Logger.recordOutput("Vision/BallTargeting/TxDegrees", getTxToClosestBall());
        Logger.recordOutput("Vision/BallTargeting/DriveCorrection", getDriveCorrection());
        Logger.recordOutput("Vision/BallTargeting/AssistEnabled", assistEnabled);
        Logger.recordOutput("Vision/BallTargeting/AssistActive", isAssistActive());

        if (bestBall != null) {
            Logger.recordOutput("Vision/BallTargeting/Area", bestBall.area());
            Logger.recordOutput("Vision/BallTargeting/Confidence", bestBall.confidence());
        } else {
            Logger.recordOutput("Vision/BallTargeting/Area", 0.0);
            Logger.recordOutput("Vision/BallTargeting/Confidence", 0.0);
        }
    }
}
