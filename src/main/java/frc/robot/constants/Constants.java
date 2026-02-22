// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final Mode currentMode = Mode.SIM; // TODO: change this if sim

    // public static final boolean isSYSID = true; // TODO: change this if sysid

    public static enum Mode {
        /** Running on a real robot. */
        COMP,

        DEV,

        /** Running a physics simulaton
         * r. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static final class SimOnlySubsystems {
        public static final boolean SWERVE = false;
        public static final boolean SHOOTER = false;
        public static final boolean KICKER = false;
        public static final boolean HOPPER = false;
        public static final boolean INTAKE = true;
        public static final boolean CLIMBER = true;
        public static final boolean VISION = false;

        private SimOnlySubsystems() {}
    }

    /**
     * Returns true if the subsystem should use simulation IO and config.
     * - If simOnlyFlag is false: use normal mode-based behavior
     * - If simOnlyFlag is true AND mode is REPLAY: use normal replay behavior (COMP config)
     * - If simOnlyFlag is true AND mode is NOT REPLAY: force simulation
     */
    public static boolean shouldUseSimulation(boolean simOnlyFlag) {
        if (!simOnlyFlag) {
            // Normal behavior: use simulation only in SIM mode
            return currentMode == Mode.SIM;
        }
        // Sim-only flag is enabled
        // In REPLAY mode, preserve normal replay behavior (don't force sim)
        if (currentMode == Mode.REPLAY) {
            return false;
        }
        // In all other modes (SIM, COMP, DEV), use simulation
        return true;
    }

    public static final double kLOOP_CYCLE_MS;
    static {
        switch (currentMode) {
            case COMP:
                kLOOP_CYCLE_MS = 0.02;

                break;

            case DEV:
                kLOOP_CYCLE_MS = 0.02;

                break;

            case SIM:
                kLOOP_CYCLE_MS = 0.02;

                break;

            case REPLAY:
                kLOOP_CYCLE_MS = 0.02;

                break;

            default:
                kLOOP_CYCLE_MS = 0.02;

                break;
        }
    }

    public static final class OperatorConstants {
        public static final int kDRIVER_CONTROLLER_PORT = 0;

        // Joystick Deadband
        // yes, this high :sob:
        public static final double LEFT_X_DEADBAND = 0.09;
        public static final double LEFT_Y_DEADBAND = 0.09;

        public static final double RIGHT_X_DEADBAND = 0.12;

        private OperatorConstants() {}
    }

    public static boolean shouldFlipPath() {
        var alliance = DriverStation.getAlliance();

        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }

        return false;
    }
}
