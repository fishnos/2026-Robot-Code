package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotState;
import frc.robot.commands.autos.AutoPaths;
import frc.robot.constants.ClimbingConstants;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.swerve.SwerveDrive;
import java.util.Set;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class AutoClimbCommand extends SequentialCommandGroup {
    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();
    private final Superstructure superstructure = Superstructure.getInstance();
    private final RobotState robotState = RobotState.getInstance();
    private String lastStage = "IDLE";

    public AutoClimbCommand(ClimbingConstants.AutoClimbTarget target) {
        addCommands(Commands.defer(() -> buildCommand(target), Set.of()));
    }

    private Command buildCommand(ClimbingConstants.AutoClimbTarget target) {
        Pose2d currentPose = robotState.getEstimatedPose();
        AutoPaths.AutoClimbPlanningResult planningResult = AutoPaths.planAutoClimb(currentPose, target);

        Logger.recordOutput("AutoClimb/currentPose", currentPose);
        Logger.recordOutput("AutoClimb/targetName", target.name());

        if (!planningResult.isAccepted()) {
            recordStage("PLAN_REJECTED");
            return new InstantCommand(() -> reportRejectedPlan(planningResult.rejectReason()));
        }

        AutoPaths.AutoClimbPlan plan = planningResult.plan();
        SwerveDrive.DesiredSystemState restoreState = determineRestoreSwerveState();
        boolean shouldWaitForInitialExtension = shouldWaitForInitialExtension();

        Logger.recordOutput("AutoClimb/rejectionReason", "NONE");
        Logger.recordOutput("AutoClimb/selectedSideName", plan.side().name());
        Logger.recordOutput("AutoClimb/selectedSideEstimatedDistanceMeters", plan.estimatedDistanceMeters());
        Logger.recordOutput("AutoClimb/shouldWaitForInitialExtension", shouldWaitForInitialExtension);
        Logger.recordOutput("AutoClimb/approachWaypointCount", plan.approachWaypoints().size());
        Logger.recordOutput("AutoClimb/finalWaypoint", plan.finalWaypoint());
        Logger.recordOutput("AutoClimb/approachPathIsValid", plan.approachPath().isValid());
        Logger.recordOutput("AutoClimb/finalPathIsValid", plan.finalPath().isValid());
        recordStage("PLAN_ACCEPTED");

        return Commands.sequence(
            logStageCommand("SEQUENCE/START"),
            runLoggedAction("SET_HOME", () -> superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.HOME)),
            // new InstantCommand(() -> superstructure.setDesiredIntakeState(Superstructure.DesiredIntakeState.STOWED)),
            runLoggedAction("SET_SWERVE_IDLE", () -> swerveDrive.setDesiredSystemState(SwerveDrive.DesiredSystemState.IDLE)),
            runLoggedAction(
                "REQUEST_CLIMB_EXTENSION",
                () -> superstructure.setDesiredClimbState(Superstructure.DesiredClimbState.EXTENDED)
            ),
            shouldWaitForInitialExtension
                ? waitUntilLogged("WAIT_FOR_INITIAL_EXTENSION", superstructure::isClimbExtended)
                : logStageCommand("WAIT_FOR_INITIAL_EXTENSION/SKIPPED"),
            traceCommand("FOLLOW_APPROACH_PATH", followPath(plan.approachPath())),
            waitUntilLogged("WAIT_FOR_EXTENSION_BEFORE_FINAL_PATH", superstructure::isClimbExtended),
            traceCommand("FOLLOW_FINAL_PATH", followPath(plan.finalPath())),
            runLoggedAction(
                "REQUEST_CLIMBED_STATE",
                () -> superstructure.setDesiredClimbState(Superstructure.DesiredClimbState.CLIMBED)
            ),
            waitUntilLogged(
                "WAIT_FOR_CLIMBED_STATE",
                () -> superstructure.getCurrentClimbState() == Superstructure.CurrentClimbState.CLIMBED
            ),
            logStageCommand("SEQUENCE/COMPLETE")
        ).finallyDo(interrupted -> {
            recordStage(interrupted ? "SEQUENCE/INTERRUPTED" : "SEQUENCE/FINALLY");
            swerveDrive.setDesiredSystemState(restoreState);
            if (interrupted && superstructure.getDesiredClimbState() != Superstructure.DesiredClimbState.CLIMBED) {
                superstructure.setDesiredClimbState(Superstructure.DesiredClimbState.EXTENDED);
            }

            Logger.recordOutput("AutoClimb/finallyInterrupted", interrupted);
            Logger.recordOutput("AutoClimb/restoreSwerveState", restoreState.name());
            recordStage(interrupted ? "SEQUENCE/RESTORED_INTERRUPTED" : "SEQUENCE/RESTORED");

            if (!interrupted && superstructure.getCurrentClimbState() != Superstructure.CurrentClimbState.CLIMBED) {
                DriverStation.reportWarning(
                    "Auto climb finished before CLIMBED. lastStage=" + lastStage,
                    false
                );
            }
        });
    }

    private Command traceCommand(String stage, Command command) {
        return logStageCommand(stage + "/START")
            .andThen(command)
            .finallyDo(interrupted -> recordStage(stage + (interrupted ? "/INTERRUPTED" : "/END")));
    }

    private Command waitUntilLogged(String stage, BooleanSupplier condition) {
        return traceCommand(stage, new WaitUntilCommand(condition));
    }

    private Command runLoggedAction(String stage, Runnable action) {
        return new InstantCommand(() -> {
            recordStage(stage + "/START");
            action.run();
            recordStage(stage + "/END");
        });
    }

    private Command logStageCommand(String stage) {
        return new InstantCommand(() -> recordStage(stage));
    }

    private Command followPath(Path path) {
        return swerveDrive.followPathCommand(path, false, false);
    }

    private boolean shouldWaitForInitialExtension() {
        return superstructure.getCurrentClimbState() == Superstructure.CurrentClimbState.CLIMBED
            || superstructure.getDesiredClimbState() == Superstructure.DesiredClimbState.CLIMBED;
    }

    private SwerveDrive.DesiredSystemState determineRestoreSwerveState() {
        SwerveDrive.DesiredSystemState currentDesiredState = swerveDrive.getDesiredSystemState();
        return currentDesiredState == SwerveDrive.DesiredSystemState.FOLLOW_PATH
            ? SwerveDrive.DesiredSystemState.IDLE
            : currentDesiredState;
    }

    private void reportRejectedPlan(AutoPaths.AutoClimbRejectReason rejectReason) {
        String message = "Auto climb rejected: " + rejectReason;
        DriverStation.reportWarning(message, false);
        Logger.recordOutput("AutoClimb/rejectionReason", rejectReason.toString());
    }

    private void recordStage(String stage) {
        lastStage = stage;
        Logger.recordOutput("AutoClimb/stage", stage);
        Logger.recordOutput("AutoClimb/stageTimestampSec", Timer.getTimestamp());
        Logger.recordOutput("AutoClimb/poseAtStage", robotState.getEstimatedPose());
        Logger.recordOutput("AutoClimb/swerveDesiredStateAtStage", swerveDrive.getDesiredSystemState().name());
        Logger.recordOutput("AutoClimb/swerveCurrentStateAtStage", swerveDrive.getCurrentSystemState().name());
        Logger.recordOutput("AutoClimb/swerveHasCurrentPathCommandAtStage", swerveDrive.hasCurrentPathCommand());
        Logger.recordOutput("AutoClimb/swerveCurrentPathCommandScheduledAtStage", swerveDrive.isCurrentPathCommandScheduled());
        Logger.recordOutput("AutoClimb/swerveCurrentPathCommandFinishedAtStage", swerveDrive.isCurrentPathCommandFinished());
        Logger.recordOutput("AutoClimb/currentClimbStateAtStage", superstructure.getCurrentClimbState().name());
        Logger.recordOutput("AutoClimb/desiredClimbStateAtStage", superstructure.getDesiredClimbState().name());
        Logger.recordOutput("AutoClimb/isClimbExtendedAtStage", superstructure.isClimbExtended());
    }
}
