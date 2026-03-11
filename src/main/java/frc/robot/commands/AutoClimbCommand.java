package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotState;
import frc.robot.constants.ClimbingConstants;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.swerve.SwerveDrive;
import org.littletonrobotics.junction.Logger;

public class AutoClimbCommand extends SequentialCommandGroup {
    private final SwerveDrive swerveDrive = SwerveDrive.getInstance();
    private final Superstructure superstructure = Superstructure.getInstance();
    private final Climber climber = Climber.getInstance();
    private final RobotState robotState = RobotState.getInstance();

    public AutoClimbCommand(ClimbingConstants.AutoClimbTarget target) {
        addRequirements(swerveDrive, superstructure, climber);
        addCommands(Commands.defer(() -> buildCommand(target), getRequirements()));
    }

    private Command buildCommand(ClimbingConstants.AutoClimbTarget target) {
        Pose2d currentPose = robotState.getEstimatedPose();
        AutoPaths.AutoClimbPlanningResult planningResult = AutoPaths.planAutoClimb(currentPose, target);

        Logger.recordOutput("AutoClimb/currentPose", currentPose);
        Logger.recordOutput("AutoClimb/targetName", target.name());

        if (!planningResult.isAccepted()) {
            return new InstantCommand(() -> reportRejectedPlan(planningResult.rejectReason()));
        }

        AutoPaths.AutoClimbPlan plan = planningResult.plan();
        SwerveDrive.DesiredSystemState restoreState = determineRestoreSwerveState();
        boolean shouldWaitForInitialExtension = shouldWaitForInitialExtension();

        Logger.recordOutput("AutoClimb/rejectionReason", "NONE");
        Logger.recordOutput("AutoClimb/shouldWaitForInitialExtension", shouldWaitForInitialExtension);
        Logger.recordOutput("AutoClimb/usesRetreatWaypoint", plan.usesRetreatWaypoint());
        Logger.recordOutput("AutoClimb/approachWaypointCount", plan.approachWaypoints().size());
        Logger.recordOutput("AutoClimb/finalWaypoint", plan.finalWaypoint());

        return Commands.sequence(
            new InstantCommand(() -> superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.HOME)),
            new InstantCommand(() -> superstructure.setDesiredIntakeState(Superstructure.DesiredIntakeState.STOWED)),
            new InstantCommand(() -> swerveDrive.setDesiredSystemState(SwerveDrive.DesiredSystemState.IDLE)),
            new InstantCommand(() -> superstructure.setDesiredClimbState(Superstructure.DesiredClimbState.EXTENDED)),
            shouldWaitForInitialExtension ? new WaitUntilCommand(superstructure::isClimbExtended) : Commands.none(),
            followPath(plan.approachPath()),
            new WaitUntilCommand(superstructure::isClimbExtended),
            followPath(plan.finalPath()),
            new InstantCommand(() -> superstructure.setDesiredClimbState(Superstructure.DesiredClimbState.CLIMBED)),
            new WaitUntilCommand(() -> superstructure.getCurrentClimbState() == Superstructure.CurrentClimbState.CLIMBED)
        ).finallyDo(interrupted -> {
            swerveDrive.setDesiredSystemState(restoreState);
            if (interrupted && superstructure.getDesiredClimbState() != Superstructure.DesiredClimbState.CLIMBED) {
                superstructure.setDesiredClimbState(Superstructure.DesiredClimbState.EXTENDED);
            }
        });
    }

    private Command followPath(Path path) {
        return new InstantCommand(() -> {
            swerveDrive.setCurrentPath(path, false);
            swerveDrive.setDesiredSystemState(SwerveDrive.DesiredSystemState.FOLLOW_PATH);
        }).andThen(
            new WaitUntilCommand(() -> swerveDrive.getCurrentSystemState() == SwerveDrive.CurrentSystemState.IDLE)
        );
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
}
