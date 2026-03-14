package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.DesiredClimbState;
import frc.robot.subsystems.Superstructure.DesiredIntakeState;
import frc.robot.subsystems.Superstructure.DesiredSystemState;
import frc.robot.subsystems.swerve.SwerveDrive;
import java.util.List;

public final class Autos {
    private Autos() {}

    public static SendableChooser<Command> getChooser() {
        SendableChooser<Command> chooser = new SendableChooser<>();
        chooser.setDefaultOption("Do Nothing", Commands.none());

        for (AutoDefinition autoDefinition : AUTOS) {
            chooser.addOption(autoDefinition.name(), autoDefinition.command());
        }

        return chooser;
    }

    private static final List<AutoDefinition> AUTOS = List.of(
        auto(
            "Simple Back",
            setSystem(DesiredSystemState.HOME),
            setIntake(DesiredIntakeState.STOWED),
            setClimber(DesiredClimbState.RETRACTED),
            followPath("simple_back", true)
        ),
        auto(
            "Citrus Sweep",
            setSystem(DesiredSystemState.HOME),
            setIntake(DesiredIntakeState.DEPLOYED),
            setClimber(DesiredClimbState.RETRACTED),
            followPath("citrus_sweep"),
            waitSeconds(0.25),
            setIntake(DesiredIntakeState.STOWED)
        )
    );

    private static AutoDefinition auto(String name, Command... commands) {
        SequentialCommandGroup auto = new SequentialCommandGroup(commands);
        auto.setName(name);
        return new AutoDefinition(name, auto);
    }

    public static Command setIntake(DesiredIntakeState intakeState) {
        return new InstantCommand(() -> Superstructure.getInstance().setDesiredIntakeState(intakeState));
    }

    public static Command setSystem(DesiredSystemState systemState) {
        return new InstantCommand(() -> Superstructure.getInstance().setDesiredSystemState(systemState));
    }

    public static Command setClimber(DesiredClimbState climbState) {
        return new InstantCommand(() -> Superstructure.getInstance().setDesiredClimbState(climbState));
    }

    public static Command followPath(String pathName) {
        return followPath(pathName, false, false);
    }

    public static Command followPath(String pathName, boolean shouldResetPose) {
        return followPath(pathName, shouldResetPose, false);
    }

    public static Command followPath(String pathName, boolean shouldResetPose, boolean shouldMirrorPath) {
        return SwerveDrive.getInstance().followPathCommand(new Path(pathName), shouldResetPose, shouldMirrorPath);
    }

    public static Command waitSeconds(double seconds) {
        return new WaitCommand(seconds);
    }

    private record AutoDefinition(String name, Command command) {}
}
