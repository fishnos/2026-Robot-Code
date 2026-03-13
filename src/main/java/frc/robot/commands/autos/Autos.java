package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.TargetState;
import frc.robot.subsystems.swerve.SwerveDrive;

public final class Autos {
    private Autos() {}

    public static SendableChooser<Command> createChooser(
        SwerveDrive swerveDrive,
        Superstructure superstructure
    ) {
        SendableChooser<Command> chooser = new SendableChooser<>();
        chooser.setDefaultOption("Citrus Sweep", citrusSweep(swerveDrive, superstructure));
        chooser.addOption("Do Nothing", Commands.none());
        chooser.addOption("Citrus Sweep Mirrored", citrusSweepMirrored(swerveDrive, superstructure));
        chooser.addOption("Simple Back", simpleBack(swerveDrive, superstructure));
        chooser.addOption("Outpost To Tower", outpostToTower(swerveDrive, superstructure));
        return chooser;
    }

    public static Command citrusSweep(SwerveDrive swerveDrive, Superstructure superstructure) {
        SequentialCommandGroup auto = new SequentialCommandGroup(
            new InstantCommand(() -> superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.HOME)),
            new InstantCommand(() -> superstructure.setDesiredIntakeState(Superstructure.DesiredIntakeState.DEPLOYED)),
            swerveDrive.followPathCommand(new Path("citrus_sweep"), false, false),
            new WaitCommand(0.25),
            new InstantCommand(() -> superstructure.setDesiredIntakeState(Superstructure.DesiredIntakeState.STOWED))
        );
        auto.setName("Citrus Sweep");
        return auto;
    }

    public static Command citrusSweepMirrored(SwerveDrive swerveDrive, Superstructure superstructure) {
        SequentialCommandGroup auto = new SequentialCommandGroup(
            new InstantCommand(() -> superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.HOME)),
            new InstantCommand(() -> superstructure.setDesiredIntakeState(Superstructure.DesiredIntakeState.DEPLOYED)),
            swerveDrive.followPathCommand(new Path("citrus_sweep"), false, true),
            new WaitCommand(0.25),
            new InstantCommand(() -> superstructure.setDesiredIntakeState(Superstructure.DesiredIntakeState.STOWED))
        );
        auto.setName("Citrus Sweep Mirrored");
        return auto;
    }

    public static Command simpleBack(SwerveDrive swerveDrive, Superstructure superstructure) {
        SequentialCommandGroup auto = new SequentialCommandGroup(
            new InstantCommand(() -> superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.HOME)),
            swerveDrive.followPathCommand(new Path("simple_back"), true, false)
        );
        auto.setName("Simple Back");
        return auto;
    }

    public static Command outpostToTower(SwerveDrive swerveDrive, Superstructure superstructure) {
        SequentialCommandGroup auto = new SequentialCommandGroup(
            new InstantCommand(() -> superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.HOME)),
            new InstantCommand(() -> superstructure.setDesiredIntakeState(Superstructure.DesiredIntakeState.STOWED)),
            swerveDrive.followPathCommand(new Path("outpost"), true, false),
            new InstantCommand(() -> superstructure.setDesiredTargetState(TargetState.HUB)),
            new InstantCommand(() -> superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.READY_FOR_SHOT)),
            new WaitCommand(0.5),
            new InstantCommand(() -> superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.SHOOTING)),
            new WaitCommand(0.5),
            new InstantCommand(() -> superstructure.setDesiredSystemState(Superstructure.DesiredSystemState.HOME)),
            swerveDrive.followPathCommand(new Path("outpost_to_tower"), false, false)
        );
        auto.setName("Outpost To Tower");
        return auto;
    }
}
