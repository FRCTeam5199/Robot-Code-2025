/// / Copyright (c) FIRST and other WPILib contributors.
/// / Open Source Software; you can modify and/or share it under the terms of
/// / the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;

public final class Autos {
    private final ArmSubsystem arm = ArmSubsystem.getInstance();
    private final ClimberSubsystem climber = ClimberSubsystem.getInstance();
    private final WristSubsystem wrist = WristSubsystem.getInstance();
    private final IntakeSubsystem intake = IntakeSubsystem.getInstance();



    private Autos() {
    NamedCommands.registerCommand("intake", new SequentialCommandGroup(new InstantCommand(() -> intake.setPercent(60))));
    NamedCommands.registerCommand("outtake", new SequentialCommandGroup(new InstantCommand(() -> intake.setPercent(60))));
    NamedCommands.registerCommand("humanPlayerStation", new SequentialCommandGroup(ScoreCommands.intakeHP()));
    NamedCommands.registerCommand("reef1", new SequentialCommandGroup(ScoreCommands.scoreL1()));
    NamedCommands.registerCommand("reef2", new SequentialCommandGroup(ScoreCommands.scoreL2()));
    NamedCommands.registerCommand("reef3", new SequentialCommandGroup(ScoreCommands.scoreL3()));
  }

  private static SendableChooser<Command> autoChooser;

  /**
   * Gets or creates the AutoChooser (Singleton Method)
   */
  public static SendableChooser<Command> getAutoChooser() {
    if (autoChooser == null) autoChooser = AutoBuilder.buildAutoChooser();
    return autoChooser;
  }


}
