/// / Copyright (c) FIRST and other WPILib contributors.
/// / Open Source Software; you can modify and/or share it under the terms of
/// / the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public final class Autos {
    private static SendableChooser<Command> autoChooser;
  
    private Autos() {
      NamedCommands.registerCommand("Intake", ScoreCommands.intake());
      NamedCommands.registerCommand("Outtake", ScoreCommands.outtake());
      NamedCommands.registerCommand("HP", new SequentialCommandGroup(ScoreCommands.intakeHP()));
      NamedCommands.registerCommand("L1", new SequentialCommandGroup(ScoreCommands.scoreL1()));
      NamedCommands.registerCommand("L2", new SequentialCommandGroup(ScoreCommands.scoreL2()));
      NamedCommands.registerCommand("L3", new SequentialCommandGroup(ScoreCommands.scoreL3()));
    }

    /**
     * Gets or creates the AutoChooser (Singleton Method)
     */
    public static SendableChooser<Command> getAutoChooser() {
      if (autoChooser == null) autoChooser = AutoBuilder.buildAutoChooser();
      return autoChooser;
    }
}
