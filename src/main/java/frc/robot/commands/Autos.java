/// / Copyright (c) FIRST and other WPILib contributors.
/// / Open Source Software; you can modify and/or share it under the terms of
/// / the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public final class Autos {
    private static SendableChooser<Command> autoChooser;

    private Autos() {}

    public static void initalizeNamedCommands() {
      NamedCommands.registerCommand("INTAKE", ScoreCommands.intake());
      NamedCommands.registerCommand("OUTTAKE", ScoreCommands.outtake());
      NamedCommands.registerCommand("HP", ScoreCommands.intakeHP());
      NamedCommands.registerCommand("L1", ScoreCommands.scoreL1());
      NamedCommands.registerCommand("L2", ScoreCommands.scoreL2());
      NamedCommands.registerCommand("L3", ScoreCommands.scoreL3());
      NamedCommands.registerCommand("L4", ScoreCommands.scoreL4());
      NamedCommands.registerCommand("ARML4", ScoreCommands.armL4());
      NamedCommands.registerCommand("ARML3", ScoreCommands.armL3());
      NamedCommands.registerCommand("ARML2", ScoreCommands.armL2());

    }

    /**
     * Gets or creates the AutoChooser (Singleton Method)
     */
    public static SendableChooser<Command> getAutoChooser() {
      if (autoChooser == null) {
        autoChooser = AutoBuilder.buildAutoChooser();
      }

      return autoChooser;
    }
}
