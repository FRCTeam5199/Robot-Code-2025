// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public final class Autos {
  private Autos() {}
  
  private static SendableChooser<Command> autoChooser;

  /**
   * Gets or creates the AutoChooser (Singleton Method)
   */
  public static SendableChooser<Command> getAutoChooser() {
    if (autoChooser == null) autoChooser = AutoBuilder.buildAutoChooser();
    return autoChooser;
  }
}
