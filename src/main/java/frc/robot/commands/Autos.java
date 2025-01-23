/// / Copyright (c) FIRST and other WPILib contributors.
/// / Open Source Software; you can modify and/or share it under the terms of
/// / the WPILib BSD license file in the root directory of this project.
//
//package frc.robot.commands;
//
//import com.fasterxml.jackson.databind.util.Named;
//import com.pathplanner.lib.auto.AutoBuilder;
//
//import com.pathplanner.lib.auto.NamedCommands;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//
//public final class Autos {
//  private Autos() {
//    NamedCommands.registerCommand("intake", new SequentialCommandGroup(null));
//    NamedCommands.registerCommand("outtake", new SequentialCommandGroup(null));
//    NamedCommands.registerCommand("humanPlayerStation", new SequentialCommandGroup(null));
//    NamedCommands.registerCommand("reef1", new SequentialCommandGroup(null));
//    NamedCommands.registerCommand("reef2", new SequentialCommandGroup(null));
//    NamedCommands.registerCommand("reef3", new SequentialCommandGroup(null));
//  }
//
//  private static SendableChooser<Command> autoChooser;
//
//  /**
//   * Gets or creates the AutoChooser (Singleton Method)
//   */
//  public static SendableChooser<Command> getAutoChooser() {
//    if (autoChooser == null) autoChooser = AutoBuilder.buildAutoChooser();
//    return autoChooser;
//  }
//
//
//}
