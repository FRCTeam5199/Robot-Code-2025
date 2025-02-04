/// / Copyright (c) FIRST and other WPILib contributors.
/// / Open Source Software; you can modify and/or share it under the terms of
/// / the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.estimator.PoseEstimator;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;


public final class Autos {
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
    private static SendableChooser<Command> autoChooser;
  
    private Autos() {
      
      
      // NamedCommands.registerCommand("INTAKE", ScoreCommands.intake());
      // NamedCommands.registerCommand("OUTTAKE", ScoreCommands.outtake());
      // NamedCommands.registerCommand("STOPINTAKE", ScoreCommands.stopIntake());
      // NamedCommands.registerCommand("ARMSTABLE", ScoreCommands.stopIntake());
      // NamedCommands.registerCommand("ARMHP", ScoreCommands.armHP());
      // NamedCommands.registerCommand("ARML1", ScoreCommands.armL1());
      // NamedCommands.registerCommand("ARML2", ScoreCommands.armL2());
      // NamedCommands.registerCommand("ARML3", ScoreCommands.armL3());
      // NamedCommands.registerCommand("ARML4", ScoreCommands.armL4());
    }

    /**
     * Gets or creates the AutoChooser (Singleton Method)
     */
    // public static SendableChooser<Command> getAutoChooser() {
    //   if (autoChooser == null) autoChooser = AutoBuilder.buildAutoChooser();
    //   return autoChooser;
    // }


    public Command onePieceBlueHPTL1(){
      return new PathPlannerAuto("1 Piece Blue HPT L1");
    }
    public Command onePieceBlueHPBL1(){
      return new PathPlannerAuto("1 Piece Blue HPB L1");
    }
    public Command onePieceBlueHPTL4(){
      return new PathPlannerAuto("1 Piece Blue HPT L4");
    }
    public Command onePieceBlueHPBL4(){
      return new PathPlannerAuto("1 Piece Blue HPB L4");
    }

    public Command onePieceRedHPTL1(){
      return new PathPlannerAuto("1 Piece Red HPT L1");
    }
    public Command onePieceRedHPBL1(){
      return new PathPlannerAuto("1 Piece Red HPB L1");
    }

    public Command onePieceRedHPTL4(){
      return new PathPlannerAuto("1 Piece Red HPT L4");
    }
    public Command onePieceRedHPBL4(){
      return new PathPlannerAuto("1 Piece Red HPB L4");
    }



}
