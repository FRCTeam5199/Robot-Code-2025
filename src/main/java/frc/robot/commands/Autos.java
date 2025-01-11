// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.config.RobotConfig;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;

// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.CommandSwerveDrivetrain;

// public final class Autos {
//   private static SendableChooser<Command> autoChooser;
//   private Autos() {


//      RobotConfig config;
//     try{
//       config = RobotConfig.fromGUISettings();
//     } catch (Exception e) {
//       // Handle exception as needed
//       e.printStackTrace();
//     }

//     // Configure AutoBuilder last
//     AutoBuilder.configure(
//             this::getP, // Robot pose supplier
//             this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
//             this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
//             (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
//             new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
//                     new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
//                     new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
//             ),
//             config, // The robot configuration
//             () -> {
//               // Boolean supplier that controls when the path will be mirrored for the red alliance
//               // This will flip the path being followed to the red side of the field.
//               // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

//               var alliance = DriverStation.getAlliance();
//               if (alliance.isPresent()) {
//                 return alliance.get() == DriverStation.Alliance.Red;
//               }
//               return false;
//             },
//             this // Reference to this subsystem to set requirements
//     );
//   }

//   /**
//    * Gets or creates the AutoChooser (Singleton Method)
//    */
//   public static SendableChooser<Command> getAutoChooser() {
//     if (autoChooser == null) autoChooser = AutoBuilder.buildAutoChooser();
//     return autoChooser;
//   }




// }
