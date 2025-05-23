// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.commands.ScoreCommands;
import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.Constants;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command autonomousCommand;

    private RobotContainer robotContainer;
    private static final CommandSwerveDrivetrain commandSwerveDrivetrain = RobotContainer.commandSwerveDrivetrain;
    private static Pair<Optional<EstimatedRobotPose>, Double> estimatePose;
    private static AprilTagSubsystem aprilTagSubsystem = AprilTagSubsystem.getInstance();

//  private IntakeSubsystem exampleSubsystem = IntakeSubsystem.getInstance();

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        UserInterface.init();

        System.setProperty("wpilog.directory", "/media/sda1/wpilog");

        DataLogManager.start();

        commandSwerveDrivetrain.configureAutoBuilder();

        System.setProperty("wpilog.directory", "/media/sda1/wpilog");

        DataLogManager.start();

        commandSwerveDrivetrain.setVisionMeasurementStdDevs(Constants.Vision.kTagStdDevs);

        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();

        commandSwerveDrivetrain.seedFieldCentric();
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        UserInterface.update();

        estimatePose = aprilTagSubsystem.getEstimatedGlobalPose();
        if (estimatePose.getFirst().isPresent()) {
            Pose2d robotPose2d = estimatePose.getFirst().get().estimatedPose.toPose2d();
            Pose2d modify = new Pose2d(robotPose2d.getX(), robotPose2d.getY(),
                    commandSwerveDrivetrain.getPose().getRotation());

            commandSwerveDrivetrain.addVisionMeasurement(modify, Utils.getCurrentTimeSeconds(),
                    aprilTagSubsystem.getEstimationStdDevs());
        }

        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        RobotContainer.periodic();
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
        SignalLogger.stop();
    }

    @Override
    public void disabledPeriodic() {
        estimatePose = aprilTagSubsystem.getEstimatedGlobalPose();
        if (estimatePose.getFirst().isPresent()) {
            Pose2d robotPose2d = estimatePose.getFirst().get().estimatedPose.toPose2d();
            Pose2d modify = new Pose2d(robotPose2d.getX(), robotPose2d.getY(),
                    commandSwerveDrivetrain.getPose().getRotation());

            commandSwerveDrivetrain.addVisionMeasurement(modify, Utils.getCurrentTimeSeconds(),
                    aprilTagSubsystem.getEstimationStdDevs());
        }
    }

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        var alliance = DriverStation.getAlliance();
        SignalLogger.start();

//        commandSwerveDrivetrain.getPigeon2().reset();

        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
            // commandSwerveDrivetrain.getPigeon2().setYaw(Math.toRadians(180));
        }


        autonomousCommand = robotContainer.getAutonomousCommand();

        // Schedule the autonomous command (example)
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
        SignalLogger.stop();
        SignalLogger.start();

        ScoreCommands.Stabling.wristandElevatorStable();

        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
//        estimatePose = aprilTagSubsystem.getEstimatedGlobalPose();
//        if (estimatePose.getFirst().isPresent()) {
//            Pose2d robotPose2d = estimatePose.getFirst().get().estimatedPose.toPose2d();
//            Pose2d modify = new Pose2d(robotPose2d.getX(), robotPose2d.getY(),
//                    commandSwerveDrivetrain.getPose().getRotation());
//
//            commandSwerveDrivetrain.addVisionMeasurement(modify, Utils.getCurrentTimeSeconds(),
//                    aprilTagSubsystem.getEstimationStdDevs());
//        }
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();

    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }

    /**
     * This function is called once when the robot is first started up.
     */
    @Override
    public void simulationInit() {
    }

    /**
     * This function is called periodically whilst in simulation.
     */
    @Override
    public void simulationPeriodic() {
    }
}
