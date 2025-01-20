// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog.MotorLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.PivotToCommand;
import frc.robot.commands.ShooterPivotAngles;
import frc.robot.constants.Constants.OperatorConstants;
// import frc.robot.commands.Autos;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
// import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.testing.LinearTestSubsystem;
import frc.robot.subsystems.testing.PivotTestSubsystem;
import frc.robot.subsystems.testing.RollerTestSubsystem;
// import tagalong.subsystems.micro.Pivot;

import javax.sound.sampled.Line;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.baseUnitMagnitude(); // kSpeedAt12VoltsMps desired top speed
    private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity


    /* Setting up bindings for necessary control of the swerve drive platform */
    private final CommandXboxController commandXboxController = new CommandXboxController(OperatorConstants.driverControllerPort); // My joystick
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(); // My drivetrain
    private final ArmSubsystem arm = ArmSubsystem.getInstance();
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDesaturateWheelSpeeds(true)
             .withDeadband(MaxSpeed * .05).withRotationalDeadband(MaxAngularRate * .05) // Add a 10% deadband
            .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage); // I want field-centric

    // driving in open loop
    private final SwerveRequest.SwerveDriveBrake brake = new com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
    private ElevatorSubsystem elevatorSubsystem = ElevatorSubsystem.getInstance();

    // The robot's subsystems and commands are defined here...
    private final Telemetry logger = new Telemetry(MaxSpeed);
    //    public static final ElevatorSubsystem elevatorSubsystem = ElevatorSubsystem.getInstance();
//    public static final RollerTestSubsystem rollerTestSubsystem = RollerTestSubsystem.getInstance();
//    public static final LinearTestSubsystem linearTestSubsystem = new LinearTestSubsystem();
    //  public static final PivotTestSubsystem pivotTestSubsystem = new PivotTestSubsystem();
    // private final SendableChooser<Command> autoChooser = Autos.getAutoChooser();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        configureBindings();
        SignalLogger.setPath("/media/LOG/ctre-logs/");
    }


    private void configureBindings() {
        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-commandXboxController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(-commandXboxController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-commandXboxController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                ));

        commandXboxController.a().onTrue(new InstantCommand(() -> elevatorSubsystem.setPosition(.25)))
                .onFalse(new InstantCommand(() -> elevatorSubsystem.setPosition(0)));

        // reset the field-centric heading on left bumper press
        commandXboxController.button(8).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        if (Utils.isSimulation()) {
            drivetrain.resetPose(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));

        }


        //     commandXboxController.a().onTrue(new InstantCommand(() -> pivotTestSubsystem.setPosition(10)))
        //             .onFalse(new InstantCommand(() -> pivotTestSubsystem.setPosition(0)));

//               commandXboxController.povLeft().onTrue(new InstantCommand(() -> elevator.setVoltage(1.175)));
        //     commandXboxController.povLeft().onTrue(armSubsystem.setGround());
        //     commandXboxController.povRight().onTrue(armSubsystem.setL1());
        //     commandXboxController.povDown().onTrue(armSubsystem.setL3());
        //  commandXboxController.povDown().onTrue(new InstantCommand(()-> linearTestSubsystem.setPosition(1)));


        commandXboxController.leftBumper().onTrue(Commands.runOnce(SignalLogger::start).alongWith(new PrintCommand("Start")));
        commandXboxController.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop).alongWith(new PrintCommand("End")));

        commandXboxController.povLeft().onTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        commandXboxController.povRight().onTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        commandXboxController.povUp().onTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
        commandXboxController.povDown().onTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        //   commandXboxController.leftBumper().toggleOnTrue(arm.)
        /*
         * Joystick Y = quasistatic forward
         * Joystick A = quasistatic reverse
         * Joystick B = dynamic forward
         * Joystick X = dyanmic reverse
         */


//        commandXboxController.y().whileTrue(elevatorSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
//        commandXboxController.a().whileTrue(elevatorSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
//        commandXboxController.b().whileTrue(elevatorSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
//        commandXboxController.x().whileTrue(elevatorSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        // return Autos.exampleAuto(armSubsystem);
        // return autoChooser.getSelected();
        return null;
    }
}
