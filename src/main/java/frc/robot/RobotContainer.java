// 
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Autos;
import frc.robot.commands.ScoreCommands;
import frc.robot.constants.Constants.OperatorConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;

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
    private final ClimberSubsystem climber = ClimberSubsystem.getInstance();
    private final WristSubsystem wrist = WristSubsystem.getInstance();
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDesaturateWheelSpeeds(true)
             .withDeadband(MaxSpeed * .05).withRotationalDeadband(MaxAngularRate * .05) // Add a 10% deadband
            .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage); // I want field-centric

    // driving in open loop
    private static final SwerveRequest.SwerveDriveBrake brake = new com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake();

    private static final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
    private static final ElevatorSubsystem elevatorSubsystem = ElevatorSubsystem.getInstance();
    private static final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();

    // private static final SendableChooser<Command> autoChooser = Autos.getAutoChooser();
    
    private static Boolean algaeControls = false;
    
    // The robot's subsystems and commands are defined here...
    private final Telemetry logger = new Telemetry(MaxSpeed);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        drivetrain.configureAutoBuilder();

        NamedCommands.registerCommand("INTAKE", ScoreCommands.intake2());
        NamedCommands.registerCommand("OUTTAKE", ScoreCommands.outtake2());
        NamedCommands.registerCommand("STOPINTAKE", ScoreCommands.stopIntake2());
        NamedCommands.registerCommand("ARMSTABLE", ScoreCommands.stopIntake2());
        NamedCommands.registerCommand("ARMHP", ScoreCommands.armHP());
        NamedCommands.registerCommand("ARML1", ScoreCommands.armL1());
        NamedCommands.registerCommand("ARML2", ScoreCommands.armL2());
        NamedCommands.registerCommand("ARML3", ScoreCommands.armL3());
        NamedCommands.registerCommand("ARML4", ScoreCommands.armL4());
        configureBindings();
        SignalLogger.setPath("/media/LOG/ctre-logs/");
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-commandXboxController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(-commandXboxController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-commandXboxController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                ));

        commandXboxController.button(7).toggleOnFalse(drivetrain.applyRequest(() -> brake)); // TESTING ONLY -> CHANGE TO onTrue AT COMP
        // reset the field-centric heading on menu button press
        commandXboxController.button(8).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        if (Utils.isSimulation()) {
            drivetrain.resetPose(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));

        }
        
        commandXboxController.rightBumper().onTrue(new InstantCommand(() -> System.out.println("Arm Degrees: " + armSubsystem.getDegrees()))
            .andThen(new InstantCommand(() -> System.out.println("Elevator Meters: " + elevatorSubsystem.getMechM())))
            .andThen(new InstantCommand(() -> System.out.println("Wrist Degrees: " + wrist.getDegrees()))));

        commandXboxController.button(9).onTrue(new InstantCommand(() -> algaeControls = false));
        commandXboxController.button(10).onTrue(new InstantCommand(() -> algaeControls = true));

        commandXboxController.a().onTrue(ScoreCommands.armHP());
        commandXboxController.b().onTrue(new ConditionalCommand(ScoreCommands.armAlgaeL1(), ScoreCommands.armL2(), () -> algaeControls));
        commandXboxController.x().onTrue(new ConditionalCommand(ScoreCommands.algaeArmL2(), ScoreCommands.armL3(), () -> algaeControls));
        commandXboxController.y().onTrue(ScoreCommands.armL4());
        
        commandXboxController.leftBumper().onTrue(new ConditionalCommand(ScoreCommands.algaeArmStable(), ScoreCommands.armStable(), () -> algaeControls));

        commandXboxController.leftTrigger().onTrue(new InstantCommand(() -> intakeSubsystem.setPercent(60)))
                .onFalse(new InstantCommand(() -> intakeSubsystem.setVoltage(0)));
        commandXboxController.rightTrigger().onTrue(new InstantCommand(() -> intakeSubsystem.setPercent(-80)))
                .onFalse(new InstantCommand(() -> intakeSubsystem.setVoltage(0)));

        commandXboxController.povUp().onTrue(new InstantCommand(() -> climber.setPercent(0.3))).onFalse(new InstantCommand(() -> climber.setPercent(0)));
        commandXboxController.povDown().onTrue(new InstantCommand(() -> climber.setPercent(-0.3))).onFalse(new InstantCommand(() -> climber.setPercent(0)));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // return autoChooser.getSelected();
        return new PathPlannerAuto("test auto red");
    }
    
    // public static Command threePieceProcessor() {                                                                                         
    //     return AutoBuilder.buildAuto("Lfue");
    // }
}
