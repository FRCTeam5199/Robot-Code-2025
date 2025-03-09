package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.constants.TunerConstants;
import frc.robot.constants.Constants.ArmConstants;
import frc.robot.constants.Constants.ElevatorConstants;
import frc.robot.constants.Constants.WristConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.template.PositionCommand;
import frc.robot.subsystems.template.VelocityCommand;
import frc.robot.utility.State;

import java.util.Map;

import static frc.robot.RobotContainer.*;

public class ScoreCommands {
    private static ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
    private static ElevatorSubsystem elevatorSubsystem = ElevatorSubsystem.getInstance();
    private static WristSubsystem wristSubsystem = WristSubsystem.getInstance();
    private static IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();
    private static ClimberSubsystem climberSubsystem = ClimberSubsystem.getInstance();

    private static AprilTagSubsystem aprilTagSubsystem = AprilTagSubsystem.getInstance();

    private static Timer timer = new Timer();

    public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.baseUnitMagnitude();

    private final static SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDesaturateWheelSpeeds(true) // Add a 10% deadband
            .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);


    public static class Drive{
        public static Command autoAlignTeleop() {
                return new FunctionalCommand(
                        () -> commandSwerveDrivetrain.resetPose(
                                new Pose2d(
                                        new Translation2d(commandSwerveDrivetrain.getPose().getX(),
                                                commandSwerveDrivetrain.getPose().getY()),
                                        new Rotation2d(Math.toRadians(aprilTagSubsystem.getRotationToAlign(aprilTagSubsystem
                                                .getClosestTagID()))))),
                        () -> {
                            if ((!elevatorSubsystem.isMechAtGoal(false)
                                    || !armSubsystem.isMechAtGoal(false)
                                    || !wristSubsystem.isMechAtGoal(false))
                                    && Math.abs(aprilTagSubsystem.getClosestTagXYYaw()[0]) < .3)
                                commandSwerveDrivetrain.setControl(
                                        drive.withVelocityX(0)
                                                .withVelocityY(yVelocity)
                                                .withRotationalRate(rotationVelocity));
                            else commandSwerveDrivetrain.setControl(
                                    drive.withVelocityX(xVelocity)
                                            .withVelocityY(yVelocity)
                                            .withRotationalRate(rotationVelocity));
                        },
                        (interrupted) -> {
                            commandSwerveDrivetrain
                                    .resetRotation(new Rotation2d(Math.toRadians(commandSwerveDrivetrain
                                            .getPigeon2().getRotation2d().getDegrees() + (DriverStation.getAlliance().isPresent()
                                            && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue) ? 0 : 180))));
                            commandSwerveDrivetrain.setControl(
                                    drive.withVelocityX(0)
                                            .withVelocityY(0)
                                            .withRotationalRate(0));
                        },
                        RobotContainer::aligned,
                        commandSwerveDrivetrain);
            }
        
            public static Command autoAlignLAuton() {
                return new FunctionalCommand(
                        () -> {
                            if (RobotContainer.autoAlignYOffset < 0) {
                                RobotContainer.autoAlignYOffset = -RobotContainer.autoAlignYOffset;
                            }
        
                            commandSwerveDrivetrain.resetPose(
                                    new Pose2d(
                                            new Translation2d(commandSwerveDrivetrain.getPose().getX(),
                                                    commandSwerveDrivetrain.getPose().getY()),
                                            new Rotation2d(Math.toRadians(aprilTagSubsystem.getRotationToAlign(aprilTagSubsystem
                                                    .getClosestTagID())))));
                        },
                        () -> RobotContainer.commandSwerveDrivetrain.setControl(
                                drive.withVelocityX(RobotContainer.xVelocity)
                                        .withVelocityY(yVelocity)
                                        .withRotationalRate(rotationVelocity)),
                        (interrupted) -> {
                            RobotContainer.commandSwerveDrivetrain
                                    .resetRotation(new Rotation2d(Math.toRadians(RobotContainer.commandSwerveDrivetrain
                                            .getPigeon2().getRotation2d().getDegrees() +
                                            (DriverStation.getAlliance().isPresent()
                                                    && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)
                                                    ? 0 : 180))));
        
                            RobotContainer.commandSwerveDrivetrain.setControl(
                                    drive.withVelocityX(0)
                                            .withVelocityY(0)
                                            .withRotationalRate(0));
                        },
                        RobotContainer::aligned,
                        RobotContainer.commandSwerveDrivetrain);
            }
        
            public static Command autoAlignRAuton() {
                return new FunctionalCommand(
                        () -> {
                            if (RobotContainer.autoAlignYOffset > 0) {
                                RobotContainer.autoAlignYOffset = -RobotContainer.autoAlignYOffset;
                            }
        
                            commandSwerveDrivetrain.resetPose(
                                    new Pose2d(
                                            new Translation2d(commandSwerveDrivetrain.getPose().getX(),
                                                    commandSwerveDrivetrain.getPose().getY()),
                                            new Rotation2d(Math.toRadians(aprilTagSubsystem.getRotationToAlign(aprilTagSubsystem
                                                    .getClosestTagID())))));
                        },
                        () ->
                                RobotContainer.commandSwerveDrivetrain.setControl(
                                        drive.withVelocityX(RobotContainer.xVelocity)
                                                .withVelocityY(yVelocity)
                                                .withRotationalRate(rotationVelocity)),
                        (interrupted) -> {
                            RobotContainer.commandSwerveDrivetrain
                                    .resetRotation(new Rotation2d(Math.toRadians(RobotContainer.commandSwerveDrivetrain
                                            .getPigeon2().getRotation2d().getDegrees() +
                                            (DriverStation.getAlliance().isPresent()
                                                    && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)
                                                    ? 0 : 180))));
        
                            RobotContainer.commandSwerveDrivetrain.setControl(
                                    drive.withVelocityX(0)
                                            .withVelocityY(0)
                                            .withRotationalRate(0));
                        },
                        RobotContainer::aligned,
                        RobotContainer.commandSwerveDrivetrain);
            }
        
            public static Command autoMoveForwardBottom() {
                return new FunctionalCommand(
                        () -> {
                        },
                        () -> RobotContainer.commandSwerveDrivetrain.setControl(
                                drive.withVelocityX(-.5)
                                        .withVelocityY(-.5)),
                        (interrupted) -> RobotContainer.commandSwerveDrivetrain.setControl(
                                drive.withVelocityX(0)
                                        .withVelocityY(0)),
                        () -> false,
                        RobotContainer.commandSwerveDrivetrain);
            }
        
        
            public static Command autoMoveForwardTop() {
                return new FunctionalCommand(
                        () -> {
                            System.out.println("Forward Start");
                        },
                        () -> RobotContainer.commandSwerveDrivetrain.setControl(
                                drive.withVelocityX(-0.5)
                                        .withVelocityY(0.5)),
                        (bool) -> {
                            RobotContainer.commandSwerveDrivetrain.setControl(
                                    drive.withVelocityX(0)
                                            .withVelocityY(0));
        
                            System.out.println("ending");
                        },
                        () -> false,
                        RobotContainer.commandSwerveDrivetrain);
            }
        
    }

    public static class Stabling{
        public static Command stable() {
                return new SequentialCommandGroup(
                        new PositionCommand(wristSubsystem, 10),
        
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, 0, false).beforeStarting(new WaitCommand(0.5))
                                        .until(() -> elevatorSubsystem.isAtBottom()
                                                && elevatorSubsystem.getMechM() < .05)
                        ),
                        new InstantCommand(() -> elevatorSubsystem.getMotor().setPosition(0)),
                        Arm.armStable()
                ).alongWith(new VelocityCommand(intakeSubsystem, 0));
            }
        
            public static Command intakeStable() {
                return new ParallelCommandGroup(
                        new PositionCommand(elevatorSubsystem, 0, false).beforeStarting(new WaitCommand(0.5))
                                .andThen(new PositionCommand(armSubsystem, 59)),
                        new PositionCommand(wristSubsystem, 10)
                );
            }
        
            public static Command stableWithArm() {
                return new ConditionalCommand(
                        new SequentialCommandGroup( //Won't clip elevator
                                new ParallelCommandGroup(
                                        new PositionCommand(wristSubsystem, 0),
                                        new PositionCommand(elevatorSubsystem, 0, false)
                                                .until(() -> elevatorSubsystem.isAtBottom() && elevatorSubsystem.getMechM() < .1)
                                ),
                                new InstantCommand(() -> elevatorSubsystem.getMotor().setPosition(0)),
                                new PositionCommand(armSubsystem, 0)
                        ),
                        new SequentialCommandGroup( //Will clip elevator
                                new PositionCommand(wristSubsystem, 0),
                                new PositionCommand(elevatorSubsystem, 0, false)
                                        .until(() -> elevatorSubsystem.isAtBottom() && elevatorSubsystem.getMechM() < .1),
                                new InstantCommand(() -> elevatorSubsystem.getMotor().setPosition(0)),
                                new PositionCommand(armSubsystem, 0)
                        ),
                        () -> wristSubsystem.getDegrees() < 50
                );
            }
        
            public static Command wristandElevatorStable() {
                return new PositionCommand(wristSubsystem, 0).andThen(new PositionCommand(elevatorSubsystem, 0, false));
            }
    }

    public static class Climber{
        public static Command drop() {
                return new FunctionalCommand(
                        () -> {
                            timer.reset();
                        },
                        () -> climberSubsystem.drop(),
                        (bool) -> climberSubsystem.stopDrop(),
                        () -> {
                            timer.start();
                            if (timer.hasElapsed(0.45)) {
                                timer.stop();
                                return true;
                            }
                            return false;
                        },
                        climberSubsystem);
            }
        
    }

    public static class Zeroing{

        public static Command zeroElevator() {
                return new FunctionalCommand(
                        () -> {
                            elevatorSubsystem.setVoltage(-2);
                            elevatorSubsystem.setOffset(0, false);
                        },
                        () -> {
                        },
                        (interrupted) -> {
                            elevatorSubsystem.setVoltage(0);
                        },
                        elevatorSubsystem::isAtBottom,
                        elevatorSubsystem
                );
            }
        
            public static Command zeroWrist() {
                return new FunctionalCommand(
                        () -> {
                            wristSubsystem.setVoltage(-1);
                            wristSubsystem.setOffset(0, false);
                        },
                        () -> {
                        },
                        (interrupted) -> {
                            wristSubsystem.setVoltage(0);
                        },
                        wristSubsystem::isAtBottom,
                        wristSubsystem
                );
            }
        
        
            public static Command zeroArm() {
                return new FunctionalCommand(
                        () -> {
                            armSubsystem.setVoltage(-4);
                            armSubsystem.setOffset(0, false);
                        },
                        () -> {
                        },
                        (interrupted) -> {
                            armSubsystem.setVelocity(0);
                        },
                        armSubsystem::isAtBottom,
                        armSubsystem
                );
            }
        
            public static Command zeroSubsystems() {
                return new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                zeroElevator(),
                                zeroArm(),
                                zeroWrist()
                        ).withTimeout(5),
                        new WaitCommand(.3),
                        new ParallelCommandGroup(
                                new InstantCommand(() -> elevatorSubsystem.getMotor().setPosition(0)),
                                new InstantCommand(() -> armSubsystem.getMotor().setPosition(0)),
                                new InstantCommand(() -> wristSubsystem.getMotor().setPosition(0))
                        ),
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, 0.12, true),
                                new PositionCommand(armSubsystem, 7),
                                new PositionCommand(wristSubsystem, 7).beforeStarting(new WaitCommand(0.5))
                        ),
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, 0, true),
                                new PositionCommand(armSubsystem, 0),
                                new PositionCommand(wristSubsystem, 0)
                        )
                );
            }
    }

    public static class Intake{
        
        public static Command intakeGround() {
                //currently ground intake
                return new SequentialCommandGroup(
                        new PositionCommand(armSubsystem, Constants.ArmConstants.GROUND),
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, Constants.ElevatorConstants.GROUND, true),
                                new PositionCommand(wristSubsystem, Constants.WristConstants.GROUND)
                        )
                ).alongWith(new InstantCommand(() -> RobotContainer.setState(State.GROUND)))
                        .alongWith(new VelocityCommand(intakeSubsystem, -75));
            }

            public static Command intakeHP() {
                return new ConditionalCommand(
                        new SequentialCommandGroup( //Going down
                                new ParallelCommandGroup(
                                        new PositionCommand(elevatorSubsystem, ElevatorConstants.HP, false)
                                                .andThen(new PositionCommand(armSubsystem, ArmConstants.HP)),
                                        new PositionCommand(wristSubsystem, WristConstants.HP).beforeStarting(new WaitCommand(0.5))
                                )
                        ),
                        new SequentialCommandGroup(
                                new ParallelCommandGroup( //Going up
                                        new PositionCommand(elevatorSubsystem, ElevatorConstants.HP, true),
                                        new PositionCommand(armSubsystem, ArmConstants.HP),
                                        new PositionCommand(wristSubsystem, WristConstants.HP).beforeStarting(new WaitCommand(0.5))
                                )
                        ),
                        () -> elevatorSubsystem.getMechM() > .04
                ).beforeStarting(new PositionCommand(wristSubsystem, 10));
            }
        
    }


    public static class Arm{

        public static Command armL1() {
                return new PositionCommand(armSubsystem, Constants.ArmConstants.L1).alongWith(
                        new InstantCommand(() -> RobotContainer.setState(State.L1)));
            }
        
            public static Command armL2() {
                return new PositionCommand(armSubsystem, Constants.ArmConstants.L2).alongWith(
                        new InstantCommand(() -> RobotContainer.setState(State.L2)));
            }
        
            public static Command armL3() {
                return new PositionCommand(armSubsystem, Constants.ArmConstants.L3)
                        .alongWith(new InstantCommand(() -> RobotContainer.setState(State.L3)));
            }
        
            public static Command armL4() {
                return new PositionCommand(armSubsystem, Constants.ArmConstants.L4)
                        .alongWith(new InstantCommand(() -> RobotContainer.setState(State.L4)));
            }

            public static Command armStable() {
                return new SelectCommand<>(
                        Map.ofEntries(
                                Map.entry(State.L1, armL1()),
                                Map.entry(State.L2, armL2()),
                                Map.entry(State.L3, armL3()),
                                Map.entry(State.L4, armL4())
                        ),
                        RobotContainer::getState
                );
            }
        

    }

    public static class Roller{
        public static Command intake() {
                return new FunctionalCommand(
                        () -> {
                            timer.reset();
                        },
                        () -> intakeSubsystem.intake(),
                        (interrupted) -> intakeSubsystem.stopIntake(),
                        () -> {
                            if (intakeSubsystem.getStatorCurrent() > 25) {
                                timer.start();
                                if (timer.hasElapsed(.3)) {
                                    timer.stop();
                                    return true;
                                }
                                return false;
                            }
                            return false;
                        },
                        intakeSubsystem);
            }
        
            public static Command outtake() {
                return new FunctionalCommand(
                        () -> {
                            timer.reset();
                        },
                        () -> intakeSubsystem.outtake(),
                        (bool) -> intakeSubsystem.stopIntake(),
                        () -> {
                            timer.start();
                            if (timer.hasElapsed(.3)) {
                                timer.stop();
                                return true;
        
                            }
                            return false;
                        },
                        intakeSubsystem);
            }
    }



    public static class Score{
        public static Command removeAlgaeHigh() {
                return new ConditionalCommand(
                        new SequentialCommandGroup( //Going down
                                new ParallelCommandGroup(
                                        new PositionCommand(elevatorSubsystem, ElevatorConstants.ALGAE_HIGH, false),
                                        new PositionCommand(wristSubsystem, WristConstants.ALGAE_HIGH).beforeStarting(new WaitCommand(0.5))
                                ),
                                new PositionCommand(armSubsystem, ArmConstants.ALGAE_HIGH)
                        ),
                        new SequentialCommandGroup( //Going up
                                new PositionCommand(armSubsystem, ArmConstants.ALGAE_HIGH),
                                new ParallelCommandGroup(
                                        new PositionCommand(elevatorSubsystem, ElevatorConstants.ALGAE_HIGH, true),
                                        new PositionCommand(wristSubsystem, WristConstants.ALGAE_HIGH).beforeStarting(new WaitCommand(0.5))
                                )
                        ),
                        () -> elevatorSubsystem.getMechM() > .55
                );
            }
        
            public static Command removeAlgaeLow() {
                return new ConditionalCommand(
                        new SequentialCommandGroup( //Going down
                                new ParallelCommandGroup(
                                        new PositionCommand(elevatorSubsystem, ElevatorConstants.ALGAE_LOW, false),
                                        new PositionCommand(wristSubsystem, WristConstants.ALGAE_LOW).beforeStarting(new WaitCommand(0.5))
                                ),
                                new PositionCommand(armSubsystem, ArmConstants.ALGAE_LOW)
                        ),
                        new SequentialCommandGroup( //Going up
                                new PositionCommand(armSubsystem, ArmConstants.ALGAE_LOW),
                                new ParallelCommandGroup(
                                        new PositionCommand(elevatorSubsystem, ElevatorConstants.ALGAE_LOW, true),
                                        new PositionCommand(wristSubsystem, WristConstants.ALGAE_LOW).beforeStarting(new WaitCommand(0.5))
                                )
                        ),
                        () -> elevatorSubsystem.getMechM() > .25
                );
            }

        public static Command scoreL1() {
                return new SequentialCommandGroup(
                        new PositionCommand(armSubsystem, Constants.ArmConstants.L1),
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, Constants.ElevatorConstants.L1, true),
                                new PositionCommand(wristSubsystem, Constants.WristConstants.L1)
                        )
                ).alongWith(new InstantCommand(() -> RobotContainer.setState(State.L1)));
            }
                      
        
        
            public static Command scoreL2() {
                return new ConditionalCommand(
                        new SequentialCommandGroup( //Going down
                                new ParallelCommandGroup(
                                        new PositionCommand(elevatorSubsystem, Constants.ElevatorConstants.L2, false),
                                        new PositionCommand(wristSubsystem, Constants.WristConstants.L2).beforeStarting(new WaitCommand(0.5))
                                ),
                                new PositionCommand(armSubsystem, Constants.ArmConstants.L2)
                        ),
                        new SequentialCommandGroup( //Going up
                                new PositionCommand(armSubsystem, Constants.ArmConstants.L2),
                                new ParallelCommandGroup(
                                        new PositionCommand(elevatorSubsystem, Constants.ElevatorConstants.L2, true),
                                        new PositionCommand(wristSubsystem, Constants.WristConstants.L2).beforeStarting(new WaitCommand(0.5))
                                        // new VelocityCommand(intakeSubsystem, 50)
                                )
        
                        ),
                        () -> elevatorSubsystem.getMechM() > .12
                ).alongWith(new InstantCommand(() -> RobotContainer.setState(State.L2)));
            }
        
            public static Command scoreL3() {
                return new ConditionalCommand(
                        new SequentialCommandGroup( //Going down
                                new ParallelCommandGroup(
                                        new PositionCommand(elevatorSubsystem, Constants.ElevatorConstants.L3, false),
                                        new PositionCommand(wristSubsystem, Constants.WristConstants.L3).beforeStarting(new WaitCommand(0.5))
                                        // new VelocityCommand(intakeSubsystem, 50)
                                ),
                                new PositionCommand(armSubsystem, Constants.ArmConstants.L3)
                        ),
                        new SequentialCommandGroup( //Going up
                                new PositionCommand(armSubsystem, Constants.ArmConstants.L3), 
                                new ParallelCommandGroup(
                                        new PositionCommand(elevatorSubsystem, Constants.ElevatorConstants.L3, true),
                                        new PositionCommand(wristSubsystem, Constants.WristConstants.L3).beforeStarting(new WaitCommand(0.5))
                                        // new VelocityCommand(intakeSubsystem, 50)
                                )
                        ),
                        () -> elevatorSubsystem.getMechM() > .46
                ).alongWith(new InstantCommand(() -> RobotContainer.setState(State.L3)));
            }
        
            public static Command scoreL4() {
                return new SequentialCommandGroup(
                        new PositionCommand(armSubsystem, Constants.ArmConstants.L4),
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, Constants.ElevatorConstants.L4, true),
                                new PositionCommand(wristSubsystem, Constants.WristConstants.L4).beforeStarting(new WaitCommand(0.5))
                        ),
                        new PositionCommand(wristSubsystem, Constants.WristConstants.L4_DUNK)
                ).alongWith(new InstantCommand(() -> RobotContainer.setState(State.L4)));
            }
        
            public static Command score() {
                return new SelectCommand<>(
                        Map.ofEntries(
                                Map.entry(State.L1, scoreL1()),
                                Map.entry(State.L2, scoreL2()),
                                Map.entry(State.L3, scoreL3()),
                                Map.entry(State.L4, scoreL4()),
                                Map.entry(State.ALGAE_HIGH, removeAlgaeHigh()),
                                Map.entry(State.ALGAE_LOW, removeAlgaeLow())
                        ),
                        RobotContainer::getState
                ).beforeStarting(new PositionCommand(wristSubsystem, 10));
            }

            public static Command scoreShoot() {
                return new ConditionalCommand(
                        new VelocityCommand(intakeSubsystem, -25),
                        new VelocityCommand(intakeSubsystem, -75),
                        () -> RobotContainer.getState() == State.L1);
            }
    }



}
