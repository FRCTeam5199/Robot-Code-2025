package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.template.PositionCommand;
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
    // public static double MaxAngularRate = TunerConstants.kRotationAt12Volts;

    private final static SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDesaturateWheelSpeeds(true) // Add a 10% deadband
            .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);

    public static Command stable() {
//        return new ConditionalCommand(
//                new SequentialCommandGroup( //Won't clip elevator
//                        new ParallelCommandGroup(
//                                new PositionCommand(wristSubsystem, 0),
//                                new PositionCommand(elevatorSubsystem, 0, false)
//                                        .until(() -> elevatorSubsystem.isAtBottom() && elevatorSubsystem.getMechM() < .05)
//                        ),
//                        new InstantCommand(() -> elevatorSubsystem.getMotor().setPosition(0)),
//                        armStable()
//                ),
//                new SequentialCommandGroup( //Will clip elevator
//                        new PositionCommand(wristSubsystem, 0),
//                        new PositionCommand(elevatorSubsystem, 0, false)
//                                .until(() -> elevatorSubsystem.isAtBottom() && elevatorSubsystem.getMechM() < .1),
//                        new InstantCommand(() -> elevatorSubsystem.getMotor().setPosition(0)),
//                        armStable()
//                ),
//                () -> wristSubsystem.getDegrees() < 50
//        );
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new PositionCommand(wristSubsystem, 0),
                        new PositionCommand(elevatorSubsystem, 0, false)
                                .until(() -> elevatorSubsystem.isAtBottom()
                                        && elevatorSubsystem.getMechM() < .05)
                ),
                new InstantCommand(() -> elevatorSubsystem.getMotor().setPosition(0)),
                armStable()
        );
    }

    public static Command intakeStable() {
        return new ParallelCommandGroup(
                new PositionCommand(elevatorSubsystem, 0, false).andThen(new PositionCommand(armSubsystem, 56.56)),
                new PositionCommand(wristSubsystem, 0)
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

    public static Command wristandElevatorHP() {
        return new PositionCommand(wristSubsystem, .004).andThen(new PositionCommand(elevatorSubsystem, .006, false));
    }

    public static Command intakeHP() {
        return new ConditionalCommand(
                new SequentialCommandGroup( //Going down
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, 0.04, false)
                                        .andThen(new PositionCommand(armSubsystem, 56.56)),
                                new PositionCommand(wristSubsystem, 0)
                        )
                ),
                new SequentialCommandGroup(
                        new ParallelCommandGroup( //Going up
                                new PositionCommand(elevatorSubsystem, 0.04, true),
                                new PositionCommand(armSubsystem, 56.56),
                                new PositionCommand(wristSubsystem, 0)
                        )
                ),
                () -> elevatorSubsystem.getMechM() > .04
        );
        // ).alongWith(new VelocityCommand(intakeSubsystem, 50));
    }

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

    public static Command scoreL1() {
        return new SequentialCommandGroup(
                new PositionCommand(armSubsystem, 0),
                new ParallelCommandGroup(
                        new PositionCommand(elevatorSubsystem, 0, true),
                        new PositionCommand(wristSubsystem, 46)
                        // new VelocityCommand(intakeSubsystem, 50)
                )
        ).alongWith(new InstantCommand(() -> RobotContainer.setState(State.L1)));
    }

    public static Command scoreL2() {
        return new ConditionalCommand(
                new SequentialCommandGroup( //Going down
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, .02, false),
                                new PositionCommand(wristSubsystem, 152)
                                // new VelocityCommand(intakeSubsystem, 50)
                        ),
                        new PositionCommand(armSubsystem, 74)
                ),
                new SequentialCommandGroup( //Going up
                        new PositionCommand(armSubsystem, 74),
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, .02, true),
                                new PositionCommand(wristSubsystem, 152)
                                // new VelocityCommand(intakeSubsystem, 50)
                        )
                ),
                () -> elevatorSubsystem.getMechM() > .07
        ).alongWith(new InstantCommand(() -> RobotContainer.setState(State.L2)));
    }

    public static Command scoreL3() {
        return new ConditionalCommand(
                new SequentialCommandGroup( //Going down
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, .32, false),
                                new PositionCommand(wristSubsystem, 157)
                                // new VelocityCommand(intakeSubsystem, 50)
                        ),
                        new PositionCommand(armSubsystem, 85)
                ),
                new SequentialCommandGroup( //Going up
                        new PositionCommand(armSubsystem, 85),
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, .32, true),
                                new PositionCommand(wristSubsystem, 157)
                                // new VelocityCommand(intakeSubsystem, 50)
                        )
                ),
                () -> elevatorSubsystem.getMechM() > .33
        ).alongWith(new InstantCommand(() -> RobotContainer.setState(State.L3)));
    }

    public static Command scoreL4() {
        return new SequentialCommandGroup(
                new PositionCommand(armSubsystem, 83),
                new ParallelCommandGroup(
                        new PositionCommand(elevatorSubsystem, 1.002, true),
                        new PositionCommand(wristSubsystem, 100)
                ),
                new PositionCommand(wristSubsystem, 165)
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
        );
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

    public static Command armL1() {
        return new PositionCommand(armSubsystem, 0).alongWith(
                new InstantCommand(() -> RobotContainer.setState(State.L1)));
    }

    public static Command armL2() {
        return new PositionCommand(armSubsystem, 74).alongWith(
                new InstantCommand(() -> RobotContainer.setState(State.L2)));
    }

    public static Command armL3() {
        return new PositionCommand(armSubsystem, 85)
                .alongWith(new InstantCommand(() -> RobotContainer.setState(State.L3)));
    }

    public static Command armL4() {
        return new PositionCommand(armSubsystem, 81)
                .alongWith(new InstantCommand(() -> RobotContainer.setState(State.L4)));
    }

    public static Command dunk() {
        return new SelectCommand<>(
                Map.ofEntries(
                        Map.entry(State.L2, new PositionCommand(armSubsystem, 65)),
                        Map.entry(State.L3, new PositionCommand(armSubsystem, 73)),
                        Map.entry(State.L4, new PositionCommand(wristSubsystem, 167))
                ),
                RobotContainer::getState
        );
    }

    public static Command unDunk() {
        return new SelectCommand<>(
                Map.ofEntries(
                        Map.entry(State.L2, new PositionCommand(armSubsystem, 74)),
                        Map.entry(State.L3, new PositionCommand(armSubsystem, 85)),
                        Map.entry(State.L4, new PositionCommand(wristSubsystem, 164))
                ),
                RobotContainer::getState
        );
    }

    public static Command removeAlgaeHigh() {
        return new ConditionalCommand(
                new SequentialCommandGroup( //Going down
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, .55, false),
                                new PositionCommand(wristSubsystem, 163)
                        ),
                        new PositionCommand(armSubsystem, 74)
                ),
                new SequentialCommandGroup( //Going up
                        new PositionCommand(armSubsystem, 74),
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, .55, true),
                                new PositionCommand(wristSubsystem, 163)
                        )
                ),
                () -> elevatorSubsystem.getMechM() > .55
        );
    }

    public static Command removeAlgaeLow() {
        return new ConditionalCommand(
                new SequentialCommandGroup( //Going down
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, .25, false),
                                new PositionCommand(wristSubsystem, 163)
                        ),
                        new PositionCommand(armSubsystem, 69)
                ),
                new SequentialCommandGroup( //Going up
                        new PositionCommand(armSubsystem, 69),
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, .25, true),
                                new PositionCommand(wristSubsystem, 163)
                        )
                ),
                () -> elevatorSubsystem.getMechM() > .25
        );
    }

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
                        new PositionCommand(elevatorSubsystem, 0.05, true),
                        new PositionCommand(armSubsystem, 7),
                        new PositionCommand(wristSubsystem, 7)
                ),
                new ParallelCommandGroup(
                        new PositionCommand(elevatorSubsystem, 0, true),
                        new PositionCommand(armSubsystem, 0),
                        new PositionCommand(wristSubsystem, 0)
                )
        );
    }

    public static Command autoAlignLAuton() {
        return new FunctionalCommand(
                () -> {
                    if (RobotContainer.autoAlignYOffset < 0) {
                        RobotContainer.autoAlignYOffset = -RobotContainer.autoAlignYOffset;
                    }

                    RobotContainer.commandSwerveDrivetrain.resetRotation(new Rotation2d(
                            Math.toRadians(aprilTagSubsystem.getRotationToAlign(aprilTagSubsystem
                                    .getClosestTagID()))));
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

                    RobotContainer.commandSwerveDrivetrain.resetRotation(new Rotation2d(
                            Math.toRadians(aprilTagSubsystem.getRotationToAlign(aprilTagSubsystem
                                    .getClosestTagID()))));
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


//    public static Command algaeL1() {
//        return new ConditionalCommand(
//                new SequentialCommandGroup(
//                        new ParallelCommandGroup(
////                                new PositionCommand(elevatorSubsystem, .2, 20, 50),
//                                new PositionCommand(wristSubsystem, 132)
//                        ),
//                        new PositionCommand(armSubsystem, 64.71)
//                ),
//                new SequentialCommandGroup(
//                        new PositionCommand(armSubsystem, 64.71),
//                        new ParallelCommandGroup(
////                                new PositionCommand(elevatorSubsystem, .2, 20, 50),
//                                new PositionCommand(wristSubsystem, 132)
//                        )
//
//                ),
//                () -> elevatorSubsystem.getMechM() > .06
//        );
//    }
//
//    public static Command algaeL2() {
//        return new ConditionalCommand(
//                new SequentialCommandGroup( //Going down
//                        new ParallelCommandGroup(
////                                new PositionCommand(elevatorSubsystem, 0.5, 36, 20),
//                                new PositionCommand(wristSubsystem, 140)
//                        ),
//                        new PositionCommand(armSubsystem, 72.21)
//                ),
//                new SequentialCommandGroup( //Going up
//                        new PositionCommand(armSubsystem, 72.21),
//                        new ParallelCommandGroup(
////                                new PositionCommand(elevatorSubsystem, 0.5, 60, 20),
//                                new PositionCommand(wristSubsystem, 140)
//                        )
//                ),
//                () -> elevatorSubsystem.getMechM() > .4
//        );
//    }
//
//    public static Command algaeStable() {
//        return new ConditionalCommand(
//                new SequentialCommandGroup( //Won't clip elevator
//                        new ParallelCommandGroup(
//                                new PositionCommand(wristSubsystem, 56)
////                                new PositionCommand(elevatorSubsystem, 0, 40, 20)
//                        ),
//                        new PositionCommand(armSubsystem, 0.5)
//                ),
//                new SequentialCommandGroup( //Will clip elevator
//                        new PositionCommand(wristSubsystem, 56),

    /// /                        new PositionCommand(elevatorSubsystem, 0, 40, 20),
//                        new PositionCommand(armSubsystem, 0.5)
//                ),
//                () -> wristSubsystem.getDegrees() < 50
//        );
//    }
    public static Command scoreL2NoDunk() {
        return new ConditionalCommand(
                new SequentialCommandGroup( //Going down
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, .08, false),
                                new PositionCommand(wristSubsystem, 140)
                                // new VelocityCommand(intakeSubsystem, 50)
                        ),
                        new PositionCommand(armSubsystem, 59) //59, .04, 148
                ),
                new SequentialCommandGroup( //Going up
                        new PositionCommand(armSubsystem, 59),
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, .08, true),
                                new PositionCommand(wristSubsystem, 140)
                                // new VelocityCommand(intakeSubsystem, 50)
                        )

                ),
                () -> elevatorSubsystem.getMechM() > .08
        ).alongWith(new InstantCommand(() -> RobotContainer.setState(State.L2)));
    }

    public static Command scoreL3NoDunk() {
        return new ConditionalCommand(
                new SequentialCommandGroup( //Going down
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, .46, false),
                                new PositionCommand(wristSubsystem, 159)
                                // new VelocityCommand(intakeSubsystem, 50)
                        ),
                        new PositionCommand(armSubsystem, 73)
                ),
                new SequentialCommandGroup( //Going up
                        new PositionCommand(armSubsystem, 73), //73, .45, 167.5
                        new ParallelCommandGroup(
                                new PositionCommand(elevatorSubsystem, .46, true),
                                new PositionCommand(wristSubsystem, 159)
                                // new VelocityCommand(intakeSubsystem, 50)
                        )
                ),
                () -> elevatorSubsystem.getMechM() > .46
        ).alongWith(new InstantCommand(() -> RobotContainer.setState(State.L3)));
    }

    public static Command scoreL4NoDunk() {
        return new SequentialCommandGroup(
                new PositionCommand(armSubsystem, 81),
                new ParallelCommandGroup(
                        new PositionCommand(elevatorSubsystem, 1.003, true),
                        new PositionCommand(wristSubsystem, 150)
                        // new VelocityCommand(intakeSubsystem, 50)
                ),
                new PositionCommand(wristSubsystem, 164.5) //81, 171
        ).alongWith(new InstantCommand(() -> RobotContainer.setState(State.L4)));
    }

    public static Command scoreL4NoDunkAuton() {
        return new SequentialCommandGroup(
                new PositionCommand(armSubsystem, 82),
                new ParallelCommandGroup(
                        new PositionCommand(elevatorSubsystem, 1.003, true),
                        new PositionCommand(wristSubsystem, 165.5)
                        // new VelocityCommand(intakeSubsystem, 50)
                ),
                new PositionCommand(wristSubsystem, 165.5) //81, 171
        ).alongWith(new InstantCommand(() -> RobotContainer.setState(State.L4)));
    }

    public static Command armL4Auton() {
        return new PositionCommand(armSubsystem, 82)
                .alongWith(new InstantCommand(() -> RobotContainer.setState(State.L4)));
    }

    public static Command scoreNoDunk() {
        SelectCommand<State> stateSelectCommand = new SelectCommand<>(
                Map.ofEntries(
                        Map.entry(State.L1, scoreL1()),
                        Map.entry(State.L2, scoreL2NoDunk()),
                        Map.entry(State.L3, scoreL3NoDunk()),
                        Map.entry(State.L4, scoreL4NoDunk()),
                        Map.entry(State.ALGAE_HIGH, removeAlgaeHigh()),
                        Map.entry(State.ALGAE_LOW, removeAlgaeLow())
                ),
                RobotContainer::getState
        );
        return stateSelectCommand;
    }

    public static Command armL2NoDunk() {
        return new PositionCommand(armSubsystem, 59).alongWith(
                new InstantCommand(() -> RobotContainer.setState(State.L2)));
    }

    public static Command armL3NoDunk() {
        return new PositionCommand(armSubsystem, 73)
                .alongWith(new InstantCommand(() -> RobotContainer.setState(State.L3)));
    }

}
