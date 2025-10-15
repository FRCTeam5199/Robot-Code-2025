package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;

import frc.robot.RobotContainer;
import frc.robot.subsystems.*;
import frc.robot.utility.ScoringPosition;

import java.util.function.Supplier;

public class DriveToPoseCommand extends Command {

    private final static AprilTagSubsystem aprilTagSubsystem = AprilTagSubsystem.getInstance();
    private final static CommandSwerveDrivetrain commandSwerveDrivetrain = RobotContainer.commandSwerveDrivetrain;
    public final static SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDesaturateWheelSpeeds(true)
            .withDeadband(RobotContainer.MaxSpeed * .05).withRotationalDeadband(RobotContainer.MaxAngularRate * .05) // Add a 10% deadband
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

    public DriveToPoseCommand() {

    }

    @Override
    public void initialize() {
        commandSwerveDrivetrain.applyRequest(() ->
                drive.withVelocityX(0).
                        withVelocityX(0).
                        withRotationalRate(0)
        ).schedule();
        if (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue))
            AutoBuilder.pathfindToPose(
                            RobotContainer.getCurrentScoringPosition().getBluePose(),
                            new PathConstraints(5.5, 5.5,
                                    Units.degreesToRadians(540d), Units.degreesToRadians(720d)), 0d)
                    .until(() -> aprilTagSubsystem.getClosestTagID()
                            == RobotContainer.getCurrentScoringPosition().getBlueAprilTagID())
                    .schedule();
        else
            AutoBuilder.pathfindToPose(
                            RobotContainer.getCurrentScoringPosition().getRedPose(),
                            new PathConstraints(5.5, 5.5,
                                    Units.degreesToRadians(540d), Units.degreesToRadians(720d)), 0d)
                    .until(() -> aprilTagSubsystem.getClosestTagID()
                            == RobotContainer.getCurrentScoringPosition().getRedAprilTagID())
                    .schedule();
    }

    @Override
    public void execute() {
        if (aprilTagSubsystem.getClosestTagXYYaw()[0] == 0) {
            if (DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue))
                if (aprilTagSubsystem.getClosestTagID()
                        == RobotContainer.getCurrentScoringPosition().getBlueAprilTagID())
                    this.end(false);
                else if (aprilTagSubsystem.getClosestTagID()
                        == RobotContainer.getCurrentScoringPosition().getRedAprilTagID())
                    this.end(false);
        }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        if (aprilTagSubsystem.getClosestTagXYYaw()[0] == 0) return false;
        if (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue))
            return aprilTagSubsystem.getClosestTagID()
                    == RobotContainer.getCurrentScoringPosition().getBlueAprilTagID();
        else
            return aprilTagSubsystem.getClosestTagID()
                    == RobotContainer.getCurrentScoringPosition().getRedAprilTagID();
    }
}
