package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;

public class GoToCommands {
    private static PathConstraints constraints = new PathConstraints(
        1.0, 4.0, // TODO: Change once we are confident with pathfinding
                Units.degreesToRadians(540), Units.degreesToRadians(720));

    public static Command GoToCommand(int tagID) {
        double[][] reefSidePoses = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, // Red Reef Poses
                                    {1.19, 1.04, 0}, {3.3, 6.07, -60}, {5.6, 5.94, -120}, {6.27, 4.06, -180}, {5.22, 2.4, 120}, {0, 0, 0}}; // Blue Reef Poses

        Pose2d targetPose2d;
        if (tagID >= 6 && tagID <= 11) {
            targetPose2d = new Pose2d(reefSidePoses[tagID - 6][0], reefSidePoses[tagID - 6][1], new Rotation2d(Math.toDegrees(reefSidePoses[tagID - 6][2])));
            System.out.println(targetPose2d);
        } else if (tagID >= 17 && tagID <= 22) {
            targetPose2d = new Pose2d(reefSidePoses[tagID - 17][0], reefSidePoses[tagID - 17][1], new Rotation2d(Math.toDegrees((reefSidePoses[tagID - 17][2]))));
        } else { return new PrintCommand("Invalid Tag ID"); }
        return AutoBuilder.pathfindToPose(
            targetPose2d,
            constraints,
            0.0);
    }
}
