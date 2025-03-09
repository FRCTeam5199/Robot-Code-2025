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
        3.0, 4.0, // TODO: Change once we are confident with pathfinding
                Units.degreesToRadians(540), Units.degreesToRadians(720));

    public static Command GoToCommand(int tagID) {
        double[][] reefSidePoses = {{}, {}, {}, {}, {}, {}, // Red Reef Poses
                                    {}, {}, {}, {}, {}, {}}; // Blue Reef Poses

        Pose2d targetPose2d;
        if (tagID >= 6 && tagID <= 11) {
            targetPose2d = new Pose2d(reefSidePoses[tagID - 6][0], reefSidePoses[tagID - 6][1], new Rotation2d(reefSidePoses[tagID - 6][2]));
        } else if (tagID >= 17 && tagID <= 22) {
            targetPose2d = new Pose2d(reefSidePoses[tagID - 11][0], reefSidePoses[tagID - 11][1], new Rotation2d(reefSidePoses[tagID - 11][2]));
        } else { return new PrintCommand("Invalid Tag ID"); }
        return AutoBuilder.pathfindToPose(
            targetPose2d,
            constraints,
            0.0);
    }
}
