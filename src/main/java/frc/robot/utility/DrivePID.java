package frc.robot.utility;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public enum DrivePID {
    AUTO_ALIGNING(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), new Pose2d(0, 0, Rotation2d.fromDegrees(0))),
    HP_RIGHT(new Pose2d(1.139, 1.003, Rotation2d.fromDegrees(55)), new Pose2d(0, 0, Rotation2d.fromDegrees(0))),
    HP_LEFT(new Pose2d(1.187, 7.060, Rotation2d.fromDegrees(-52.943)), new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

    final Pose2d bluePose, redPose;

    DrivePID(Pose2d bluePose, Pose2d redPose) {
        this.bluePose = bluePose;
        this.redPose = redPose;
    }

    public Pose2d getRedPose() {
        return redPose;
    }

    public Pose2d getBluePose() {
        return bluePose;
    }
}
