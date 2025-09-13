package frc.robot.utility;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public enum ScoringPosition {
    REEF_SIDE_A(new Pose2d(2.882, 4.176, new Rotation2d(0)), new Pose2d(0, 0, new Rotation2d(0)), false),
    REEF_SIDE_B(new Pose2d(2.882, 3.834, new Rotation2d(0)), new Pose2d(0, 0, new Rotation2d(0)), true),
    REEF_SIDE_C(new Pose2d(3.511, 2.721, new Rotation2d(60)), new Pose2d(0, 0, new Rotation2d(0)), false),
    REEF_SIDE_D(new Pose2d(3.776, 2.592, new Rotation2d(60)), new Pose2d(0, 0, new Rotation2d(0)), true),
    REEF_SIDE_E(new Pose2d(5.113, 2.525, new Rotation2d(120)), new Pose2d(0, 0, new Rotation2d(0)), false),
    REEF_SIDE_F(new Pose2d(5.403, 2.704, new Rotation2d(120)), new Pose2d(0, 0, new Rotation2d(0)), true),
    REEF_SIDE_G(new Pose2d(6.101, 3.850, new Rotation2d(180)), new Pose2d(0, 0, new Rotation2d(0)), false),
    REEF_SIDE_H(new Pose2d(6.101, 4.168, new Rotation2d(180)), new Pose2d(0, 0, new Rotation2d(0)), true),
    REEF_SIDE_I(new Pose2d(5.422, 5.339, new Rotation2d(240)), new Pose2d(0, 0, new Rotation2d(0)), false),
    REEF_SIDE_J(new Pose2d(5.135, 5.503, new Rotation2d(240)), new Pose2d(0, 0, new Rotation2d(0)), true),
    REEF_SIDE_K(new Pose2d(3.802, 5.500, new Rotation2d(300)), new Pose2d(0, 0, new Rotation2d(0)), false),
    REEF_SIDE_L(new Pose2d(3.571, 5.503, new Rotation2d(300)), new Pose2d(0, 0, new Rotation2d(0)), true);

    final Pose2d bluePose;
    final Pose2d redPose;

    final boolean isRightSide;

    ScoringPosition(Pose2d bluePose, Pose2d redPose, boolean isRightSide) {
        this.bluePose = bluePose;
        this.redPose = redPose;

        this.isRightSide = isRightSide;
    }

    public Pose2d getRedPose() {
        return redPose;
    }

    public Pose2d getBluePose() {
        return bluePose;
    }

    public boolean isRightSide() {
        return isRightSide;
    }

}
