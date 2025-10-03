package frc.robot.utility;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public enum ScoringPosition {
    //Coral Setpoints
    REEF_SIDE_A(new Pose2d(2.882, 4.176, new Rotation2d(Math.toRadians(0))), new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))), false),
    REEF_SIDE_B(new Pose2d(2.882, 3.834, new Rotation2d(Math.toRadians(0))), new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))), true),
    REEF_SIDE_C(new Pose2d(3.511, 2.721, new Rotation2d(Math.toRadians(60))), new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))), false),
    REEF_SIDE_D(new Pose2d(3.776, 2.592, new Rotation2d(Math.toRadians(60))), new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))), true),
    REEF_SIDE_E(new Pose2d(5.113, 2.525, new Rotation2d(Math.toRadians(120))), new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))), false),
    REEF_SIDE_F(new Pose2d(5.403, 2.704, new Rotation2d(Math.toRadians(120))), new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))), true),
    REEF_SIDE_G(new Pose2d(6.101, 3.850, new Rotation2d(Math.toRadians(180))), new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))), false),
    REEF_SIDE_H(new Pose2d(6.101, 4.168, new Rotation2d(Math.toRadians(180))), new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))), true),
    REEF_SIDE_I(new Pose2d(5.422, 5.339, new Rotation2d(Math.toRadians(240))), new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))), false),
    REEF_SIDE_J(new Pose2d(5.135, 5.503, new Rotation2d(Math.toRadians(240))), new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))), true),
    REEF_SIDE_K(new Pose2d(3.802, 5.500, new Rotation2d(Math.toRadians(300))), new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))), false),
    REEF_SIDE_L(new Pose2d(3.571, 5.503, new Rotation2d(Math.toRadians(300))), new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))), true),

    //Algae Setpoints
    REEF_SIDE_AB(new Pose2d(2.882, 4.005, new Rotation2d(Math.toRadians(0))), new Pose2d(0, 0, new Rotation2d(Math.toRadians(0)))),
    REEF_SIDE_CD(new Pose2d(3.644, 2.657, new Rotation2d(Math.toRadians(60))), new Pose2d(0, 0, new Rotation2d(Math.toRadians(0)))),
    REEF_SIDE_EF(new Pose2d(5.258, 2.615, new Rotation2d(Math.toRadians(120))), new Pose2d(0, 0, new Rotation2d(Math.toRadians(0)))),
    REEF_SIDE_GH(new Pose2d(6.101, 4.009, new Rotation2d(Math.toRadians(180))), new Pose2d(0, 0, new Rotation2d(Math.toRadians(0)))),
    REEF_SIDE_IJ(new Pose2d(5.279, 5.421, new Rotation2d(Math.toRadians(240))), new Pose2d(0, 0, new Rotation2d(Math.toRadians(0)))),
    REEF_SIDE_KL(new Pose2d(3.687, 5.502, new Rotation2d(Math.toRadians(300))), new Pose2d(0, 0, new Rotation2d(Math.toRadians(0)))),

    BARGE(new Pose2d(7.55, 5.3, new Rotation2d(Math.toRadians(180))), new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))));

    final Pose2d bluePose;
    final Pose2d redPose;

    final boolean isRightSide;

    ScoringPosition(Pose2d bluePose, Pose2d redPose, boolean isRightSide) {
        this.bluePose = bluePose;
        this.redPose = redPose;

        this.isRightSide = isRightSide;
    }

    ScoringPosition(Pose2d bluePose, Pose2d redPose) {
        this.bluePose = bluePose;
        this.redPose = redPose;

        //non-relevant for Algae
        isRightSide = false;
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
