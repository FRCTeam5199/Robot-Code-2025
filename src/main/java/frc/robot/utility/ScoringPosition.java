package frc.robot.utility;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public enum ScoringPosition {
    //Coral Setpoints
    //TODO: change red side to middle
    REEF_SIDE_A(new Pose2d(2.697, 4.019, new Rotation2d(Math.toRadians(0))), new Pose2d(14.650, 3.859, new Rotation2d(Math.toRadians(180))), false),
    REEF_SIDE_B(new Pose2d(2.697, 4.019, new Rotation2d(Math.toRadians(0))), new Pose2d(14.650, 4.176, new Rotation2d(Math.toRadians(180))), true),
    REEF_SIDE_C(new Pose2d(3.608, 2.461, new Rotation2d(Math.toRadians(60))), new Pose2d(13.976, 5.339, new Rotation2d(Math.toRadians(240))), false),
    REEF_SIDE_D(new Pose2d(3.608, 2.461, new Rotation2d(Math.toRadians(60))), new Pose2d(13.699, 5.519, new Rotation2d(Math.toRadians(240))), true),
    REEF_SIDE_E(new Pose2d(5.370, 2.437, new Rotation2d(Math.toRadians(120))), new Pose2d(12.440, 5.546, new Rotation2d(Math.toRadians(300))), false),
    REEF_SIDE_F(new Pose2d(5.370, 2.437, new Rotation2d(Math.toRadians(120))), new Pose2d(12.108, 5.325, new Rotation2d(Math.toRadians(300))), true),
    REEF_SIDE_G(new Pose2d(6.353, 4.043, new Rotation2d(Math.toRadians(180))), new Pose2d(11.450, 4.171, new Rotation2d(Math.toRadians(0))), false),
    REEF_SIDE_H(new Pose2d(6.353, 4.043, new Rotation2d(Math.toRadians(180))), new Pose2d(11.450, 3.887, new Rotation2d(Math.toRadians(0))), true),
    REEF_SIDE_I(new Pose2d(5.382, 5.613, new Rotation2d(Math.toRadians(240))), new Pose2d(12.136, 2.697, new Rotation2d(Math.toRadians(60))), false),
    REEF_SIDE_J(new Pose2d(5.382, 5.613, new Rotation2d(Math.toRadians(240))), new Pose2d(12.413, 2.531, new Rotation2d(Math.toRadians(60))), true),
    REEF_SIDE_K(new Pose2d(3.608, 5.673, new Rotation2d(Math.toRadians(300))), new Pose2d(13.685, 2.504, new Rotation2d(Math.toRadians(120))), false),
    REEF_SIDE_L(new Pose2d(3.608, 5.673, new Rotation2d(Math.toRadians(300))), new Pose2d(13.990, 2.697, new Rotation2d(Math.toRadians(120))), true),

    //Algae Setpoints
    REEF_SIDE_AB(new Pose2d(2.882, 4.005, new Rotation2d(Math.toRadians(0))), new Pose2d(13.650, 4.075, new Rotation2d(Math.toRadians(180)))),
    REEF_SIDE_CD(new Pose2d(3.644, 2.657, new Rotation2d(Math.toRadians(60))), new Pose2d(13.838, 5.429, new Rotation2d(Math.toRadians(240)))),
    REEF_SIDE_EF(new Pose2d(5.258, 2.615, new Rotation2d(Math.toRadians(120))), new Pose2d(12.274, 5.436, new Rotation2d(Math.toRadians(300)))),
    REEF_SIDE_GH(new Pose2d(6.101, 4.009, new Rotation2d(Math.toRadians(180))), new Pose2d(11.450, 4.029, new Rotation2d(Math.toRadians(0)))),
    REEF_SIDE_IJ(new Pose2d(5.279, 5.421, new Rotation2d(Math.toRadians(240))), new Pose2d(12.275, 2.614, new Rotation2d(Math.toRadians(60)))),
    REEF_SIDE_KL(new Pose2d(3.687, 5.502, new Rotation2d(Math.toRadians(300))), new Pose2d(13.838, 2.600, new Rotation2d(Math.toRadians(120)))),
    BARGE(new Pose2d(7.4, 4.80, new Rotation2d(Math.toRadians(200))), new Pose2d(0, 0, new Rotation2d(Math.toRadians(0)))),
    BARGE2(new Pose2d(7.45, 4.80, new Rotation2d(Math.toRadians(200))), new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))));

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
