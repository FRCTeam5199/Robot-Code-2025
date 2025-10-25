package frc.robot.utility;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public enum ScoringPosition {
    //Coral Setpoints
    REEF_SIDE_A(new Pose2d(2.4, 4.019, new Rotation2d(Math.toRadians(0))), new Pose2d(14.937, 4.103, new Rotation2d(Math.toRadians(180))), 18, 7, false),
    REEF_SIDE_B(new Pose2d(2.4, 4.019, new Rotation2d(Math.toRadians(0))), new Pose2d(14.937, 4.103, new Rotation2d(Math.toRadians(180))), 18, 7, true),
    REEF_SIDE_C(new Pose2d(3.451, 2.260, new Rotation2d(Math.toRadians(60))), new Pose2d(13.956, 5.603, new Rotation2d(Math.toRadians(240))), 17, 6, false),
    REEF_SIDE_D(new Pose2d(3.451, 2.260, new Rotation2d(Math.toRadians(60))), new Pose2d(13.956, 5.603, new Rotation2d(Math.toRadians(240))), 17, 6, true),
    REEF_SIDE_E(new Pose2d(5.450, 2.319, new Rotation2d(Math.toRadians(120))), new Pose2d(12.168, 5.613, new Rotation2d(Math.toRadians(300))), 22, 11, false),
    REEF_SIDE_F(new Pose2d(5.450, 2.319, new Rotation2d(Math.toRadians(120))), new Pose2d(12.168, 5.613, new Rotation2d(Math.toRadians(300))), 22, 11, true),
    REEF_SIDE_G(new Pose2d(6.494, 4.074, new Rotation2d(Math.toRadians(180))), new Pose2d(11.150, 4.029, new Rotation2d(Math.toRadians(0))), 21, 10, false),
    REEF_SIDE_H(new Pose2d(6.494, 4.074, new Rotation2d(Math.toRadians(180))), new Pose2d(11.150, 4.029, new Rotation2d(Math.toRadians(0))), 21, 10, true),
    REEF_SIDE_I(new Pose2d(5.538, 5.819, new Rotation2d(Math.toRadians(240))), new Pose2d(12.133, 2.437, new Rotation2d(Math.toRadians(60))), 20, 9, false),
    REEF_SIDE_J(new Pose2d(5.538, 5.819, new Rotation2d(Math.toRadians(240))), new Pose2d(12.133, 2.437, new Rotation2d(Math.toRadians(60))), 20, 9, true),
    REEF_SIDE_K(new Pose2d(3.422, 5.858, new Rotation2d(Math.toRadians(300))), new Pose2d(13.973, 2.437, new Rotation2d(Math.toRadians(120))), 19, 8, false),
    REEF_SIDE_L(new Pose2d(3.422, 5.858, new Rotation2d(Math.toRadians(300))), new Pose2d(13.973, 2.437, new Rotation2d(Math.toRadians(120))), 19, 8, true),

    //Algae Setpoints
    REEF_SIDE_AB(new Pose2d(2.882, 4.005, new Rotation2d(Math.toRadians(0))), new Pose2d(14.937, 4.103, new Rotation2d(Math.toRadians(180)))),
    REEF_SIDE_CD(new Pose2d(3.644, 2.657, new Rotation2d(Math.toRadians(60))), new Pose2d(13.956, 5.603, new Rotation2d(Math.toRadians(240)))),
    REEF_SIDE_EF(new Pose2d(5.258, 2.615, new Rotation2d(Math.toRadians(120))), new Pose2d(12.168, 5.613, new Rotation2d(Math.toRadians(300)))),
    REEF_SIDE_GH(new Pose2d(6.101, 4.009, new Rotation2d(Math.toRadians(180))), new Pose2d(11.150, 4.029, new Rotation2d(Math.toRadians(0)))),
    REEF_SIDE_IJ(new Pose2d(5.790, 5.410, new Rotation2d(Math.toRadians(240))), new Pose2d(12.133, 2.437, new Rotation2d(Math.toRadians(60)))),
    REEF_SIDE_KL(new Pose2d(3.687, 5.502, new Rotation2d(Math.toRadians(300))), new Pose2d(13.973, 2.437, new Rotation2d(Math.toRadians(120)))),
    BARGE(new Pose2d(7.4, 4.80, new Rotation2d(Math.toRadians(200))), new Pose2d(10.15, 3.2, new Rotation2d(Math.toRadians(20)))),
    BARGE2(new Pose2d(7.4, 4.80, new Rotation2d(Math.toRadians(200))), new Pose2d(10.05, 3.2, new Rotation2d(Math.toRadians(20))));

    final Pose2d bluePose;
    final Pose2d redPose;

    int blueAprilTagID;
    int redAprilTagID;

    final boolean isRightSide;

    ScoringPosition(Pose2d bluePose, Pose2d redPose, int blueAprilTagID, int redAprilTagID, boolean isRightSide) {
        this.bluePose = bluePose;
        this.redPose = redPose;
        this.blueAprilTagID = blueAprilTagID;
        this.redAprilTagID = redAprilTagID;
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

    public int getBlueAprilTagID() {
        return blueAprilTagID;
    }

    public int getRedAprilTagID() {
        return redAprilTagID;
    }
}
