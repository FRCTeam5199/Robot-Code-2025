package frc.robot.utility;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public enum ScoringPosition {
    //Coral Setpoints
    REEF_SIDE_A(new Pose2d(2.697, 4.019, new Rotation2d(Math.toRadians(0))), new Pose2d(14.937, 4.103, new Rotation2d(Math.toRadians(180))),
            new Pose2d(2.661, 3.755, new Rotation2d(Math.toRadians(180))), new Pose2d(14.865, 4.307, new Rotation2d(Math.toRadians(0))), 18, 7, false),
    REEF_SIDE_B(new Pose2d(2.697, 4.019, new Rotation2d(Math.toRadians(0))), new Pose2d(14.937, 4.103, new Rotation2d(Math.toRadians(180))),
            new Pose2d(2.661, 3.755, new Rotation2d(Math.toRadians(180))), new Pose2d(14.865, 4.307, new Rotation2d(Math.toRadians(0))), 18, 7, true),
    REEF_SIDE_C(new Pose2d(3.608, 2.461, new Rotation2d(Math.toRadians(60))), new Pose2d(13.956, 5.603, new Rotation2d(Math.toRadians(240))),
            new Pose2d(3.932, 2.389, new Rotation2d(Math.toRadians(240))), new Pose2d(13.606, 5.793, new Rotation2d(Math.toRadians(60))), 17, 8, false),
    REEF_SIDE_D(new Pose2d(3.608, 2.461, new Rotation2d(Math.toRadians(60))), new Pose2d(13.956, 5.603, new Rotation2d(Math.toRadians(240))),
            new Pose2d(3.932, 2.389, new Rotation2d(Math.toRadians(240))), new Pose2d(13.606, 5.793, new Rotation2d(Math.toRadians(60))), 17, 8, true),
    REEF_SIDE_E(new Pose2d(5.370, 2.437, new Rotation2d(Math.toRadians(120))), new Pose2d(12.168, 5.613, new Rotation2d(Math.toRadians(300))),
            new Pose2d(5.586, 2.628, new Rotation2d(Math.toRadians(300))), new Pose2d(11.868, 5.410, new Rotation2d(Math.toRadians(120))), 22, 9, false),
    REEF_SIDE_F(new Pose2d(5.370, 2.437, new Rotation2d(Math.toRadians(120))), new Pose2d(12.168, 5.613, new Rotation2d(Math.toRadians(300))),
            new Pose2d(5.586, 2.628, new Rotation2d(Math.toRadians(300))), new Pose2d(11.868, 5.410, new Rotation2d(Math.toRadians(120))), 22, 9, true),
    REEF_SIDE_G(new Pose2d(6.353, 4.043, new Rotation2d(Math.toRadians(180))), new Pose2d(11.150, 4.029, new Rotation2d(Math.toRadians(0))),
            new Pose2d(6.246, 4.307, new Rotation2d(Math.toRadians(0))), new Pose2d(11.268, 3.695, new Rotation2d(Math.toRadians(180))), 21, 10, false),
    REEF_SIDE_H(new Pose2d(6.353, 4.043, new Rotation2d(Math.toRadians(180))), new Pose2d(11.150, 4.029, new Rotation2d(Math.toRadians(0))),
            new Pose2d(6.246, 4.307, new Rotation2d(Math.toRadians(0))), new Pose2d(11.268, 3.695, new Rotation2d(Math.toRadians(180))), 21, 10, true),
    REEF_SIDE_I(new Pose2d(5.382, 5.410, new Rotation2d(Math.toRadians(240))), new Pose2d(12.133, 2.437, new Rotation2d(Math.toRadians(60))),
            new Pose2d(5.107, 5.757, new Rotation2d(Math.toRadians(60))), new Pose2d(12.431, 2.317, new Rotation2d(Math.toRadians(240))), 20, 11, false),
    REEF_SIDE_J(new Pose2d(5.382, 5.410, new Rotation2d(Math.toRadians(240))), new Pose2d(12.133, 2.437, new Rotation2d(Math.toRadians(60))),
            new Pose2d(5.107, 5.757, new Rotation2d(Math.toRadians(60))), new Pose2d(12.431, 2.317, new Rotation2d(Math.toRadians(240))), 20, 11, true),
    REEF_SIDE_K(new Pose2d(3.608, 5.673, new Rotation2d(Math.toRadians(300))), new Pose2d(13.973, 2.437, new Rotation2d(Math.toRadians(120))),
            new Pose2d(3.369, 5.386, new Rotation2d(Math.toRadians(120))), new Pose2d(14.241, 2.628, new Rotation2d(Math.toRadians(300))), 19, 6, false),
    REEF_SIDE_L(new Pose2d(3.608, 5.673, new Rotation2d(Math.toRadians(300))), new Pose2d(13.973, 2.437, new Rotation2d(Math.toRadians(120))),
            new Pose2d(3.369, 5.386, new Rotation2d(Math.toRadians(120))), new Pose2d(14.241, 2.628, new Rotation2d(Math.toRadians(300))), 19, 6, true),

    //Algae Setpoints
    REEF_SIDE_AB(new Pose2d(2.882, 4.005, new Rotation2d(Math.toRadians(0))), new Pose2d(14.937, 4.103, new Rotation2d(Math.toRadians(180)))),
    REEF_SIDE_CD(new Pose2d(3.644, 2.657, new Rotation2d(Math.toRadians(60))), new Pose2d(13.956, 5.603, new Rotation2d(Math.toRadians(240)))),
    REEF_SIDE_EF(new Pose2d(5.258, 2.615, new Rotation2d(Math.toRadians(120))), new Pose2d(12.168, 5.613, new Rotation2d(Math.toRadians(300)))),
    REEF_SIDE_GH(new Pose2d(6.101, 4.009, new Rotation2d(Math.toRadians(180))), new Pose2d(11.150, 4.029, new Rotation2d(Math.toRadians(0)))),
    REEF_SIDE_IJ(new Pose2d(5.790, 5.410, new Rotation2d(Math.toRadians(240))), new Pose2d(12.133, 2.437, new Rotation2d(Math.toRadians(60)))),
    REEF_SIDE_KL(new Pose2d(3.687, 5.502, new Rotation2d(Math.toRadians(300))), new Pose2d(13.973, 2.437, new Rotation2d(Math.toRadians(120)))),
    BARGE(new Pose2d(7.4, 4.80, new Rotation2d(Math.toRadians(200))), new Pose2d(10.05, 3.2, new Rotation2d(Math.toRadians(20)))),
    BARGE2(new Pose2d(7.4, 4.80, new Rotation2d(Math.toRadians(200))), new Pose2d(10.05, 3.2, new Rotation2d(Math.toRadians(20))));

    final Pose2d bluePose;
    final Pose2d redPose;

    Pose2d bluePoseBackwards;
    Pose2d redPoseBackwards;

    int blueAprilTagID;
    int redAprilTagID;

    final boolean isRightSide;

    ScoringPosition(Pose2d bluePose, Pose2d redPose, Pose2d bluePoseBackwards, Pose2d redPoseBackwards, int blueAprilTagID, int redAprilTagID, boolean isRightSide) {
        this.bluePose = bluePose;
        this.redPose = redPose;
        this.bluePoseBackwards = bluePoseBackwards;
        this.redPoseBackwards = redPoseBackwards;
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


    public Pose2d getRedPoseBackwards() {
        return redPoseBackwards;
    }

    public Pose2d getBluePoseBackwards() {
        return bluePoseBackwards;
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
