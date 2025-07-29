package frc.robot.utility;

public enum ScoringPosition {
    REEF_SIDE_A_BLUE(0, 0, 0, false),
    REEF_SIDE_B_BLUE(0, 0, 0, true),
    REEF_SIDE_C_BLUE(0, 0, 0, false),
    REEF_SIDE_D_BLUE(3.776, 2.592, 60, true),
    REEF_SIDE_E_BLUE(0, 0, 0, false),
    REEF_SIDE_F_BLUE(0, 0, 0, true),
    REEF_SIDE_G_BLUE(0, 0, 0, false),
    REEF_SIDE_H_BLUE(0, 0, 0, true),
    REEF_SIDE_I_BLUE(0, 0, 0, false),
    REEF_SIDE_J_BLUE(0, 0, 0, true);

    final double goalX, goalY, goalDegrees;
    final boolean isRightSide;

    ScoringPosition(double goalX, double goalY, double goalDegrees, boolean isRightSide) {
        this.goalX = goalX;
        this.goalY = goalY;
        this.goalDegrees = goalDegrees;
        this.isRightSide = isRightSide;
    }

    public boolean isRightSide() {
        return isRightSide;
    }

    public double getGoalDegrees() {
        return goalDegrees;
    }

    public double getGoalY() {
        return goalY;
    }

    public double getGoalX() {
        return goalX;
    }
}
