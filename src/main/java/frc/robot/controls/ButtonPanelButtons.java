package frc.robot.controls;

public enum ButtonPanelButtons {
    SETMODE_CORAL(1, 4),
    SETMODE_ALGAE(1, 4),
    MOVE_WRIST_INCREASE(1, 4),
    MOVE_WRIST_DECREASE(1, 4),
    MOVE_ELEVATOR_INCREASE(1, 4),
    MOVE_ELEVATOR_DECREASE(1, 4),
    MOVE_ARM_INCREASE(1, 4),
    MOVE_ARM_DECREASE(1, 4),
    MOVE_CLIMB_INCREASE(1, 3),
    MOVE_CLIMB_DECREASE(1, 6),
    SETPOINT_INTAKE_HP(1, 4),
    REEF_SCORE_L1(1, 4),
    REEF_SCORE_L2(1, 4),
    REEF_SCORE_L3(1, 4),
    REEF_SCORE_L4(1, 4),
    REEF_SIDE_A(1, 4),
    REEF_SIDE_B(1, 4),
    REEF_SIDE_C(1, 4),
    REEF_SIDE_D(1, 4),
    REEF_SIDE_E(1, 4),
    REEF_SIDE_F(1, 4),
    REEF_SIDE_G(1, 4),
    REEF_SIDE_H(1, 4),
    REEF_SIDE_I(1, 4),
    REEF_SIDE_J(1, 4),
    REEF_SIDE_K(1, 4),
    REEF_SIDE_L(1, 4);

    final int arduinoID;
    final int buttonID;

    ButtonPanelButtons(int arduinoID, int buttonID) {
        this.arduinoID = arduinoID;
        this.buttonID = buttonID;
    }
}
