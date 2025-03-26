package frc.robot.controls;

public enum ButtonPanelButtons {
    SETMODE_CORAL(1, 14),
    SETMODE_ALGAE(1, 19),
    MOVE_WRIST_INCREASE(1, 7),
    MOVE_WRIST_DECREASE(1, 6),
    MOVE_ELEVATOR_INCREASE(1, 4),
    MOVE_ELEVATOR_DECREASE(1, 3),
    MOVE_ARM_INCREASE(1, 10),
    MOVE_ARM_DECREASE(1, 13),
    MOVE_CLIMB_INCREASE(1, 22),
    MOVE_CLIMB_DECREASE(1, 21),
    SETPOINT_INTAKE_HP(1, 11),
    REEF_SCORE_L1(1, 12),
    REEF_SCORE_L2(1, 5),
    REEF_SCORE_L3(1, 8),
    REEF_SCORE_L4(1, 9),
    REEF_SIDE_A(2, 11),
    REEF_SIDE_B(2, 10),
    REEF_SIDE_C(2, 9),
    REEF_SIDE_D(2, 3),
    REEF_SIDE_E(2, 4),
    REEF_SIDE_F(2, 5),
    REEF_SIDE_G(2, 22),
    REEF_SIDE_H(2, 21),
    REEF_SIDE_I(2, 20),
    REEF_SIDE_J(2, 6),
    REEF_SIDE_K(2, 13),
    REEF_SIDE_L(2, 12),
    AUX_LEFT(1, 20),
    AUX_RIGHT(2, 7);

    final int arduinoID;
    final int buttonID;

    ButtonPanelButtons(int arduinoID, int buttonID) {
        this.arduinoID = arduinoID;
        this.buttonID = buttonID;
    }
}
