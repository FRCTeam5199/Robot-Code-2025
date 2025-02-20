package frc.robot.controls;

public enum ButtonPanelButtons {
    REEF_A(1, 4),
    REEF_B(1, 4),
    REEF_C(1, 4),
    REEF_D(1, 4),
    REEF_E(1, 4),
    REEF_F(1, 4),
    REEF_G(1, 4),
    REEF_H(1, 4),
    REEF_I(1, 4),
    REEF_J(1, 4),
    REEF_K(1, 4),
    REEF_L(1, 4);
    //TODO: Add buttons

    final int arduinoID;
    final int buttonID;

    ButtonPanelButtons(int arduinoID, int buttonID) {
        this.arduinoID = arduinoID;
        this.buttonID = buttonID;
    }
}
