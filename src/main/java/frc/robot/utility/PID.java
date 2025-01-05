package frc.robot.utility;

import com.ctre.phoenix6.configs.Slot0Configs;

public class PID {
    public final double P, I, D;
    private Slot0Configs slot0Configs;

    public PID(double p, double i, double d) {
        P = p;
        I = i;
        D = d;

        slot0Configs = new Slot0Configs();
        slot0Configs.kP = p;
        slot0Configs.kI = i;
        slot0Configs.kD = d;
    }

    public double getP() {
        return P;
    }

    public double getI() {
        return I;
    }

    public double getD() {
        return D;
    }

    public Slot0Configs getSlot0Configs() {
        return slot0Configs;
    }
}
