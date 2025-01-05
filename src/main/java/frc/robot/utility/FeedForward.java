package frc.robot.utility;

public class FeedForward {
    private double kS, kV, kG, kA;

    public FeedForward(double kS, double kV) {
        this.kS = kS;
        this.kV = kV;
    }

    public FeedForward(double kS, double kG, double kV) {
        this.kS = kS;
        this.kG = kG;
        this.kV = kV;
    }

    public FeedForward(double kS, double kG, double kV, double kA) {
        this.kS = kS;
        this.kG = kG;
        this.kV = kV;
        this.kA = kA;
    }

    public double getkS() {
        return kS;
    }

    public double getkG() {
        return kG;
    }

    public double getkV() {
        return kV;
    }

    public double getkA() {
        return kA;
    }
}
