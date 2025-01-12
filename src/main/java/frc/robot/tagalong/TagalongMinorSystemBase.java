package frc.robot.tagalong;

public class TagalongMinorSystemBase {
    /* -------- MinorSystem specific disablements -------- */
    // Configured disablement for explicit on off
    // Disablement state -- based off hardware disconnects
    protected boolean _isMinorSystemDisabled = true;

    // null parser is a configured disablement
    public TagalongMinorSystemBase() {
        _isMinorSystemDisabled = false;
    }

    public boolean motorResetConfig() {
        return !_isMinorSystemDisabled;
    }

    public boolean checkInitStatus() {
        return !_isMinorSystemDisabled;
    }

    /* -------- Math utilities -------- */
    public double clamp(double target, double min, double max) {
        return Math.max(min, Math.min(max, target));
    }

    public boolean inTolerance(double position, double lowerBound, double upperBound) {
        return (position >= lowerBound) && (position <= upperBound);
    }
}
