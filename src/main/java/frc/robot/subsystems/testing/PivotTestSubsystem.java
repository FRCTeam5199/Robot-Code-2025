package frc.robot.subsystems.testing;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.template.TemplateSubsystem;
import frc.robot.utility.FeedForward;
import frc.robot.utility.PID;
import frc.robot.utility.Type;

public class PivotTestSubsystem extends TemplateSubsystem {
    public PivotTestSubsystem() {
        super(Type.PIVOT, 24,
                new TrapezoidProfile.Constraints(1, 2),
                new PID(50, 0, 0), new FeedForward(.02003604, .49003605, 2.5),
                .75, .75, new double[][]{{1, 42.4286}});
        configureMotor(false, true, 80, 80);
        configurePivot(0, 57, 13.5);
        configureEncoder(30, "rio", .930176, 1, 42.4286);
    }

    @Override
    public void periodic() {
        super.periodic();
        System.out.println("Encoder rotations: " + getEncoder().getPosition().getValueAsDouble());
    }
}
