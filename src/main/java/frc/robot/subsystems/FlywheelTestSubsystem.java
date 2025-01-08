package frc.robot.subsystems;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.template.AbstractSubsystem;
import frc.robot.utility.FeedForward;
import frc.robot.utility.PID;
import frc.robot.utility.Type;

public class FlywheelTestSubsystem extends AbstractSubsystem {
    public FlywheelTestSubsystem() {
        super(Type.FLYWHEEL, new int[]{26},
                new TrapezoidProfile.Constraints(90, 500),
                new PID(0, 0, 0), new FeedForward(.08, 0.16393442622950819672131147540984),
                5, 5, new double[][]{{1, 1}});
        configureMotors(false, true, new double[][]{{80, 80}});
    }
}
