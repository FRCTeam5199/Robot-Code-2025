package frc.robot.subsystems;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.template.AbstractSubsystem;
import frc.robot.utility.FeedForward;
import frc.robot.utility.PID;
import frc.robot.utility.Type;

public class LinearTestSubsystem extends AbstractSubsystem {
    public LinearTestSubsystem() {
        super(Type.LINEAR, new int[]{22},
                new TrapezoidProfile.Constraints(3, 7.5),
                new PID(2, 0, 0), new FeedForward(.4, .205, 8.3333333333333333333333333333333),
                .015, .015, new double[][]{{48, 14}, {48, 24}});
        configureMotors(false, true, new double[][]{{80, 80}});
        configureFollowerMotor(23);
        configureLinearMech(1.273, 0, 100);
    }
}
