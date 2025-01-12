package frc.robot.subsystems.testing;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.template.TemplateSubsystem;
import frc.robot.utility.FeedForward;
import frc.robot.utility.PID;
import frc.robot.utility.Type;

public class LinearTestSubsystem extends TemplateSubsystem {
    private static LinearTestSubsystem linearTestSubsystem;

    public static LinearTestSubsystem getInstance() {
        if (linearTestSubsystem == null) linearTestSubsystem = new LinearTestSubsystem();
        return linearTestSubsystem;
    }

    public LinearTestSubsystem() {
        super(Type.LINEAR, 22,
                new TrapezoidProfile.Constraints(150, 300),
                new PID(100, 0, 0), new FeedForward(.0225, .1525, 8.3333333333333333333333333333333),
                .015, .015, new double[][]{{15, 56}, {20, 60}});
        configureMotor(false, true, 80, 80);
        configureFollowerMotor(23);
        configureLinearMech(.0364 * Math.PI, 0, .44);
    }
    //.08928571428571419642857142857143

    @Override
    public void periodic() {
        super.periodic();
    }
}
