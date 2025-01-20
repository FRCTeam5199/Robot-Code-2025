package frc.robot.subsystems.testing;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.template.TemplateSubsystem;
import frc.robot.utility.FeedForward;
import frc.robot.utility.PID;
import frc.robot.utility.Type;

public class RollerTestSubsystem extends TemplateSubsystem {
    private static RollerTestSubsystem rollerTestSubsystem;

    public static RollerTestSubsystem getInstance() {
        if (rollerTestSubsystem == null) rollerTestSubsystem = new RollerTestSubsystem();
        return rollerTestSubsystem;
    }

    public RollerTestSubsystem() {
        super(Type.ROLLER, 25,
                new TrapezoidProfile.Constraints(90, 200),
                new FeedForward(0.23, 0.11904761904761904761904761904762),
                3, 3, new double[][]{{1, 1}});
        configureMotor(true, false, 80, 80, new PID(0, 0, 0).getSlot0Configs());
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}
