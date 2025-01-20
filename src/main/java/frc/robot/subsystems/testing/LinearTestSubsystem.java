package frc.robot.subsystems.testing;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
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
//new FeedForward(ElevatorConstants.ELEVATOR_FF.getkS(), ElevatorConstants.ELEVATOR_FF.getkG(), ElevatorConstants.ELEVATOR_FF.getkV(), ElevatorConstants.ELEVATOR_FF.getkA()),
                new FeedForward(.0225, .1525, 11.1297), //8.8 rotations for kV to meters
                .015, 100, new double[][]{{15, 56}, {20, 60}});
        configureMotor(false, true, 80, 80, ElevatorConstants.ELEVATOR_PID.getSlot0Configs());
        configureFollowerMotor(23, true);
        configureLinearMech(.0364 * Math.PI, 0, .44);
    }
    //.08928571428571419642857142857143

    @Override
    public void periodic() {
        super.periodic();
    }

    public Command go() {
        return this.runOnce(() -> setPosition(5));
    }
}
