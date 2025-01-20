package frc.robot.subsystems.testing;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.Constants.ArmConstants;
import frc.robot.subsystems.template.TemplateSubsystem;
import frc.robot.utility.FeedForward;
import frc.robot.utility.PID;
import frc.robot.utility.Type;

public class PivotTestSubsystem extends TemplateSubsystem {
    //old FF values: ks: .02003604, Kg: 0.49003604, KV: 2.5
    // kg = .348, ks = .14816, Kv = 6.16786
    public PivotTestSubsystem() {
        super(Type.PIVOT, 24,
                new TrapezoidProfile.Constraints(1, 2),
                new PID(50, 0, 0), new FeedForward(.14816, .348, 2.5),
                .75, .75, new double[][]{{1, 42.4286}},
                "Pivot test"
                
            );
        configureMotor(true, true, 80, 80, ArmConstants.ARM_PID.getSlot0Configs());
        configurePivot(0, 57, 0);
        configureEncoder(30, "rio", ArmConstants.ARM_CANCODER_MAGNET_OFFSET, 1, 1 / 42.4286);
    }

    @Override
    public void periodic() {
        super.periodic();
        System.out.println("Encoder degrees: " + Units.rotationsToDegrees(getEncoder().getAbsolutePosition().getValueAsDouble()));

        if (isProfileFinished()) {
//            setVoltage(ArmConstants.ARM_FF.getkG() / Math.cos(Units.rotationsToRadians(getAbsPosition())));

        }
    }
}
