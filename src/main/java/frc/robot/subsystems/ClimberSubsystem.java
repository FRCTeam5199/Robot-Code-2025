package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.ClimberConstants;
import frc.robot.subsystems.template.TemplateSubsystem;
import frc.robot.utility.Type;

public class ClimberSubsystem extends TemplateSubsystem {
    private static ClimberSubsystem climber;

    public ClimberSubsystem() {
        super(Type.PIVOT,
                Constants.ClimberConstants.CLIMBER_ID,
                Constants.ClimberConstants.CLIMBER_CONSTRAINTS,
                Constants.ClimberConstants.CLIMBER_FF,
                Constants.ClimberConstants.CLIMBER_lowerTOLERANCE,
                Constants.ClimberConstants.CLIMBER_upperTOLERANCE,
                new double[][]{{125, 1}},
                "Climber"
        );

        configureMotor(
                ClimberConstants.CLIMBER_INVERT,
                ClimberConstants.CLIMBER_BRAKE,
                ClimberConstants.CLIMBER_SUPPLY_CURRENT_LIMIT,
                ClimberConstants.CLIMBER_STATOR_CURRENT_LIMIT,
                ClimberConstants.CLIMBER_SLOT0_CONFIGS
        );

        configurePivot(
                ClimberConstants.CLIMBER_LOW_LIMIT,
                ClimberConstants.CLIMBER_HIGH_LIMIT,
                0
        );
    }

    public static ClimberSubsystem getInstance() {
        if (climber == null) {
            climber = new ClimberSubsystem();
        }
        return climber;
    }

    public void drop() {
        setPercent(0.45);
    }
    public void stopDrop() {
        setPercent(0);
    }
    public void retract() {
        setPercent(-0.3);
    }
}