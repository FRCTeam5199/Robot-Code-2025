package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;

import frc.robot.constants.Constants.IntakeConstants;
import frc.robot.subsystems.template.TemplateSubsystem;
import frc.robot.utility.Type;

public class IntakeSubsystem extends TemplateSubsystem {
    public static IntakeSubsystem intakeSubsystem;
    public AnalogInput intakeSensor;
    private boolean hasCoral;
    private boolean isAboveSpeed = false;
    private int isAboveSpeedCounter = 0;

    public boolean isScoringAlgae() {
        return isScoringAlgae;
    }

    public void setScoringAlgae(boolean scoringAlgae) {
        isScoringAlgae = scoringAlgae;
    }

    private boolean isScoringAlgae = false;

    public IntakeSubsystem() {
        super(Type.ROLLER,
                IntakeConstants.INTAKE_ID,
                IntakeConstants.INTAKE_CONSTRAINTS,
                IntakeConstants.INTAKE_FF,
                IntakeConstants.INTAKE_LOWER_TOLERANCE,
                IntakeConstants.INTAKE_UPPER_TOLERANCE,
                IntakeConstants.INTAKE_GEAR_RATIO,
                "Intake");

        configureMotor(
                IntakeConstants.INTAKE_INVERT,
                IntakeConstants.INTAKE_BRAKE,
                IntakeConstants.INTAKE_SUPPLY_CURRENT_LIMIT,
                IntakeConstants.INTAKE_STATOR_CURRENT_LIMIT,
                IntakeConstants.INTAKE_SLOT0_CONFIGS
        );

        configureSecondaryMotor(
                IntakeConstants.INTAKE_SECONDARY_ID,
                IntakeConstants.INTAKE_SECONDARY_FF,
                IntakeConstants.INTAKE_SECONDARY_INVERT,
                IntakeConstants.INTAKE_BRAKE,
                IntakeConstants.INTAKE_SUPPLY_CURRENT_LIMIT,
                IntakeConstants.INTAKE_STATOR_CURRENT_LIMIT,
                IntakeConstants.INTAKE_SECONDARY_SLOT0_CONFIGS
        );

        intakeSensor = new AnalogInput(IntakeConstants.INTAKE_SENSOR_ID);
    }

    public void periodic() {
        super.periodic();
    }

    public static IntakeSubsystem getInstance() {
        if (intakeSubsystem == null) {
            intakeSubsystem = new IntakeSubsystem();
        }
        return intakeSubsystem;
    }

    public boolean hasCoral() {
        return hasCoral;
    }

    public boolean isCurrentSpiked() {
        return getSupplyCurrent() > 10;
    }
}