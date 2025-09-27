package frc.robot.subsystems;

import au.grapplerobotics.CanBridge;
import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;

import frc.robot.constants.Constants.IntakeConstants;
import frc.robot.subsystems.template.TemplateSubsystem;
import frc.robot.utility.Type;

public class IntakeSubsystem extends TemplateSubsystem {
    public static IntakeSubsystem intakeSubsystem;
    public LaserCan intakeSensor;
    private boolean hasCoral;

    public boolean isScoringAlgae() {
        return isScoringAlgae;
    }

    public void setScoringAlgae(boolean scoringAlgae) {
        isScoringAlgae = scoringAlgae;
    }

    private boolean isScoringAlgae = false;

    public IntakeSubsystem() {
        super(Type.ROLLER, //Left Motor
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

        configureSecondaryMotor( //Right Motor
                IntakeConstants.INTAKE_SECONDARY_ID,
                IntakeConstants.INTAKE_SECONDARY_FF,
                IntakeConstants.INTAKE_SECONDARY_INVERT,
                IntakeConstants.INTAKE_BRAKE,
                IntakeConstants.INTAKE_SUPPLY_CURRENT_LIMIT,
                IntakeConstants.INTAKE_STATOR_CURRENT_LIMIT,
                IntakeConstants.INTAKE_SECONDARY_SLOT0_CONFIGS
        );

        intakeSensor = new LaserCan(IntakeConstants.INTAKE_SENSOR_ID);
        try {
            intakeSensor.setRangingMode(LaserCan.RangingMode.SHORT);
            intakeSensor.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            intakeSensor.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException ignored) {
        }
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
        return intakeSensor.getMeasurement().distance_mm < 5;
    }

    public boolean isCurrentSpiked() {
        return getSupplyCurrent() > 10;
    }
}