package frc.robot.subsystems;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;

import frc.robot.RobotContainer;
import frc.robot.constants.Constants.IntakeConstants;
import frc.robot.subsystems.template.TemplateSubsystem;
import frc.robot.utility.Type;

public class IntakeSubsystem extends TemplateSubsystem {
    public static IntakeSubsystem intakeSubsystem;
    public LaserCan coralSensor;
    public LaserCan algaeSensor;
    private double currentSpike = 0;

    public boolean isScoringAlgae() {
        return isScoringAlgae;
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

        coralSensor = new LaserCan(IntakeConstants.CORAL_SENSOR_ID);
        try {
            coralSensor.setRangingMode(LaserCan.RangingMode.SHORT);
            coralSensor.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 8, 8));
            coralSensor.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException ignored) {
        }

        algaeSensor = new LaserCan(IntakeConstants.ALGAE_SENSOR_ID);
        try {
            algaeSensor.setRangingMode(LaserCan.RangingMode.SHORT);
            algaeSensor.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            algaeSensor.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException ignored) {
        }
    }

    public void periodic() {
        super.periodic();

        if (getStatorCurrent() < 0) currentSpike++;
        else currentSpike = 0;

//        System.out.println("has coral: " + hasCoral());
//        System.out.println("distance: " + algaeSensor.getMeasurement().distance_mm);
    }

    public static IntakeSubsystem getInstance() {
        if (intakeSubsystem == null) {
            intakeSubsystem = new IntakeSubsystem();
        }
        return intakeSubsystem;
    }

    public boolean hasCoral() {
        return coralSensor.getMeasurement().distance_mm < 5;
    }

    public boolean hasAlgae() {
        return RobotContainer.aligned();
    }

    public void setIntakeMotors(double rps, double secondaryRPS) {
        intakeSubsystem.setVelocity(rps);
        intakeSubsystem.setSecondaryVelocity(secondaryRPS);
    }

    public boolean isCurrentSpiked() {
        return getSupplyCurrent() > 10;
    }
}