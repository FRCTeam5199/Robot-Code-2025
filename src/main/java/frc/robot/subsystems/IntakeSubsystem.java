package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.AnalogInput;

import frc.robot.RobotContainer;
import frc.robot.constants.Constants.IntakeConstants;
import frc.robot.subsystems.template.TemplateSubsystem;
import frc.robot.utility.State;
import frc.robot.utility.Type;

public class IntakeSubsystem extends TemplateSubsystem {
    public static IntakeSubsystem intakeSubsystem;
    public TalonFX intake_motor = new TalonFX(IntakeConstants.INTAKE_ID);
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

        intakeSensor = new AnalogInput(IntakeConstants.INTAKE_SENSOR_ID);
    }

    public void periodic() {
        super.periodic();

        hasCoral = intakeSensor.getValue() > 100;

        if (super.isAboveSpeed()) isAboveSpeedCounter++;
        else isAboveSpeedCounter = 0;

        isAboveSpeed = isAboveSpeedCounter > 2;

        if ((RobotContainer.getState() == State.ALGAE_LOW
                || RobotContainer.getState() == State.ALGAE_HIGH
                || RobotContainer.getState() == State.BARGE
                || RobotContainer.getState() == State.PROCESSOR) && !isScoringAlgae) setVelocity(-50);
    }

    public static IntakeSubsystem getInstance() {
        if (intakeSubsystem == null) {
            intakeSubsystem = new IntakeSubsystem();
        }
        return intakeSubsystem;
    }

    public void intake() {
        setVelocity(75);
    }

    public void stopIntake() {
        setPercent(0);
    }

    public void outtake() {
        setVelocity(-75);
    }

    public boolean hasCoral() {
        return hasCoral;
    }

    @Override
    public boolean isAboveSpeed() {
        return isAboveSpeed;
    }

    public boolean isCurrentSpiked() {
        return getSupplyCurrent() > 10;
    }
}