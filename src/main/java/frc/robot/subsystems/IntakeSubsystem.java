package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.constants.Constants.IntakeConstants;
import frc.robot.subsystems.template.TemplateSubsystem;
import frc.robot.utility.Type;

public class IntakeSubsystem extends TemplateSubsystem {
    public static IntakeSubsystem intakeSubsystem;
    public TalonFX intake_motor = new TalonFX(IntakeConstants.INTAKE_ID);

    public IntakeSubsystem() {
        super(Type.ROLLER,
                IntakeConstants.INTAKE_ID,
                IntakeConstants.INTAKE_CONSTRAINTS,
                IntakeConstants.INTAKE_FEEDFORWARD,
                IntakeConstants.INTAKE_lowerTOLERANCE,
                IntakeConstants.INTAKE_upperTOLERANCE,
                IntakeConstants.INTAKE_gearRatios,
                "Intake");

        configureMotor(
                IntakeConstants.INTAKE_INVERT,
                IntakeConstants.INTAKE_BRAKE,
                IntakeConstants.INTAKE_SUPPLY_CURRENT_LIMIT,
                IntakeConstants.INTAKE_STATOR_CURRENT_LIMIT,
                IntakeConstants.INTAKE_SLOT0_CONFIGS
        );
    }

    public void periodic() {
        super.periodic();

//        System.out.println("Intake: " + getMechVelocity());

    }

    public static IntakeSubsystem getInstance() {
        if (intakeSubsystem == null) {
            intakeSubsystem = new IntakeSubsystem();
        }
        return intakeSubsystem;
    }

    public void intake() {
        setPercent(1);
    }

    public void stopIntake() {
        setPercent(0);
    }

    public void outtake() {
        setPercent(-1);
    }

    //Peak naming :fire:
    public boolean isIntooken() {
        return getStatorCurrent() > 25;
    }
}