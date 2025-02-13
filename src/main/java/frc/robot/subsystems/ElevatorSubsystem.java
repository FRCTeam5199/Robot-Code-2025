package frc.robot.subsystems;

import frc.robot.constants.Constants.ElevatorConstants;
import frc.robot.subsystems.template.TemplateSubsystem;
import frc.robot.utility.Type;

public class ElevatorSubsystem extends TemplateSubsystem {
    public static ElevatorSubsystem elevatorSubsystem;
    private int currentSpike = 0;
    private int noCurrentSpike = 0;

    public ElevatorSubsystem() {
        super(Type.LINEAR,
                ElevatorConstants.ELEVATOR1_ID,
                ElevatorConstants.ELEVATOR_CONSTRAINTS,
                ElevatorConstants.ELEVATOR_FF,
                ElevatorConstants.ELEVATOR_MIN,
                ElevatorConstants.ELEVATOR_MAX,
                ElevatorConstants.ELEVATOR_GEARING,
                "Elevator"
        );

        configureMotor(
                ElevatorConstants.INVERT,
                ElevatorConstants.ELEVATOR_BRAKE,
                ElevatorConstants.SUPPLY_CURRENT_LIMIT,
                ElevatorConstants.STATOR_CURRENT_LIMIT,
                ElevatorConstants.ELEVATOR_SLOT0_CONFIGS);

        configureFollowerMotor(ElevatorConstants.ELEVATOR2_ID,
                ElevatorConstants.FOLLOWER_OPPOSE_MASTER_DIRECTION);

        configureLinearMech(ElevatorConstants.DRUM_CIRCUMFERENCE,
                ElevatorConstants.ELEVATOR_LOWER_TOLERANCE,
                ElevatorConstants.ELEVATOR_UPPER_TOLERANCE);
    }

    public void periodic() {
        super.periodic();
        // System.out.println("is at goal position: " + isMechAtGoal(false));
        // System.out.println("meters: " + getMechM());

//        System.out.println("Elevator Mech M: " + getMechM());

        if (getSupplyCurrent() > 25) currentSpike++;
        else noCurrentSpike++;

        if (noCurrentSpike >= 3) {
            currentSpike = 0;
            noCurrentSpike = 0;
        }
    }

    public static ElevatorSubsystem getInstance() {
        if (elevatorSubsystem == null) {
            elevatorSubsystem = new ElevatorSubsystem();
        }
        return elevatorSubsystem;
    }

    public boolean isAtBottom() {
        return currentSpike >= 10;
    }
}