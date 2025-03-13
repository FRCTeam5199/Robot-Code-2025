package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.UserInterface;
import frc.robot.constants.Constants.ElevatorConstants;
import frc.robot.subsystems.template.TemplateSubsystem;
import frc.robot.utility.Type;

public class ElevatorSubsystem extends TemplateSubsystem {
    public static ElevatorSubsystem elevatorSubsystem;
    private int currentSpike = 0;
    private int noCurrentSpike = 0;

    public ElevatorSubsystem() {
        super(Type.LINEAR,
                ElevatorConstants.ELEVATOR_LEFT_ID,
                ElevatorConstants.ELEVATOR_CONSTRAINTS,
                ElevatorConstants.ELEVATOR_FF,
                ElevatorConstants.ELEVATOR_LOWER_TOLERANCE,
                ElevatorConstants.ELEVATOR_UPPER_TOLERANCE,
                ElevatorConstants.ELEVATOR_GEARING,
                "Elevator"
        );

        configureMotor(
                ElevatorConstants.INVERT,
                ElevatorConstants.ELEVATOR_BRAKE,
                ElevatorConstants.ELEVATOR_SUPPLY_CURRENT_LIMIT,
                ElevatorConstants.ELEVATOR_STATOR_CURRENT_LIMIT,
                ElevatorConstants.ELEVATOR_SLOT0_CONFIGS);

        configureFollowerMotor(ElevatorConstants.ELEVATOR_RIGHT_ID,
                ElevatorConstants.FOLLOWER_OPPOSE_MASTER_DIRECTION);

        configureLinearMech(ElevatorConstants.DRUM_CIRCUMFERENCE,
                ElevatorConstants.ELEVATOR_MIN,
                ElevatorConstants.ELEVATOR_MAX);
    }

    public void periodic() {
        super.periodic();

        if (getSupplyCurrent() > 2.75) currentSpike++;
        else noCurrentSpike++;

        if (noCurrentSpike >= 3) {
            currentSpike = 0;
            noCurrentSpike = 0;
        }

        // if (!DriverStation.isFMSAttached()) {
        //     if (UserInterface.getTestComponent("Offset Elevator").getString("") != "") {
        //         this.setOffset(UserInterface.getTestComponent("Offset Elevator").getDouble(0));
        //     }
        //     if (UserInterface.getTestComponent("Set Elevator").getString("") != "") {
        //         this.setPosition(UserInterface.getTestComponent("Set Elevator").getDouble(0));
        //     }
        // }
    }

    public static ElevatorSubsystem getInstance() {
        if (elevatorSubsystem == null) {
            elevatorSubsystem = new ElevatorSubsystem();
        }
        return elevatorSubsystem;
    }

    public boolean isAtBottom() {
        return currentSpike >= 5;
    }
}