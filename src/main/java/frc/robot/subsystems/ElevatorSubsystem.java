package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.UserInterface;
import frc.robot.constants.Constants.ElevatorConstants;
import frc.robot.subsystems.template.AbstractSubsystem;
import frc.robot.utility.SubsystemPrint;
import frc.robot.utility.Type;

public class ElevatorSubsystem extends AbstractSubsystem {
    private static ElevatorSubsystem elevatorSubsystem;
    private static boolean subsystemOk = false;

    private ElevatorSubsystem(){
        super(Type.LINEAR, 
        ElevatorConstants.ELEVATOR_ID, 
        ElevatorConstants.ELEVATOR_CONSTRAINTS, 
        ElevatorConstants.ELEVATOR_PID, 
        ElevatorConstants.ELEVATOR_FF,
        ElevatorConstants.ELEVATOR_MIN, 
        ElevatorConstants.ELEVATOR_MAX, 
        ElevatorConstants.ELEVATOR_GEARING);

        configureMotor(ElevatorConstants.INVERT, ElevatorConstants.BRAKE, ElevatorConstants.SUPPLY_CURRENT_LIMIT, ElevatorConstants.STATOR_CURRENT_LIMIT);
    }

    public static ElevatorSubsystem getInstance() {
        if (elevatorSubsystem == null) elevatorSubsystem = new ElevatorSubsystem();
        return elevatorSubsystem;
    }

    /**
   * The subsystem's initalize method.
   */
    public void init() {
        try {
        configureMotor(false, false, 80, 80);

        initComponents();

        new SubsystemPrint(this, "Initalized");
        subsystemOk = true;
        } catch (Exception e) {
            System.err.println(e.getStackTrace());
        }
    }
    
    public void initComponents() {
        UserInterface.createTestComponent("Elevator Status", false, BuiltInWidgets.kBooleanBox, 0, 0, 1, 1, null);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (subsystemOk) subsystemPeriodic();
        else UserInterface.setTestComponent("Elevator Status", false);
    }

    /**
     * The subsystem's periodic method. Only runs when the subsystem is ok with no
     * errors.
     */
    private void subsystemPeriodic() {
        UserInterface.setTestComponent("Elevator Status", true);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public Command setL1(){
        return new InstantCommand(()-> setPosition(0));
    
    }

    public Command setL2(){
        return new InstantCommand(()-> setPosition(0));

    }

    public Command setL3(){
        return new InstantCommand(()-> setPosition(0));

    }

    public Command setL4(){
        return new InstantCommand(()-> setPosition(0));

    }

    public Command setHP(){
        return new InstantCommand(()-> setPosition(0));

    }

    public Command setHeight(double position){
        return new InstantCommand(()-> setPosition(position));
    }
}