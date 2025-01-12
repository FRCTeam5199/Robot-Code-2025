package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants.ElevatorConstants;
import frc.robot.subsystems.template.TemplateSubsystem;
import frc.robot.utility.Type;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

public class ElevatorSubsystem extends TemplateSubsystem {
    public static ElevatorSubsystem elevatorSubsystem;
    public TalonFX elevator_motor = new TalonFX(ElevatorConstants.ELEVATOR_ID);

    SysIdRoutineLog elevatorLOG = new SysIdRoutineLog("Elevator Motor");

    public final SysIdRoutine sysIdRoutineArm = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volts.per(Second).of(.2),        // Use default ramp rate (1 V/s)
                    Volts.of(.6), // Reduce dynamic step voltage to 4 V to prevent brownout

                    null,        // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("Elevator_State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                    output -> elevator_motor.setControl(new VoltageOut(output)),
                    null,
                    // log->
                    //{

                    // log.motor("Pivot Motor")
                    // .voltage(Volts.mutable(0).mut_replace(arm_motor.getMotorVoltage().getValueAsDouble(), Volts))
                    // .angularPosition(Radians.mutable(0).mut_replace(arm_motor.getPosition().getValueAsDouble(), Rotations))
                    // .angularVelocity(RadiansPerSecond.mutable(0).mut_replace(arm_motor.getVelocity().getValueAsDouble(), RadiansPerSecond));
                    // },
                    this
            )
    );

    public ElevatorSubsystem(){
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

    public static ElevatorSubsystem getInstance() {
        if (elevatorSubsystem == null) {
            elevatorSubsystem = new ElevatorSubsystem();
        }
        return elevatorSubsystem;
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutineArm.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutineArm.dynamic(direction);
    }

}