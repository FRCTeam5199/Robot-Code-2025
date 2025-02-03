package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants.ElevatorConstants;
import frc.robot.subsystems.template.TemplateSubsystem;
import frc.robot.utility.Type;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.concurrent.TimeUnit;

public class ElevatorSubsystem extends TemplateSubsystem {
    public static ElevatorSubsystem elevatorSubsystem;
    public TalonFX elevator1_motor = new TalonFX(ElevatorConstants.ELEVATOR1_ID);
    public TalonFX elevator2_motor = new TalonFX(ElevatorConstants.ELEVATOR2_ID);


    SysIdRoutineLog elevatorLOG = new SysIdRoutineLog("Elevator Motor");

    public final SysIdRoutine sysIdRoutineElevator = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volts.per(Second).of(.01
                    ),        // Use default ramp rate (1 V/s)
                    Volts.of(1), // Reduce dynamic step voltage to 4 V to prevent brownout

                    Time.ofBaseUnits(10000, Seconds),  // default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("Elevator_State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                    output -> {
                        elevator1_motor.setControl(new VoltageOut(output).withEnableFOC(true));
                        elevator2_motor.setControl(new Follower(ElevatorConstants.ELEVATOR1_ID, true));
                    },
                    null,
                    // log->
                    //{

                    // log.motor("Pivot Motor")
                    // .voltage(Volts.mutable(0).mut_replace(elevator_motor.getMotorVoltage().getValueAsDouble(), Volts))
                    // .angularPosition(Radians.mutable(0).mut_replace(elevator_motor.getPosition().getValueAsDouble(), Rotations))
                    // .angularVelocity(RadiansPerSecond.mutable(0).mut_replace(elevator_motor.getVelocity().getValueAsDouble(), RadiansPerSecond));
                    // },
                    this
            )
    );


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
    }

    public Command setL1() {
        return new InstantCommand(() -> setPosition(0));

    }

    public Command setL2() {
        return new InstantCommand(() -> setPosition(.2));

    }

    public Command setL3() {
        return new InstantCommand(() -> setPosition(.3));

    }

    public Command setL4() {
        return new InstantCommand(() -> setPosition(6));

    }

    public Command setHP() {
        return new InstantCommand(() -> setPosition(.5));

    }

    public Command setBase() {
        return new InstantCommand(() -> setPosition(0));
    }

    public Command setHeight(double position) {
        return new InstantCommand(() -> setPosition(position));
    }

    public static ElevatorSubsystem getInstance() {
        if (elevatorSubsystem == null) {
            elevatorSubsystem = new ElevatorSubsystem();
        }
        return elevatorSubsystem;
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutineElevator.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutineElevator.dynamic(direction);
    }

    public Command sysId() {
        return Commands.sequence(
                sysIdRoutineElevator
                        .quasistatic(SysIdRoutine.Direction.kForward)
                        .until(() -> (elevator1_motor.getPosition().getValueAsDouble() > 34)),
                sysIdRoutineElevator
                        .quasistatic(SysIdRoutine.Direction.kReverse)
                        .until(() -> (elevator1_motor.getPosition().getValueAsDouble() < 1)),
                sysIdRoutineElevator
                        .dynamic(SysIdRoutine.Direction.kForward)
                        .until(() -> (elevator1_motor.getPosition().getValueAsDouble() > 34)),
                sysIdRoutineElevator
                        .dynamic(SysIdRoutine.Direction.kReverse)
                        .until(() -> (elevator1_motor.getPosition().getValueAsDouble() < 1)));
    }


}