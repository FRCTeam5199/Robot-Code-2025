package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants.WristConstants;
import frc.robot.utility.Type;

import frc.robot.subsystems.template.TemplateSubsystem;


import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

public class WristSubsystem extends TemplateSubsystem {
    private static WristSubsystem wristSubsystem;
    private int currentSpike = 0;
    private int noCurrentSpike = 0;

    public WristSubsystem() {

        super(
                Type.PIVOT,
                WristConstants.WRIST_MOTOR_ID,
                WristConstants.WRIST_CONSTRAINTS,
                WristConstants.WRIST_FF,
                WristConstants.WRIST_LOWER_TOLERANCE,
                WristConstants.WRIST_UPPER_TOLERANCE,
                WristConstants.WRIST_GEAR_RATIO,
                "Wrist"
        );

        configureMotor(
                WristConstants.WRIST_INVERTED,
                WristConstants.WRIST_BRAKE,
                WristConstants.WRIST_SUPPLY_CURRENT_LIMIT,
                WristConstants.WRIST_STATOR_CURRENT_LIMIT,
                WristConstants.WRIST_SLOT0_CONFIGS
        );

        configurePivot(
                WristConstants.WRIST_MIN,
                WristConstants.WRIST_MAX,
                WristConstants.WRIST_FF_OFFSET
        );
    }

    public void periodic() {
        super.periodic();

        if (getSupplyCurrent() > 1) currentSpike++;
        else noCurrentSpike++;

        if (noCurrentSpike >= 2) {
            currentSpike = 0;
            noCurrentSpike = 0;
        }

//        System.out.println("wrist degrees: " + wristSubsystem.getDegrees());
    }


    public static WristSubsystem getInstance() {
        if (wristSubsystem == null) {
            wristSubsystem = new WristSubsystem();
        }
        return wristSubsystem;
    }


    public final SysIdRoutine sysIdRoutineWrist = new SysIdRoutine(

            new SysIdRoutine.Config(
                    Volts.per(Second).of(.2),        // Use default ramp rate (1 V/s)
                    Volts.of(.6), // Reduce dynamic step voltage to 4 V to prevent brownout

                    null,        // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("Wrist_State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                    output -> setVoltage(output.baseUnitMagnitude()),
                    null,
                    // log->
                    //{

                    // log.motor("Pivot Motor")
                    // .voltage(Volts.mutable(0).mut_replace(wrist_motor.getMotorVoltage().getValueAsDouble(), Volts))
                    // .angularPosition(Radians.mutable(0).mut_replace(wrist_motor.getPosition().getValueAsDouble(), Rotations))
                    // .angularVelocity(RadiansPerSecond.mutable(0).mut_replace(wrist_motor.getVelocity().getValueAsDouble(), RadiansPerSecond));
                    // },
                    this
            )
    );

    public Command sysId() {
        return Commands.sequence(
                sysIdRoutineWrist
                        .quasistatic(SysIdRoutine.Direction.kForward)
                        .until(() -> (getMotorRot() > 120)),
                sysIdRoutineWrist
                        .quasistatic(SysIdRoutine.Direction.kReverse)
                        .until(() -> (getMotorRot() < 3)),
                sysIdRoutineWrist
                        .dynamic(SysIdRoutine.Direction.kForward)
                        .until(() -> (getMotorRot() > 120)),
                sysIdRoutineWrist
                        .dynamic(SysIdRoutine.Direction.kReverse)
                        .until(() -> (getMotorRot() < 3)));
    }

    public boolean isAtBottom() {
        return currentSpike >= 7;
    }
}