package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.Constants.ArmConstants;
import frc.robot.subsystems.template.TemplateSubsystem;
import frc.robot.utility.Type;
import edu.wpi.first.math.util.Units;


public class ArmSubsystem extends TemplateSubsystem {
    private static ArmSubsystem armSubsystem;

    //.08, .5438
    public ArmSubsystem() {
        super(
                Type.PIVOT,
                ArmConstants.ARM_MOTOR_ID,
                ArmConstants.ARM_CONSTRAINTS,
                ArmConstants.ARM_PID,
                ArmConstants.ARM_FF,
                ArmConstants.ARM_LOWER_TOLERANCE,
                ArmConstants.ARM_UPPER_TOLERANCE,
                ArmConstants.MOTOR_TO_MECH_GEAR_RATIO
        );

        configureMotor(
                ArmConstants.ARM_INVERTED,
                ArmConstants.ARM_BRAKE,
                ArmConstants.ARM_SUPPLY_CURRENT_LIMIT,
                ArmConstants.ARM_STATOR_CURRENT_LIMIT
        );

        configurePivot(
                ArmConstants.ARM_MIN,
                ArmConstants.ARM_MAX,
                ArmConstants.ARM_FF_OFFSET
        );

        configureEncoder(
                ArmConstants.ARM_CANCODER_ID,
                ArmConstants.ARM_CANCODER_CANBUS,
                ArmConstants.ARM_CANCODER_MAGNET_OFFSET,
                ArmConstants.ARM_SENSOR_TO_MECH_GEAR_RATIO,
                ArmConstants.ARM_MOTOR_TO_SENSOR_GEAR_RATIO
        );


    }


    @Override
    public void periodic() {
        super.periodic();
        if (isProfileFinished()) {
//            setVoltage(.05 + (ArmConstants.ARM_FF.getkG() * Math.cos(Units.rotationsToRadians(getAbsPosition()))));

        }
        System.out.println("Mech Degrees: " + getEncoderRot() * 360d);

    }


    public Command setGround() {
        return new InstantCommand(() -> setPosition(ArmConstants.GROUND));
    }

    public Command setGroundBack() {
        return new InstantCommand(() -> setPosition(ArmConstants.GROUND_2));
    }

    public Command setL1() {
        return new InstantCommand(() -> setPosition(ArmConstants.L1));
    }

    public Command setL2() {
        return new InstantCommand(() -> setPosition(ArmConstants.L2));
    }

    public Command setL3() {
        return new InstantCommand(() -> setPosition(ArmConstants.L3));
    }

    public Command setL4() {
        return new InstantCommand(() -> setPosition(ArmConstants.L4));
    }


    public static ArmSubsystem getInstance() {
        if (armSubsystem == null) {
            armSubsystem = new ArmSubsystem();
        }
        return armSubsystem;
    }


//         public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
//             return sysIdRoutineArm.quasistatic(direction).until(()-> (arm_motor.getPosition().getValueAsDouble() > .003418) || (arm_motor.getPosition().getValueAsDouble() < -0.000247));
//         }

//         /**
//          * Runs the SysId Dynamic test in the given direction for the routine
//          * specified by {@link #m_sysIdRoutineToApply}.
//          *
//          * @param direction Direction of the SysId Dynamic test
//          * @return Command to run
//          */
//         public Command sysIdDynamic(SysIdRoutine.Direction direction) {
//             return sysIdRoutineArm.dynamic(direction).until(()-> (arm_motor.getPosition().getValueAsDouble() > .003418) || (arm_motor.getPosition().getValueAsDouble() < 0));
//         }


//         public final SysIdRoutine sysIdRoutineArm = new SysIdRoutine(


//         new SysIdRoutine.Config(
//              null,        // Use default ramp rate (1 V/s)
//             Volts.of(1), // Reduce dynamic step voltage to 4 V to prevent brownout

//             null,        // Use default timeout (10 s)
//             // Log state with SignalLogger class
//             state -> SignalLogger.writeString("Arm_State", state.toString())
//         ),
//         new SysIdRoutine.Mechanism(
//             output -> arm_motor.setControl(new VoltageOut(output).withEnableFOC((true))),
//             null,
//         //    (SysIdRoutineLog log)-> 
//         //     {

//         //     log.motor("Pivot Motor")
//         //     .voltage(BaseUnits.VoltageUnit.of(arm_motor.getMotorVoltage().getValueAsDouble()))
//         //     .angularPosition(Units.Rotation.of(arm_motor.getPosition().getValueAsDouble()))
//         //     .angularVelocity(Rotations.per(Second).of(arm_motor.getVelocity().getValueAsDouble()));
//         //     },
//             this
//         )
//     );

//          public Command sysId() {
//     return Commands.sequence(
//         sysIdRoutineArm
//             .quasistatic(SysIdRoutine.Direction.kForward)
//             .until(() -> (arm_motor.getPosition().getValueAsDouble() > .00341)),
//         sysIdRoutineArm
//             .quasistatic(SysIdRoutine.Direction.kReverse)
//             .until(() -> (arm_motor.getPosition().getValueAsDouble() < 0)),
//         sysIdRoutineArm
//             .dynamic(SysIdRoutine.Direction.kForward)
//             .until(() -> (arm_motor.getPosition().getValueAsDouble() > .003418)),
//         sysIdRoutineArm
//             .dynamic(SysIdRoutine.Direction.kReverse)
//             .until(() -> (arm_motor.getPosition().getValueAsDouble() < 0)));
//   }


}