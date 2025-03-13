package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.UserInterface;
import frc.robot.commands.PositionCommand;
import frc.robot.constants.Constants.ArmConstants;
import frc.robot.subsystems.template.TemplateSubsystem;
import frc.robot.utility.Type;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;


public class ArmSubsystem extends TemplateSubsystem {
    private static ArmSubsystem armSubsystem;
    private int currentSpike = 0;
    private int noCurrentSpike = 0;


    //.08, .5438
    public ArmSubsystem() {
        super(
                Type.PIVOT,
                ArmConstants.ARM_MOTOR_ID,
                ArmConstants.ARM_CONSTRAINTS,
                ArmConstants.ARM_FF,
                ArmConstants.ARM_LOWER_TOLERANCE,
                ArmConstants.ARM_UPPER_TOLERANCE,
                ArmConstants.ARM_GEAR_RATIO,
                "Arm Subsystem"
        );

        configureMotor(
                ArmConstants.LEFT_ARM_INVERTED,
                ArmConstants.ARM_BRAKE,
                ArmConstants.ARM_SUPPLY_CURRENT_LIMIT,
                ArmConstants.ARM_STATOR_CURRENT_LIMIT,
                ArmConstants.ARM_SLOT0_CONFIGS
        );
        configureFollowerMotor(
                ArmConstants.ARM_FOLLOW_MOTOR_ID,
                ArmConstants.ARM_FOLLOWER_INVERTED
        );

        configurePivot(
                ArmConstants.ARM_MIN,
                ArmConstants.ARM_MAX,
                ArmConstants.ARM_FF_OFFSET
        );

//        configureEncoder(
//                ArmConstants.ARM_CANCODER_ID,
//                ArmConstants.ARM_CANCODER_CANBUS,
//                ArmConstants.ARM_CANCODER_MAGNET_OFFSET,
//                ArmConstants.ARM_SENSOR_TO_MECH_GEAR_RATIO,
//                ArmConstants.ARM_MOTOR_TO_SENSOR_GEAR_RATIO
//        );


    }


    @Override
    public void periodic() {
        super.periodic();
        if (isProfileFinished()) {
            //   setVoltage((ArmConstants.ARM_FF.getkG()) / Math.cos(Units.rotationsToRadians(getEncoderRot())));

        }
        if (getStatorCurrent() > 45) currentSpike++;
        else noCurrentSpike++;

        if (noCurrentSpike >= 2) {
            currentSpike = 0;
            noCurrentSpike = 0;
        }

        // if (!DriverStation.isFMSAttached()) {
        //     if (UserInterface.getTestComponent("Offset Arm").getString("") != "") {
        //         this.setOffset(UserInterface.getTestComponent("Offset Arm").getDouble(0));
        //     }

        //     if (UserInterface.getTestComponent("Set Arm").getString("") != "") {
        //         this.setPosition(UserInterface.getTestComponent("Set Arm").getDouble(0));
        //     }
        // }
    }

    public static ArmSubsystem getInstance() {
        if (armSubsystem == null) {
            armSubsystem = new ArmSubsystem();
        }
        return armSubsystem;
    }


    public final SysIdRoutine sysIdRoutineArm = new SysIdRoutine(


            new SysIdRoutine.Config(
                    null,        // Use default ramp rate (1 V/s)
                    Volts.of(1), // Reduce dynamic step voltage to 4 V to prevent brownout

                    null,        // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("Arm_State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                    output -> setVoltage(output.baseUnitMagnitude()),
                    null,
                    //    (SysIdRoutineLog log)->
                    //     {

                    //     log.motor("Pivot Motor")
                    //     .voltage(BaseUnits.VoltageUnit.of(arm_motor.getMotorVoltage().getValueAsDouble()))
                    //     .angularPosition(Units.Rotation.of(arm_motor.getPosition().getValueAsDouble()))
                    //     .angularVelocity(Rotations.per(Second).of(arm_motor.getVelocity().getValueAsDouble()));
                    //     },
                    this
            )
    );

    public Command sysId() {
        return Commands.sequence(
                sysIdRoutineArm
                        .quasistatic(SysIdRoutine.Direction.kForward)
                        .until(() -> (getMotorRot() > 120)),
                sysIdRoutineArm
                        .quasistatic(SysIdRoutine.Direction.kReverse)
                        .until(() -> (getMotorRot() < 3)),
                sysIdRoutineArm
                        .dynamic(SysIdRoutine.Direction.kForward)
                        .until(() -> (getMotorRot() > 120)),
                sysIdRoutineArm
                        .dynamic(SysIdRoutine.Direction.kReverse)
                        .until(() -> (getMotorRot() < 3)));
    }

    public boolean isAtBottom() {
        return currentSpike >= 3;
    }


    // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    //     return sysIdRoutineArm.quasistatic(direction).until(()-> (getRotorPosit > .003418) || (getPosition().getValueAsDouble() < -0.000247));
    // }

    // /**
    //  * Runs the SysId Dynamic test in the given direction for the routine
    //  * specified by {@link #m_sysIdRoutineToApply}.
    //  *
    //  * @param direction Direction of the SysId Dynamic test
    //  * @return Command to run
    //  */
    // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    //     return sysIdRoutineArm.dynamic(direction).until(()-> (getPosition().getValueAsDouble() > .003418) || (getPosition().getValueAsDouble() < 0));
    // }


}