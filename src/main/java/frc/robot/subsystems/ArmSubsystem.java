package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecondSquared;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants.ArmConstants;
import frc.robot.tagalong.PivotAugment;
import frc.robot.tagalong.TagalongPivot;
import edu.wpi.first.networktables.GenericEntry;
import frc.robot.tagalong.TagalongSubsystemBase;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;


public class ArmSubsystem extends TagalongSubsystemBase implements PivotAugment {
    private final TagalongPivot arm;
    private static ArmSubsystem armSubsystem;

    public TalonFX arm_motor = new TalonFX(ArmConstants.ARM_MOTOR_ID);
    public CANcoder arm_CANCoder = new CANcoder(ArmConstants.ARM_CANCODER_ID);

    SysIdRoutineLog armLOG = new SysIdRoutineLog("Arm Motor");

    double prevPosition;

    public final SysIdRoutine sysIdRoutineArm = new SysIdRoutine(
        
        
    new SysIdRoutine.Config(
         null,        // Use default ramp rate (1 V/s)
        Volts.of(1), // Reduce dynamic step voltage to 4 V to prevent brownout
        
        null,        // Use default timeout (10 s)
        // Log state with SignalLogger class
        state -> SignalLogger.writeString("Arm_State", state.toString())
    ),
    new SysIdRoutine.Mechanism(
        output -> arm_motor.setControl(new VoltageOut(output)),
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


    public ArmSubsystem() {
        super();
        ArmConstants.TALON_FX_CONFIGURATION.CurrentLimits.SupplyCurrentLimit
                = ArmConstants.ARM_SUPPLY_CURRENT_LIMIT;
        ArmConstants.TALON_FX_CONFIGURATION.CurrentLimits.StatorCurrentLimit
                = ArmConstants.ARM_STATOR_CURRENT_LIMIT;
        ArmConstants.TALON_FX_CONFIGURATION.CurrentLimits.SupplyCurrentLimitEnable = true;
        ArmConstants.TALON_FX_CONFIGURATION.CurrentLimits.StatorCurrentLimitEnable = true;
        ArmConstants.TALON_FX_CONFIGURATION.Feedback.FeedbackRemoteSensorID
                = ArmConstants.ARM_CANCODER_ID;
        ArmConstants.TALON_FX_CONFIGURATION.Feedback.FeedbackSensorSource
                = FeedbackSensorSourceValue.FusedCANcoder;
        ArmConstants.TALON_FX_CONFIGURATION.Feedback.SensorToMechanismRatio
                = ArmConstants.ARM_SENSOR_TO_MECH_GEAR_RATIO;
        ArmConstants.TALON_FX_CONFIGURATION.Feedback.RotorToSensorRatio
                = ArmConstants.ARM_MOTOR_TO_SENSOR_GEAR_RATIO;
        ArmConstants.TALON_FX_CONFIGURATION.MotorOutput.NeutralMode
                = NeutralModeValue.Brake;
        ArmConstants.TALON_FX_CONFIGURATION.MotorOutput.Inverted =
                ArmConstants.ARM_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        ArmConstants.ARM_CANCODER_CONFIGURATION.MagnetSensor.AbsoluteSensorDiscontinuityPoint
                = ArmConstants.ARM_CANCODER_DISCONTINUITY_POINT;
        ArmConstants.ARM_CANCODER_CONFIGURATION.MagnetSensor.SensorDirection
                = ArmConstants.ARM_CANCODER_DIRECTION;
        ArmConstants.ARM_CANCODER_CONFIGURATION.MagnetSensor.MagnetOffset
                = ArmConstants.ARM_CANCODER_MAGNET_OFFSET;


        arm = new TagalongPivot(ArmConstants.ARM_MOTOR_ID, ArmConstants.ARM_CANCODER_ID,
                ArmConstants.ARM_FF, ArmConstants.ARM_FF_OFFSET,
                ArmConstants.ARM_LOWER_TOLERANCE, ArmConstants.ARM_UPPER_TOLERANCE,
                ArmConstants.ARM_MIN, ArmConstants.ARM_MAX,
                ArmConstants.ARM_MAX_VELOCITY, ArmConstants.ARM_MAX_ACCELERATION,
                ArmConstants.ARM_MOTOR_TO_MECH_GEAR_RATIO,
                ArmConstants.TALON_FX_CONFIGURATION, ArmConstants.ARM_CANCODER_CONFIGURATION,
                ArmConstants.ARM_SLOT0_CONFIGS);

    }



    @Override
    public void periodic() {
//        System.out.println(arm.getVoltage());

    }


    public Command setGround() {
        return new InstantCommand(() -> arm.setPivotProfile(ArmConstants.GROUND));
    }

    public Command setGroundBack() {
        return new InstantCommand(() -> arm.setPivotProfile(ArmConstants.GROUND_2));
    }

    public Command setL1() {
        return new InstantCommand(() -> arm.setPivotProfile(ArmConstants.L1));
    }

    public Command setL2() {
        return new InstantCommand(() -> arm.setPivotProfile(ArmConstants.L2));
    }

    public Command setL3() {
        return new InstantCommand(() -> arm.setPivotProfile(ArmConstants.L3));
    }

    public Command setL4() {
        return new InstantCommand(() -> arm.setPivotProfile(ArmConstants.L4));
    }


    public static ArmSubsystem getInstance() {
        if (armSubsystem == null) {
            armSubsystem = new ArmSubsystem();
        }
        return armSubsystem;
    }


    @Override
    public TagalongPivot getPivot() {
        return arm;
    }


    @Override
    public TagalongPivot getPivot(int i) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPivot'");
    }



 


        public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
            return sysIdRoutineArm.quasistatic(direction).until(()-> (arm_motor.getPosition().getValueAsDouble() > .003418) || (arm_motor.getPosition().getValueAsDouble() < -0.000247));
        }
    
        /**
         * Runs the SysId Dynamic test in the given direction for the routine
         * specified by {@link #m_sysIdRoutineToApply}.
         *
         * @param direction Direction of the SysId Dynamic test
         * @return Command to run
         */
        public Command sysIdDynamic(SysIdRoutine.Direction direction) {
            return sysIdRoutineArm.dynamic(direction).until(()-> (arm_motor.getPosition().getValueAsDouble() > .003418) || (arm_motor.getPosition().getValueAsDouble() < 0));
        }

         public Command sysId() {
    return Commands.sequence(
        sysIdRoutineArm
            .quasistatic(SysIdRoutine.Direction.kForward)
            .until(() -> (arm_motor.getPosition().getValueAsDouble() > .00341)),
        sysIdRoutineArm
            .quasistatic(SysIdRoutine.Direction.kReverse)
            .until(() -> (arm_motor.getPosition().getValueAsDouble() < 0)),
        sysIdRoutineArm
            .dynamic(SysIdRoutine.Direction.kForward)
            .until(() -> (arm_motor.getPosition().getValueAsDouble() > .003418)),
        sysIdRoutineArm
            .dynamic(SysIdRoutine.Direction.kReverse)
            .until(() -> (arm_motor.getPosition().getValueAsDouble() < 0)));
  }




}