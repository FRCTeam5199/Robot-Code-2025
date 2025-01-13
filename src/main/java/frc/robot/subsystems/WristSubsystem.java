package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants.WristConstants;
import frc.robot.constants.Constants.WristConstants;
import frc.robot.tagalong.WristParser;
import frc.robot.tagalong.PivotAugment;
import frc.robot.tagalong.PivotParser;
import frc.robot.tagalong.TagalongPivot;
import frc.robot.tagalong.TagalongSubsystemBase;
import frc.robot.tagalong.WristParser;
import frc.robot.tagalong.WristParser;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

public class WristSubsystem extends TagalongSubsystemBase implements PivotAugment{
    private final TagalongPivot wrist;
        private static WristSubsystem wristSubsystem;

    public TalonFX wrist_motor = new TalonFX(WristConstants.WRIST_MOTOR_ID);
    public CANcoder wrist_CANCoder = new CANcoder(WristConstants.WRIST_CANCODER_ID);

    SysIdRoutineLog wristLOG = new SysIdRoutineLog("Wrist Motor");

    public final SysIdRoutine sysIdRoutineWrist = new SysIdRoutine(

            new SysIdRoutine.Config(
                    Volts.per(Second).of(.2),        // Use default ramp rate (1 V/s)
                    Volts.of(.6), // Reduce dynamic step voltage to 4 V to prevent brownout

                    null,        // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("Wrist_State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                    output -> wrist_motor.setControl(new VoltageOut(output)),
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





    public WristSubsystem(){
        super();
        
        WristConstants.TALON_FX_CONFIGURATION.CurrentLimits.SupplyCurrentLimit
                = WristConstants.WRIST_SUPPLY_CURRENT_LIMIT;
        WristConstants.TALON_FX_CONFIGURATION.CurrentLimits.StatorCurrentLimit
                = WristConstants.WRIST_STATOR_CURRENT_LIMIT;
        WristConstants.TALON_FX_CONFIGURATION.CurrentLimits.SupplyCurrentLimitEnable = true;
        WristConstants.TALON_FX_CONFIGURATION.CurrentLimits.StatorCurrentLimitEnable = true;
        WristConstants.TALON_FX_CONFIGURATION.Feedback.FeedbackRemoteSensorID
                = WristConstants.WRIST_CANCODER_ID;
        WristConstants.TALON_FX_CONFIGURATION.Feedback.FeedbackSensorSource
                = FeedbackSensorSourceValue.FusedCANcoder;
        WristConstants.TALON_FX_CONFIGURATION.Feedback.SensorToMechanismRatio
                = WristConstants.WRIST_SENSOR_TO_MECH_GEAR_RATIO;
        WristConstants.TALON_FX_CONFIGURATION.Feedback.RotorToSensorRatio
                = WristConstants.WRIST_MOTOR_TO_SENSOR_GEAR_RATIO;
        WristConstants.TALON_FX_CONFIGURATION.MotorOutput.NeutralMode
                = NeutralModeValue.Brake;
        WristConstants.TALON_FX_CONFIGURATION.MotorOutput.Inverted =
                WristConstants.WRIST_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        WristConstants.WRIST_CANCODER_CONFIGURATION.MagnetSensor.AbsoluteSensorDiscontinuityPoint
                = WristConstants.WRIST_CANCODER_DISCONTINUITY_POINT;
        WristConstants.WRIST_CANCODER_CONFIGURATION.MagnetSensor.SensorDirection
                = WristConstants.WRIST_CANCODER_DIRECTION;
        WristConstants.WRIST_CANCODER_CONFIGURATION.MagnetSensor.MagnetOffset
                = WristConstants.WRIST_CANCODER_MAGNET_OFFSET;


        wrist = new TagalongPivot(WristConstants.WRIST_MOTOR_ID, WristConstants.WRIST_CANCODER_ID,
                WristConstants.WRIST_FF, WristConstants.WRIST_FF_OFFSET,
                WristConstants.WRIST_LOWER_TOLERANCE, WristConstants.WRIST_UPPER_TOLERANCE,
                WristConstants.WRIST_MIN, WristConstants.WRIST_MAX,
                WristConstants.WRIST_MAX_VELOCITY, WristConstants.WRIST_MAX_ACCELERATION,
                WristConstants.WRIST_MOTOR_TO_MECH_GEAR_RATIO,
                WristConstants.TALON_FX_CONFIGURATION, WristConstants.WRIST_CANCODER_CONFIGURATION,
                WristConstants.WRIST_SLOT0_CONFIGS);


        setName("Wrist");
        BaseStatusSignal.setUpdateFrequencyForAll(250,
                wrist_motor.getPosition(),
                wrist_motor.getVelocity(),
                wrist_motor.getMotorVoltage());

        wrist_motor.optimizeBusUtilization();



    }


    public Command setGround(){
        return new InstantCommand(()->wrist.setPivotProfile(WristConstants.GROUND));
    }

    public Command setGroundBack(){
        return new InstantCommand(()->wrist.setPivotProfile(WristConstants.GROUND_2));
    }
    public Command setL1(){
        return new InstantCommand(()->wrist.setPivotProfile(WristConstants.L1));
    }
    public Command setL2(){
        return new InstantCommand(()->wrist.setPivotProfile(WristConstants.L2));
    }
    public Command setL3(){
        return new InstantCommand(()->wrist.setPivotProfile(WristConstants.L3));
    }
    public Command setL4(){
        return new InstantCommand(()->wrist.setPivotProfile(WristConstants.L4));
    }


    public static WristSubsystem getInstance() {
   
        return wristSubsystem;
    }



    @Override
    public TagalongPivot getPivot() {
        return wrist;
    }


    @Override
    public TagalongPivot getPivot(int i) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPivot'");
    }


    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutineWrist.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutineWrist.dynamic(direction);
    }




}