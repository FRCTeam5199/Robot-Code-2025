package frc.robot.subsystems.template;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.FeedForward;
import frc.robot.utility.Type;

public class TemplateSubsystem extends SubsystemBase {
    private TalonFX motor;
    private TalonFXConfiguration motorConfig;

    private TalonFX followerMotor;
    private Follower follower;
    private TalonFXConfiguration followerConfig;

    private CANcoder encoder;
    private CANcoderConfiguration encoderConfig;

    private TrapezoidProfile profile;
    private TrapezoidProfile.State initState;
    private TrapezoidProfile.State currentState;
    private TrapezoidProfile.State goalState;
    private TrapezoidProfile.Constraints constraints;
    private double goal;
    private boolean followLastMechProfile = false;

    private PositionVoltage positionVoltage;
    private VelocityVoltage velocityVoltage;
    private MotionMagicVoltage magicMan;

    private SimpleMotorFeedforward simpleMotorFF;
    private ElevatorFeedforward linearFF;
    private ArmFeedforward pivotFF;

    private double lowerTolerance;
    private double upperTolerance;
    private double mechMin;
    private double mechMax;
    private double sensorToMechRatio;
    private double offset;
    private boolean changedOffset = false;

    private double gearRatio = 1d;
    private double drumCircumference;
    private double ffOffset;

    double zero = 0;


    private Type type;
    private String name;

    NetworkTableInstance inst;

    NetworkTable systemStateTable;
    DoublePublisher systemPose;
    DoublePublisher systemSpeeds;
    DoublePublisher systemTimestamp;
    DoublePublisher systemStatorCurrent;
    DoublePublisher systemVoltage;
    DoublePublisher systemStatorVoltage;

    public TemplateSubsystem(Type type, int id, TrapezoidProfile.Constraints constraints, FeedForward feedForward,
                             double lowerTolerance, double upperTolerance,
                             double[][] gearRatios, String SubsystemName) {
        this.type = type;

        motor = new TalonFX(id);
        motorConfig = new TalonFXConfiguration();

        profile = new TrapezoidProfile(constraints);
        initState = new TrapezoidProfile.State(0.0, 0.0);
        goalState = new TrapezoidProfile.State(0.0, 0.0);
        currentState = new TrapezoidProfile.State(0.0, 0.0);
        this.constraints = constraints;

        switch (type) {
            case ROLLER -> simpleMotorFF = new SimpleMotorFeedforward(
                    feedForward.getkS(), feedForward.getkV());
            case LINEAR -> linearFF = new ElevatorFeedforward(
                    feedForward.getkS(), feedForward.getkG(), feedForward.getkV());
            case PIVOT -> pivotFF = new ArmFeedforward(
                    feedForward.getkS(), feedForward.getkG(), feedForward.getkV());
        }

        positionVoltage = new PositionVoltage(0).withSlot(0).withEnableFOC(true);
        velocityVoltage = new VelocityVoltage(0).withSlot(0).withEnableFOC(true);
        magicMan = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);

        this.lowerTolerance = lowerTolerance;
        this.upperTolerance = upperTolerance;

        for (double[] ratio : gearRatios) {
            this.gearRatio *= (ratio[1] / ratio[0]);
        }

        inst = NetworkTableInstance.getDefault();

        /* Robot swerve drive state */
        systemStateTable = inst.getTable(SubsystemName);
        systemPose = systemStateTable.getDoubleTopic("Position").publish();
        systemSpeeds = systemStateTable.getDoubleTopic("Speeds").publish();
        systemTimestamp = systemStateTable.getDoubleTopic("Timestamp").publish();
        this.name = SubsystemName;
    }

    public String getName() {
        return name;
    }

    //Configurations
    public void configureMotor(boolean isInverted, boolean isBrakeMode,
                               double supplyCurrentLimit, double statorCurrentLimit,
                               Slot0Configs slot0Configs) {
        motorConfig.MotorOutput.Inverted =
                isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = isBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        motorConfig.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;
        motorConfig.CurrentLimits.StatorCurrentLimit = statorCurrentLimit;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfig.Slot0 = slot0Configs;

        motor.getConfigurator().apply(motorConfig);
        motor.setPosition(0);
    }


    public void configureLinearMech(double drumCircumference, double mechMinM, double mechMaxM) {
        this.drumCircumference = drumCircumference;
        this.mechMin = mechMinM;
        this.mechMax = mechMaxM;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = getMotorRotFromMechM(mechMaxM);
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = getMotorRotFromMechM(mechMinM);
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    }

    public void configurePivot(double mechMinDegrees, double mechMaxDegrees, double ffOffset) {
        this.mechMin = mechMinDegrees;
        this.mechMax = mechMaxDegrees;
        this.ffOffset = ffOffset;

        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = getMotorRotFromDegrees(mechMaxDegrees);
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = getMotorRotFromDegrees(mechMinDegrees);
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    }

    public void configureFollowerMotor(int followerMotorId, boolean opposeMasterDirection) {
        followerMotor = new TalonFX(followerMotorId);
        follower = new Follower(motor.getDeviceID(), opposeMasterDirection);

        followerMotor.setControl(follower);
    }

    public void configureEncoder(int encoderId, String canbus, double magnetOffset,
                                 double sensorToMechRatio, double motorToSensorRatio) {
        encoder = new CANcoder(encoderId, canbus);
        encoderConfig = new CANcoderConfiguration();

        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        encoderConfig.MagnetSensor.MagnetOffset = magnetOffset;

        encoder.getConfigurator().apply(encoderConfig);

        motorConfig.Feedback.FeedbackRemoteSensorID = encoderId;
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        motorConfig.Feedback.SensorToMechanismRatio = sensorToMechRatio;
        motorConfig.Feedback.RotorToSensorRatio = motorToSensorRatio;

        this.sensorToMechRatio = sensorToMechRatio;

        motor.getConfigurator().apply(motorConfig);
        gearRatio = motorToSensorRatio;
    }

    public void setPercent(double percent) {
        followLastMechProfile = false;
        if (percent > 1) percent /= 100;
        motor.set(percent);
    }

    public void setVoltage(double output) {
        followLastMechProfile = false;
        motor.setVoltage(output);
    }

    public void setVelocity(double rps) {
        this.goal = rps;
        followLastMechProfile = false;
        if (rps == 0) setPercent(0);
        else motor.setControl(velocityVoltage.withVelocity(rps)
                .withFeedForward(calculateFF(getMotorVelocity(), rps)));
    }

//    private double calculateFF(double rps, double acceleration) {
//        switch (type) {
//            case LINEAR -> {
//                return linearFF.calculate(rps, acceleration);
//            }
//            case PIVOT -> {
//                if (encoder == null)
//                    return pivotFF.calculate(Units.degreesToRadians(getDegrees()),
//                            Units.degreesToRadians(getDegreesFromMotorRot(rps)), Units.degreesToRadians(getDegreesFromMotorRot(acceleration)));
//                return pivotFF.calculate(Units.degreesToRadians(getEncoderDegrees()),
//                    Units.degreesToRadians(getDegreesFromEncoderRot(rps)), Units.degreesToRadians(getDegreesFromEncoderRot(acceleration)));
//            }
//            default -> {
//                return simpleMotorFF.calculate(rps, acceleration);
//            }
//        }
//    }

    private double calculateFF(double currentVelocity, double nextVelocity) {
        switch (type) {
            case LINEAR -> {
                return linearFF.calculateWithVelocities(currentVelocity, nextVelocity);
            }
            case PIVOT -> {
                if (encoder == null)
                    return pivotFF.calculateWithVelocities(Units.degreesToRadians(getDegrees()),
                            Units.degreesToRadians(getDegreesFromMotorRot(currentVelocity)), Units.degreesToRadians(getDegreesFromMotorRot(nextVelocity)));
                return pivotFF.calculateWithVelocities(Units.degreesToRadians(getEncoderDegrees()),
                        Units.degreesToRadians(getDegreesFromEncoderRot(currentVelocity)), Units.degreesToRadians(getDegreesFromEncoderRot(nextVelocity)));
            }
            default -> {
                return simpleMotorFF.calculateWithVelocities(currentVelocity, nextVelocity);
            }
        }
    }

    public void setPosition(double goal) {
        if (type == Type.ROLLER) return;

        profile = new TrapezoidProfile(constraints);

        switch (type) {
            case LINEAR -> goalState.position = getMotorRotFromMechM(goal + offset);
            case PIVOT -> goalState.position = encoder == null ? getMotorRotFromDegrees(goal + offset)
                    : getEncoderRotFromDegrees(goal + offset);
            default -> goalState.position = getMotorRotFromMechRot(goal);
        }

        goalState.velocity = 0;
        this.goal = goal;

        currentState = profile.calculate(0, currentState, goalState);
        if (encoder == null) currentState.position = getMotorRot();
        else currentState.position = getEncoderRot();

        followLastMechProfile = true;
    }

    //Used if velocity/acceleration constraint needs to be changed
    public void setPosition(double goal, boolean holdPosition, double vel, double acc) {
        if (type == Type.ROLLER) return;

        switch (type) {
            case LINEAR -> goalState.position = getMotorRotFromMechM(goal + offset);
            case PIVOT -> goalState.position = encoder == null ? getMotorRotFromDegrees(goal + offset)
                    : getEncoderRotFromDegrees(goal + offset);
            default -> goalState.position = getMotorRotFromMechRot(goal);
        }

        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(vel, acc));

        goalState.velocity = 0;
        this.goal = goal;
        currentState = profile.calculate(0, currentState, goalState);

        if (encoder == null) currentState.position = getMotorRot();
        else currentState.position = getEncoderRot();

        followLastMechProfile = holdPosition;
    }

    public void setPosition(double goal, boolean holdPosition) {
        if (type == Type.ROLLER) return;

        profile = new TrapezoidProfile(constraints);

        switch (type) {
            case LINEAR -> goalState.position = getMotorRotFromMechM(goal + offset);
            case PIVOT -> goalState.position = encoder == null ? getMotorRotFromDegrees(goal + offset)
                    : getEncoderRotFromDegrees(goal + offset);
            default -> goalState.position = getMotorRotFromMechRot(goal);
        }

        goalState.velocity = 0;
        this.goal = goal;

        currentState = profile.calculate(0, currentState, goalState);
        if (encoder == null) currentState.position = getMotorRot();
        else currentState.position = getEncoderRot();

        followLastMechProfile = holdPosition;
    }

    public void followLastMechProfile() {
        if (type == Type.ROLLER) return;

        TrapezoidProfile.State nextState = profile.calculate(.02, currentState, goalState);
        motor.setControl(
                positionVoltage.withPosition(nextState.position)
                        .withFeedForward(calculateFF(currentState.velocity, nextState.velocity)));

        currentState = nextState;
    }

    public boolean isProfileFinished() {
        return currentState.position == goalState.position && currentState.velocity == goalState.velocity;
    }

    public boolean isMechAtGoal(boolean isVelocity) {
        switch (type) {
            case LINEAR -> {
                return isProfileFinished() &&
                        getMechM() >= goal - lowerTolerance && getMechM() <= goal + upperTolerance;
            }
            case PIVOT -> {
                return isProfileFinished() &&
                        getDegrees() >= goal - lowerTolerance && getDegrees() <= goal + upperTolerance;
            }
            default -> {
                if (isVelocity) return getMechVelocity() >= goal - lowerTolerance
                        && getMechVelocity() <= goal - upperTolerance;
                else return getMechRot() >= goal - lowerTolerance
                        && getMechRot() <= goal + upperTolerance;
            }
        }
    }

    public boolean isMechGreaterThanPosition(double position) {
        switch (type) {
            case LINEAR -> {
                return getMechM() >= position;
            }
            case PIVOT -> {
                return getDegrees() >= position;
            }
            default -> {
                return false;
            }
        }
    }

    public boolean isAboveSpeed() {
        if (type != Type.ROLLER) return false;
        return getMechVelocity() > goal - 20; //TODO: fix
    }


    public void setOffset(double offset) {
        this.offset = offset;
        changedOffset = true;
    }

    public void setOffset(double offset, boolean changedOffset) {
        this.offset = offset;
        this.changedOffset = changedOffset;
    }

    public double getOffset() {
        return offset;
    }

    public double getGoal() {
        return goal;
    }

    public void setFollowLastMechProfile(boolean followLastMechProfile) {
        this.followLastMechProfile = followLastMechProfile;
    }


    //Unit Conversions
    public double getDegrees() {
        return encoder == null ? motor.getRotorPosition().getValueAsDouble() * gearRatio * 360d
                : getEncoderRot() * sensorToMechRatio * 360d;
    }

    /**
     * @return Gets the position of the actual motor of the mechanism.
     */

    public double getMotorRot() {
        return motor.getRotorPosition().getValueAsDouble();
    }

    /**
     * @return The rotation of the mechanism itself (Accounts for gear ratios and stuff)
     */
    public double getMechRot() {
        return motor.getRotorPosition().getValueAsDouble() * gearRatio;
    }

    public double getDegreesFromMechRot(double mechRot) {
        return mechRot * 360d;
    }

    public double getDegreesFromMotorRot(double motorRot) {
        return motorRot * gearRatio * 360d;
    }

    public double getMechRotFromDegrees(double degrees) {
        return degrees / 360d;
    }

    public double getMechRotFromMotorRot(double motorRot) {
        return motorRot * gearRatio;
    }

    public double getMotorRotFromDegrees(double degrees) {
        return degrees / 360d / gearRatio;
    }

    public double getMotorRotFromMechRot(double mechRot) {
        return mechRot / gearRatio;
    }

    /**
     * @return the current height of the elevator as a double
     */
    public double getMechM() {
        if (type != Type.LINEAR) return 0;
        return motor.getRotorPosition().getValueAsDouble() * drumCircumference * gearRatio;
    }

    /**
     * @return a constantly updating value for the height of the elevator as a double.
     */
    public DoubleSupplier getMechMeter() {
        if (type != Type.LINEAR) return () -> 0;
        return () -> motor.getRotorPosition().getValueAsDouble() * drumCircumference * gearRatio;
    }

    public double getMechMFromMotorRot(double motorRot) {
        if (type != Type.LINEAR) return 0;
        return motorRot * drumCircumference * gearRatio;
    }

    public double getMotorRotFromMechM(double mechM) {
        if (type != Type.LINEAR) return 0;
        return mechM / drumCircumference / gearRatio;
    }

    //Motor Values
    public double getMotorVelocity() {
        return motor.getVelocity().getValueAsDouble();
    }

    public double getMotorVoltage() {
        return motor.getMotorVoltage().getValueAsDouble();
    }

    public TalonFX getMotor() {
        return motor;
    }

    public TalonFX getFollowerMotor() {
        return followerMotor;
    }

    public double getSupplyVoltage() {
        return motor.getSupplyVoltage().getValueAsDouble();
    }

    public double getSupplyCurrent() {
        return motor.getSupplyCurrent().getValueAsDouble();
    }

    public double getStatorCurrent() {
        return motor.getStatorCurrent().getValueAsDouble();
    }


    public double getMechVelocity() {
        return getMechRotFromMotorRot(motor.getVelocity().getValueAsDouble());
    }

    public CANcoder getEncoder() {
        return encoder;
    }

    public double getEncoderRot() {
        return encoder.getAbsolutePosition().getValueAsDouble();
    }

    public double getEncoderRotFromDegrees(double degrees) {
        if (encoder == null) return getMotorRotFromDegrees(degrees);
        return degrees / 360 * sensorToMechRatio;
    }

    public double getDegreesFromEncoderRot(double encoderRot) {
        if (encoder == null) return getDegreesFromMotorRot(encoderRot);
        return encoderRot * 360 * sensorToMechRatio;
    }

    public double getEncoderDegrees() {
        return encoder.getAbsolutePosition().getValueAsDouble() * 360 * sensorToMechRatio;
    }

    @Override
    public void periodic() {
        if (changedOffset) {
            setPosition(goal);
            changedOffset = false;
        }
        if (followLastMechProfile) followLastMechProfile();

        systemPose.set(getMotorRot());
        systemSpeeds.set(getMotorVelocity());
        systemTimestamp.set(Timer.getFPGATimestamp());
    }

    public void setControl(ControlRequest control) {
        motor.setControl(control);
    }
}