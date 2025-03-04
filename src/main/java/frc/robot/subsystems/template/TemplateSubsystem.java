package frc.robot.subsystems.template;

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
import frc.robot.constants.Constants.ArmConstants;
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

    private double velocity;
    private double acceleration;
    double zero = 0;


    private Timer timer;
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

    /**
     * Creates a motor based subsystem and configures a single motor
     * @param type The type of the subsystem, AFFECTS THE UNITS OF THE SUBSYSTEM TO MATCH THE TYPE.
     * @param id The motor ID to be controlled by the subsystem
     * @param constraints The motion constraints of the motor
     * @param feedForward The feedforward configuration of the motor
     * @param lowerTolerance (Degrees or Meters) The lower limit of the motor to be considered at it's goal
     * @param upperTolerance (Degrees or Meters) The lower limit of the motor to be considered at it's goal
     * @param gearRatios A matrix of gear ratios with two columns and many rows, in the format gearRatios[x][0] : gearRatios[x][1]
     * @param SubsystemName The name of the subsystem
     */
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

        timer = new Timer();

        inst = NetworkTableInstance.getDefault();

        /* Robot swerve drive state */
        systemStateTable = inst.getTable(SubsystemName);
        systemPose = systemStateTable.getDoubleTopic("Position").publish();
        systemSpeeds = systemStateTable.getDoubleTopic("Speeds").publish();
        systemTimestamp = systemStateTable.getDoubleTopic("Timestamp").publish();
        this.name = SubsystemName;
    }

    /**
     * @return The subsystem's name set at configuration.
     */
    public String getName() {
        return name;
    }

    //Configurations:

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

    /**
     * Configures the subsystem for use as a linear mech
     */
    public void configureLinearMech(double drumCircumference, double mechMinM, double mechMaxM) {
        this.drumCircumference = drumCircumference;
        this.mechMin = mechMinM;
        this.mechMax = mechMaxM;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = getMotorRotFromMechM(mechMaxM);
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = getMotorRotFromMechM(mechMinM);
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    }

    /**
     * Configures the subsystem to work as a Pivot.
     * @param mechMinDegrees
     * @param mechMaxDegrees
     * @param ffOffset
     */
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


    public void configureFollowerMotor(int followerMotorId, boolean invert, boolean brake, Slot0Configs slot0Configs) {
        followerMotor = new TalonFX(followerMotorId);
        followerConfig.MotorOutput.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        followerMotor.getConfigurator().apply(followerConfig);
        follower = new Follower(motor.getDeviceID(), invert);

        followerMotor.setControl(follower);
    }

    public void configureEncoder(int encoderId, String canbus, double magnetOffset,
                                 double sensorToMechRatio, double motorToSensorRatio) {
        encoder = new CANcoder(encoderId, canbus);
        encoderConfig = new CANcoderConfiguration();

        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        encoderConfig.MagnetSensor.MagnetOffset = magnetOffset;

        encoder.getConfigurator().apply(encoderConfig);

        motorConfig.Feedback.FeedbackRemoteSensorID = encoderId;
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        motorConfig.Feedback.SensorToMechanismRatio = sensorToMechRatio;
        motorConfig.Feedback.RotorToSensorRatio = motorToSensorRatio;

        this.sensorToMechRatio = sensorToMechRatio;

        motor.getConfigurator().apply(motorConfig);
        gearRatio = sensorToMechRatio;
    }

    public void setPercent(double percent) {
        followLastMechProfile = false;
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
                .withFeedForward(calculateFF(rps, 0)));
    }

    private double calculateFF(double rps, double acceleration) {
        switch (type) {
            case LINEAR -> {
                return linearFF.calculate(rps, acceleration);
            }
            case PIVOT -> {
                return pivotFF.calculate(Units.degreesToRadians(getDegrees() + Units.degreesToRotations(ffOffset)),
                        Units.degreesToRadians(getDegreesFromMotorRot(rps)), Units.degreesToRadians(getDegreesFromMotorRot(acceleration)));
            }
            default -> {
                return simpleMotorFF.calculate(rps, acceleration);
            }
        }
    }

//    private double calculateFF(double... velocities) {
//        switch (type) {
//            case LINEAR -> {
//                return linearFF.calculateWithVelocities(getMechMFromMotorRot(velocities[0]),
//                        getMechMFromMotorRot(velocities[1]));
//            }
//            case PIVOT -> {
//                //return pivotFF.calculate(Math.toRadians(getDegrees() + ffOffset),
//                //  rps * 2 * Math.PI, acceleration * 2 * Math.PI);
//                //The feedforward calculation below is untested, use above if it doesn't work

    /// /                return pivotFF.calculateWithVelocities(Math.toRadians(getDegrees() + ffOffset),
    /// /                        velocities[0] * 2 * Math.PI, velocities[1] * 2 * Math.PI);
//            }
//            default -> {
//                return simpleMotorFF.calculate(velocities[0]);
//            }
//        }
//    }

    /**
     * Sets the position, and holds it.
     * @param goal Degrees or Meters based on the Subsystem's {@link frc.robot.utility.Type}
     */
    public void setPosition(double goal) {
        setPosition(goal, true);
    }

    /**
     * Use if velocity/acceleration constraint needs to be changed. DOESN'T DO ANYTHING WITH ROLLERS!
     * @param goal Goal in either degrees or meters
     * @param holdPosition Whether the mech should hold the position it is set to
     * @param vel Goal velocity (RPS or MPS based on the Subsystem's {@link frc.robot.utility.Type})
     * @param acc Goal acceleration (RPS^2 or MPS^2 based on the Subsystem's {@link frc.robot.utility.Type})
     */
    public void setPosition(double goal, boolean holdPosition, double vel, double acc) {
        if (type == Type.ROLLER) return;

        switch (type) {
            case LINEAR -> goalState.position = getMotorRotFromMechM(goal + offset);
            case PIVOT -> goalState.position = getMotorRotFromDegrees(goal + offset);
            default -> goalState.position = getMotorRotFromMechRot(goal) + zero;
        }

        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(vel, acc));
        setPosition(goal, holdPosition);
    }
    /**
     * DOESN'T DO ANYTHING WITH ROLLERS!
     * @param goal Goal in either degrees or meters
     * @param holdPosition Whether the mech should hold the position it is set to
     * @param vel Goal velocity (RPS or MPS based on the Subsystem's {@link frc.robot.utility.Type})
     * @param acc Goal acceleration (RPS^2 or MPS^2 based on the Subsystem's {@link frc.robot.utility.Type})
     */
    public void setPosition(double goal, boolean holdPosition) {
        if (type == Type.ROLLER) return;

        switch (type) {
            case LINEAR -> goalState.position = getMotorRotFromMechM(goal + offset);
            case PIVOT -> goalState.position = getMotorRotFromDegrees(goal + offset);
            default -> goalState.position = getMotorRotFromMechRot(goal);
        }

        goalState.velocity = 0;
        this.goal = goal;

        currentState = profile.calculate(0, currentState, goalState);
        if (encoder == null) currentState.position = getMotorRot();
        else currentState.position = getEncoderRot();

        followLastMechProfile = holdPosition;
    }
    /**
     * Used to hold the mech profile's after it is set.
     */
    public void followLastMechProfile() {
        if (type == Type.ROLLER) return;  

        TrapezoidProfile.State nextState = profile.calculate(timer.get(), currentState, goalState);
        motor.setControl(
                positionVoltage.withPosition(nextState.position)
                        .withFeedForward(calculateFF(nextState.velocity,
                                (nextState.velocity - currentState.velocity) / timer.get())));

        currentState = nextState;
        timer.restart();
    }
    /**
     * @return A boolean telling you if the mech has reached the exact goal.
     */
    public boolean isProfileFinished() {
        return currentState.position == goalState.position && currentState.velocity == goalState.velocity;
    }
    /**
     * @param isVelocity if the goal is a velocity
     * @return if the mech is within the tolerance limits 
     */
    public boolean isMechAtGoal(boolean isVelocity) {
        switch (type) {
            case LINEAR -> {
                return isProfileFinished() &&
                        getMechM() >= goal - lowerTolerance && getMechM() <= goal + upperTolerance;
            }
            case PIVOT -> {
                if (encoder != null)
                    return isProfileFinished() &&
                            Units.rotationsToDegrees(getEncoderRot()) >= goal - lowerTolerance && Units.rotationsToDegrees(getEncoderRot()) <= goal + upperTolerance;
                else
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
    /**
     * Sets the periodic method to hold the mech profile
     * @param followLastMechProfile should the subsystem periodic method hold the mech profile
     */
    public void setFollowLastMechProfile(boolean followLastMechProfile) {
        this.followLastMechProfile = followLastMechProfile;
    }


    //Unit Conversions
    public double getDegrees() {
        return getMotorRot() * gearRatio * 360d;
    }

    public double getMotorRot() {
        return motor.getRotorPosition().getValueAsDouble();
    }

    public double getMechRot() {
        return getMotorRot() * gearRatio;
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
     * @return The position of the mech in Meters, returns 0 if there the subsystem's type is not linear.
     */
    public double getMechM() {
        if (type != Type.LINEAR) return 0;
        return motor.getRotorPosition().getValueAsDouble() * drumCircumference * gearRatio;
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
        return degrees / 360 * sensorToMechRatio;
    }

    @Override
    public void periodic() {
        if (changedOffset) {
            setPosition(goal);
            changedOffset = false;
            System.out.println("Goal: " + goal + offset);
        }
        if (followLastMechProfile) followLastMechProfile();

        systemPose.set(getMotorRot());
        systemSpeeds.set(getMotorVelocity());
        systemTimestamp.set(Timer.getFPGATimestamp());
    }

    public void setConstraints(double vel, double accel) {
        velocity = vel;
        acceleration = accel;
    }

    public void setControl(ControlRequest control) {
        motor.setControl(control);
    }
}