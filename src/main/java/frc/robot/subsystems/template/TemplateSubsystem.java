package frc.robot.subsystems.template;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.ArmConstants;
import frc.robot.subsystems.testing.PivotTestSubsystem;
import frc.robot.utility.FeedForward;
import frc.robot.utility.PID;
import frc.robot.utility.Type;

public class TemplateSubsystem extends SubsystemBase {
    private TalonFX motor;
    private TalonFXConfiguration motorConfig;

    private TalonFX followerMotor;
    private Follower follower;

    private CANcoder encoder;
    private CANcoderConfiguration encoderConfig;

    private TrapezoidProfile profile;
    public TrapezoidProfile.State currentState;
    public TrapezoidProfile.State goalState;
    public double goal;
    public boolean followLastMechProfile = false;

    private PositionVoltage positionVoltage;
    private PositionTorqueCurrentFOC positionTorque;
    private VelocityVoltage velocityVoltage;
    private Slot0Configs slot0Configs;

    private SimpleMotorFeedforward simpleMotorFF;
    private ElevatorFeedforward linearFF;
    private ArmFeedforward pivotFF;

    private double lowerTolerance;
    private double upperTolerance;
    private double mechMin;
    private double mechMax;

    private double gearRatio = 1d;
    private double drumCircumference;
    private double ffOffset;

    private Timer timer;
    private Type type;

    /**
     * 
     * @param type What type of Mechanism is it? Linear, Pivot, or Roller
     * @param id What is the ID of the master motor
     * @param constraints What are the Trapezoidal Profile constraints of the mechanism
     * @param pid Give a PID object
     * @param feedForward Give a Feedforward object
     * @param lowerTolerance The lower range of how much you want to allow your input value to be off. Be reasonable and don't just put 0 lmao.
     * @param upperTolerance What is the upper range of how much you want your mechanism to be off from a desired value.
     * @param gearRatios What is the gear ratio from motor to mechanism.
     */
    public TemplateSubsystem(Type type, int id, TrapezoidProfile.Constraints constraints,
                             PID pid, FeedForward feedForward,
                             double lowerTolerance, double upperTolerance,
                             double[][] gearRatios) {
        this.type = type;

        motor = new TalonFX(id);
        motorConfig = new TalonFXConfiguration();

        profile = new TrapezoidProfile(constraints);
        goalState = new TrapezoidProfile.State(0.0, 0.0);
        currentState = new TrapezoidProfile.State(0.0, 0.0);

        slot0Configs = pid.getSlot0Configs();
        motor.getConfigurator().apply(slot0Configs);

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

        this.lowerTolerance = lowerTolerance;
        this.upperTolerance = upperTolerance;

        for (double[] ratio : gearRatios) {
            this.gearRatio *= ratio[0] / ratio[1];
        }

        timer = new Timer();
    }

    //Configurations
    /**
     * 
     * @param isInverted Is the motor inverted true or false
     * @param isBrakeMode When not running should the motor be in brake or coast mode.
     * @param supplyCurrentLimit What is the absolute limit on current going to the motor. This is useful for preventing brown out
     * @param statorCurrentLimit What is the limit on current being outputted by the motor. This is useful for prevent burnout of the motor.
     */
    public void configureMotor(boolean isInverted, boolean isBrakeMode, double supplyCurrentLimit, double statorCurrentLimit) {
        motorConfig.MotorOutput.Inverted =
                isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = isBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        motorConfig.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;
        motorConfig.CurrentLimits.StatorCurrentLimit = statorCurrentLimit;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        motor.getConfigurator().apply(motorConfig);
        motor.setPosition(0);
    }

    /**
     * 
     * @param drumCircumference idk man this is rohans thing
     * @param mechMinM What is the lowest position you want the motor to be allowed to run.
     * @param mechMaxM what is the maximum position you want the motor to run.
     */

    public void configureLinearMech(double drumCircumference, double mechMinM, double mechMaxM) {
        this.drumCircumference = drumCircumference;
        this.mechMin = mechMinM;
        this.mechMax = mechMaxM;
    }

    /**
     * 
     * @param mechMinDegrees What is lowest you want your arm to go in Degrees
     * @param mechMaxDegrees What is the highest you want your arm to go in degrees
     * @param ffOffset If the encoder being used does not go to zero when the arm is horizontal to the ground this is the value it is at when it is horizontal to the ground. In degrees.
     */

    public void configurePivot(double mechMinDegrees, double mechMaxDegrees, double ffOffset) {
        this.mechMin = mechMinDegrees;
        this.mechMax = mechMaxDegrees;
        this.ffOffset = ffOffset;
    }

    /**
     * 
     * @param followerMotorId If there is more than one motor what is the second motor in the system.
     * @param invert Does the motor run opposite to the master when following. invert it.
     */

    public void configureFollowerMotor(int followerMotorId, boolean invert) {
        followerMotor = new TalonFX(followerMotorId);
        follower = new Follower(motor.getDeviceID(), invert);
        followerMotor.setControl(follower);
    }

    /**
     * 
     * @param encoderId What is the id of the encoder.
     * @param canbus What canbus is the encoder on
     * @param magnetOffset When using absolute position how far off is the resting position of the encoder from 0. You can find this using Phoenix Tuner if its a cancoder.
     * @param sensorToMechRatio What is the gear ratio between the sensor and mechanism
     * @param motorToSensorRatio What is the gear ratio between the motor and the sensor.
     */

    public void configureEncoder(int encoderId, String canbus, double magnetOffset,
                                 double sensorToMechRatio, double motorToSensorRatio, SensorDirectionValue direction ) {
        encoder = new CANcoder(encoderId, canbus);
        encoderConfig = new CANcoderConfiguration();

        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        encoderConfig.MagnetSensor.SensorDirection = direction;
        encoderConfig.MagnetSensor.MagnetOffset = magnetOffset;

        encoder.getConfigurator().apply(encoderConfig);

        motorConfig.Feedback.FeedbackRemoteSensorID = encoderId;
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        motorConfig.Feedback.SensorToMechanismRatio = sensorToMechRatio;
        motorConfig.Feedback.RotorToSensorRatio = motorToSensorRatio;

        motor.getConfigurator().apply(motorConfig);
        gearRatio = sensorToMechRatio;
    }

    public void setPercent(double percent) {
        motor.set(percent);
    }

    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    /**
     * 
     * @param rps What is the desired velocity in rotations per second
     */

    public void setVelocity(double rps) {
        if (type == Type.LINEAR || type == Type.PIVOT) return;

        this.goal = rps;
        followLastMechProfile = false;
        motor.setNeutralMode(NeutralModeValue.Coast);
        motor.setControl(velocityVoltage.withVelocity(rps)
                .withFeedForward(calculateFF(rps, 0)));
    }

    /**
     * 
     * @param rps What is the desired velocity while moving. This changes depending on the mechanism. A pivot uses rot per sec.
     * 
     * @param acceleration What is the desired acceleration of the motor while moving. Changes dependign on the mechanism
     * @return Returns a value to be used for feed forward calculations
     */

    private double calculateFF(double rps, double acceleration) {
        switch (type) {
            case LINEAR -> {
                return linearFF.calculate(rps, acceleration);
            }
            case PIVOT -> {
                return pivotFF.calculate(Units.rotationsToRadians(getAbsPosition() + Units.degreesToRotations(ffOffset)),
                        Units.degreesToRadians(rps), Units.degreesToRadians(acceleration));
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
//                return pivotFF.calculateWithVelocities(Math.toRadians(getDegrees()),
//                        velocities[0] * 2 * Math.PI, velocities[1] * 2 * Math.PI);
//            }
//            default -> {
//                return simpleMotorFF.calculate(velocities[0]);
//            }
//        }
//    }

/**
 * @param goal The position you want the arm to go to in degrees.
 * @param goal The position you want the elevator to go to in meters.
 */
    public void setPosition(double goal) {
        if (type == Type.ROLLER) return;

        if (type == Type.LINEAR && goal < mechMin) goal = mechMin;
        else if (type == Type.LINEAR && goal > mechMax) goal = mechMax;

        if (type == Type.PIVOT && goal < mechMin) goal = mechMin;
        else if (type == Type.PIVOT && goal > mechMax) goal = mechMax;

        followLastMechProfile = true;

        switch (type) {
            case LINEAR -> goalState.position = getMotorRotFromMechM(goal);
      //      case PIVOT -> goalState.position = getMotorRotFromDegrees(goal);
           case PIVOT -> goalState.position = 
           goal;
            default -> goalState.position = getMotorRotFromMechRot(goal);
        }

        goalState.velocity = 0;
        this.goal = goal;

        currentState = profile.calculate(0, currentState, goalState);


        currentState.position = Units.rotationsToDegrees(getAbsPosition());

    
        timer.restart();
    }

    public void followLastMechProfile() {
        if (type == Type.ROLLER) return;

        TrapezoidProfile.State nextState = profile.calculate(timer.get(), currentState, goalState);
        motor.setControl(
                positionVoltage.withPosition(nextState.position)
                        .withFeedForward(calculateFF(/*currentState.velocity, nextState.velocity*/
                                nextState.velocity,
                                (nextState.velocity - currentState.velocity) / timer.get())));

        currentState = nextState;

        timer.restart();
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
                        Units.rotationsToDegrees(getAbsPosition()) >= goal - lowerTolerance && Units.rotationsToDegrees(getAbsPosition())  <= goal + upperTolerance;
            }
            default -> {
                if (isVelocity) return getMechVelocity() >= goal - lowerTolerance
                        && getMechVelocity() <= goal - upperTolerance;
                else return getMechRot() >= goal - lowerTolerance
                        && getMechRot() <= goal + upperTolerance;
            }
        }
    }


    //Unit Conversions
    public double getDegrees() {
        return motor.getPosition().getValueAsDouble() * gearRatio * 360d;
    }

    /**
     * Returns the absolute position of the encoder in rotations.
     */
    public double getAbsPosition(){
        return encoder.getAbsolutePosition().getValueAsDouble();
    }

    public double getMotorRot() {
        return motor.getPosition().getValueAsDouble();
    }

    public double getMechRot() {
        return motor.getPosition().getValueAsDouble() * gearRatio;
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

    public double getMechM() {
        if (type != Type.LINEAR) return 0;
        return motor.getPosition().getValueAsDouble() * drumCircumference * gearRatio;
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

    public double getSupplyVoltage() {
        return motor.getSupplyVoltage().getValueAsDouble();
    }

    public double getSupplyCurrent() {
        return motor.getSupplyCurrent().getValueAsDouble();
    }

    public double getMechVelocity() {
        return getMechRotFromMotorRot(motor.getVelocity().getValueAsDouble());
    }

    public CANcoder getEncoder() {
        return encoder;
    }

    @Override
    public void periodic() {
            if (followLastMechProfile) followLastMechProfile();
     
    }
}