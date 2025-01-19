package frc.robot.subsystems.template;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.ArmConstants;
import frc.robot.tagalong.GeometricUtils;
import frc.robot.tagalong.MathUtils;
import frc.robot.utility.FeedForward;
import frc.robot.utility.PID;
import frc.robot.utility.Type;

public class TemplateSubsystem extends SubsystemBase {
    private TalonFX motor;
    private TalonFXConfiguration motorConfig;

    private TalonFX followerMotor;
    private Follower follower;
    private TalonFXConfiguration followerConfig;

    private CANcoder encoder;
    private CANcoderConfiguration encoderConfig;


    private ProfiledPIDController pidProfile;
    private TrapezoidProfile profile;
    public TrapezoidProfile.State initState;
    public TrapezoidProfile.State currentState;
    public TrapezoidProfile.State goalState;
    public double goal;
    public boolean followLastMechProfile = false;

    private PositionVoltage positionVoltage;
    private VelocityVoltage velocityVoltage;
    private Slot0Configs slot0Configs;

    private SimpleMotorFeedforward simpleMotorFF;
    private ElevatorFeedforward linearFF;
    private ArmFeedforward pivotFF;

    private double lowerTolerance;
    private double upperTolerance;
    private double mechMin;
    private double mechMax;
    private double sensorToMechRatio;

    private double gearRatio = 1d;
    private double drumCircumference;
    private double ffOffset;

    private Timer timer;
    private double[] _values;
    private int[] _ids;


    private Type type;

    public TemplateSubsystem(Type type, int id, TrapezoidProfile.Constraints constraints,
                             PID pid, FeedForward feedForward,
                             double lowerTolerance, double upperTolerance,
                             double[][] gearRatios) {
        this.type = type;

        motor = new TalonFX(id);
        motorConfig = new TalonFXConfiguration();

        profile = new TrapezoidProfile(constraints);
        initState = new TrapezoidProfile.State(0.0,0.0);
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


        double minAbs = MathUtils.cppMod(mechMin, 1.0);
        double maxAbs = MathUtils.cppMod(mechMax, 1.0);
        double halfUnusedRange = (mechMax - mechMin) / 2.0;
        double midUnused = maxAbs + halfUnusedRange;

        if (midUnused > 1.0) {
            _values = new double[]{midUnused - 1.0, minAbs, maxAbs, 1.0};
            _ids = new int[]{2, 0, 1, 2};
        } else if (mechMin > 0.0) {
            _values = new double[]{minAbs, maxAbs, midUnused, 1.0};
            _ids = new int[]{0, 1, 2, 0};
        } else {
            _values = new double[]{maxAbs, midUnused, minAbs, 1.0};
            _ids = new int[]{1, 2, 0, 1};
        }
    }

    //Configurations
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

    public void configureMotor(boolean isInverted, boolean isBrakeMode, double supplyCurrentLimit, double statorCurrentLimit, int cancoderID, Slot0Configs slot0Configs) {
        motorConfig.MotorOutput.Inverted = isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = isBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        motorConfig.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;
        motorConfig.CurrentLimits.StatorCurrentLimit = statorCurrentLimit;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfig.Feedback.RotorToSensorRatio = ArmConstants.ARM_MOTOR_TO_SENSOR_GEAR_RATIO;
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        motorConfig.Feedback.FeedbackRemoteSensorID = cancoderID;
        motorConfig.Slot0 = slot0Configs;

        motor.getConfigurator().apply(motorConfig);
        motor.setPosition(0);
    }



    public void configureLinearMech(double drumCircumference, double mechMinM, double mechMaxM) {
        this.drumCircumference = drumCircumference;
        this.mechMin = mechMinM;
        this.mechMax = mechMaxM;
    }

    public void configurePivot(double mechMinDegrees, double mechMaxDegrees, double ffOffset) {
        this.mechMin = mechMinDegrees;
        this.mechMax = mechMaxDegrees;
        this.ffOffset = ffOffset;
    }

    public void configureFollowerMotor(int followerMotorId, boolean invert){
        followerMotor = new TalonFX(followerMotorId);
        follower = new Follower(motor.getDeviceID(), invert);

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
        motor.set(percent);
    }

    public void setVoltage(double output) {
        motor.setVoltage(output);
    }

    public void setVelocity(double rps) {
        if (type == Type.LINEAR || type == Type.PIVOT) return;

        this.goal = rps;
        followLastMechProfile = false;
        motor.setNeutralMode(NeutralModeValue.Coast);
        motor.setControl(velocityVoltage.withVelocity(rps)
                .withFeedForward(calculateFF(rps, 0)));
    }

    private double calculateFF(double rps, double acceleration) {
        switch (type) {
            case LINEAR -> {
                return linearFF.calculate(rps, acceleration);
            }
            case PIVOT -> {
                return pivotFF.calculate(Units.rotationsToRadians(getEncoderRot() + Units.degreesToRotations(ffOffset)),
                        rps , acceleration);
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
    public void setPosition(double goal) {
        if (type == Type.ROLLER) return;

        if (goal < mechMin) goal = mechMin;
        else if (goal > mechMax) goal = mechMax;

        followLastMechProfile = true;

        if (type == Type.LINEAR) currentState.position = getMotorRot();
        else currentState.position = Units.rotationsToRadians(getEncoderRot());

        switch (type) {
            case LINEAR -> goalState.position = getMotorRotFromMechM(goal);
            case PIVOT -> goalState.position = Units.degreesToRadians(goal);
            default -> goalState.position = getMotorRotFromMechRot(goal);
        }

        goalState.velocity = 0;
        this.goal = goal;
        

        currentState = profile.calculate(0, currentState, goalState);
    


    }

    public void followLastMechProfile() {
        if (type == Type.ROLLER) return;

         TrapezoidProfile.State nextState = profile.calculate(timer.get(), currentState, goalState);
         switch(type){
            case LINEAR ->  {motor.setControl(
                positionVoltage.withPosition(nextState.position)
                        .withFeedForward(calculateFF(nextState.velocity,
                                (nextState.velocity - currentState.velocity) / timer.get())));
            }
            case PIVOT ->{
                motor.setControl(
                    positionVoltage.withSlot(0).withPosition(Units.radiansToRotations(nextState.position))
                            .withFeedForward(calculateFF(nextState.velocity,
                                    (nextState.velocity - currentState.velocity) / timer.get())/3));
    
            System.out.println("Next state position: " + Units.radiansToDegrees(nextState.position));
            System.out.println("FF calculations: " + calculateFF(nextState.velocity,
                    (nextState.velocity - currentState.velocity) / timer.get()));
            currentState = nextState;

            }
            default -> {motor.setControl(
                positionVoltage.withPosition(nextState.position)
                        .withFeedForward(calculateFF(nextState.velocity,
                                (nextState.velocity - currentState.velocity) / timer.get())));
            }

         }




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
                        Units.rotationsToDegrees(getEncoderRot()) >= goal - lowerTolerance && Units.rotationsToDegrees(getEncoderRot()) <= goal + upperTolerance;
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

    public double getMotorRot() {
        return motor.getPosition().getValueAsDouble();
    }

    public double getRotorPosition() {
        return motor.getRotorPosition().getValueAsDouble();
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

    public double getEncoderRot() {
        return encoder.getAbsolutePosition().getValueAsDouble();
    }

    public double getEncoderRotFromDegrees(double degrees) {
        return degrees / 360 * sensorToMechRatio;
    }

    @Override
    public void periodic() {
        if (followLastMechProfile) followLastMechProfile();
    }

        public double absoluteClamp(double value) {
        double abs = MathUtils.cppMod(value, 1.0);
        int i = 0;
        while (abs >= _values[i] && i < _values.length) {
            i++;
        }
        switch (_ids[i]) {
            case 2:
                return mechMax;
            case 1:
                return abs;
            case 0:
            default:
                return mechMin;
        }
    }

    public void setControl(ControlRequest control){
        motor.setControl(control);
    }
}