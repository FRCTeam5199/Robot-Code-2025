package frc.robot.subsystems.template;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utility.FeedForward;
import frc.robot.utility.PID;
import frc.robot.utility.SubsystemPrint;
import frc.robot.utility.Type;

public abstract class AbstractSubsystem extends SubsystemBase {
    private List<TalonFX> motors = new ArrayList<>();
    private TalonFXConfiguration motorConfig;

    private TalonFX followerMotors[];
    private Follower followers[];

    private CANcoder encoder;
    private CANcoderConfiguration encoderConfig;

    private TrapezoidProfile profile;
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

    private double gearRatio = 1d;
    private double drumCircumference;
    private double ffOffset;

    private Timer timer;
    private Type type;

    public AbstractSubsystem(Type type, int[] motorIDs, TrapezoidProfile.Constraints constraints,
                             PID pid, FeedForward feedForward,
                             double lowerTolerance, double upperTolerance,
                             double[][] gearRatios) {
        this.type = type;

        for (int i = 0; i < motorIDs.length; i++) {
            motors.add(new TalonFX(motorIDs[i]));
        }
        motorConfig = new TalonFXConfiguration();

        profile = new TrapezoidProfile(constraints);
        goalState = new TrapezoidProfile.State(0.0, 0.0);
        currentState = new TrapezoidProfile.State(0.0, 0.0);

        slot0Configs = pid.getSlot0Configs();
        for (TalonFX motor : motors) {
            motor.getConfigurator().apply(slot0Configs);
        }

        switch (type) {
            case ROLLER, FLYWHEEL -> simpleMotorFF = new SimpleMotorFeedforward(
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
            this.gearRatio *= ratio[1] / ratio[0];
        }

        timer = new Timer();
    }

    //Configurations
    /**
     * @param isInverted
     * @param isBrakeMode
     * @param currentLimits in format [{supplyCurrentLimit, statorCurrentLimit}]
     */
    public void configureMotors(boolean isInverted, boolean isBrakeMode, double[][] currentLimits) {
        motorConfig.MotorOutput.Inverted =
                isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = isBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        for (TalonFX motor : motors) {
            motorConfig.CurrentLimits.SupplyCurrentLimit = currentLimits[motors.indexOf(motor)][0];
            motorConfig.CurrentLimits.StatorCurrentLimit = currentLimits[motors.indexOf(motor)][1];
            motor.getConfigurator().apply(motorConfig);
        }
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

    public void configureFollowerMotor(int... followerMotorIds) {
        for (int i = 0; i < followerMotors.length; i++) {
            followerMotors[i] = new TalonFX(followerMotorIds[i]);
            followers[i] = new Follower(motors.get(i).getDeviceID(), true);
            followerMotors[i].setControl(followers[i]);
        }
    }

    public void configureEncoder(int encoderId, String canbus, double magnetOffset,
                                 double sensorToMechRatio, double motorToSensorRatio) {
        encoder = new CANcoder(encoderId, canbus);
        encoderConfig = new CANcoderConfiguration();

        // encoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        encoderConfig.MagnetSensor.MagnetOffset = magnetOffset;

        encoder.getConfigurator().apply(encoderConfig);

        motorConfig.Feedback.FeedbackRemoteSensorID = encoderId;
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        motorConfig.Feedback.SensorToMechanismRatio = sensorToMechRatio;
        motorConfig.Feedback.RotorToSensorRatio = motorToSensorRatio;

        for (TalonFX motor : motors) {
            motor.getConfigurator().apply(motorConfig);
        }
        gearRatio = sensorToMechRatio;
    }

    public void setPercent(int motorIndex, double percent) {
        motors.get(motorIndex).set(percent);
        System.out.println(motorIndex);
        System.out.println(motors.get(motorIndex).getDeviceID());
    }

    public void setVelocity(int motorIndex, double rps) {
        if (type == Type.LINEAR || type == Type.PIVOT) return;

        this.goal = rps;
        followLastMechProfile = false;
        motors.get(motorIndex).setNeutralMode(NeutralModeValue.Coast);
        if (type == Type.ROLLER)
            motors.get(motorIndex).setControl(velocityVoltage.withVelocity(rps)
                    .withFeedForward(calculateFF(motorIndex, rps, 0)));
        else
            motors.get(motorIndex).setControl(velocityVoltage.withOverrideBrakeDurNeutral(true)
                    .withVelocity(rps)
                    .withFeedForward(calculateFF(motorIndex, rps, 0)));
    }

    private double calculateFF(int motorIndex, double rps, double acceleration) {
        switch (type) {
            case LINEAR -> {
                return linearFF.calculate(rps, acceleration);
            }
            case PIVOT -> {
                return pivotFF.calculate(Math.toRadians(getMechDegrees(motorIndex) + ffOffset),
                        rps * 2 * Math.PI, acceleration * 2 * Math.PI);
            }
            default -> {
                return simpleMotorFF.calculate(rps, acceleration);
            }
        }
    }

    public void setPosition(double goal) {
        if (type == Type.FLYWHEEL) return;

        if (type == Type.LINEAR && goal < mechMin) goal = mechMin;
        else if (type == Type.LINEAR && goal > mechMax) goal = mechMax;

        followLastMechProfile = true;

        switch (type) {
            case LINEAR -> goalState.position = getMotorRotFromMechM(goal);
            case PIVOT -> goalState.position = getMotorRotFromDegrees(goal);
            default -> goalState.position = getMotorRotFromMechRot(goal);
        }

        goalState.velocity = 0;
        this.goal = goal;

        currentState = profile.calculate(0, currentState, goalState);
        timer.restart();
    }

    public void followLastMechProfile(int motorIndex) {
        if (type == Type.FLYWHEEL) return;

        TrapezoidProfile.State nextState = profile.calculate(timer.get(), currentState, goalState);
        motors.get(motorIndex).setControl(
                positionVoltage.withPosition(nextState.position)
                        .withFeedForward(calculateFF(motorIndex, nextState.velocity,
                                (nextState.velocity - currentState.velocity) / timer.get())));

        if (isProfileFinished()) followLastMechProfile = false;
        currentState = nextState;
        timer.restart();
    }

    public boolean isProfileFinished() {
        return currentState.position == goalState.position && currentState.velocity == goalState.velocity;
    }

    public boolean isMechAtGoal(int motorIndex, boolean isVelocity) {
        switch (type) {
            case LINEAR -> {
                return isProfileFinished() &&
                        getMechM(motorIndex) >= goal - lowerTolerance && getMechM(motorIndex) <= goal + upperTolerance;
            }
            case PIVOT -> {
                return isProfileFinished() &&
                        getMechDegrees(motorIndex) >= goal - lowerTolerance && getMechDegrees(motorIndex) <= goal + upperTolerance;
            }
            default -> {
                if (isVelocity) return getMechVelocity(motorIndex) >= goal - lowerTolerance
                        && getMechVelocity(motorIndex) <= goal - upperTolerance;
                else return getMechRot(motorIndex) >= goal - lowerTolerance
                        && getMechRot(motorIndex) <= goal + upperTolerance;
            }
        }
    }


    //Unit Conversions
    public double getMechDegrees(int motorIndex) {
        return motors.get(motorIndex).getPosition().getValueAsDouble() * gearRatio * 360d;
    }

    public double getMotorRot(int motorIndex) {
        return motors.get(motorIndex).getPosition().getValueAsDouble();
    }

    public double getMechRot(int motorIndex) {
        return motors.get(motorIndex).getPosition().getValueAsDouble() * gearRatio;
    }

    public double getDegreesFromMechRot(double mechRot) {
        return mechRot * gearRatio * 360d;
    }

    public double getDegreesFromMotorRot(double motorRot) {
        return motorRot * 360d;
    }

    public double getMechRotFromDegrees(double degrees) {
        return degrees / 360d / gearRatio;
    }

    public double getMotorRotFromDegrees(double degrees) {
        return degrees / 360d;
    }

    public double getMechRotFromMotorRot(double motorRot) {
        return motorRot * gearRatio;
    }

    public double getMotorRotFromMechRot(double mechRot) {
        return mechRot / gearRatio;
    }

    public double getMechM(int motorIndex) {
        if (type != Type.LINEAR) return 0;
        return motors.get(motorIndex).getPosition().getValueAsDouble() * drumCircumference / gearRatio;
    }

    public double getMechMFromMotorRot(double motorRot) {
        if (type != Type.LINEAR) return 0;
        return motorRot * drumCircumference / gearRatio;
    }

    public double getMotorRotFromMechM(double mechM) {
        if (type != Type.LINEAR) return 0;
        return mechM / drumCircumference * gearRatio;
    }

    //Motor Values
    public double getMotorVelocity(int motorIndex) {
        return motors.get(motorIndex).getVelocity().getValueAsDouble();
    }

    public double getMotorVoltage(int motorIndex) {
        return motors.get(motorIndex).getMotorVoltage().getValueAsDouble();
    }

    public double getSupplyVoltage(int motorIndex) {
        return motors.get(motorIndex).getSupplyVoltage().getValueAsDouble();
    }

    public double getMechVelocity(int motorIndex) {
        return getMechRotFromMotorRot(motors.get(motorIndex).getVelocity().getValueAsDouble());
    }

    @Override
    public void periodic() {
        for (int i = 0; i < motors.size(); i++) {
            if (followLastMechProfile) {
                followLastMechProfile(i);
            }
            if (type == Type.ROLLER && getMotorVelocity(i) == 0 && getMotorRot(i) != 0) {
                for (TalonFX motor : motors) {
                    motor.setPosition(0d);
                }
                currentState.position = 0;
                currentState.velocity = 0;
            }
        }
    }
}