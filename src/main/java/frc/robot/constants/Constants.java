// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.utility.FeedForward;
import frc.robot.utility.PID;

import frc.robot.utility.Type;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double LOOP_PERIOD_MS = 20.0;
    public static final double LOOP_PERIOD_S = Units.millisecondsToSeconds(LOOP_PERIOD_MS);

    public static class OperatorConstants {
        public static final int driverControllerPort = 0;
    }

    public static class ElevatorConstants {
        public static final int ELEVATOR1_ID = 22;
        public static final int ELEVATOR2_ID = 23;

        public static final TrapezoidProfile.Constraints ELEVATOR_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 3);
        public static final PID ELEVATOR_PID = new PID(100, 0, 0); //95.748, 5.4125
        public static final FeedForward ELEVATOR_FF = new FeedForward(0.043, 0.102, .111, 0.108);
        public static final double ELEVATOR_MIN = 0;
        public static final double ELEVATOR_MAX = 0.595;
        public static final double[][] ELEVATOR_GEARING = new double[][]{{48, 14}, {48, 24}};
        public static final boolean INVERT = false;
        public static final boolean BRAKE = false;
        public static final double SUPPLY_CURRENT_LIMIT = 60;
        public static final double STATOR_CURRENT_LIMIT = 100;
        public static final double DRUM_CIRCUMFERENCE = .0364;

    }

    public static class ArmConstants {
        public static final int ARM_MOTOR_ID = 14;
        public static final int ARM_FOLLOW_MOTOR_ID = 15;
        public static final int ARM_CANCODER_ID = 30;
        public static final TalonFXConfiguration TALON_FX_CONFIGURATION = new TalonFXConfiguration();
        public static final double ARM_STATOR_CURRENT_LIMIT = 60;
        public static final double ARM_SUPPLY_CURRENT_LIMIT = 60;
        public static final boolean LEFT_ARM_INVERTED = true;
        public static final boolean ARM_FOLLOWER_INVERTED = true;

        public static final boolean ARM_BRAKE = false;
        public static final boolean ARM_FOLLOW_BRAKE = false;


        public static final PID ARM_PID = new PID(0, 0, 0);


        //   public static final double ARM_FF_OFFSET = Units.degreesToRadians(13.5);

        public static final double ARM_FF_OFFSET = 0;
        public static final CANcoderConfiguration ARM_CANCODER_CONFIGURATION = new CANcoderConfiguration();

        /*This determines the range the cancoder records in rotations.
         * -1 = -1 to 0
         * 0 = -.5 to .5
         * 1 = 0 to 1
         */
        public static final double ARM_CANCODER_DISCONTINUITY_POINT = 1;

        //What direction should be considered postive.
//        public static final SensorDirectionValue ARM_CANCODER_DIRECTION = SensorDirectionValue.Clockwise_Positive;

        //When the arm is resting if the cancoder does not record 0 for absolute position multiply the value it record by -1 and put it here. This must be in rotations.
        public static final double ARM_CANCODER_MAGNET_OFFSET = -0.064453125;

        //The speed and acceleration the arm should move at.
        public static final TrapezoidProfile.Constraints ARM_CONSTRAINTS = new TrapezoidProfile.Constraints(800, 1600);


// kg = .348, ks = .14816, Kv = 6.16786
        // These are the values that will be factored into the arm ff equation. There is a separate documet to find these.

        public static final FeedForward ARM_FF = new FeedForward(0.14, 0, .123407);

        //degrees. check super for template subsystem
        public static final double ARM_LOWER_TOLERANCE = 0.001;
        public static final double ARM_UPPER_TOLERANCE = 0.001;

        //Degrees check super for template subsystem
        public static final double ARM_MIN = 0.61524;
        public static final double ARM_MAX = 57; //fix


        public static final double ARM_MAX_VELOCITY = 500;
        public static final double ARM_MAX_ACCELERATION = 750;
        public static final double ARM_MOTOR_TO_MECH_GEAR_RATIO = 42.4286;
        public static final double ARM_MOTOR_TO_SENSOR_GEAR_RATIO = 42.4286;
        public static final double ARM_SENSOR_TO_MECH_GEAR_RATIO = 1;

        public static final double[][] MOTOR_TO_MECH_GEAR_RATIO = {{125, 1}};

        //Value the arm should move to for a wanted position.
        public static final double GROUND = 5;
        public static final double L1 = 10;
        public static final double L2 = 15;
        public static final double L3 = 20;
        public static final double L4 = 25;
        public static final double HP = 35 / 3;
        public static final double GROUND_2 = 0;
        public static final String ARM_CANCODER_CANBUS = "rio";

        public static final Slot0Configs ARM_SLOT0_CONFIGS = new Slot0Configs()
                .withKP(2.5)
                .withKI(0)
                .withKD(0)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
                .withKS(ARM_FF.getkS())
                .withKG(ARM_FF.getkG())
                .withKV(ARM_FF.getkV())
                .withKA(ARM_FF.getkA())
                .withGravityType(GravityTypeValue.Arm_Cosine);


    }

    public static class ClawConstants {
        public static final Type CLAW_TYPE = Type.LINEAR;
        public static final int CLAW_ID = 0;
        public static final TrapezoidProfile.Constraints CLAW_CONSTRAINTS = new TrapezoidProfile.Constraints(0, 0);
        public static final PID CLAW_PID = new PID(0, 0, 0);
        public static final FeedForward CLAW_FEEDFORWARD = new FeedForward(0, 0, 0);
        public static final double CLAW_lowerTOLERANCE = 0.0;
        public static final double CLAW_upperTOLERANCE = 0.0;
        public static final double[][] CLAW_gearRatios = {{0}, {0}};
    }

    public static class AlgaeIntakeConstants {
        public static final Type ALGAEINTAKESUBYSTEM_TYPE = Type.LINEAR;
        public static final int ALGAEINTAKESUBYSTEM_ID = 0;
        public static final TrapezoidProfile.Constraints ALGAEINTAKESUBYSTEM_CONSTRAINTS = new TrapezoidProfile.Constraints(0, 0);
        public static final PID ALGAEINTAKESUBYSTEM_PID = new PID(0, 0, 0);
        public static final FeedForward ALGAEINTAKESUBYSTEM_FEEDFORWARD = new FeedForward(0, 0, 0);
        public static final double ALGAEINTAKESUBYSTEM_lowerTOLERANCE = 0.0;
        public static final double ALGAEINTAKESUBYSTEM_upperTOLERANCE = 0.0;
        public static final double[][] ALGAEINTAKESUBYSTEM_gearRatios = {{0}, {0}};
    }

    public static class ClimberConstants {
        public static final Type CLIMBER_TYPE = Type.PIVOT;
        public static final int CLIMBER_ID = 0;
        public static final TrapezoidProfile.Constraints CLIMBER_CONSTRAINTS = new TrapezoidProfile.Constraints(0, 0);
        public static final PID CLIMBER_PID = new PID(0, 0, 0);
        public static final FeedForward CLIMBER_FEEDFORWARD = new FeedForward(0, 0, 0);
        public static final double CLIMBER_lowerTOLERANCE = 0.0;
        public static final double CLIMBER_upperTOLERANCE = 0.0;
        public static final double[][] CLIMBER_gearRatios = {{0}, {0}};
    }

    public static class WristConstants {

        //TODO: Change all of these constants as they were just copied from arm

        public static final int WRIST_MOTOR_ID = 240000;
        public static final int WRIST_CANCODER_ID = 300000;
        public static final TalonFXConfiguration TALON_FX_CONFIGURATION = new TalonFXConfiguration();
        public static final double WRIST_STATOR_CURRENT_LIMIT = 80;
        public static final double WRIST_SUPPLY_CURRENT_LIMIT = 80;
        public static final boolean WRIST_INVERTED = true;

        public static final Slot0Configs WRIST_SLOT0_CONFIGS = new Slot0Configs()
                .withKP(135)
                .withKI(0)
                .withKD(0)
                .withKS(0)
                .withKG(0)
                .withKV(0)
                .withKA(0);


        public static final double WRIST_FF_OFFSET = Units.degreesToRadians(13.5);

        public static final CANcoderConfiguration WRIST_CANCODER_CONFIGURATION = new CANcoderConfiguration();
        public static final double WRIST_CANCODER_DISCONTINUITY_POINT = 1;
        public static final SensorDirectionValue WRIST_CANCODER_DIRECTION
                = SensorDirectionValue.Clockwise_Positive;

        //rotations
        public static final double WRIST_CANCODER_MAGNET_OFFSET = 0.930176;


        public static final ArmFeedforward WRIST_FF = new ArmFeedforward(.175, .707, 2.5, 0.0);
        //degrees
        public static final double WRIST_LOWER_TOLERANCE = .75;
        public static final double WRIST_UPPER_TOLERANCE = 0.75;
        //Degrees
        public static final double WRIST_MIN = 0.0;
        public static final double WRIST_MAX = 57;

        public static final double WRIST_MAX_VELOCITY = 500;
        public static final double WRIST_MAX_ACCELERATION = 750;
        public static final double WRIST_MOTOR_TO_MECH_GEAR_RATIO = 42.4286;
        public static final double WRIST_MOTOR_TO_SENSOR_GEAR_RATIO = 1;
        public static final double WRIST_SENSOR_TO_MECH_GEAR_RATIO = 42.4286;

        public static final double
                GROUND = 0,
                L1 = 0,
                L2 = 0,
                L3 = 0,
                L4 = 0,
                HP = 0,
                GROUND_2 = 0;


    }

    public static final String Vision = null;
}
