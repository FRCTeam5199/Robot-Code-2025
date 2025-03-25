// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.utility.FeedForward;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static class OperatorConstants {
        public static final int driverControllerPort = 0;
        public static final int operatorControllerPort = 3;
        public static final int buttonPanel1Port = 1;
        public static final int buttonPanel2Port = 2;
    }

    public static class ElevatorConstants {
        public static final int ELEVATOR_LEFT_ID = 17;
        public static final int ELEVATOR_RIGHT_ID = 18;

        public static final TrapezoidProfile.Constraints ELEVATOR_CONSTRAINTS
                = new TrapezoidProfile.Constraints(75, 150);
        public static final FeedForward ELEVATOR_FF = new FeedForward(.42, .18, .16666666666666666666666666666667);
        public static final double ELEVATOR_MIN = 0;
        public static final double ELEVATOR_MAX = 1.002;
        public static final double ELEVATOR_LOWER_TOLERANCE = 0.05;
        public static final double ELEVATOR_UPPER_TOLERANCE = 0.05;
        public static final double[][] ELEVATOR_GEARING = new double[][]{{3, 1}};
        public static final boolean INVERT = true;
        public static final boolean FOLLOWER_OPPOSE_MASTER_DIRECTION = true;
        public static final boolean ELEVATOR_BRAKE = true;
        public static final double ELEVATOR_SUPPLY_CURRENT_LIMIT = 45;
        public static final double ELEVATOR_STATOR_CURRENT_LIMIT = 45;
        public static final double DRUM_CIRCUMFERENCE = .119694706;

        public static final Slot0Configs ELEVATOR_SLOT0_CONFIGS = new Slot0Configs()
                .withKP(.75)
                .withKI(0)
                .withKD(0)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
                .withKS(ELEVATOR_FF.getkS())
                .withKG(ELEVATOR_FF.getkG())
                .withKV(ELEVATOR_FF.getkV())
                .withGravityType(GravityTypeValue.Elevator_Static);

        public static final double STABLE = 0;
        public static final double GROUND = .12; //.17

        public static final double L1 = 0;
        public static final double L2 = .14;
        public static final double L3 = .4;
        public static final double L4 = 1.002;
        public static final double HP = .0;

        public static final double ALGAE_LOW = .13;
        public static final double ALGAE_HIGH = .45;
        public static final double BARGE = 1.002;
        public static final double PROCESSOR = .12;
    }

    public static class ArmConstants {
        public static final int ARM_MOTOR_ID = 14;
        public static final int ARM_FOLLOW_MOTOR_ID = 15;
        public static final double ARM_STATOR_CURRENT_LIMIT = 60;
        public static final double ARM_SUPPLY_CURRENT_LIMIT = 60;
        public static final boolean LEFT_ARM_INVERTED = true;
        public static final boolean ARM_FOLLOWER_INVERTED = true;

        public static final boolean ARM_BRAKE = true;

        public static final double ARM_FF_OFFSET = 0;

        public static final TrapezoidProfile.Constraints ARM_CONSTRAINTS
                = new TrapezoidProfile.Constraints(800, 1600);
        public static final FeedForward ARM_FF = new FeedForward(0.2, 0, .1098901098901099);

        public static final double ARM_LOWER_TOLERANCE = 2;
        public static final double ARM_UPPER_TOLERANCE = 2;

        public static final double STABLE = 0;
        public static final double GROUND = 11;

        public static final double L1 = 18;
        public static final double L2 = 44;
        public static final double L3 = 64;
        public static final double L4 = 78;
        public static final double HP = 63;

        public static final double ALGAE_LOW = 60;
        public static final double ALGAE_HIGH = 70;
        public static final double BARGE = 90;
        public static final double PROCESSOR = 0;

        public static final double ARM_MIN = 0.61524;
        public static final double ARM_MAX = 180; //fix

        public static final double[][] ARM_GEAR_RATIO = {{240, 1}};

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

    public static class IntakeConstants {
        public static final int INTAKE_ID = 20;
        public static final int INTAKE_SENSOR_ID = 2;
        public static final TrapezoidProfile.Constraints INTAKE_CONSTRAINTS = new TrapezoidProfile.Constraints(0, 0);
        public static final FeedForward INTAKE_FF = new FeedForward(0.27, 0, .09090909090909090909090909090909);
        public static final double INTAKE_LOWER_TOLERANCE = 5;
        public static final double INTAKE_UPPER_TOLERANCE = 5;
        public static final double[][] INTAKE_GEAR_RATIO = {{1, 1}};

        public static final boolean INTAKE_INVERT = false;
        public static final boolean INTAKE_BRAKE = true;
        public static final double INTAKE_STATOR_CURRENT_LIMIT = 100;
        public static final double INTAKE_SUPPLY_CURRENT_LIMIT = 100;

        public static final Slot0Configs INTAKE_SLOT0_CONFIGS = new Slot0Configs()
                .withKP(.75)
                .withKI(0)
                .withKD(0)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
                .withKS(INTAKE_FF.getkS())
                .withKG(INTAKE_FF.getkG())
                .withKV(INTAKE_FF.getkV());
    }

    public static class ClimberConstants {
        public static final int CLIMBER_ID = 16;
        public static final TrapezoidProfile.Constraints CLIMBER_CONSTRAINTS = new TrapezoidProfile.Constraints(0, 0);
        public static final FeedForward CLIMBER_FF = new FeedForward(0, 0, 0);
        public static final double CLIMBER_LOWER_TOLERANCE = 0.0;
        public static final double CLIMBER_UPPER_TOLERANCE = 0.0;
        public static final double[][] CLIMBER_GEAR_RATIO = {{125, 1}};
        public static final double CLIMBER_LOW_LIMIT = 10;
        public static final double CLIMBER_HIGH_LIMIT = 110;


        public static final boolean CLIMBER_INVERT = true;
        public static final boolean CLIMBER_BRAKE = true;
        public static final double CLIMBER_STATOR_CURRENT_LIMIT = 100;
        public static final double CLIMBER_SUPPLY_CURRENT_LIMIT = 100;

        public static final Slot0Configs CLIMBER_SLOT0_CONFIGS = new Slot0Configs()
                .withKP(2.5)
                .withKI(0)
                .withKD(0)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
                .withKS(CLIMBER_FF.getkS())
                .withKG(CLIMBER_FF.getkG())
                .withKV(CLIMBER_FF.getkV())
                .withKA(CLIMBER_FF.getkA())
                .withGravityType(GravityTypeValue.Arm_Cosine);
    }

    public static class WristConstants {
        public static final int WRIST_MOTOR_ID = 19;
        public static final double WRIST_STATOR_CURRENT_LIMIT = 20;
        public static final double WRIST_SUPPLY_CURRENT_LIMIT = 20;
        public static final boolean WRIST_INVERTED = false;

        public static final double STABLE = 10;
        public static final double GROUND = 202;
        public static final double L1 = 10;
        public static final double L2 = 68;
        public static final double L3 = 94;
        public static final double PREVIOUS_L4 = 100;
        public static final double L4 = 135;
        public static final double HP = 62;

        public static final double ALGAE_LOW = 200;
        public static final double ALGAE_HIGH = 200;
        public static final double BARGE = 215;
        public static final double PROCESSOR = 200;

        public static final boolean WRIST_BRAKE = true;

        public static final Slot0Configs WRIST_SLOT0_CONFIGS = new Slot0Configs()
                .withKP(2)
                .withKI(0)
                .withKD(0.01)
                .withKS(0)
                .withKG(0)
                .withKV(0)
                .withKA(0);


        public static final double WRIST_FF_OFFSET = Units.degreesToRadians(0);
        public static final TrapezoidProfile.Constraints WRIST_CONSTRAINTS = new TrapezoidProfile.Constraints(75, 500);

        public static final FeedForward WRIST_FF = new FeedForward(.16, .18, .11764705882352941176470588235294);

        public static final double WRIST_LOWER_TOLERANCE = 2;
        public static final double WRIST_UPPER_TOLERANCE = 2;

        public static final double WRIST_MIN = 0.0;
        public static final double WRIST_MAX = 215; //unknown

        public static final double[][] WRIST_GEAR_RATIO = {{72, 10}, {72, 20}, {48, 24}};
    }


    public static class Vision {
        public static final String CAMERA_NAME = "Camera";
        public static final Transform3d CAMERA_POSE =
                new Transform3d(-.0318, 0, .174625, new Rotation3d(0, Math.toRadians(6), 0));
        public static final double CAMERA_TO_FRONT_DISTANCE = .46355;
        public static final Matrix<N3, N1> kTagStdDevs = VecBuilder.fill(3.5, 3.5, 999);
    }
}

