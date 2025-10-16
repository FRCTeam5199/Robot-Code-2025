// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
                = new TrapezoidProfile.Constraints(150, 300);
        public static final FeedForward ELEVATOR_FF = new FeedForward(.4, .3, .178);
        public static final double ELEVATOR_MIN = 0;
        public static final double ELEVATOR_MAX = 1.01;
        public static final double ELEVATOR_LOWER_TOLERANCE = 0.05;
        public static final double ELEVATOR_UPPER_TOLERANCE = 0.05;
        public static final double[][] ELEVATOR_GEARING = new double[][]{{3, 1}};
        public static final boolean INVERT = true;
        public static final boolean FOLLOWER_OPPOSE_MASTER_DIRECTION = true;
        public static final boolean ELEVATOR_BRAKE = true;
        public static final double ELEVATOR_SUPPLY_CURRENT_LIMIT = 100;
        public static final double ELEVATOR_STATOR_CURRENT_LIMIT = 100;
        public static final double DRUM_CIRCUMFERENCE = .119694706;

        public static final Slot0Configs ELEVATOR_SLOT0_CONFIGS = new Slot0Configs()
                .withKP(2)
                .withKI(0)
                .withKD(0.1)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
                .withKS(ELEVATOR_FF.getkS())
                .withKG(ELEVATOR_FF.getkG())
                .withKV(ELEVATOR_FF.getkV())
                .withGravityType(GravityTypeValue.Elevator_Static);

        public static final double STABLE = 0;
        public static final double ALGAE_STABLE = .17;
        public static final double GROUND = .11;
        public static final double ALGAE_GROUND = .12;

        public static final double L1 = 0;
        public static final double L2 = .19;
        public static final double L3 = .44;
        public static final double L4 = 1.01;
        public static final double HP = 0;

        public static final double ALGAE_LOW = .37;
        public static final double ALGAE_HIGH = .75;
        public static final double BARGE = 1.002;
        public static final double PROCESSOR = .22;
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
                = new TrapezoidProfile.Constraints(150, 300);
        public static final FeedForward ARM_FF = new FeedForward(0.23, 0.07, .01938507206859285189665004237877);

        public static final double ARM_LOWER_TOLERANCE = 4;
        public static final double ARM_UPPER_TOLERANCE = 4;

        // public static final double STABLE = 0;
        public static final double GROUND = 4;
        public static final double ALGAE_GROUND = 11;

        public static final double L1 = 16;
        public static final double L2 = 32;
        public static final double L3 = 54;
        public static final double L4 = 72;
        public static final double HP = 63;
        public static final double HP_C = 75;

        public static final double ALGAE_LOW = 61;
        public static final double ALGAE_HIGH = 68;
        public static final double BARGE = 85;
        public static final double PROCESSOR = 30;

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
        public static final int INTAKE_ID = 21;
        public static final int INTAKE_SECONDARY_ID = 20;
        public static final int CORAL_SENSOR_ID = 0;
        public static final int ALGAE_SENSOR_ID = 1;
        public static final TrapezoidProfile.Constraints INTAKE_CONSTRAINTS = new TrapezoidProfile.Constraints(0, 0);
        public static final FeedForward INTAKE_FF = new FeedForward(.55, 0, .1960784313725490196078431372549);
        public static final FeedForward INTAKE_SECONDARY_FF = new FeedForward(.4, 0, .13333333333333333333333333333333);
        public static final double INTAKE_LOWER_TOLERANCE = 5;
        public static final double INTAKE_UPPER_TOLERANCE = 5;
        public static final double[][] INTAKE_GEAR_RATIO = {{1, 1}};

        public static final boolean INTAKE_INVERT = true;
        public static final boolean INTAKE_SECONDARY_INVERT = false;
        public static final boolean INTAKE_BRAKE = true;
        public static final double INTAKE_STATOR_CURRENT_LIMIT = 75;
        public static final double INTAKE_SUPPLY_CURRENT_LIMIT = 75;

        public static final Slot0Configs INTAKE_SLOT0_CONFIGS = new Slot0Configs()
                .withKP(15)
                .withKI(0)
                .withKD(0)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
                .withKS(INTAKE_FF.getkS())
                .withKG(INTAKE_FF.getkG())
                .withKV(INTAKE_FF.getkV());

        public static final Slot0Configs INTAKE_SECONDARY_SLOT0_CONFIGS = new Slot0Configs()
                .withKP(15)
                .withKI(0)
                .withKD(0)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
                .withKS(INTAKE_SECONDARY_FF.getkS())
                .withKG(INTAKE_SECONDARY_FF.getkG())
                .withKV(INTAKE_SECONDARY_FF.getkV());
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
        public static final double WRIST_STATOR_CURRENT_LIMIT = 30;
        public static final double WRIST_SUPPLY_CURRENT_LIMIT = 30;
        public static final boolean WRIST_INVERTED = false;

        public static final double STABLE = 10;
        public static final double GROUND = 179;
        public static final double ALGAE_GROUND = 185;

        public static final double L1 = 10;
        public static final double L2 = 61;
        public static final double L3 = 88;
        public static final double L4 = 128.5;

        public static final double HP = 61;
        public static final double HP_C = 67;

        public static final double ALGAE_LOW = 230;
        public static final double ALGAE_HIGH = 243;
        public static final double BARGE = 140;
        public static final double PROCESSOR = 185;
        public static final double ALGAE_STABLE = 160;
        public static final double ALGAE_PREP = 120;

        public static final boolean WRIST_BRAKE = true;

        public static final double WRIST_FF_OFFSET = Units.degreesToRadians(0);
        public static final TrapezoidProfile.Constraints WRIST_CONSTRAINTS
                = new TrapezoidProfile.Constraints(100, 150);

        public static final FeedForward WRIST_FF = new FeedForward(.16, .3, .04405673165); //.134
        public static final Slot0Configs WRIST_SLOT0_CONFIGS = new Slot0Configs()
                .withKP(2.8)
                .withKI(0)
                .withKD(0.01)
                .withKS(WRIST_FF.getkS())
                .withKG(WRIST_FF.getkG())
                .withKV(WRIST_FF.getkV())
                .withKA(0)
                .withGravityType(GravityTypeValue.Arm_Cosine);

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


        public static final String BACK_CAMERA_NAME = "BackCamera";
        public static final Transform3d BACK_CAMERA_POSE =
                new Transform3d(-.2794, .3175, .19685, new Rotation3d(0, Math.toRadians(0), Math.toRadians(180)));
        public static final double BACK_CAMERA_TO_BACK_DISTANCE = .05;

        public static final String LIMELIGHT_NAME = "limelight";
        public static final double LIMELIGHT_X_AIM = -29;

        public static final double AUTO_ALIGN_X = -0.05;
        public static final double AUTO_ALIGN_X_BACK = -.3;
        public static final double AUTO_ALIGN_Y = .15;

        public static final Pose2d ALGAE_BLUE_POSE
                = new Pose2d(7.265, 3.851, new Rotation2d(Math.toRadians(180)));
        public static final Pose2d ALGAE_RED_POSE
                = new Pose2d(10.400, 3.851, new Rotation2d(0));
    }
}

