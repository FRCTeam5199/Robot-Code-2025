// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

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

  public static class ElevatorConstants{
    public static final int ELEVATOR_ID = 0;
    public static final TrapezoidProfile.Constraints ELEVATOR_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 3);
    public static final PID ELEVATOR_PID = new PID(0, 0, 0);
    public static final FeedForward ELEVATOR_FF = new FeedForward(0, 0, 0);
    public static final double ELEVATOR_MIN = 0;
    public static final double ELEVATOR_MAX = 0;
    public static final double[][] ELEVATOR_GEARING = new double[][]{{0, 0,}, {0, 0}};
    public static final boolean INVERT = false;
    public static final boolean BRAKE = false;
    public static final double SUPPLY_CURRENT_LIMIT = 60;
    public static final double STATOR_CURRENT_LIMIT = 100;
  }

  public static class ArmConstants{
    public static final int ARM_ID = 0;
    public static final double GROUND = 0;
    public static final double L1 = 0;
    public static final double L2 = 0;
    public static final double L3 = 0;
    public static final double L4 = 0;
    public static final double HP = 0;
    public static final double GROUND_2 = 0;

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
    public static final int WRIST_ID = 0;
    public static final double GROUND = 0;
    public static final double L1 = 0;
    public static final double L2 = 0;
    public static final double L3 = 0;
    public static final double L4 = 0;
    public static final double HP = 0;
    public static final double GROUND_2 = 0;

  }

  public static final String Vision = null;
}
