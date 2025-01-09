// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.FeedForward;
import frc.robot.utils.PID;

import edu.wpi.first.math.util.Units;

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

  public static final String Vision = null;
}
