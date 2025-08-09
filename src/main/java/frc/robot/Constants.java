// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  // TODO: Insert DriveConstants here...
  public static class DriveConstants {
    public static final int kLeftLeaderMotorPort = 1;
    public static final int kLeftFollowerMotorPort = 2;
    public static final int kRightLeaderMotorPort = 3;
    public static final int kRightFollowerMotorPort = 4;
  }

  // Arm Constants
  public static class ArmConstants {
    public final static int canID = 12;
    public final static double gearRatio = 25;
    public static double kP = 100;
    public static double kI = 2;
    public static double kD = 1;
    public final static double maxVelocity = 100;
    public final static double maxAcceleration = 100; 
    public final static boolean brakeMode = true;
    public final static boolean enableStatorLimit = true;
    public final static double statorCurrentLimit = 40;
    public final static boolean enableSupplyLimit = false;
    public final static double supplyCurrentLimit = 40;
    public final static double armLength = 0.5;
    public final static double minAngleDeg = 0;
    public final static double maxAngleDeg = 90;
    public final static double kS = 0.1;
    public final static double kG = 0.2;
    public final static double kA = 0.3;
    public final static double kV = 0.4;
  } 
}
