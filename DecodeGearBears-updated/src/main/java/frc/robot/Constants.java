// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final int kDriverControllerPort = 1;
    public static final int kMechanismControllerPort = 0;
  }

  public static class ShooterConstants {
    public static final int numGearTurretMotor = 12;
    public static final int numGearTurretTrack = 24;
    public static final double turretRotationGearRatio = (double)23/(double)130; // 1/5.41
    public static final double flyWheelCircumfrance = 2;

    public static final double constantHorizontalVelocity = 2;
    public static final double height = 1.1176;

    public static final double turretKp = 0.005;
    public static final double turretKi = 0.0;
    public static final double turretKd = 0.000;

    public static final double hoodKp = 0.1;
    public static final double hoodKi = 0.0;
    public static final double hoodKd = 0.0;

    public static final double flyWheelKp = 0.1;
    public static final double flyWheelKi = 0.0;
    public static final double flyWheelKd = 0.0;
  }

  public static class PhysicalConstants {
    public static final int neoMaxRPM = 5676;
  }

  public static class IntakeConstants {
    // PID CONSTANTS
    public static final double armUpMotor_kP = 0.075;
    public static final double armUpMotor_kI = 0.0;
    public static final double armUpMotor_kD = 0.015;
    
    public static final double armDownMotor_kP = 0.075;
    public static final double armDownMotor_kI = 0.0;
    public static final double armDownMotor_kD = 0.0;

    public static final double velocityMotor_kP = 0.1;
    public static final double velocityMotor_kI = 0.0;
    public static final double velocityMotor_kD = 0.0;

    public static final double armGearRatio = (double)14/(double)36;
  }

  public static class HopperConstants {
    public static final double spinKp = 0.1;
    public static final double spinKi = 0.0;
    public static final double spinKd = 0.0;

    public static final double kickKp = 0.1;
    public static final double kickKi = 0.0;
    public static final double kickKd = 0.0;

  }

  public static class ClimberConstants {
    public static final double armKp = 0.1;
    public static final double armKi = 0.0;
    public static final double armKd = 0.0;
    
    public static final double climbKp = 0.1;
    public static final double climbKi = 0.0;
    public static final double climbKd = 0.0;
  }
}

