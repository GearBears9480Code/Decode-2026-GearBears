// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 1;
    public static final int kMechanismControllerPort = 0;
  }

  public static class VisionConstants {
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0.002, 0.006, 0.002);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.001, 0.001, 0.0008);

    // camera one - angle 230 y = -11.5 x =16
    // camera two - angle 310 x = -3.5 y = -12.5

    // public static final Pose2d camOnePose = new Pose2d(Units.inchesToMeters(16), Units.inchesToMeters(-11.5), new Rotation2d(Units.degreesToRadians(230)));
    // public static final Pose2d camTwoPose = new Pose2d(Units.inchesToMeters(-3.5), Units.inchesToMeters(-12.5), new Rotation2d(Units.degreesToRadians(310)));
  }

  public static class ShooterConstants {
    public static final int numGearTurretMotor = 12;
    public static final int numGearTurretTrack = 24;
    public static final double turretRotationGearRatio = (double)23/(double)130; // 1/5.41
    public static final double flyWheelCircumfrance = 2;

    public static final double constantHorizontalVelocity = 2;
    public static final double height = 1.1176;

    public static final double turretKp = 0.01;
    public static final double turretKi = 0.0;
    public static final double turretKd = 0.00075;

    public static final double turretX = Units.inchesToMeters(-4);
    public static final double turretY = Units.inchesToMeters(8);

    public static final double hoodKp = 0.1;
    public static final double hoodKi = 0.0;
    public static final double hoodKd = 0.0;

    public static final double flyWheelKp = 10;
    public static final double flyWheelKi = 0.0;
    public static final double flyWheelKd = 0.0;

    public static final Pose2d hubBlue = new Pose2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84), new Rotation2d(0));
    public static final Pose2d hubRed = new Pose2d(Units.inchesToMeters(182.11 + 287), Units.inchesToMeters(158.84), new Rotation2d(0));
    
  }

  public static class PhysicalConstants {
    public static final int neoMaxRPM = 5676;
    public static final double hubWidth = 1.1938;
  }

  public static class IntakeConstants {
    // PID CONSTANTS
    public static final double armMotor_kP = 1;
    public static final double armMotor_kI = 0.0;
    public static final double armMotor_kD = 0.05;

    public static final double velocityMotor_kP = 0.1;
    public static final double velocityMotor_kI = 0.0;
    public static final double velocityMotor_kD = 0.0;

    public static final double distancePerRotation = 3.25; // inches

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

