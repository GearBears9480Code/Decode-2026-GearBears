package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {
    // the starting number = 4

    // motors
    private final SparkMax rotationMotor = new SparkMax(40, MotorType.kBrushless);
    private final SparkMax hoodMotor = new SparkMax(41, MotorType.kBrushless);
    private final SparkMax shooterMotor = new SparkMax(42, MotorType.kBrushless);

    // encoders
    private final RelativeEncoder rotationEncoder = rotationMotor.getEncoder();
    private final RelativeEncoder hoodEncoder = hoodMotor.getEncoder();

    public ShooterSubsystem() {
        // initialize stuff possibly
    }
    
    // used to get rotation of the turret based on the current rotation of the motor
    public double getRotation() {
        double motorRotation = rotationEncoder.getPosition() * 360;
        return motorRotation * ShooterConstants.turretRotationGearRatio;
    }
    
    // starts the shooter
    public void shoot() {
        shooterMotor.set(0.5);
    }
    
    // stops the shooter
    public void stop() {
        shooterMotor.set(0.0);
    }

    public void rotateHood(double velocity) {
        hoodMotor.set(velocity);
    }

    public void rotateTurret(double velocity) {
        rotationMotor.set(velocity);
    }

    public double getHoodRotation() {
        return hoodEncoder.getPosition() * 360;
    }
}
