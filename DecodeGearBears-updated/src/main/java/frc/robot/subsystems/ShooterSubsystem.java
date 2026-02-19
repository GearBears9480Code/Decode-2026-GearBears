package frc.robot.subsystems;

import java.time.chrono.MinguoEra;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.setRotation;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {
    // the starting number = 4

    // motors
    private final SparkMax rotationMotor = new SparkMax(45, MotorType.kBrushed);
    // private final SparkMax hoodMotor = new SparkMax(41, MotorType.kBrushless);
    // private final SparkMax shooterMotor = new SparkMax(42, MotorType.kBrushless);
    private SparkMax hoodMotor;
    private SparkMax shooterMotor;
    // encoders
    private RelativeEncoder rotationEncoder = rotationMotor.getEncoder();
    private RelativeEncoder hoodEncoder;
    public setRotation turretPID;

    // angle limits
    private double minRotation = -90;
    private double maxRotation = 90;

    private ClientSubsystem client;

    public ShooterSubsystem(ClientSubsystem cli) {
        // initialize stuff possibly
        rotationEncoder.setPosition(0);
        rotateTurret(0);
        turretPID = new setRotation(this::getRotation, this::rotateTurret, 0, this, true);
        
        client = cli;
    }

    public void resetMotors() {
        rotationMotor.set(0);
    }
    
    // used to get rotation of the turret based on the current rotation of the motor
    public double getRotation() {
        double motorRotation = ((rotationEncoder.getPosition() / 4) * -360);
        SmartDashboard.putNumber("motor angle", motorRotation);
        SmartDashboard.putNumber("turret angle", motorRotation * ShooterConstants.turretRotationGearRatio);
        return motorRotation * ShooterConstants.turretRotationGearRatio;
    }
    
    // starts the shooter
    public void shoot() {
        shooterMotor.set(-1);
    }
    
    // stops the shooter
    public void stop() {
        shooterMotor.set(0.0);
    }

    public void rotateHood(double velocity) {
        hoodMotor.set(velocity);
    }

    public void rotateTurret(double velocity) {
        SmartDashboard.putNumber("turret velocity", velocity);
        rotationMotor.set(velocity);
    }

    public double getHoodRotation() {
        return hoodEncoder.getPosition() * 360;
    }

    public void periodic() {
        double angle = getRotation() + client.getDesiredAngle(false);
        
        if (angle < minRotation) {
            angle = minRotation;
        } else if (angle > maxRotation) {
            angle = maxRotation;
        }

        turretPID.changeSetpoint(angle);
        // turretPID.changeSetpoint(angle);
    }
}
