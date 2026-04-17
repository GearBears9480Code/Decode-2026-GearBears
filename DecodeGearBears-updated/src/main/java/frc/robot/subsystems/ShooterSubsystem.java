package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.ShooterPIDCommand;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {
    // the starting number = 4

    // motors
    private final SparkMax rotationMotor = new SparkMax(40, MotorType.kBrushless);
    private final SparkMax hoodMotor = new SparkMax(41, MotorType.kBrushed);
    private final TalonFX shooterMotor = new TalonFX(42);
    // encoders
    private final RelativeEncoder rotationEncoder = rotationMotor.getAlternateEncoder();
    private final RelativeEncoder hoodEncoder = hoodMotor.getEncoder();

    Pose2d hubPose;

    public ShooterPIDCommand pid;
    double distFromHub;
    HopperSubsystem hopper;


    public double velocity = 0;

    private VisionSubsystem vision;

    public ShooterSubsystem(VisionSubsystem v, HopperSubsystem hop) {
        // initialize stuff possibly
        resetPositions();
        
        SmartDashboard.putBoolean("can shoot", true);
        
        hopper = hop;
        vision = v;
        pid = new ShooterPIDCommand(this);

        hubPose = DriverStation.getAlliance().isPresent() && (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) ? ShooterConstants.hubBlue : ShooterConstants.hubRed;
    }

    public void getHubPose() {
        hubPose = DriverStation.getAlliance().isPresent() && (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) ?
                 ShooterConstants.hubBlue : ShooterConstants.hubRed;
    }
    public void resetPositions() {
        rotationEncoder.setPosition(0);
        // hoodEncoder.setPosition(0);
    }
    
    public void resetMotors() {
        rotationMotor.set(0);
        velocity = 0;
        hoodMotor.set(0);
        shooterMotor.set(0);
    }
    
    public void setRotationEncoder(double degrees) {
        double r = (degrees / ShooterConstants.turretRotationGearRatio) / (double)-360;

        rotationEncoder.setPosition(r);
    }

    // used to get rotation of the turret based on the current rotation of the motor
    public double getRotation() {
        double motorRotation = ((rotationEncoder.getPosition()) * -360);
        SmartDashboard.putNumber("motor angle", motorRotation);
        SmartDashboard.putNumber("turret angle", motorRotation * ShooterConstants.turretRotationGearRatio);
        return motorRotation * ShooterConstants.turretRotationGearRatio;
    }
    
    public void rotateTurret(double velocity) {
        SmartDashboard.putNumber("turret velocity", velocity);
        rotationMotor.set(velocity);
    }

    public void rotateHood(double velocity) {
        hoodMotor.set(velocity);
        SmartDashboard.putNumber("hood velocity", velocity);
    }
    

    public double getHoodRotation() {
        double angle = (hoodEncoder.getPosition() * ShooterConstants.hoodGearRatio) * -360;
        SmartDashboard.putNumber("hood angle", angle);
        return angle;
    }

    // starts the shooter
    public void shoot(double velocity) {
        shooterMotor.set(velocity);
        if (velocity != 0) {
            hopper.hopperPIDCommand.changeKickerSpeed(3000);
        } else {
            hopper.hopperPIDCommand.changeKickerSpeed(0);
        }

    }

    public double getCalcVelocity() {
        if (distFromHub > 1.989 && distFromHub <= 4.1) {
            SmartDashboard.putBoolean("can shoot", true);
            pid.changeHoodAngle(0);
            hopper.hopperPIDCommand.changeKickerSpeed(3000);
            return (0.238314 * distFromHub) - 0.000473725;
        } else if (distFromHub > 4.1 && distFromHub <= 4.7) {
            SmartDashboard.putBoolean("can shoot", true);
            pid.changeHoodAngle(3);
            hopper.hopperPIDCommand.changeKickerSpeed(3000);
            return (0.138599 * distFromHub) + 0.159585;
        } else if (distFromHub > 4.7 && distFromHub <= 5.12) {
            SmartDashboard.putBoolean("can shoot", true);
            pid.changeHoodAngle(5.13);
            hopper.hopperPIDCommand.changeKickerSpeed(400);
            return (0.138599 * distFromHub) + 0.119585;
        } else if (distFromHub > 5.12) {
            SmartDashboard.putBoolean("can shoot", true);
            pid.changeHoodAngle(5);
            hopper.hopperPIDCommand.changeKickerSpeed(800);
            return (0.138599 * distFromHub) + 0.129585;
        } else {
            SmartDashboard.putBoolean("can shoot", false);
            pid.changeHoodAngle(0);
            hopper.hopperPIDCommand.changeKickerSpeed(0);
            return 0;
        }
    }

    // stops the shooter
    public void stop() {
        velocity = 0;
        shooterMotor.set(0.0);
    }
    
    
    public void periodic() {
        // get the position of the turret in field space
        Matrix<N2, N1> turretPosition = vision.getPointFieldOriented(ShooterConstants.turretX, ShooterConstants.turretY);
        
        // get angle of 
        double x = hubPose.getX() - turretPosition.getData()[0];
        double y = hubPose.getY() - turretPosition.getData()[1];
        
        distFromHub = Math.hypot(x, y);
        // System.out.println(distFromHub);
        
        double angle = Math.toDegrees(Math.atan2(y, x));
        // System.out.println("x: " + x  + ", y: " + y + ", angle: " + angle);
        double desiredAngle = angle - vision.getVisionPose().getRotation().getDegrees();
        
        desiredAngle = desiredAngle % 360;
        
        if (desiredAngle < 0) {
            desiredAngle = 360 + desiredAngle;
        }

        getRotation();
        getHoodRotation();

        pid.changeTurretAngle(desiredAngle);
    }
}
