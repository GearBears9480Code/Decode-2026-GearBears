package frc.robot.subsystems;

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
    private final SparkMax shooterMotor = new SparkMax(42, MotorType.kBrushless);
    // encoders
    private final RelativeEncoder rotationEncoder = rotationMotor.getAlternateEncoder();
    private final RelativeEncoder hoodEncoder = hoodMotor.getEncoder();
    private final RelativeEncoder velocityEncoder = shooterMotor.getEncoder();

    Pose2d hubPose;

    public ShooterPIDCommand pid;
    double distFromHub;

    private ClientSubsystem client;

    public double velocity = 0;

    private SwerveSubsystem swerve;

    public ShooterSubsystem(ClientSubsystem cli, SwerveSubsystem s) {
        // initialize stuff possibly
        resetPositions();
        rotateTurret(0);
        
        client = cli;
        swerve = s;
        pid = new ShooterPIDCommand(this);

        hubPose = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? ShooterConstants.hubBlue : ShooterConstants.hubRed;
    }

    public void resetPositions() {
        rotationEncoder.setPosition(0);
    }

    public void resetMotors() {
        rotationMotor.set(0);
        velocity = 0;
        hoodMotor.set(0);
        shooterMotor.set(0);
    }
    
    // used to get rotation of the turret based on the current rotation of the motor
    public double getRotation() {
        double motorRotation = ((rotationEncoder.getPosition()) * -360);
        SmartDashboard.putNumber("motor angle", motorRotation);
        SmartDashboard.putNumber("turret angle", motorRotation * ShooterConstants.turretRotationGearRatio);
        return motorRotation * ShooterConstants.turretRotationGearRatio;
    }
    
    // starts the shooter
    public void shoot(double velocity) {
        shooterMotor.set(velocity);
    }

    public double getCalcVelocity() {
        if (distFromHub > 2.4) {
            return (0.0951046 * distFromHub) + 0.325379;
        } else {
            return 0;
        }
    }

    public double getEstimatedVelocity() {
        if (distFromHub >= 2.421 && distFromHub < 2.6) {
            return 0.57;
        } else if (distFromHub >= 2.6 && distFromHub < 2.9) {
            return 0.6;
        } else if (distFromHub >= 2.9 && distFromHub < 3.5) {
            return 0.65;
        } else if (distFromHub >= 3.5 && distFromHub < 3.8) {
            return 0.71;
        } else if (distFromHub >= 4.0824 && distFromHub < 5.67) {
            return 0.71;
        } else if (distFromHub > 5.67) {
            return 0.9;
        } else {
            return 0.0;
        }
    }

    public double getVelocity() {
        // equation to convert RPM -> m/s or velocity
        // the encoder reads RPM for velocity not actually m/s
        // return (velocityEncoder.getVelocity() * ShooterConstants.flyWheelCircumfrance) / 60;
        return velocityEncoder.getVelocity();
    }
    
    // stops the shooter
    public void stop() {
        velocity = 0;
        shooterMotor.set(0.0);
    }

    public void rotateHood(double velocity) {
        hoodMotor.set(velocity);
        SmartDashboard.putNumber("hood velocity", velocity);
    }

    public void rotateTurret(double velocity) {
        SmartDashboard.putNumber("turret velocity", velocity);
        rotationMotor.set(velocity);
    }

    public double getHoodRotation() {
        double angle = (hoodEncoder.getPosition() * 2) * -360;
        SmartDashboard.putNumber("hood angle", angle);
        return angle;
    }

    public double getHoodAngle() {
        double distance = client.getDistance();
        double time = distance / ShooterConstants.constantHorizontalVelocity;
        double input = ((2 * time * ShooterConstants.height) + (9.8 * (Math.pow(time, 3)))) / (2 * distance * time);
        return Math.toDegrees(Math.atan(input));
    }

    public double getFlywheelVelocity(double angle) {
        return ShooterConstants.constantHorizontalVelocity / Math.cos(angle);
    }

    public void periodic() {
        // double desiredAngle = client.getDesiredAngle(false);
        // SmartDashboard.putNumber("apriltag-angle", desiredAngle);
        // double angle = -1 * (getRotation() + desiredAngle);

        // print out desired angle
        Matrix<N2, N1> turretPosition = swerve.getPointFieldOriented(ShooterConstants.turretX, ShooterConstants.turretY);

        // get angle of 
        double x = hubPose.getX() - turretPosition.getData()[0];
        double y = hubPose.getY() - turretPosition.getData()[1];

        distFromHub = Math.hypot(x, y);
        System.out.println(distFromHub);

        double angle = Math.toDegrees(Math.atan2(y, x));
        double desiredAngle = angle - swerve.getPose().getRotation().getDegrees();

        desiredAngle = desiredAngle % 360;

        if (desiredAngle < 0) {
            desiredAngle = 360 + desiredAngle;
        }

    
        // if (angle < minRotationTurret) {
        //     angle = minRotationTurret;
        // } else if (angle > maxRotationTurret) {
        //     angle = maxRotationTurret;
        // }

        getRotation();

        // System.out.println(desiredAngle + " " + angle);
        // pid.changeTurretAngle(desiredAngle);

        // if (pid.enterManual && getRotation() >= maxRotationTurret && turretVelocity > 0) {
        //     rotateTurret(0);
        // } else if (pid.enterManual && getRotation() <= minRotationTurret && turretVelocity < 0) {
        //     rotateTurret(0);
        // }

        // System.out.println(getHoodRotation());



        pid.changeTurretAngle(desiredAngle);
    }
}
