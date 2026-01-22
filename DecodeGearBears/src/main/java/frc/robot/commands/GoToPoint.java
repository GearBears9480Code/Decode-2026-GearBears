package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class GoToPoint extends Command {
    private PIDController xPIDControl;
    private PIDController yPidControl;
    private double deltaX;
    private double deltaY;

    private SwerveSubsystem swerve;

    private Pose2d startPoint;
    private Pose2d setPoint;

    private double minimumAxisSpeed = 0.5;
    private double maxSpeed = 3/5; // numerator denotes m/s denominator denotes how many times it will update per second

    public GoToPoint(SwerveSubsystem swerveDrive, double deltaX, double deltaY) {
        xPIDControl = new PIDController(0.9, 0.0, 0.0);
        yPidControl = new PIDController(0.9, 0.0, 0.0);
        this.deltaX = deltaX;
        this.deltaY = deltaY;
        swerve = swerveDrive;
    }

    public void initialize() {
        startPoint = swerve.getPose();

        setPoint = new Pose2d(startPoint.getX() + this.deltaX, startPoint.getY() +  this.deltaY, new Rotation2d(0));
        xPIDControl.setSetpoint(setPoint.getX());
        yPidControl.setSetpoint(setPoint.getY());

        xPIDControl.setTolerance(0.5);
        yPidControl.setTolerance(0.5);

    }

    public void execute() {
        Pose2d currentPos = swerve.getPose();

        double xSpeed = xPIDControl.calculate(currentPos.getX());
        double ySpeed = yPidControl.calculate(currentPos.getY());

        double hypot = Math.hypot(xSpeed, ySpeed);
        if (hypot > maxSpeed) {
            xSpeed = xSpeed / hypot * maxSpeed;
            ySpeed = ySpeed / hypot * maxSpeed;
        }

        xSpeed = xSpeed >= minimumAxisSpeed ? xSpeed : 0;
        ySpeed = ySpeed >= minimumAxisSpeed ? ySpeed : 0;

        swerve.driveRobotRelative(new ChassisSpeeds(xSpeed, ySpeed, 0));
    }

    public void end(boolean interrupt) {
        swerve.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
    }

    public boolean isFinished() {
        return xPIDControl.atSetpoint() && yPidControl.atSetpoint();
    }
}
