package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterPIDCommand extends Command {
    private final PIDController turretPID = new PIDController(ShooterConstants.turretKp, ShooterConstants.turretKi, ShooterConstants.turretKd);
    private final PIDController hoodPID = new PIDController(ShooterConstants.hoodKp, ShooterConstants.hoodKi, ShooterConstants.hoodKd);
    private final PIDController flyWheelPID = new PIDController(ShooterConstants.flyWheelKp, ShooterConstants.flyWheelKi, ShooterConstants.flyWheelKd);

    private double turretSetpoint = 0;
    private double hoodSetpoint = 0;
    private double flyWheelSetvelocity = 0;

    private ShooterSubsystem shooter;

    public ShooterPIDCommand(ShooterSubsystem shoot) {
        shooter = shoot;
        turretPID.setSetpoint(turretSetpoint);
        hoodPID.setSetpoint(hoodSetpoint);
        flyWheelPID.setSetpoint(flyWheelSetvelocity);

        addRequirements(shooter);
    }

    public void changeTurretAngle(double angle) {
        turretSetpoint = angle;
        turretPID.setSetpoint(angle);
        SmartDashboard.putNumber("turret setpoint", angle);
    }

    public void initialize() {

    }

    public void execute() {
        double turretPosition = shooter.getRotation();
        // double hoodRotation = shooter.getHoodRotation();

        double turretVelocity = turretPID.calculate(turretPosition);
        System.out.println(turretVelocity);
        // double hoodVelocity = hoodPID.calculate(hoodRotation);

        shooter.rotateTurret(turretVelocity);
        // shooter.rotateHood(hoodVelocity);

        // double shooterVelocity = shooter.getVelocity();
        // double deltaVelocity = flyWheelPID.calculate(shooterVelocity);
        
        // shooter.shoot(deltaVelocity);
    }

    public void end(boolean interrupted) {

    }

    public boolean isFinished() {
        return false;
    }
}
