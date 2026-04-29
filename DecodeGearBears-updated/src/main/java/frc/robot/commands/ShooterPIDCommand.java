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
    public boolean enterManual = true;

    private double speedTolerance = 0.025;

    private ShooterSubsystem shooter;

    public ShooterPIDCommand(ShooterSubsystem shoot) {
        shooter = shoot;
        turretPID.setSetpoint(turretSetpoint);
        turretPID.setIZone(17);
        hoodPID.setSetpoint(hoodSetpoint);
        flyWheelPID.setSetpoint(flyWheelSetvelocity);

        addRequirements(shooter);
        
        SmartDashboard.putBoolean("manual", true);
    }

    public void manualToggle() {
        enterManual = !enterManual;
        shooter.rotateTurret(0);
        shooter.rotateHood(0);
        SmartDashboard.putBoolean("manual", enterManual);
    }

    public void changeTurretAngle(double angle) {
        turretSetpoint = angle;
        turretPID.setSetpoint(angle);
        SmartDashboard.putNumber("turret setpoint", angle);
    }

    public void changeHoodAngle(double angle) {
        hoodSetpoint = angle;
        hoodPID.setSetpoint(angle);
        SmartDashboard.putNumber("hood setpoint", angle);
    }

    public void changeSpeed(double velocity) {
        flyWheelSetvelocity = velocity;
        System.out.println(velocity);
        flyWheelPID.setSetpoint(flyWheelSetvelocity);
    }

    public void initialize() {

    }

    public void execute() {
        if (!enterManual) {
            double turretPosition = shooter.getRotation();

            double turretVelocity = turretPID.calculate(turretPosition);

            if (Math.abs(turretVelocity) > speedTolerance && Math.abs(turretVelocity) < ShooterConstants.turretKs) {
                turretVelocity = turretVelocity > 0 ? ShooterConstants.turretKs : -ShooterConstants.turretKs;
            }

            shooter.rotateTurret(turretVelocity);
        }

        double hoodPosition = shooter.getHoodRotation();
        double hoodVelocity = hoodPID.calculate(hoodPosition);
            
        shooter.rotateHood(hoodVelocity);
    }

    public void end(boolean interrupted) {

    }

    public boolean isFinished() {
        return false;
    }
}
