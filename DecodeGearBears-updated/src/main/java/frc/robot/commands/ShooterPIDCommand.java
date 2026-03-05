package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterPIDCommand extends Command {
    private final PIDController turretPID = new PIDController(ShooterConstants.turretKp, ShooterConstants.turretKi, ShooterConstants.turretKd);
    private final PIDController hoodPID = new PIDController(ShooterConstants.hoodKp, ShooterConstants.hoodKi, ShooterConstants.hoodKd);
    private final PIDController flyWheelPID = new PIDController(ShooterConstants.flyWheelKp, ShooterConstants.flyWheelKi, ShooterConstants.flyWheelKd);

    private double turretSetpoint = 0;
    private double hoodSetpoint = 0;
    private double flyWheelSetvelocity = 0;
    private boolean enterManual = false;

    private ShooterSubsystem shooter;

    public ShooterPIDCommand(ShooterSubsystem shoot) {
        shooter = shoot;
        turretPID.setSetpoint(turretSetpoint);
        turretPID.setIZone(17);
        hoodPID.setSetpoint(hoodSetpoint);
        flyWheelPID.setSetpoint(flyWheelSetvelocity);

        addRequirements(shooter);
    }

    public void manualToggle() {
        enterManual = !enterManual;
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
        flyWheelPID.setSetpoint(flyWheelSetvelocity);
    }

    public void initialize() {

    }

    public void execute() {
        if (!enterManual) {
            double turretPosition = shooter.getRotation();
            double hoodRotation = shooter.getHoodRotation();

            double turretVelocity = turretPID.calculate(turretPosition);
            double hoodVelocity = hoodPID.calculate(hoodRotation);

            shooter.rotateTurret(turretVelocity);
            // shooter.rotateHood(hoodVelocity);
        }

        double shooterVelocity = shooter.getVelocity();
        double deltaVelocity = flyWheelPID.calculate(shooterVelocity);
        
        shooter.shoot(deltaVelocity / PhysicalConstants.neoMaxRPM);
    }

    public void end(boolean interrupted) {

    }

    public boolean isFinished() {
        return false;
    }
}
