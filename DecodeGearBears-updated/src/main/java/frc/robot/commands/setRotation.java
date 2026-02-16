package frc.robot.commands;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class setRotation extends Command{
    private DoubleSupplier getRotation;
    private DoubleConsumer setRotation;

    private double setPoint;
    private PIDController pid;

    private boolean runConstant;

    public setRotation(DoubleSupplier getRotation, DoubleConsumer setRotation, double setPoint, Subsystem subsyst, boolean runConstantly) {
        this.getRotation = getRotation; // gets the rotation in degrees
        this.setRotation = setRotation; // sets the rotation in degrees

        this.setPoint = setPoint; // setpoint in degrees
        pid = new PIDController(0.015, 0.003, 0.000005); // sets up the pid constants
        runConstant = runConstantly;
        addRequirements(subsyst);
    }

    public void changeSetpoint(double angle) {
        setPoint = angle;
        pid.setSetpoint(angle);
        SmartDashboard.putNumber("setpoint", setPoint);
    }

    public void initialize() {
        pid.setSetpoint(setPoint);
        pid.setIZone(5);
        SmartDashboard.putNumber("setpoint", setPoint);
    }

    public void execute() {
        double currentPosition = getRotation.getAsDouble();
        
        double speed = pid.calculate(currentPosition);
        
        setRotation.accept(speed);
        pid.calculate(getRotation.getAsDouble());
        SmartDashboard.putNumber("error", pid.getError());
    }

    public void end(boolean interrupted) {
        setRotation.accept(0);
    }

    public boolean isFinished() {
        double rotation = getRotation.getAsDouble();
        return (setPoint - 1 <= rotation && rotation <= setPoint + 1) &&! runConstant;
    }
}
