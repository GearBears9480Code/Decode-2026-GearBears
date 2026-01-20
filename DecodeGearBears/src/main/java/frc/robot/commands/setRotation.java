package frc.robot.commands;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class setRotation extends Command{
    private DoubleSupplier getRotation;
    private DoubleConsumer setRotation;

    private double setPoint;
    private PIDController pid;

    public setRotation(DoubleSupplier getRotation, DoubleConsumer setRotation, double setPoint) {
        this.getRotation = getRotation; // gets the rotation in degrees
        this.setRotation = setRotation; // sets the rotation in degrees

        this.setPoint = setPoint; // setpoint in degrees
        pid = new PIDController(0.0, 0.0, 0.0); // sets up the pid constants
    }

    public void initialize() {
        pid.setSetpoint(setPoint);
        pid.setTolerance(5); // degrees
    }

    public void execute() {
        double currentPosition = getRotation.getAsDouble();
        
        double speed = pid.calculate(currentPosition);

        setRotation.accept(speed);
    }

    public void end(boolean interrupted) {

    }

    public boolean isFinished() {
        return pid.atSetpoint();
    }
}
