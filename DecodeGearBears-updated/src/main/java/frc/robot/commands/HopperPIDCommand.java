package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HopperConstants;
import frc.robot.subsystems.HopperSubsystem;

public class HopperPIDCommand extends Command {
    private final PIDController spindexerPID = new PIDController(
        HopperConstants.spinKp, 
        HopperConstants.spinKi, 
        HopperConstants.spinKd
    );

    private double spindexerSpeed = 0;

    private HopperSubsystem hopper;

    public HopperPIDCommand(HopperSubsystem hop) {
        hopper = hop;
        spindexerPID.setSetpoint(spindexerSpeed);
    }

    public void changeSpindexerSpeed(double velocity) {
        spindexerSpeed = velocity;
        spindexerPID.setSetpoint(velocity);
    }

    public void initialize() {

    }

    public void execute() {
        double spinMes = hopper.getSpinVelocity();
        double spinOut = spindexerPID.calculate(spinMes);
        hopper.setSpinVelocity(spinOut);
    }

    public void end(boolean interupt) {

    }

    public boolean isFinished() {
        return false;
    }
}
