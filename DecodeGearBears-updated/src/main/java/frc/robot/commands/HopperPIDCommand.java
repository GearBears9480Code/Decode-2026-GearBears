package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.subsystems.HopperSubsystem;

public class HopperPIDCommand extends Command {
    private final PIDController spindexerPID = new PIDController(
        HopperConstants.spinKp, 
        HopperConstants.spinKi, 
        HopperConstants.spinKd
    );
    private final PIDController kickerPID = new PIDController(
        HopperConstants.kickKp, 
        HopperConstants.kickKi, 
        HopperConstants.kickKd
    );

    private double spindexerSpeed = 0;
    private double kickerSpeed = 0;

    private HopperSubsystem hopper;

    public HopperPIDCommand(HopperSubsystem hop) {
        hopper = hop;
        spindexerPID.setSetpoint(spindexerSpeed);
        kickerPID.setSetpoint(kickerSpeed);

        addRequirements(hopper);
    }

    public void changeSpindexerSpeed(double velocity) {
        if (velocity > 1200) {
            velocity = 1200;
        } else if (velocity < -1200) {
            velocity = -1200;
        }
        spindexerSpeed = velocity;
        spindexerPID.setSetpoint(velocity);
    }

    public void changeKickerSpeed(double velocity) {
        if (velocity > 1200) {
            velocity = 1200;
        } else if (velocity < -1200) {
            velocity = -1200;
        }
        kickerSpeed = velocity;
        kickerPID.setSetpoint(velocity);
    }

    public void initialize() {

    }

    public void execute() {
        double spinMes = hopper.getSpinVelocity();
        double spinOut = spindexerPID.calculate(spinMes);
        hopper.setSpinVelocity(spinOut / PhysicalConstants.neoMaxRPM);

        double kickMes = hopper.getKickVelocity();
        double kickOut = kickerPID.calculate(kickMes);
        hopper.setKickVelocity(kickOut / PhysicalConstants.neoMaxRPM);
    }

    public void end(boolean interupt) {

    }

    public boolean isFinished() {
        return false;
    }
}
