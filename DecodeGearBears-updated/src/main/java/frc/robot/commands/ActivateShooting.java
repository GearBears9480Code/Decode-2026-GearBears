package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ActivateShooting extends Command {
    boolean finished = false;
    double startingTime;

    HopperSubsystem hopper;
    ShooterSubsystem shooter;

    public ActivateShooting(HopperSubsystem hop, ShooterSubsystem shoot) {
        hopper = hop;
        shooter = shoot;
    }

    public void stop() {
        finished = true;
        shooter.stop();
    }

    public void initialize() {
        startingTime = Timer.getTimestamp();
        shooter.pid.changeSpeed(600);
        finished = false;
    }

    public void execute() {
        double currentTime = Timer.getTimestamp();
        if (currentTime - startingTime >= 1) {
            hopper.hopperPIDCommand.changeSpindexerSpeed(1200);
        } else if (currentTime - startingTime >= 0.5) {
            hopper.hopperPIDCommand.changeKickerSpeed(3600);
        }
    }

    public void end(boolean interupt) {
        hopper.hopperPIDCommand.changeSpindexerSpeed(0);
        hopper.hopperPIDCommand.changeKickerSpeed(0);
        shooter.pid.changeSpeed(0);
    }

    public boolean isFinished() {
        return finished;
    }
}