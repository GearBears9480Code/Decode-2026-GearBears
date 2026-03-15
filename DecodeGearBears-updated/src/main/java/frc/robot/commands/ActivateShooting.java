package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ActivateShooting extends Command {
    boolean finished = false;
    double startingTime;

    HopperSubsystem hopper;
    ShooterSubsystem shooter;

    boolean on = false;

    public ActivateShooting(HopperSubsystem hop, ShooterSubsystem shoot) {
        hopper = hop;
        shooter = shoot;
    }

    public void toggleFlywheel() {
        if (on) {
            shooter.velocity = 0;
            shooter.pid.changeSpeed(0);
            on = false;
        } else {
            shooter.pid.changeSpeed(335);
            on = true;
        }
    }

    public void stop() {
        finished = true;
    }

    public void initialize() {
        startingTime = Timer.getTimestamp();
        hopper.hopperPIDCommand.changeKickerSpeed(2700);
        finished = false;
    }

    public void execute() {
        double currentTime = Timer.getTimestamp();
        if (currentTime - startingTime >= 1) {
            hopper.hopperPIDCommand.changeSpindexerSpeed(4000);
        }
    }

    public void end(boolean interupt) {
        hopper.hopperPIDCommand.changeSpindexerSpeed(0);
        hopper.hopperPIDCommand.changeKickerSpeed(0);
    }

    public boolean isFinished() {
        return finished;
    }
}