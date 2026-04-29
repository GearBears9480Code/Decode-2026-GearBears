package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ActivateShooting extends Command {
    boolean finished = false;
    double startingTime;

    HopperSubsystem hopper;
    ShooterSubsystem shooter;

    boolean on = false;

    public ActivateShooting(HopperSubsystem hop, ShooterSubsystem shoot) {
        hopper = hop;
        shooter = shoot;
        SmartDashboard.putBoolean("flywheel on", on);
    }

    public void toggleFlywheel() {
        if (on) {
            shooter.velocity = 0;
            shooter.shoot(0);
            shooter.pid.changeHoodAngle(0);
            hopper.hopperPIDCommand.changeKickerSpeed(0);
            on = false;
        } else {
            shooter.shoot(shooter.getCalcVelocity());
            on = true;
        }

        SmartDashboard.putBoolean("flywheel on", on);
    }

    public void stop() {
        finished = true;
    }

    public void initialize() {
        startingTime = Timer.getTimestamp();
        hopper.hopperPIDCommand.changeSpindexerSpeed(1000);
        finished = false;
    }

    public void execute() {
        if (on) {
            shooter.shoot(shooter.getCalcVelocity());
        }
    }

    public void end(boolean interupt) {
        hopper.hopperPIDCommand.changeSpindexerSpeed(0);
    }

    public boolean isFinished() {
        return finished;
    }
}