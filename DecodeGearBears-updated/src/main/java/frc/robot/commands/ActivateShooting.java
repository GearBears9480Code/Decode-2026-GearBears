package frc.robot.commands;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperSubsystem;

public class ActivateShooting extends Command {
    
    SparkMax shooter = new SparkMax(42, MotorType.kBrushless);
    boolean finished = false;
    double startingTime;

    HopperSubsystem hopper;

    public ActivateShooting(HopperSubsystem hop) {
        hopper = hop;
    }

    public void stop() {
        finished = true;
    }

    public void initialize() {
        startingTime = Timer.getTimestamp();
        shooter.set(0.5);
    }

    public void execute() {
        double currentTime = Timer.getTimestamp();
        if (currentTime - startingTime >= 2.0) {
            hopper.hopperPIDCommand.changeSpindexerSpeed(600);
        }
    }

    public void end(boolean interupt) {
        finished = false;
        shooter.set(0);
        hopper.hopperPIDCommand.changeSpindexerSpeed(0);
    }

    public boolean isFinished() {
        return finished;
    }
}