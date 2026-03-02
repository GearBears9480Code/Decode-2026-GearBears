package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimberPIDCommand extends Command {
    private final PIDController armPID = new PIDController(ClimberConstants.armKp, ClimberConstants.armKi, ClimberConstants.armKd);
    private final PIDController climbPID = new PIDController(ClimberConstants.climbKp, ClimberConstants.climbKi, ClimberConstants.climbKd);
    private ClimbSubsystem climber;
    private double setPoint = 0;
    private double setPosition = 0;


    public ClimberPIDCommand(ClimbSubsystem climb) {
        climber = climb;
        armPID.setSetpoint(setPoint);
        climbPID.setSetpoint(setPosition);

        addRequirements(climb);
    }

    public void changeClimbSetpoint(double position) {
        climbPID.setSetpoint(position);
        setPosition = position;
    }

    public void changeArmSetpoint(double angle) {
        armPID.setSetpoint(angle);
        setPoint = angle;
    }

    public void initialize() {

    }

    public void execute() {
        double armPosition = climber.getArmPosition();
        double armVelocity = armPID.calculate(armPosition);
        climber.setArmVelocity(armVelocity);

        double climbPosition = climber.getClimbPosition();
        double climbVelocity = armPID.calculate(climbPosition);
        climber.setClimbVelocity(climbVelocity);
    }

    public void end(boolean interrupted) {

    }

    public boolean isFinished() {
        return false;
    }
}
