package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakePIDCommand extends Command {
    private final PIDController armPID = new PIDController(
            IntakeConstants.armMotor_kP, 
            IntakeConstants.armMotor_kI, 
            IntakeConstants.armMotor_kD
        );
    private IntakeSubsystem intake;
    private double setPoint = 0;

    public IntakePIDCommand(IntakeSubsystem intake) {
        this.intake = intake;
        armPID.setSetpoint(setPoint);
        addRequirements(intake);
    }

    public void changeArmSetpoint(double angle) {
        armPID.setSetpoint(angle);
        setPoint = angle;
    }

    public void initialize() {

    }

    public void execute() {
        double armPosition = intake.getArmPosition();
        double armVelocity = armPID.calculate(armPosition);
        intake.setArmVelocity(armVelocity);
    }

    public void end(boolean interrupted) {

    }

    public boolean isFinished() {
        return false;
    }
}

