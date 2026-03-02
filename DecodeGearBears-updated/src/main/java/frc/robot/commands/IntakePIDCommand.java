package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakePIDCommand extends Command {
    private final PIDController armPID = new PIDController(
            IntakeConstants.armMotor_kP, 
            IntakeConstants.armMotor_kI, 
            IntakeConstants.armMotor_kD
        );
    private final PIDController velocityPID = new PIDController(
            IntakeConstants.velocityMotor_kP, 
            IntakeConstants.velocityMotor_kI, 
            IntakeConstants.velocityMotor_kD
        );
    private IntakeSubsystem intake;
    private double setPoint = 0;
    private double velocitySetPoint = 0;

    public IntakePIDCommand(IntakeSubsystem intake) {
        this.intake = intake;
        armPID.setSetpoint(setPoint);
        velocityPID.setSetpoint(velocitySetPoint);
        addRequirements(intake);
    }

    public void changeArmSetpoint(double angle) {
        armPID.setSetpoint(angle);
        setPoint = angle;
    }

    public void changeVelocity(double velocity) {
        velocityPID.setSetpoint(velocity);
        velocitySetPoint = velocity;
    }

    public void initialize() {

    }

    public void execute() { //test
        double armPosition = intake.getArmPosition();
        double armVelocity = armPID.calculate(armPosition);

        double intakeVelocity = intake.getVelocity();
        double newVelocity = velocityPID.calculate(intakeVelocity);

        intake.setArmVelocity(armVelocity);
        intake.setVelocity(newVelocity/PhysicalConstants.neoMaxRPM);
    }

    public void end(boolean interrupted) {

    }

    public boolean isFinished() {
        return false;
    }
}

