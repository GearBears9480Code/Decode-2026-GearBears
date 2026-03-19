package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
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

    private double downPosition = 1015;
    private double upPosition = 0;

    private boolean up = false;
    private double deadbandArmDown = 15;
    private double deadbandArmUp = 5;

    public IntakePIDCommand(IntakeSubsystem intake) {
        this.intake = intake;
        armPID.setSetpoint(setPoint);
        velocityPID.setSetpoint(velocitySetPoint);
        armPID.setTolerance(10);
        addRequirements(intake);
    }

    public void togglePID() {
        if (!up) {
            changeArmSetpoint(downPosition);
            up = true;
        } else {
            changeArmSetpoint(upPosition);
            up = false;
        }
    }

    public void changeArmSetpoint(double angle) {
        armPID.setSetpoint(angle);
        setPoint = angle;
    }

    public void changeVelocity(double velocity) {
        if (velocity > 1200) {
            velocity = 1200;
        } else if (velocity < -1200) {
            velocity = -1200;
        }
        velocityPID.setSetpoint(velocity);
        velocitySetPoint = velocity;
    }

    public void initialize() {

    }

    public void execute() { //test
        double armPosition = intake.getArmPosition();
        SmartDashboard.putNumber("arm pos", armPosition);
        double armVelocity = armPID.calculate(armPosition);

        if (up && Math.abs(armPID.getError()) < deadbandArmDown) {
            armVelocity = 0;
        }
        if (!up && Math.abs(armPID.getError()) < deadbandArmUp) {
            armVelocity = 0;
        }


        intake.setArmVelocity(armVelocity);
        // intake.setVelocity(newVelocity/PhysicalConstants.neoMaxRPM);
    }

    public void end(boolean interrupted) {

    }

    public boolean isFinished() {
        return false;
    }
}

