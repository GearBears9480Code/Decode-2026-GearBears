package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.IntakePIDCommand;


public class IntakeSubsystem extends SubsystemBase {

    SparkMax armMotor = new SparkMax (61, MotorType.kBrushed);
    SparkMax vacMotor = new SparkMax(60, MotorType.kBrushless);

    double vacVel = 0;

	RelativeEncoder armEncoder = armMotor.getEncoder();
    RelativeEncoder vacEncoder = vacMotor.getEncoder();

    public IntakePIDCommand pid = new IntakePIDCommand(this);

    public double getArmPosition() {
        return armEncoder.getPosition();
    }

    public void setArmRotation(double velocity) {
        armMotor.set(velocity);
    }
  
    public void setVelocity(double velocity) {
        vacVel += velocity;
        vacMotor.set(vacVel);
    }

    public double getVelocity() {
        return vacEncoder.getVelocity();
    }

   public IntakeSubsystem(){

   }

    public void setArmVelocity(double armVelocity) {
        armMotor.set(armVelocity);
    }
}