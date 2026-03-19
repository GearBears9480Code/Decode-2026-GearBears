package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.IntakePIDCommand;


public class IntakeSubsystem extends SubsystemBase {

    SparkMax armMotor = new SparkMax (61, MotorType.kBrushed); // change to brushless in both code and REV
    public SparkMax vacMotor = new SparkMax(60, MotorType.kBrushless);

	RelativeEncoder armEncoder = armMotor.getEncoder();
    RelativeEncoder vacEncoder = vacMotor.getEncoder();
    boolean vacOn = false;

    public IntakePIDCommand pid = new IntakePIDCommand(this);
    
    public IntakeSubsystem(){
        armEncoder.setPosition(0);
        SmartDashboard.putBoolean("vac on", vacOn);
    }

    public double getArmPosition() {
        double pos =  (armEncoder.getPosition() / 11.4625) * 360 * IntakeConstants.armGearRatio;
        SmartDashboard.putNumber("intake position", pos);
        return pos;
    }

    public void toggleVacume() {
        if (vacOn) {
            vacMotor.set(0);
            vacOn = false;
        } else {
            vacMotor.set(-1);
            vacOn = true;
        }
        SmartDashboard.putBoolean("vac on", vacOn);
    }

    public void setArmRotation(double velocity) {
        armMotor.set(velocity);
    }
  
    public void setVelocity(double velocity) {
        vacMotor.set(velocity);
    }

    public double getVelocity() {
        return vacEncoder.getVelocity();
    }


    public void setArmVelocity(double armVelocity) {
        armMotor.set(armVelocity);
    }

    public void periodic() {
        getArmPosition();
    }
}