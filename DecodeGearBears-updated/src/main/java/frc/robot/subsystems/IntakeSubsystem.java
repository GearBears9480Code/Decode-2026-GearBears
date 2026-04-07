package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.IntakePIDCommand;


public class IntakeSubsystem extends SubsystemBase {
    private TalonFX vacMotor = new TalonFX(60);
    private TalonFX slideMotor = new TalonFX(61);
    private SparkMax oldSlideMotor = new SparkMax(61, MotorType.kBrushed);
    private RelativeEncoder armEncoder = oldSlideMotor.getEncoder();
    boolean vacOn = false;

    public IntakePIDCommand pid = new IntakePIDCommand(this);
    
    public IntakeSubsystem(){
        SmartDashboard.putBoolean("vac on", vacOn);
    }

    public double getArmPosition() {
        double pos =  (armEncoder.getPosition() / 11.4625) * 360 * IntakeConstants.armGearRatio;
        // double pos = slideMotor.getPosition().getValueAsDouble();
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
  
    public void setVelocity(double velocity) {
        vacMotor.set(velocity);
    }

    public void setArmVelocity(double armVelocity) {
        oldSlideMotor.set(armVelocity);
    }

    public void periodic() {
        getArmPosition();
    }
}