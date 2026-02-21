package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeSubsystem extends SubsystemBase {


    SparkMax armMotor = new SparkMax (1, MotorType.kBrushless);
    SparkMax vacMotor = new SparkMax(2, MotorType.kBrushless);

	RelativeEncoder armEncoder = armMotor.getEncoder();

    public double getArmPosition() {
        return armEncoder.getPosition();
    }

    public void setArmRotation(double velocity) {
        armMotor.set(velocity);
    }
  
    public void startVacMotor() {
        vacMotor.set(1);
    }

    public void stopVacMotor() {
        vacMotor.set(0);
    }

   public IntakeSubsystem(){
       
   }

    public void setArmVelocity(double armVelocity) {
        
    }
}