package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeSubsystem extends SubsystemBase {


    SparkMax armMotor = new SparkMax (61, MotorType.kBrushed);
    SparkMax vacMotor = new SparkMax(60, MotorType.kBrushless);

	RelativeEncoder armEncoder = armMotor.getEncoder();

    public double getArmPosition() {
        return armEncoder.getPosition();
    }

    public void setArmRotation(double velocity) {
        armMotor.set(velocity);
    }
  
    public void startVacMotor() {
        vacMotor.set(-1);
    }

    public void stopVacMotor() {
        vacMotor.set(0);
    }

   public IntakeSubsystem(){
       
   }

    public void setArmVelocity(double armVelocity) {
        armMotor.set(armVelocity);
    }
}