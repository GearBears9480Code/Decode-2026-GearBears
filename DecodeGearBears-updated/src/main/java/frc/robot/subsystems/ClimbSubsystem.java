package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ClimbSubsystem extends SubsystemBase{
   final SparkMax armMotor = new SparkMax(0, MotorType.kBrushless);
   final SparkMax climbMotor = new SparkMax(1, MotorType.kBrushless);


   RelativeEncoder armEncoder = armMotor.getEncoder();
   RelativeEncoder climbEncoder = climbMotor.getEncoder();


   public ClimbSubsystem() {

   }


   public void setArmVelocity(double velocity) {
       armMotor.set(velocity);
   }


   public void setClimbVelocity(double velocity) {
       climbMotor.set(velocity);
   }


   public double getArmPosition() {
       return armEncoder.getPosition();
   }


   public double getClimbPosition() {
       return climbEncoder.getPosition();
   }


}