package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase{
   final SparkMax climbMotor = new SparkMax(50, MotorType.kBrushless);
   final SparkMax armMotor = new SparkMax(51, MotorType.kBrushed);
   boolean deployed = false;


   public ClimbSubsystem() {

   }


   public void setArmVelocity(double velocity) {
       armMotor.set(velocity);
   }


   public void setClimbVelocity(double velocity) {
       climbMotor.set(velocity);
   }
}