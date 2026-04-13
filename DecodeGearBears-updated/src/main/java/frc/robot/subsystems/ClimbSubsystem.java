package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import swervelib.SwerveInputStream;


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

   public SwerveInputStream deploy(SwerveInputStream input) {
        setArmVelocity(-0.5);
        SwerveInputStream output;
        if (!deployed) {
            deployed = true;
            output = input.copy().scaleTranslation(0.1);
        } else {
            output = input;
        }
        return output;
    }
}