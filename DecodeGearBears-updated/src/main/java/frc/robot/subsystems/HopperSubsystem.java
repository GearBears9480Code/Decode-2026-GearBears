package frc.robot.subsystems;


import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;


import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class HopperSubsystem extends SubsystemBase {
   final SparkMax spindexer = new SparkMax(48, MotorType.kBrushless);
   final SparkMax kicker = new SparkMax(49, MotorType.kBrushless);


   public HopperSubsystem() {

    spin();
   }


   public void spin() {
       spindexer.set(0.3);
   }


   public void stopSpin() {
       spindexer.set(0);
   }


   public void kick() {
       kicker.set(1);
   }


   public void stopKick() {
       kicker.set(0);
   }
}
