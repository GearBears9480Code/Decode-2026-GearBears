package frc.robot.subsystems;


import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.HopperPIDCommand;


public class HopperSubsystem extends SubsystemBase {
   final SparkMax spindexer = new SparkMax(48, MotorType.kBrushless);
   final SparkMax kicker = new SparkMax(49, MotorType.kBrushless);

   final RelativeEncoder spinEncoder = spindexer.getEncoder();
   final RelativeEncoder kickEncoder = kicker.getEncoder();

   double spinVel = 0;
   double kickVel = 0;

   public final HopperPIDCommand hopperPIDCommand = new HopperPIDCommand(this);

   public HopperSubsystem() {
   }


   public void setSpinVelocity(double velocity) {
    spinVel += velocity;
    spindexer.set(spinVel);
   }

   public double getSpinVelocity() {
    return spinEncoder.getVelocity();
   }

   public void setKickVelocity(double velocity) {
    kickVel += velocity;
    kicker.set(kickVel);
   }

   public double getKickVelocity() {
    return kickEncoder.getVelocity();
   }
}
