package frc.robot.subsystems;


import frc.robot.Constants.MotorsContants;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

public class ArmSubsystem {
  public static VictorSP arm_motor = new VictorSP(MotorsContants.arm_motor);

  
    


    public void runArm(double arm_speed){
        arm_motor.set(arm_speed);
      }


    
    }




