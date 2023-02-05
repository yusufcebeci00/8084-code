package frc.robot.subsystems;

import frc.robot.Constants.MotorsContants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

public class GripperSubsystem {
  public static TalonFX gripper_motor = new TalonFX(   5);

  
  


    public GripperSubsystem() {
   
    }


    public void runGripper(double gripper_speed){
        gripper_motor.set(ControlMode.PercentOutput, gripper_speed);
      }


    
    }




