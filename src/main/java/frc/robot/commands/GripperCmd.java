package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.MotorVeriablesContants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;

public class GripperCmd extends CommandBase{
    private final GripperSubsystem a_gripperSubsystem;
    private final double a_speed;
    
    public GripperCmd(GripperSubsystem gripperSubsystem, double speed) {
        this.a_gripperSubsystem = gripperSubsystem;
        this.a_speed = speed;
        addRequirements(gripperSubsystem);
    }
private Timer timer = new Timer();


private void addRequirements(GripperSubsystem gripperSubsystem) {
    }
   

@Override
public void initialize(){

}


@Override
public void execute() {
   timer.reset();
   timer.start();
  
  if(timer.get() > 0.5) {
  a_gripperSubsystem.runGripper(a_speed);
  }
  else {
    a_gripperSubsystem.runGripper(0);
  }





}
@Override
public void end(boolean interrupted) {
    a_gripperSubsystem.runGripper(MotorVeriablesContants.motor_off);

}

@Override
  public boolean isFinished() {
    return false;
  }
}
