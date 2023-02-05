package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.MotorVeriablesContants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElongatedArmSubsystem;
import frc.robot.subsystems.FoldingSubsystem;
import frc.robot.subsystems.GripperSubsystem;

public class ElongatedArmCmd extends CommandBase{
    private final ElongatedArmSubsystem a_elongatedArmSubsystem;
    private final double a_speed;
    
    public ElongatedArmCmd(ElongatedArmSubsystem elongatedArmSubsystem, double speed) {
        this.a_elongatedArmSubsystem = elongatedArmSubsystem;
        this.a_speed = speed;
        addRequirements(elongatedArmSubsystem);
    }


private void addRequirements(ElongatedArmSubsystem elongatedArmSubsystem) {
    }
   

@Override
public void initialize(){

}


@Override
public void execute() {
   a_elongatedArmSubsystem.runElongatedArm(a_speed);
}
@Override
public void end(boolean interrupted) {
    a_elongatedArmSubsystem.runElongatedArm(MotorVeriablesContants.motor_off);

}

@Override
  public boolean isFinished() {
    return false;
  }
}
