package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.MotorVeriablesContants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.FoldingSubsystem;
import frc.robot.subsystems.GripperSubsystem;

public class FoldingCmd extends CommandBase{
    private final FoldingSubsystem a_foldingSubsystem;
    private final double a_speed;
    
    public FoldingCmd(FoldingSubsystem foldingSubsystem, double speed) {
        this.a_foldingSubsystem = foldingSubsystem;
        this.a_speed = speed;
        addRequirements(foldingSubsystem);
    }


private void addRequirements(FoldingSubsystem foldingSubsystem) {
    }
   

@Override
public void initialize(){

}


@Override
public void execute() {
   a_foldingSubsystem.runFolding(a_speed);
}
@Override
public void end(boolean interrupted) {
    a_foldingSubsystem.runFolding(MotorVeriablesContants.motor_off);

}

@Override
  public boolean isFinished() {
    return false;
  }
}
