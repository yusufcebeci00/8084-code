package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.MotorVeriablesContants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCmd extends CommandBase{
    private final ArmSubsystem armSubsystem;
    private final double a_speed;
    
    public ArmCmd(ArmSubsystem armSubsystem, double speed) {
        this.armSubsystem = armSubsystem;
        this.a_speed = speed;
        addRequirements(armSubsystem);
    }


private void addRequirements(ArmSubsystem elevatorSubsystem) {
    }


@Override
public void initialize(){

}


@Override
public void execute() {
   armSubsystem.runArm(a_speed);
}
@Override
public void end(boolean interrupted) {
    armSubsystem.runArm(MotorVeriablesContants.motor_off);

}

@Override
  public boolean isFinished() {
    return false;
  }
}
