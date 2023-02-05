package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.BalanceContants;
import frc.robot.Constants.MotorVeriablesContants;
import frc.robot.subsystems.BalanceSubsystem;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
public class BalanceCmd extends CommandBase{
    private final BalanceSubsystem a_balance;
   
    private final AHRS gyro = new AHRS(SPI.Port.kMXP); 

    public BalanceCmd(BalanceSubsystem balanceSubsystem) {
        this.a_balance = balanceSubsystem;
       

        addRequirements(balanceSubsystem);
    }

    private final Timer timer = new Timer();



private void addRequirements(BalanceSubsystem balanceSubsystem) {
    }


@Override
public void initialize(){
System.out.println("Balance Command Started!");
}


@Override
public void execute() {
    timer.reset();
    timer.start();
    
double error = 0 + gyro.getPitch();
 


a_balance.runBalance(BalanceContants.kP * gyro.getPitch(),  -BalanceContants.kP * gyro.getPitch());





}

@Override
public void end(boolean interrupted) {
    a_balance.runBalance(MotorVeriablesContants.motor_off,MotorVeriablesContants.motor_off);

}
@Override
  public boolean isFinished() {
    return false;
  }


}
