package frc.robot.commands;


import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.JoysticksConstants;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArcadeDriveCmd extends CommandBase{

    private final DriveSubsystem drive = new DriveSubsystem();

    private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

    @Override
    public void initialize(){
    System.out.println("Arcade Drive Started!");
    }
    
    
    @Override
    public void execute() {
    
    

    final var xSpeed = -m_speedLimiter.calculate(JoysticksConstants.operatorcontroller.getY()) * DriveSubsystem.kMaxSpeed;

    final var rot = -m_rotLimiter.calculate(JoysticksConstants.operatorcontroller.getX()) * DriveSubsystem.kMaxAngularSpeed;

    drive.drive(xSpeed, rot);
    }
    
    @Override
    public void end(boolean interrupted) {
       
    
    }
    @Override
      public boolean isFinished() {
        return false;
      }


}
