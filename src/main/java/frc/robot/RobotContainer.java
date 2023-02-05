
package frc.robot;

import frc.robot.Constants.JoysticksConstants;
import frc.robot.Constants.MotorVeriablesContants;
import frc.robot.commands.ArmCmd;
import frc.robot.commands.BalanceCmd;
import frc.robot.commands.GripperCmd;
import frc.robot.subsystems.BalanceSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.*;




public class RobotContainer {
  private SendableChooser<Command> chooser = new SendableChooser<>();
  private Command test1 ;
  private Command test2 ;
  

private ArmSubsystem armSubsystem = new ArmSubsystem();
private BalanceSubsystem balanceSubsystem = new BalanceSubsystem();
private GripperSubsystem gripperSubsystem = new GripperSubsystem();





  public static final Joystick drivercontroller = new Joystick(JoysticksConstants.DriverController);
  public static final Joystick operatorcontroller = new Joystick(JoysticksConstants.OperatorController);
  private final AHRS gyro = new AHRS(SPI.Port.kMXP); 


  public RobotContainer() {
    chooser.addOption("test1", test1);
    chooser.addOption("test2", test2);

    gyro.calibrate();
    
  }

  
  
  
  
  
  






  private void configureBindings() {

    
    new JoystickButton(operatorcontroller, 10).whileTrue(new BalanceCmd(balanceSubsystem));
    new JoystickButton(operatorcontroller, 2).whileTrue(new ArmCmd(armSubsystem, MotorVeriablesContants.cw_arm_motor_speed));
    new JoystickButton(operatorcontroller, 4).whileTrue(new ArmCmd(armSubsystem, MotorVeriablesContants.ccw_arm_motor_speed));
    new JoystickButton(operatorcontroller, 5).whileTrue(new GripperCmd(gripperSubsystem, MotorVeriablesContants.cw_gripper_motor_speed));
    new JoystickButton(operatorcontroller, 6).whileFalse(new GripperCmd(gripperSubsystem, MotorVeriablesContants.ccw_gripper_motor_speed));
  }
  
  
  
  
 
}
