package frc.robot.subsystems;


import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.MotorDriver;
import frc.robot.Constants.MotorsContants;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ElongatedArmSubsystem {
    public static VictorSP elongatedarm_motor = new VictorSP(MotorsContants.gripper_motor);




    public void runElongatedArm(double a_speed) {

    elongatedarm_motor.set(a_speed);

    }
}
