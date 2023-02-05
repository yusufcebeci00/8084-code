package frc.robot.subsystems;


import frc.robot.Constants.MotorsContants;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

public class FoldingSubsystem {
    public static VictorSP folding_motor = new VictorSP(MotorsContants.folding_motor);




    public void runFolding(double a_speed) {

    folding_motor.set(a_speed);

    }
}
