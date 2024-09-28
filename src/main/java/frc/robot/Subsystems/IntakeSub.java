package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSub extends SubsystemBase {
    public 
    public IntakeSub(){

    }
    public void intake(){
        intakeOne.set(0.2);
        intakeTwo.set(0.2);
    }
}
