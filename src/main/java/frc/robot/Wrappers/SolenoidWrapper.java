package frc.robot.wrappers;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;


public class SolenoidWrapper extends Wrapper {
    Solenoid solenoid;

    public SolenoidWrapper(int id) {
        solenoid = new Solenoid(PneumaticsModuleType.REVPH, id);
    }

    public void set(boolean mode) {
        solenoid.set(mode);
    }
}