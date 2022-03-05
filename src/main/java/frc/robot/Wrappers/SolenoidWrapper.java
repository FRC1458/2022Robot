package frc.robot.wrappers;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DriverStation;


public class SolenoidWrapper extends Wrapper {
    Solenoid solenoid;

    public SolenoidWrapper(int id) {
        try{
            solenoid = new Solenoid(PneumaticsModuleType.REVPH, id);
            isInitialized = true;
        }
        catch (RuntimeException ex ) {
            DriverStation.reportError("Error Initiating Solenoid:  " + ex.getMessage(), true);
        }
    }

    public void set(boolean mode) {
        if (isInitialized) solenoid.set(mode);
    }
}