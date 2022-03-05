import com.ctre.phoenix.motorcontrol.ControlMode;

public class SolenoidWrapper extends Wrapper {
    Solenoid Solenoid;

    public SolenoidWrapper(int id) {
        solenoid = new Solenoid(id);
    }

    public void set(bool mode) {
        solenoid.set(mode);
    }
}