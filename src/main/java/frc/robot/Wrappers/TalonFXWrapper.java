package frc.robot;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;


public class TalonFXWrapper extends Wrapper {
    TalonFX talon;

    public TalonFXWrapper(int port) {
        talon = new TalonFX(port);
    }
    public void set(int speed) {
        talon.set(ControlMode.PercentOutput, speed);
    }
}