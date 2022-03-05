package frc.robot;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class TalonSRXWrapper extends Wrapper {
    WPI_TalonSRX talon;

    public TalonSRXWrapper (int port) {
        talon = new WPI_TalonSRX(port);
    }
    public void set(int speed) {
        talon.set(ControlMode.PercentOutput, speed);
    }
}

