package frc.robot.wrappers;

import edu.wpi.first.wpilibj.Joystick;

public class JoystickWrapper extends Wrapper{
    
    Joystick js;
    double deadZone;
    int port;

    public JoystickWrapper(int port) {
        this.port = port;
        js = new Joystick(this.port);
        deadZone = 0.1;
    }

    public double getRawAxis(int ID) {
        if (Math.abs(js.getRawAxis(ID)) < deadZone) {
            return 0.0;
        }
        return js.getRawAxis(ID);
    }

    public boolean getRawButton(int ID) {
        return js.getRawButton(ID);
    }

    public double getPOV() {
        return js.getPOV();
    }
}
