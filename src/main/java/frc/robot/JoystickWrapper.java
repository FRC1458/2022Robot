package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class JoystickWrapper {
    
    Joystick js;
    double deadZone;

    public JoystickWrapper(int ID) {
        js = new Joystick(ID);
        deadZone = 0.1;
    }

    public double getRawAxis(int ID) {
        if (Math.abs(js.getRawAxis(ID)) < 0.05) {
            return 0.0;
        }
        return js.getRawAxis(ID);
    }

    public boolean getRawButton(int ID) {
        return js.getRawButton(ID);
    }
}
