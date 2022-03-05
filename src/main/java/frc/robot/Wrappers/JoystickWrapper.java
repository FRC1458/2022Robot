package frc.robot.wrappers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation;

public class JoystickWrapper extends Wrapper{
    
    Joystick js;
    double deadZone;
    int port;

    public JoystickWrapper(int port) {
        try{
            this.port = port;
            js = new Joystick(this.port);
            deadZone = 0.1;
        }
        catch (RuntimeException ex ) {
            DriverStation.reportError("Error Initiating CAN Spark Max:  " + ex.getMessage(), true);
        }
    }

    public double getRawAxis(int ID) {
        if (isInitialized) {
            if (Math.abs(js.getRawAxis(ID)) < deadZone) {
                return 0.0;
            }
            return js.getRawAxis(ID);
        }  
        return 0; 
    }

    public boolean getRawButton(int ID) {
        if (isInitialized) return js.getRawButton(ID);
        return false;
    }

    public double getPOV() {
        if (isInitialized) return js.getPOV();
        return -1;
    }
}
