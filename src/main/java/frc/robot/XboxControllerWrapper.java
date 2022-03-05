package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

public class XboxControllerWrapper extends Wrapper{
    
    XboxController xbox;
    double deadZone;
    int port;

    public XboxControllerWrapper(int port) {
        this.port = port;
        xbox = new XboxController(this.port);
        deadZone = 0.15;
    }

    public double getRightX() {
        if (Math.abs(xbox.getRightX()) < deadZone) {
            return 0.0;
        }
        return xbox.getRightX();
    }
    
    public double getRightY() {
        if (Math.abs(xbox.getRightY()) < deadZone) {
            return 0.0;
        }
        return xbox.getRightY();
    }
    
    public double getLeftX() {
        if (Math.abs(xbox.getLeftX()) < deadZone) {
            return 0.0;
        }
        return xbox.getLeftX();
    }
    
    public double getLeftY() {
        if (Math.abs(xbox.getLeftY()) < deadZone) {
            return 0.0;
        }
        return xbox.getLeftY();
    }

    public boolean getAButton() {
        return xbox.getAButton();
    }
    
    public boolean getXButton() {
        return xbox.getXButton();
    }
    
    public boolean getBButton() {
        return xbox.getBButton();
    }
    
    public boolean getYButton() {
        return xbox.getYButton();
    }

    // public boolean getRawButton(int ID) {
    //     return js.getRawButton(ID);
    // }

    // public double getPOV() {
    //     return js.getPOV();
    // }

    public double getRightTriggerAxis() {
        return xbox.getRightTriggerAxis();
    }
    
    public double getLeftTriggerAxis() {
        return xbox.getLeftTriggerAxis();
    }
    public boolean getRightBumper() {
        return xbox.getRightBumper();
    }
}
