package frc.robot.swervedrive;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;

//Help me I'm trapped in a WheelWrapper factory.

public class WheelWrapper {
    public Wheel wheel;
    
    public WheelWrapper (int angleMotorID, int speedMotorID, String wheelName) {
        wheel = new Wheel(angleMotorID, speedMotorID, wheelName);
        try{
            wheel = new Wheel(angleMotorID, speedMotorID, wheelName);
        }
        catch (RuntimeException ex ) {
            DriverStation.reportError("Error Initiating Wheel:  " + ex.getMessage(), true);
        }
    }

    public void drive(SwerveModuleState state) {
        wheel.drive(state);
    }
    
    public void zeroEncoders() {
        wheel.zeroEncoders();
    }

}
