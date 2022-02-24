package frc.robot.swervedrive;
import edu.wpi.first.math.kinematics.SwerveModuleState;


public class WheelWrapper {
    public Wheel wheel;
    
    public WheelWrapper (int angleMotorID, int speedMotorID, String wheelName) {
        wheel = new Wheel(angleMotorID, speedMotorID, wheelName);
        
    }

    public void drive(SwerveModuleState state) {
        wheel.drive(state);
    }
    
}
