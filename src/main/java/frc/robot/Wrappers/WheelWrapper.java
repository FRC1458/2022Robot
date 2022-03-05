package frc.robot.wrappers;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.swervedrive.Wheel;

//Help me I'm trapped in a WheelWrapper factory.

public class WheelWrapper extends Wrapper{
    public Wheel wheel;
    
    public WheelWrapper (int angleMotorID, int speedMotorID, int absoluteEncoderID, String wheelName) {
        try{
            wheel = new Wheel(angleMotorID, speedMotorID, absoluteEncoderID, wheelName);
            isInitialized = true;
        }
        catch (RuntimeException ex ) {
            DriverStation.reportError("Error Initiating Wheel:  " + ex.getMessage(), true);
        }
    }

    public void drive(SwerveModuleState state) {
        if (isInitialized) wheel.drive(state.speedMetersPerSecond, state.angle.getDegrees());
    }
    
    public void setEncoders(double offset) {
        if (isInitialized) wheel.setEncoders(offset);
    }

}
