package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.swervedrive.SwerveDrive;

public class Diagnostic {
    public void angleMotorDiagnostic(SwerveDrive sd) {
        SmartDashboard.putNumber("", sd.backLeft.angleMotorDiagnostic());
    }
}
