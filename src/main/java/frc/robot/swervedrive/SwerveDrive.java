package frc.robot.swervedrive;

public class SwerveDrive {
    ChassisSpeeds speeds;
    SwerveModuleState[] moduleStates;

    public void drive(double x, double y, double r) {
        speeds.vxMetersPerSecond = x;
        speeds.vyMetersPerSecond = y;
        speeds.omegaRadiansPerSecond = r;

        moduleStates = kinematics.toSwerveModuleStates(speeds);
    }
}