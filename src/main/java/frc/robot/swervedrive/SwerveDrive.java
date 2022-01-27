package frc.robot.swervedrive;

public class SwerveDrive {
    ChassisSpeeds speeds;
    SwerveModuleState[] moduleStates;
    Wheel frontLeft;
    Wheel frontRight;
    Wheel backLeft;
    Wheel backRight;

    public SwerveDrive(Wheel frontLeft, Wheel frontRight, Wheel backRight, Wheel backLeft) {
        this.frontRight = frontRight;
        this.frontLeft = frontLeft;
        this.backRight = backRight;
        this.backLeft = backLeft;
    }

    public void drive(double x, double y, double r) {
        speeds.vxMetersPerSecond = x;
        speeds.vyMetersPerSecond = y;
        speeds.omegaRadiansPerSecond = r;

        moduleStates = kinematics.toSwerveModuleStates(speeds);
    }
}