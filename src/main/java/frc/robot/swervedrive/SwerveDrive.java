package frc.robot.swervedrive;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

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

        SmartDashboard.putNumber("X", x);
        SmartDashboard.putNumber("Y", y);
        SmartDashboard.putNumber("Rotation", r);

        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);

        SmartDashboard.putNumber("SpeedMotor", moduleStates[0].speedMetersPerSecond);
        SmartDashboard.putNumber("AngleMotor", moduleStates[0].angle.getDegrees());

        frontLeft.drive(moduleStates[0]);
        //frontRight.drive(moduleStates[1]);
        //backLeft.drive(moduleStates[2]);
        //backRight.drive(moduleStates[3]);
    }
}