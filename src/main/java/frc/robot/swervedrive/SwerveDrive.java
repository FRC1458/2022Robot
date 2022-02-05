package frc.robot.swervedrive;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.RobotConstants;
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveDrive {
    ChassisSpeeds speeds;
    Wheel frontLeft;
    Wheel frontRight;
    Wheel backLeft;
    Wheel backRight;
    SwerveDriveKinematics kinematics;

    public SwerveDrive() {
        //frontRight = new Wheel(RobotConstants.frontRightAngleID, RobotConstants.frontRightSpeedID);
        frontLeft = new Wheel(RobotConstants.frontLeftAngleID, RobotConstants.frontLeftSpeedID);
        //backRight = new Wheel(RobotConstants.backRightAngleID, RobotConstants.backRightSpeedID);
        //backLeft = new Wheel(RobotConstants.backLeftAngleID, RobotConstants.backLeftSpeedID);

        // Locations for the swerve drive modules relative to the robot center.
        Translation2d frontLeftLocation = new Translation2d(RobotConstants.frontLeftXMeters, RobotConstants.frontLeftYMeters);
        Translation2d frontRightLocation = new Translation2d(RobotConstants.frontRightXMeters, RobotConstants.frontLeftYMeters);
        Translation2d backLeftLocation = new Translation2d(RobotConstants.backLeftXMeters, RobotConstants.backLeftYMeters);
        Translation2d backRightLocation = new Translation2d(RobotConstants.backRightXMeters, RobotConstants.backLeftYMeters);

        // Creating my kinematics object using the module locations
        kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

        speeds = new ChassisSpeeds();
    }

    public void drive(double x, double y, double r) {
        speeds.vxMetersPerSecond = x;
        speeds.vyMetersPerSecond = y;
        speeds.omegaRadiansPerSecond = r;

        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);

        frontLeft.drive(moduleStates[0]);
        //frontRight.drive(moduleStates[1]);
        //backLeft.drive(moduleStates[2]);
        //backRight.drive(moduleStates[3]);
    }
}