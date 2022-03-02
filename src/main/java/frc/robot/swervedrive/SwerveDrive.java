package frc.robot.swervedrive;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import frc.robot.RobotConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;

public class SwerveDrive {
    ChassisSpeeds speedsFront;
    ChassisSpeeds speedsBack;
    Wheel frontLeft;
    Wheel frontRight;
    Wheel backLeft;
    Wheel backRight;
    SwerveDriveKinematics kinematics;
    SwerveDriveOdometry odometry;
    Pose2d pose;
    AHRS ahrs;

    public SwerveDrive() {
        frontLeft = new Wheel(RobotConstants.frontLeftAngleID, RobotConstants.frontLeftSpeedID, RobotConstants.frontLeftAbsoluteEncoderID, "Front Left (1)");
        frontRight = new Wheel(RobotConstants.frontRightAngleID, RobotConstants.frontRightSpeedID, RobotConstants.frontRightAbsoluteEncoderID, "Front Right (2)");
        backLeft = new Wheel(RobotConstants.backLeftAngleID, RobotConstants.backLeftSpeedID, RobotConstants.backLeftAbsoluteEncoderID, "Back Left (3)");
        backRight = new Wheel(RobotConstants.backRightAngleID, RobotConstants.backRightSpeedID, RobotConstants.backRightAbsoluteEncoderID, "Back Right (4)");

        // Locations for the swerve drive modules relative to the robot center.
        Translation2d frontLeftLocation = new Translation2d(RobotConstants.frontLeftXMeters, RobotConstants.frontLeftYMeters);
        Translation2d frontRightLocation = new Translation2d(RobotConstants.frontRightXMeters, RobotConstants.frontRightYMeters);
        Translation2d backLeftLocation = new Translation2d(RobotConstants.backLeftXMeters, RobotConstants.backLeftYMeters);
        Translation2d backRightLocation = new Translation2d(RobotConstants.backRightXMeters, RobotConstants.backRightYMeters);

        // Creating my kinematics object using the module locations
        kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);
        odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(), new Pose2d(5.0, 13.5, new Rotation2d()));

        speedsFront = new ChassisSpeeds();
        speedsBack = new ChassisSpeeds();
    }

    public void drive(double x, double y, double r) {
        speedsFront.vxMetersPerSecond = x;
        speedsFront.vyMetersPerSecond = y;
        speedsFront.omegaRadiansPerSecond = r;

        speedsBack.vxMetersPerSecond = x;
        speedsBack.vyMetersPerSecond = y;
        speedsBack.omegaRadiansPerSecond = -r;

        SmartDashboard.putNumber("X", x);
        SmartDashboard.putNumber("Y", y);
        SmartDashboard.putNumber("R", r);
        // SmartDashboard.putNumber("Robot Angle", ahrs.getYaw());

        SwerveModuleState[] moduleStatesFront = kinematics.toSwerveModuleStates(speedsFront);
        SwerveModuleState[] moduleStatesBack = kinematics.toSwerveModuleStates(speedsBack);

        SmartDashboard.putNumber("SpeedMotorFront", moduleStatesFront[1].speedMetersPerSecond);
        SmartDashboard.putNumber("AngleMotorFront", moduleStatesFront[1].angle.getDegrees());

        SmartDashboard.putNumber("SpeedMotorBack", moduleStatesFront[1].speedMetersPerSecond);
        SmartDashboard.putNumber("AngleMotorBack", moduleStatesFront[1].angle.getDegrees());

        //Set to angle that we get from the NavX
        //double angle = 0;

        //Rotation2d gyroAngle = Rotation2d.fromDegrees(angle);

        // Update the pose
        //pose = odometry.update(gyroAngle, moduleStates[0], moduleStates[1], moduleStates[2], moduleStates[3]);

        frontLeft.drive(moduleStatesFront[2].speedMetersPerSecond, moduleStatesFront[2].angle.getDegrees());
        frontRight.drive(moduleStatesFront[0].speedMetersPerSecond, moduleStatesFront[0].angle.getDegrees());
        backLeft.drive(moduleStatesFront[3].speedMetersPerSecond, moduleStatesFront[3].angle.getDegrees());
        backRight.drive(moduleStatesFront[1].speedMetersPerSecond, moduleStatesFront[1].angle.getDegrees());
    }

    public void setEncoders2() {
        frontLeft.setEncoders(SmartDashboard.getNumber("front left offset", 0));
        frontRight.setEncoders(SmartDashboard.getNumber("front right offset", 0));
        backLeft.setEncoders(SmartDashboard.getNumber("back left offset", 0));
        backRight.setEncoders(SmartDashboard.getNumber("back right offset", 0));
    }

    public void setEncoders() {
        frontLeft.setEncoders(RobotConstants.frontLeftAngleOffset);
        frontRight.setEncoders(RobotConstants.frontRightAngleOffset);
        backLeft.setEncoders(RobotConstants.backLeftAngleOffset);
        backRight.setEncoders(RobotConstants.backRightAngleOffset);
    }

    public void readAbsoluteEncoder() {
        SmartDashboard.putNumber("front left absolute", frontLeft.getAbsoluteEncoderValue());
        SmartDashboard.putNumber("front right absolute", frontRight.getAbsoluteEncoderValue());
        SmartDashboard.putNumber("back left absolute", backLeft.getAbsoluteEncoderValue());
        SmartDashboard.putNumber("back right absolute", frontRight.getAbsoluteEncoderValue());

    }
}