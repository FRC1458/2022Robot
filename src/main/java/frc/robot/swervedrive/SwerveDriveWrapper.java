package frc.robot.swervedrive;

public class SwerveDriveWrapper {
   
    public SwerveDrive swervedrive;

    public SwerveDriveWrapper () {
        swervedrive = new SwerveDrive() ;

    }

    public void drive(double x, double y, double r) {
        swervedrive.drive(x, y, r);
    }

    public void zeroEncoders() {
        swervedrive.zeroEncoders();
    }
}
