import edu.wpi.first.wpilibj.DriverStation;

public class NavXWrapper {
    private NavX navx;
    public NavXWrapper(){
        try{
            navx = new NavX();
        }
        catch (RuntimeException ex ) {
            DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
        }
    }
    public void operatorControl() {
        navx.operatorControl();
    }
}