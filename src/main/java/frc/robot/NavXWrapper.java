public class NavXWrapper {
    private NavX navx;
    public NavXWrapper(){
        try{
            navx = new NavX();
        }
        catch{
            DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
        }
    }
    public operatorControl(){
        navx.operatorControl();
    }
}
