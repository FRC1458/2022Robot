package frc.robot.swervedrive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
//import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotConstants;

public class Wheel {
    private CANSparkMax angleMotor;
    private CANSparkMax speedMotor;
    private SparkMaxPIDController pidController;
    private RelativeEncoder encoder;
    //private ControlType controltype;

    private double speed;
    private double goalAngle;

    private String wheelName;

    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

    public Wheel (int angleMotorID, int speedMotorID, String wheelName) {
        this.angleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
        this.speedMotor = new CANSparkMax(speedMotorID, MotorType.kBrushless);

        this.wheelName = wheelName;
        
        angleMotor.restoreFactoryDefaults();

        pidController = angleMotor.getPIDController();
        encoder = angleMotor.getEncoder();

        SmartDashboard.putNumber("Rotations", 0);

        kP = RobotConstants.kP; 
        kI = RobotConstants.kI;
        kD = RobotConstants.kD; 
        kIz = 0; 
        kFF = 0.000156; 
        kMaxOutput = 1; 
        kMinOutput = -1;
        maxRPM = 5700;

        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setIZone(kIz);
        pidController.setFF(kFF);
        pidController.setOutputRange(kMinOutput, kMaxOutput);

        int smartMotionSlot = 0;
        pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("P Gain", kP);
        SmartDashboard.putNumber("I Gain", kI);
        SmartDashboard.putNumber("D Gain", kD);
        /*
        SmartDashboard.putNumber("I Zone", kIz);
        SmartDashboard.putNumber("Feed Forward", kFF);
        SmartDashboard.putNumber("Max Output", kMaxOutput);
        SmartDashboard.putNumber("Min Output", kMinOutput);
        

        // display Smart Motion coefficients
        SmartDashboard.putNumber("Max Velocity", maxVel);
        SmartDashboard.putNumber("Min Velocity", minVel);
        SmartDashboard.putNumber("Max Acceleration", maxAcc);
        SmartDashboard.putNumber("Allowed Closed Loop Error", allowedErr);
        SmartDashboard.putNumber("Set Position", 0);
        SmartDashboard.putNumber("Set Velocity", 0);
        */
    }

    public void drive (SwerveModuleState state) {

        speed = state.speedMetersPerSecond;
        goalAngle = state.angle.getDegrees() + 180;
        
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        /*
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);
        double maxV = SmartDashboard.getNumber("Max Velocity", 0);
        double minV = SmartDashboard.getNumber("Min Velocity", 0);
        double maxA = SmartDashboard.getNumber("Max Acceleration", 0);
        double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", 0);
        */
    
        // if PID coefficients on SmartDashboard have changed, write new values to controller
        
        if((p != kP)) { pidController.setP(p); kP = p; }
        if((i != kI)) { pidController.setI(i); kI = i; }
        if((d != kD)) { pidController.setD(d); kD = d; }
        /*
        if((iz != kIz)) { pidController.setIZone(iz); kIz = iz; }
        if((ff != kFF)) { pidController.setFF(ff); kFF = ff; }
        if((max != kMaxOutput) || (min != kMinOutput)) { 
          pidController.setOutputRange(min, max); 
          kMinOutput = min; kMaxOutput = max; 
        }
        if((maxV != maxVel)) 
        { 
            pidController.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; 
        }
        if((minV != minVel)) 
        { 
            pidController.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; 
        }
        if((maxA != maxAcc)) 
        { 
            pidController.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; 
        }
        if((allE != allowedErr)) 
        { 
            pidController.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr = allE; 
        }
        */
        /*
        double setPoint;
        setPoint = SmartDashboard.getNumber("Set Position", 0);
        pidController.setReference(setPoint, CANSparkMax.ControlType.kPosition);
        
        SmartDashboard.putNumber("SetPoint", setPoint);
        */
        double processVariable = encoder.getPosition();
        SmartDashboard.putNumber("Encoder" + wheelName, processVariable);
        SmartDashboard.putNumber("Output" + wheelName, angleMotor.getAppliedOutput());

        SmartDashboard.putNumber("Angle Motor Current (Amps)" + wheelName, angleMotor.getOutputCurrent());
        SmartDashboard.putNumber("Speed Motor Current (Amps)", speedMotor.getOutputCurrent());
        /*
        double fullRotations = Math.signum(processVariable) * Math.floor((Math.abs(processVariable) + RobotConstants.swerveDriveGearRatio / 4) / RobotConstants.swerveDriveGearRatio);

        double radians = goalAngle;

        double rotations = radians * RobotConstants.swerveDriveGearRatio / (2 * Math.PI);
        rotations += fullRotations * RobotConstants.swerveDriveGearRatio;

        
        if (rotations > 0 && Math.abs(rotations - processVariable) > Math.abs(rotations - RobotConstants.swerveDriveGearRatio - processVariable)) {
            rotations = rotations - RobotConstants.swerveDriveGearRatio;
            SmartDashboard.putNumber("Funny Number" + wheelName, 1);
        }
        else if (rotations < 0 && Math.abs(rotations - processVariable) > Math.abs(rotations + RobotConstants.swerveDriveGearRatio - processVariable)) {
            rotations = rotations + RobotConstants.swerveDriveGearRatio;
            SmartDashboard.putNumber("Funny Number" + wheelName, -1);
        }
        else {
            SmartDashboard.putNumber("Funny Number" + wheelName, 0);
        }
        */
        
        double currentAngle = (processVariable * (360 / RobotConstants.swerveDriveGearRatio)) % 360;

        double rotation1 = currentAngle - goalAngle;
        double rotation2 = 0;

        if (rotation1 < 0) {
            rotation2 = rotation1 + 360;
        }
        else {
            rotation2 = rotation1 - 360;
        }
        
        double rotation = 0;

        if (Math.abs(rotation1) > Math.abs(rotation2)) {
            rotation = rotation2;
        }
        else {
            rotation = rotation1;
        }

        double realGoalAngle = processVariable + rotation * (RobotConstants.swerveDriveGearRatio / 360);

        SmartDashboard.putNumber("Current Angle", currentAngle);

        SmartDashboard.putNumber("Goal Angle", goalAngle);

        SmartDashboard.putNumber("Real Goal Angle", realGoalAngle);

        SmartDashboard.putNumber("Rotation", rotation);

        SmartDashboard.putNumber("SPEED" + wheelName, speed);

        //rotations = SmartDashboard.getNumber("Rotations", 0);

        pidController.setReference(realGoalAngle, CANSparkMax.ControlType.kPosition);
        speedMotor.set(speed);
    }

    public void zeroEncoders() {
        encoder.setPosition(0);
    }
}