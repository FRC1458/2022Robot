package frc.robot.swervedrive;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
//import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import edu.wpi.first.math.geometry.Rotation2d;

//Wrapper
import frc.robot.Wrapper;


public class Wheel{
    private CANSparkMax angleMotor;
    private CANSparkMax speedMotor;
    private SparkMaxPIDController pidController;
    private RelativeEncoder encoder;
    private TalonSRX absoluteEncoder;
    
    //private ControlType controltype;


    private double speed;
    private double goalAngle;

    private String wheelName;

    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

    public Wheel (int angleMotorID, int speedMotorID, int absoluteEncoderID, String wheelName) {
        this.angleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
        this.speedMotor = new CANSparkMax(speedMotorID, MotorType.kBrushless);
        this.absoluteEncoder = new TalonSRX(absoluteEncoderID);

        this.absoluteEncoder.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

        this.wheelName = wheelName;
        
        //angleMotor.restoreFactoryDefaults();

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

    public void drive (double speed, double angle) {

        this.speed = speed;
        
        goalAngle = angle;
        
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
        // SmartDashboard.putNumber("Output" + wheelName, angleMotor.getAppliedOutput());

        // SmartDashboard.putNumber("Angle Motor Current (Amps)" + wheelName, angleMotor.getOutputCurrent());
        // SmartDashboard.putNumber("Speed Motor Current (Amps)", speedMotor.getOutputCurrent());

        
        double currentAngle = (processVariable * (360 / RobotConstants.swerveDriveGearRatio));


        double diff = (currentAngle - goalAngle) % 360;

        if (Math.abs(diff) > 180) {
          diff = diff - 360*Math.signum(diff); // add or subtract 360 so the difference is always smaller than 180
        }

        double realGoalRotations = (currentAngle - diff) * RobotConstants.swerveDriveGearRatio/360;

        SmartDashboard.putNumber("Current Angle", currentAngle);

        SmartDashboard.putNumber("Goal Angle", goalAngle);

        SmartDashboard.putNumber("Real Goal Angle", realGoalRotations);

        SmartDashboard.putNumber("difference", diff);

        SmartDashboard.putNumber("SPEED" + wheelName, speed);

        // rotations = SmartDashboard.getNumber("Rotations", 0);

        if (speed != 0){
            pidController.setReference(realGoalRotations, CANSparkMax.ControlType.kPosition);
        }
        speedMotor.set(speed);
    }

    public void setEncoders(double offset) {
        encoder.setPosition(offset + (absoluteEncoder.getSelectedSensorPosition(0) % 4096)*RobotConstants.swerveDriveGearRatio/4096.0);
        SwerveModuleState state = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
        this.drive(state.speedMetersPerSecond, state.angle.getDegrees());
    }

    public double getAbsoluteEncoderValue() {
        return ((absoluteEncoder.getSelectedSensorPosition(0) % 4096)*RobotConstants.swerveDriveGearRatio/4096.0);
    }
}