package frc.robot.swervedrive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
//import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Wheel {
    //private CANSparkMax angleMotor;
    private CANSparkMax speedMotor;
    private SparkMaxPIDController pidController;
    private RelativeEncoder encoder;
    //private ControlType controltype;

    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

    public Wheel (int angleMotorID, int speedMotorID) {
        //this.angleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
        this.speedMotor = new CANSparkMax(speedMotorID, MotorType.kBrushless);
        /* 
        angleMotor.restoreFactoryDefaults();

        pidController = angleMotor.getPIDController();
        encoder = angleMotor.getEncoder();

        kP = 5e-5; 
        kI = 1e-6;
        kD = 0; 
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

        // button to toggle between velocity and smart motion modes
        SmartDashboard.putBoolean("Mode", true);
        */
    }

    public void drive (SwerveModuleState state) {
        /*
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);
        double maxV = SmartDashboard.getNumber("Max Velocity", 0);
        double minV = SmartDashboard.getNumber("Min Velocity", 0);
        double maxA = SmartDashboard.getNumber("Max Acceleration", 0);
        double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", 0);
    
        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if((p != kP)) { pidController.setP(p); kP = p; }
        if((i != kI)) { pidController.setI(i); kI = i; }
        if((d != kD)) { pidController.setD(d); kD = d; }
        if((iz != kIz)) { pidController.setIZone(iz); kIz = iz; }
        if((ff != kFF)) { pidController.setFF(ff); kFF = ff; }
        if((max != kMaxOutput) || (min != kMinOutput)) { 
          pidController.setOutputRange(min, max); 
          kMinOutput = min; kMaxOutput = max; 
        }
        if((maxV != maxVel)) { pidController.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
        if((minV != minVel)) { pidController.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; }
        if((maxA != maxAcc)) { pidController.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
        if((allE != allowedErr)) { pidController.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr = allE; }

        double setPoint, processVariable;
        boolean mode = SmartDashboard.getBoolean("Mode", false);
        if(mode) {
        setPoint = SmartDashboard.getNumber("Set Velocity", 0);
        pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
        processVariable = encoder.getVelocity();
        } else {
        setPoint = SmartDashboard.getNumber("Set Position", 0);
        /**
         * As with other PID modes, Smart Motion is set by calling the
         * setReference method on an existing pid object and setting
         * the control type to kSmartMotion
         */
        /*
        pidController.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
        processVariable = encoder.getPosition();
        }
        
        SmartDashboard.putNumber("SetPoint", setPoint);
        SmartDashboard.putNumber("Process Variable", processVariable);
        SmartDashboard.putNumber("Output", angleMotor.getAppliedOutput());
        */

        speedMotor.set(state.speedMetersPerSecond);
    }
}