package frc.robot.swervedrive;

import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Wheel {
    private CANSparkMax angleMotor;
    private CANSparkMax speedMotor;
    private PIDController pidController;

    private final double MAX_VOLTS = 0;

    public Wheel (int angleMotor, int speedMotor, int encoder) {
        this.angleMotor = new CANSparkMax(angleMotor, MotorType.kBrushless);
        this.speedMotor = new CANSparkMax(speedMotor, MotorType.kBrushless);

        //Everything below this point is copypasted, need to match to our equipment
        pidController = new PIDController (1, 0, 0, new AnalogInput(encoder), this.angleMotor);
    
        pidController.setOutputRange (-1, 1);
        pidController.setContinuous ();
        pidController.enable ();
    }

    public void drive (SwerveModuleState state) {
        double speed = state.speedMetersPerSecond;
        double angle = state.angle.getRadians;

        speedMotor.set(speed);
        
        double setpoint = angle * (MAX_VOLTS * 0.5) + (MAX_VOLTS * 0.5); // Optimization offset can be calculated here.
        if (setpoint < 0) {
            setpoint = MAX_VOLTS + setpoint;
        }
        if (setpoint > MAX_VOLTS) {
            setpoint = setpoint - MAX_VOLTS;
        }

        pidController.setSetpoint (setpoint);
    }
}