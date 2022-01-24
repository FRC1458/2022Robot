package frc.robot.swervedrive;

import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class WheelDrive {
    private CANSparkMax angleMotor;
    private CANSparkMax speedMotor;
    private PIDController pidController;

    public WheelDrive (int angleMotor, int speedMotor, int encoder) {
        this.angleMotor = new CANSparkMax(angleMotor, MotorType.kBrushless);
        this.speedMotor = new CANSparkMax(speedMotor, MotorType.kBrushless);

        //Everything below this point is copypasted, need to match to our equipment
        pidController = new PIDController (1, 0, 0, new AnalogInput(encoder), this.angleMotor);
    
        pidController.setOutputRange (-1, 1);
        pidController.setContinuous ();
        pidController.enable ();
}