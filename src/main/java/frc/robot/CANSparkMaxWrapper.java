package frc.robot;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class CANSparkMaxWrapper {
    public CANSparkMax spark;

    public CANSparkMaxWrapper (int num) {
        spark = new CANSparkMax(num, MotorType.kBrushless);
    }

    public void getAppliedOutput() {
        spark.getAppliedOutput();
    }

    public void getOutputCurrent() {
        spark.getOutputCurrent();
    }

    public void getPIDController() {
        spark.getPIDController();
    }

    public void getEncoder() {
        spark.getEncoder();
    }

    public void set(double speed) {
        spark.set(speed);
    }
}