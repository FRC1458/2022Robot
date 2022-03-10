// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project. (THE GAME)

package frc.robot;

import javax.net.ssl.CertPathTrustManagerParameters;
import javax.xml.transform.SourceLocator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.RobotConstants;
import frc.robot.swervedrive.SwerveDrive;
import frc.robot.swervedrive.Wheel;
import frc.robot.wrappers.*;

import com.ctre.phoenix.motorcontrol.ControlMode;

//import com.cuforge.libcu.Lasershark;

//Xbox support
import edu.wpi.first.wpilibj.XboxController;

//import edu.wpi.first.wpilibj.Ultrasonic;

import edu.wpi.first.wpilibj.Solenoid;

//class Camera;
/**sssP
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  //define variables
  enum States {
    AUTONOMOUS,
    MANUAL,
    DETECT_BALL,
    MOVE_TO_BALL,
    PICK_UP_BALL,
    GO_TO_HUB,
    DROP_BALL,
    AIM,
    SHOOT,
    GO_TO_HUMAN;


  }
  private final SolenoidWrapper leftIntakeSolenoid;
  private final SolenoidWrapper rightIntakeSolenoid;
  private final SolenoidWrapper leftElevatorSolenoid;
  private final SolenoidWrapper rightElevatorSolenoid;


  States state;
  private TalonSRXWrapper leftMotor;
  private TalonSRXWrapper leftMotor2;
  private TalonSRXWrapper rightMotor;
  private TalonSRXWrapper rightMotor2;
  private JoystickWrapper leftStick;
  private JoystickWrapper rightStick;
  private XboxControllerWrapper xboxController;
  private double leftAxis;
  private double rightAxis;

  //private CameraWrapper camera;

  private boolean depositButton;
  private boolean elevatorUpButton;
  private boolean elevatorDownButton;
  private boolean climbButton;
  private boolean dropBall;
  private boolean resetNavX;
  private boolean speedReduceButton;

  //private NavX navx;

  //Camera topCam;
  //Camera bottomCam;
  CameraWrapper ballCamera;
  SwerveDrive swerveDrive;

  //Lasershark shark;

  //Set Controller Type
  int controllerType;

  private TalonSRXWrapper intakeMotor;
  private TalonSRXWrapper leftDepositorMotor;
  private TalonSRXWrapper rightDepositorMotor;
  private TalonFXWrapper leftElevatorMotor; //to go up go clockwise
  private TalonFXWrapper rightElevatorMotor; //to go up go counter-clockwise
  
  private DigitalInput bottomLimitSwitch;
  private DigitalInput middleLimitSwitch;
  private DigitalInput topLimitSwitch;




  boolean turnTo = false;

  public Robot() {
    super(0.03);
    //create variables
    leftStick = new JoystickWrapper(0);
    rightStick = new JoystickWrapper(1);
    xboxController = new XboxControllerWrapper(0);
    //topCam = new Camera();
    //bottomCam = new Camera();
    //Ball = new Ball();
    state = States.MANUAL;
    swerveDrive = new SwerveDrive();
    ballCamera = new CameraWrapper(true);

    //navx = new NavX();

    //shark = new Lasershark(0);

    leftIntakeSolenoid = new SolenoidWrapper(RobotConstants.leftIntakeSolenoidID);
    rightIntakeSolenoid = new SolenoidWrapper(RobotConstants.rightIntakeSolenoidID);
    leftElevatorSolenoid = new SolenoidWrapper(RobotConstants.leftElevatorSolenoidID);
    rightElevatorSolenoid = new SolenoidWrapper(RobotConstants.rightElevatorSolenoidID);

    intakeMotor = new TalonSRXWrapper(RobotConstants.intakeMotorID);
    leftDepositorMotor = new TalonSRXWrapper(RobotConstants.leftDepositorMotorID);
    rightDepositorMotor = new TalonSRXWrapper(RobotConstants.rightDepositorMotorID);
    leftElevatorMotor = new TalonFXWrapper(RobotConstants.leftElevatorMotorID);
    rightElevatorMotor = new TalonFXWrapper(RobotConstants.rightElevatorMotorID);
    bottomLimitSwitch = new DigitalInput(0);
    middleLimitSwitch = new DigitalInput(1);
    topLimitSwitch = new DigitalInput(2);
  }


  @Override
  public void robotInit() {
    //set to defaults
    autonomousInit();
    swerveDrive.setEncoders();
  }

  @Override
  public void teleopInit() {
    ballCamera = new CameraWrapper(true);
    leftIntakeSolenoid.set(false);
    rightIntakeSolenoid.set(false);

  }


  @Override
  public void teleopPeriodic() {
    double xAxis;
    double yAxis;
    double rAxis;


    SmartDashboard.putNumber("BallX", ballCamera.getBallX());


    //SET CONTROLLER TYPE HERE
    //SET TO 0 FOR XBOX CONTROLLER
    //SET TO 1 FOR EVERYTHING ELSE

    controllerType = 0;

    //Controllers
    if (controllerType == 0) {
      xAxis = xboxController.getLeftX();
      yAxis = xboxController.getLeftY();
      rAxis = xboxController.getRightX();
      depositButton = xboxController.getAButton();
      elevatorUpButton = xboxController.getBButton();
      climbButton = xboxController.getXButton();
      elevatorDownButton = xboxController.getYButton();
      dropBall = xboxController.getRightBumper();
      resetNavX = xboxController.getStartButton();
      speedReduceButton = (xboxController.getRightTriggerAxis() > 0.7);

    }
    else if (controllerType == 1) {
      xAxis = leftStick.getRawAxis(0);
      yAxis = leftStick.getRawAxis(1);
      rAxis = rightStick.getRawAxis(0);
      depositButton = leftStick.getRawButton(0);
      elevatorUpButton = leftStick.getRawButton(1);
      climbButton = leftStick.getRawButton(2);
      elevatorDownButton = leftStick.getRawButton(3);
      speedReduceButton = rightStick.getRawButton(7);
      resetNavX = rightStick.getRawButton(4);

    }
    else {
      xAxis = 0;
      yAxis = 0;
      rAxis = 0;
    }

    // Setting speed of depositor motors
    if (depositButton) {
      leftDepositorMotor.set(0.5);
      rightDepositorMotor.set(-0.5);
      intakeMotor.set(0.5);
    }
    else if (dropBall) {
      leftDepositorMotor.set(-0.5);
      rightDepositorMotor.set(0.5);
      intakeMotor.set(0);
    }
    else {
      leftDepositorMotor.set(0);
      rightDepositorMotor.set(0);
      intakeMotor.set(0);
    }
    if (xboxController.getRightTriggerAxis() > 0.7) {
      leftElevatorMotor.set(1);
      rightElevatorMotor.set(-1);
    } else if (xboxController.getLeftTriggerAxis() > 0.7) {
      leftElevatorMotor.set(-1);
      rightElevatorMotor.set(1);
    } else {
      leftElevatorMotor.set(0);
      rightElevatorMotor.set(0);
    }
    if (resetNavX) {
      swerveDrive.resetNavX();
      swerveDrive.setEncoders();
    }
    //double distanceToBall = shark.getDistanceCentimeters();
    //SmartDashboard.putNumber("distanceToBall", distanceToBall);

    //navx.operatorControl();

    /*
    double pov = leftStick.getPOV();

    turnTo = (pov != -1) && (rAxis == 0);

    if (turnTo) {
      rAxis = swerveDrive.turnToAngle(pov);
      if (rAxis == 0) {
        turnTo = false;
      }
    }
    
    */
    double x,y,r,speedReduce;
    speedReduce = 1;

    if(speedReduceButton){
      speedReduce = 0.25;
    }
    x = -(Math.abs(xAxis)*xAxis) * speedReduce;
    y= Math.abs(yAxis)*yAxis * speedReduce;
    r= Math.abs(rAxis)*rAxis * speedReduce;


    swerveDrive.drive(x, y, r, true);
  }

  public void setMotorSpeed (double speed) {
    if (speed > 0) {
      if (bottomLimitSwitch.get()) {
        // Set Motor Speeds Accordingly
        
      }
      else {
        // Set Motor Speeds Accordingly
      }
    }
  }

  @Override
  public void autonomousInit() {
    ballCamera = new CameraWrapper(true);
  //   SmartDashboard.putNumber("FL angle", 0);
  //   SmartDashboard.putNumber("FR angle", 0);
  //   SmartDashboard.putNumber("BL angle", 0);
  //   SmartDashboard.putNumber("BR angle", 0);
  }

  @Override
  public void autonomousPeriodic() {
    // swerveDrive.frontLeft.printTalon();
    // swerveDrive.frontRight.printTalon();
    // swerveDrive.backLeft.printTalon();
    // swerveDrive.backRight.printTalon();

    // swerveDrive.frontLeft.drive(0.1, SmartDashboard.getNumber("FL angle", 0));
    // swerveDrive.frontRight.drive(0.1, SmartDashboard.getNumber("FR angle", 0));
    // swerveDrive.backLeft.drive(0.1, SmartDashboard.getNumber("BL angle", 0));
    // swerveDrive.backRight.drive(0.1, SmartDashboard.getNumber("BR angle", 0));

    SmartDashboard.putNumber("BallX", ballCamera.getBallX());
    if (Math.abs(ballCamera.getBallX()) < 1) {
      if (ballCamera.getBallX() > .1) {
        swerveDrive.drive(0, 0, -.1, false);
      } else if (ballCamera.getBallX() < -.1) {
        swerveDrive.drive(0, 0, .1, false);
      }
      else {
        swerveDrive.drive(0, -.1, 0, false);
      }
    }
    else {
      swerveDrive.drive(0, 0, 0, false);
    }
  }

  @Override
  public void teleopExit() {
    ballCamera.endCamera();
  }
  
  @Override
  public void autonomousExit() {
    ballCamera.endCamera();
  }

  public void elevatorDepositer(boolean input) {
    //Button to make elevator go up and down (has to stop when the middle limit switch is activated)
    if (input) { //If true, go up
      leftElevatorMotor.set(1);
      rightElevatorMotor.set(-1);
    } else if (input == false) { //If false, go down
      leftElevatorMotor.set(-1);
      rightElevatorMotor.set(1);
    }
  }

}