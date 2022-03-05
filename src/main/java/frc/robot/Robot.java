// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.net.ssl.CertPathTrustManagerParameters;
import javax.xml.transform.SourceLocator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

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
import frc.robot.JoystickWrapper;
import frc.robot.NavXWrapper;

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
  private final Solenoid leftIntakeSolenoid;
  private final Solenoid rightIntakeSolenoid;
  private final Solenoid leftElevatorSolenoid;
  private final Solenoid rightElevatorSolenoid;

  
  States state;
  private WPI_TalonSRX leftMotor;
  private WPI_TalonSRX leftMotor2;
  private WPI_TalonSRX rightMotor;
  private WPI_TalonSRX rightMotor2;
  private JoystickWrapper leftStick;
  private XboxControllerWrapper xboxController;
  private double leftAxis;
  private double rightAxis;

  private boolean depositButton;
  private boolean elevatorUpButton;
  private boolean elevatorDownButton;
  private boolean climbButton;
  private boolean dropBall;

  //private NavX navx;

  //Camera topCam;
  //Camera bottomCam;
  //Camera ballCamera;
  SwerveDrive swerveDrive;

  //Lasershark shark;

  //Set Controller Type
  int controllerType;

  private WPI_TalonSRX intakeMotor;
  private WPI_TalonSRX leftDepositorMotor;
  private WPI_TalonSRX rightDepositorMotor;
  private TalonFX leftElevatorMotor; //to go up go clockwise
  private TalonFX rightElevatorMotor; //to go up go counter-clockwise

  



  boolean turnTo = false;

  public Robot() {
    super(0.03);
    //create variables
    leftStick = new JoystickWrapper(0);
    xboxController = new XboxControllerWrapper(0);
    //topCam = new Camera();
    //bottomCam = new Camera();
    //Ball = new Ball();
    state = States.MANUAL;
    swerveDrive = new SwerveDrive();

    //navx = new NavX();

    //shark = new Lasershark(0);

    leftIntakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, RobotConstants.leftIntakeSolenoidID);
    rightIntakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, RobotConstants.rightIntakeSolenoidID);
    leftElevatorSolenoid = new Solenoid(PneumaticsModuleType.REVPH, RobotConstants.leftElevatorSolenoidID);
    rightElevatorSolenoid = new Solenoid(PneumaticsModuleType.REVPH, RobotConstants.rightElevatorSolenoidID);

    intakeMotor = new WPI_TalonSRX(RobotConstants.intakeMotorID);
    leftDepositorMotor = new WPI_TalonSRX(RobotConstants.leftDepositorMotorID);
    rightDepositorMotor = new WPI_TalonSRX(RobotConstants.rightDepositorMotorID);
    leftElevatorMotor = new TalonFX(RobotConstants.leftElevatorMotorID);
    rightElevatorMotor =new TalonFX(RobotConstants.rightElevatorMotorID);
  }


  @Override
  public void robotInit() {
    //set to defaults
    autonomousInit();
  }

  @Override
  public void teleopInit() {
    leftIntakeSolenoid.set(false);
    rightIntakeSolenoid.set(false);
    leftElevatorSolenoid.set(false);
    rightElevatorSolenoid.set(false);
  }
  

  @Override
  public void teleopPeriodic() {
    double xAxis;
    double yAxis;
    double rAxis;



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
    }
    else if (controllerType == 1) {
      xAxis = leftStick.getRawAxis(0);
      yAxis = leftStick.getRawAxis(1);
      rAxis = leftStick.getRawAxis(3);
      depositButton = leftStick.getRawButton(0);
      elevatorUpButton = leftStick.getRawButton(1);
      climbButton = leftStick.getRawButton(2);
      elevatorDownButton = leftStick.getRawButton(3);
    }
    else {
      xAxis = 0;
      yAxis = 0;
      rAxis = 0;
    }
    
    // Setting speed of depositor motors
    if (depositButton == true) {
      leftDepositorMotor.set(0.5);
      rightDepositorMotor.set(-0.5);
      intakeMotor.set(0.5);
    }
    else if (xboxController.getBButton() == true) {
      leftDepositorMotor.set(-0.5);
      rightDepositorMotor.set(0.5);
    }
    else {
      leftDepositorMotor.set(0);
      rightDepositorMotor.set(0);
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
    
    swerveDrive.drive(-xAxis, yAxis, rAxis);

    

    /*
    switch(state) {
      case MANUAL:
        manualControl();
        break;

      case AUTONOMOUS:
        
        //autonomousPeriodic();
        state = States.DETECT_BALL;
        break;
      case DETECT_BALL:
        detectBall();
        break;
      case GO_TO_HUMAN:
        goToHuman();
        break;
      case MOVE_TO_BALL:
        moveToBall();
        break;
      case PICK_UP_BALL:
        pickUpBall();
        break;
      case GO_TO_HUB:
        goToHub();
        break;
      case DROP_BALL:
        dropBall();
        break;
    }*/
  }
  /*
  public void manualControl() {
    if (button1) {
      leftMotor.set(0.3);
      leftMotor2.set(0.3);
      rightMotor.set(-0.3);
      rightMotor2.set(-0.3);
    }
    else if (button2) {
      leftMotor.set(-0.3);
      leftMotor2.set(-0.3);
      rightMotor.set(0.3);
      rightMotor2.set(0.3);
    }
    else if (button3) {
      leftMotor.set(0);
      leftMotor2.set(0);
      rightMotor.set(0);
      rightMotor2.set(0);
    }
    if (leftAxis == -1) {
      leftMotor.set(-0.1);
      leftMotor2.set(-0.1);
    }
    else if (leftAxis < -0.5) {
      leftMotor.set(0.1);
      leftMotor2.set(0.1);
    }
    else if (leftAxis > -0.5) {
      leftMotor.set(0);
      leftMotor.set(0);
    }
    else if (leftAxis == 1) {
      rightMotor.set(0.1);
      rightMotor2.set(0.1);
    }
    else if (leftAxis > 0.5) {
      rightMotor.set(-0.1);
      rightMotor2.set(-0.1);
    }
    else if (leftAxis > 0) {
      rightMotor.set(0);
      rightMotor2.set(0);
    }
    else if (leftAxis == 0) {
      leftMotor.set(0.3);
      leftMotor2.set(0.3);
      rightMotor.set(-0.3);
      rightMotor2.set(-0.3);
    }
  }*/
  /*
  public void detectBall() {

    if (findBall()) {
      state = States.MOVE_TO_BALL;
    }
    else {
      state = States.GO_TO_HUMAN;
    }
  }
  public void goToHuman() {
    
    
  }

  public void moveToBall() {
    if (reachedBall()) {
      state = States.PICK_UP_BALL;
    }
    else {
      state = States.DETECT_BALL;
    }
  }

  public void pickUpBall() {
    if (ballPickedUp()) {
      state = States.GO_TO_HUB;
    }
    else {
      state = States.DETECT_BALL;
    }
  }

  public void goToHub() {
    if (reachedHub()) {
      state = States.DROP_BALL;
    }
  }

  public void dropBall() {
    if (ballDropped()) {
      state = States.DETECT_BALL;
    }
  }
  int count = 0;
  public boolean findBall(){
    count += 1;
    int turn = topCam.isBallPresent();
    if (turn == 0) {
      return true;
    }
    else if (turn > 0) {
      swerveDrive.turn_right(0.1);
    }
    else {
      swerveDrive.turn_left(0.1);
    }
    if (count > 1000) {
      state = States.GO_TO_HUMAN;
    }
    return false;
  }
  public boolean reachedBall(){
    int distanceToBall;
    if (distanceToBall == 0){
      return true;
    }
    return false;
  }
  public boolean ballPickedUp(){
    int pickedUp = bottomCam.isBallPresent();
    if (pickedUp == 0){
      return true;
    }
    return false;
  }
  public boolean reachedHub(){
    int distanceToHub;
    if (distanceToHub == 0) {
      return true;
    }
    return false; 
  }
  public boolean ballDropped(){
    boolean hasBall;
    if (hasBall == false) {
      return true;
    }
    return false;
  }
  */
  
  @Override
  public void autonomousInit() {
    
  }

  @Override
  public void autonomousPeriodic() {
  }
}