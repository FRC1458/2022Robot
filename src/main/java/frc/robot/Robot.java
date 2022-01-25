// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.net.ssl.CertPathTrustManagerParameters;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.Ultrasonic;


//class Camera;
/**
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

  States state;
  private WPI_TalonSRX leftMotor;
  private WPI_TalonSRX leftMotor2;
  private WPI_TalonSRX rightMotor;
  private WPI_TalonSRX rightMotor2;
  private Joystick leftStick;
  private boolean button1;
  private boolean button2;
  private boolean button3;
  private float leftAxis;
  Camera topCam;
  Camera bottomCam;
  Camera ballCamera;
  Ball Ball;
  Movement swerveDrive;
  Movement leftFrontWheel;
  Movement leftBackWheel;
  Movement rightFrontWheel;
  Movement rightBackWheel;
  
  public Robot() {
    super(0.03);
    //create variables
    leftMotor = new WPI_TalonSRX(RobotConstants.leftMotorID);
    leftMotor2 = new WPI_TalonSRX(RobotConstants.leftMotor2ID);
    rightMotor = new WPI_TalonSRX(RobotConstants.rightMotorID);
    rightMotor2 = new WPI_TalonSRX(RobotConstants.rightMotor2ID);
    leftStick = new Joystick(0);
    topCam = new Camera();
    bottomCam = new Camera();
    Ball = new Ball();
    swerveDrive  = new Movement();
    leftFrontWheel = new Movement();
    leftBackWheel = new Movement();
    rightBackWheel = new Movement();
    rightFrontWheel = new Movement();
    state = States.MANUAL;
  }

  @Override
  public void robotInit() {
    //set to defaults
    autonomousInit();

  }

  @Override
  public void teleopInit() {
    //set defaults for teleop
  }

  @Override
  public void teleopPeriodic() {
    button1 = leftStick.getRawButton(1);
    button2 = leftStick.getRawButton(2);
    button3 = leftStick.getRawButton(3);
    leftAxis = leftStick.getRawAxis(1)
    switch(state) {
      case MANUAL:
        manualControl();
        break;

      case AUTONOMOUS:
        
        //autonomousPeriodic();
        state = State.DETECT_BALL;
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
    }
  }

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
}

  public void detectBall() {

    if (findBall()) {
      state = State.MOVE_TO_BALL;
    }
    else {
      state = State.GO_TO_HUMAN;
    }
  }
  public void goToHuman() {


    }
  }
  public void moveToBall() {
    if (reachedBall()) {
      state = State.PICK_UP_BALL;
    }
    else {
      state = State.DETECT_BALL;
    }
  }

  public void pickUpBall() {
    if (ballPickedUp()) {
      state = State.GO_TO_HUB;
    }
    else {
      state = State.DETECT_BALL;
    }
  }

  public void goToHub() {
    if (reachedHub()) {
      state = State.DROP_BALL;
    }
  }

  public void dropBall() {
    if (ballDropped()) {
      state = State.DETECT_BALL;
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
      state = State.GO_TO_HUMAN;
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
  
  @Override
  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic() {
    
  }
}