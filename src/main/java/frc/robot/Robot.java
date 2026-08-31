// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.POSITIONS;

public class Robot extends TimedRobot {
  private final XboxController m_controller = new XboxController(0);
  private final Drivetrain m_mecanum = new Drivetrain();
  private final Arm m_arm = new Arm();
  Constants.POSITIONS armPos = Constants.POSITIONS.STOWED;
  private final Timer m_enabledTimer = new Timer();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(10);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(10);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(10);

  public void autonomousInit() {
        m_enabledTimer.reset();
        m_enabledTimer.start();
        armPos = Constants.POSITIONS.STOWED;
        m_arm.resetArmEncoder();
    }

  @Override
  public void autonomousPeriodic() {
    if(!m_enabledTimer.hasElapsed(0.5)){
      m_arm.runArmVolts(3.5);
      m_arm.setWrist(Constants.POSITIONS.STOWED.wristPos);
    }else if(!m_enabledTimer.hasElapsed(1.75)){
      m_arm.runArmVolts(4.5);
      m_arm.setWrist(Constants.POSITIONS.L2.wristPos);    
    }else if(!m_enabledTimer.hasElapsed(15.0)){
      m_arm.setPosition(Constants.POSITIONS.L3_MID);
    }else if(!m_enabledTimer.hasElapsed(20.0)){
      m_arm.setPosition(Constants.POSITIONS.L3_PLACED);
    }else{
      m_arm.setWrist(Constants.POSITIONS.STOWED.wristPos);
      m_arm.runArmVolts(-3.0);
    }
      
    if(m_enabledTimer.hasElapsed(10.0) && !m_enabledTimer.hasElapsed(14.0)){
      drive(4.0, 0.0, 0.0, false);
    }else if(m_enabledTimer.hasElapsed(16.0) && !m_enabledTimer.hasElapsed(18.5)){
      drive(-3.0, 0.0, 0.0, false);
    }else{
      drive(0.0, 0.0, 0.0, false);
    }
  }

  private double deadbandController(double val){
    if(Math.abs(val) < 0.1){
      return 0.0;
    }
    return val;
  }

  public void teleopInit() {
        // 2. Reset and start the timer when teleop is enabled
        m_enabledTimer.reset();
        m_enabledTimer.start();
        armPos = Constants.POSITIONS.STOWED;
    }

  @Override
  public void teleopPeriodic() {

    if(m_controller.getYButton() && m_controller.getBButton()){
      m_mecanum.resetGyro();
      System.out.println("Resetting Gyro...");
    }

    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed = -m_xspeedLimiter.calculate(deadbandController(m_controller.getLeftY())) * Drivetrain.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed = -m_yspeedLimiter.calculate(deadbandController(m_controller.getLeftX())) * Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot = -m_rotLimiter.calculate(deadbandController(m_controller.getRightX())) * Drivetrain.kMaxSpeed;

    drive(xSpeed * 0.5, ySpeed * 0.5, rot * 0.5, true);

    switch (armPos) {
      case STOWED:
        if(m_controller.getRightBumperButtonPressed()){
          armPos = Constants.POSITIONS.WAIT_COLLECT;
          m_arm.resetArmEncoder();
        }
        break;
      case WAIT_COLLECT:
        if(m_controller.getRightBumperButtonPressed()){
          armPos = Constants.POSITIONS.COLLECT;
        }else if(m_controller.getLeftBumperButtonPressed()){
          armPos = Constants.POSITIONS.GROUND_COLECT;
        }
        break;
      case GROUND_COLECT:
        if(m_controller.getRightBumperButtonPressed()){
          armPos = Constants.POSITIONS.COLLECT;
        }
      case COLLECT:
        if(m_controller.getRightBumperButtonPressed()){
          armPos = Constants.POSITIONS.WAIT_COLLECT;
        }else if(m_controller.getPOV() == 0){
          armPos = Constants.POSITIONS.L3;
        }else if(m_controller.getPOV() == 270){
          armPos = Constants.POSITIONS.L2;
        }else if(m_controller.getPOV() == 180){
          armPos = Constants.POSITIONS.L1;
        }
        break;
      case L1:
        if(m_controller.getXButtonPressed()){
          armPos = POSITIONS.L1_MID;
        }

      case L1_MID:
        if(m_controller.getRightBumperButtonPressed()){
          armPos = Constants.POSITIONS.WAIT_COLLECT;
        }else if(m_controller.getPOV() == 0){
          armPos = Constants.POSITIONS.L3;
        }else if(m_controller.getPOV() == 270){
          armPos = Constants.POSITIONS.L2;
        }else if(m_controller.getPOV() == 180){
          armPos = Constants.POSITIONS.L1;
        }

        break;
      case L2:
        if(m_controller.getXButtonPressed()){
          armPos = POSITIONS.L2_MID;
        }
      case L2_MID:
        if(m_controller.getRightBumperButtonPressed()){
          armPos = Constants.POSITIONS.L2_PLACED;
        }else if(m_controller.getPOV() == 0){
          armPos = Constants.POSITIONS.L3;
        }else if(m_controller.getPOV() == 270){
          armPos = Constants.POSITIONS.L2;
        }else if(m_controller.getPOV() == 180){
          armPos = Constants.POSITIONS.L1;
        }
        
        break;
      case L3:
        if(m_controller.getXButtonPressed()){
          armPos = POSITIONS.L3_MID;
        }
      case L3_MID:
        if(m_controller.getRightBumperButtonPressed()){
          armPos = Constants.POSITIONS.L3_PLACED;
        }else if(m_controller.getPOV() == 0){
          armPos = Constants.POSITIONS.L3;
        }else if(m_controller.getPOV() == 270){
          armPos = Constants.POSITIONS.L2;
        }else if(m_controller.getPOV() == 180){
          armPos = Constants.POSITIONS.L1;
        }

        break;
      case L1_PLACED:
      case L2_PLACED:
      case L3_PLACED:
        if(m_controller.getRightBumperButtonPressed()){
          armPos = Constants.POSITIONS.WAIT_COLLECT;
        }
        
        break;
        
      default:
        break;
    }

    if(m_controller.getLeftBumperButtonPressed() && !armPos.equals(Constants.POSITIONS.WAIT_COLLECT)){
      armPos = Constants.POSITIONS.WAIT_COLLECT;
    }
    if(!armPos.equals(Constants.POSITIONS.STOWED)){
      m_arm.setPosition(armPos);
    }else{
      m_arm.setWrist(Constants.POSITIONS.STOWED.wristPos);
      m_arm.runArmVolts(-5.0);
    }
    
  }

  private void drive(double x, double y, double rot, boolean fieldRelative) {
    m_mecanum.drive(x, y, rot, fieldRelative, getPeriod());
  }
}
