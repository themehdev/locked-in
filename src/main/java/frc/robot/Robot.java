// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

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

  private final boolean LEFT_SIDE_AUTO = false;

  private final double LEFT_SIDE_AUTO_DELAY = 4;

  private final double AUTO_DELAY_MULTIPLIER = 7.5/(Drivetrain.kMaxSpeed - 0.75);

  public void autonomousInit() {
        m_enabledTimer.reset();
        m_enabledTimer.start();
        armPos = Constants.POSITIONS.STOWED;
        //m_arm.resetArmEncoder();
    }

  @Override
  public void autonomousPeriodic() {

    if(LEFT_SIDE_AUTO){
      // if(!m_enabledTimer.hasElapsed((4.0 + LEFT_SIDE_AUTO_DELAY)*AUTO_DELAY_MULTIPLIER) && m_enabledTimer.hasElapsed((LEFT_SIDE_AUTO_DELAY)*AUTO_DELAY_MULTIPLIER)){
      //   drive(Drivetrain.kMaxSpeed, 0, 0, false);
      // }else if (m_enabledTimer.hasElapsed((10.0 + LEFT_SIDE_AUTO_DELAY)*AUTO_DELAY_MULTIPLIER) && !m_enabledTimer.hasElapsed((13.0 + LEFT_SIDE_AUTO_DELAY)*AUTO_DELAY_MULTIPLIER) ){
      //   drive(-Drivetrain.kMaxSpeed, 0, -1.25 * 1/AUTO_DELAY_MULTIPLIER, false);
      // }else{
      //   drive(0, 0, 0, false);
      // }
    }else{
      // if(!m_enabledTimer.hasElapsed(1.3*AUTO_DELAY_MULTIPLIER)){
      //   drive(0, -Drivetrain.kMaxSpeed, 0, false);
      // }else if (!m_enabledTimer.hasElapsed(5.5*AUTO_DELAY_MULTIPLIER)){
      //   drive(Drivetrain.kMaxSpeed, 0, 0.5 * 1/AUTO_DELAY_MULTIPLIER, false);
      // }else if (!m_enabledTimer.hasElapsed(6.2*AUTO_DELAY_MULTIPLIER)){
      //   drive(0, 0, -Drivetrain.kMaxSpeed, false);
      // }else if (!m_enabledTimer.hasElapsed(7.0*AUTO_DELAY_MULTIPLIER)){
      //   drive(Drivetrain.kMaxSpeed, 0, 0, false);
      // }else if (!m_enabledTimer.hasElapsed(10.0*AUTO_DELAY_MULTIPLIER)){
      //   drive(-Drivetrain.kMaxSpeed * 0.8, -Drivetrain.kMaxSpeed * 0.25, 0, false);
      // }else{
      //   drive(0, 0, 0, false);
      // }
    }
    

    //m_mecanum.updateOdometry();
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
        //TODO: REMOVE LATER
        armPos = Constants.POSITIONS.STOWED;
        //m_arm.resetArmEncoder();
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

    drive(xSpeed, ySpeed, rot, true);

    switch (armPos) {
      case STOWED:
        if(m_controller.getRightBumperButtonPressed()){
          armPos = Constants.POSITIONS.WAIT_COLLECT;
        }
        break;
      case WAIT_COLLECT:
        if(m_controller.getRightBumperButtonPressed()){
          armPos = Constants.POSITIONS.COLLECT;
        }
        break;
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
        if(m_controller.getRightBumperButtonPressed()){
          armPos = Constants.POSITIONS.L1_PLACED;
        }else if(m_controller.getPOV() == 0){
          armPos = Constants.POSITIONS.L3;
        }else if(m_controller.getPOV() == 270){
          armPos = Constants.POSITIONS.L2;
        }else if(m_controller.getPOV() == 180){
          armPos = Constants.POSITIONS.L1;
        }

        break;
      case L2:
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

    m_arm.setPosition(armPos);

  }

  private void drive(double x, double y, double rot, boolean fieldRelative) {
    m_mecanum.drive(x, y, rot, fieldRelative, getPeriod());
  }
}
