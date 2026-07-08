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
  private final Timer m_enabledTimer = new Timer();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(10);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(10);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(10);

  public void autonomousInit() {
        m_enabledTimer.reset();
        m_enabledTimer.start();
    }

  @Override
  public void autonomousPeriodic() {
    if(!m_enabledTimer.hasElapsed(5.0) && m_enabledTimer.hasElapsed(1.0)){
      drive(Drivetrain.kMaxSpeed, 0, 0, false);
    }else if (m_enabledTimer.hasElapsed(6.0) && !m_enabledTimer.hasElapsed(9) ){
      drive(-Drivetrain.kMaxSpeed, 0, -0.75, false);
    }else{
      drive(0, 0, 0, false);
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
    }

  @Override
  public void teleopPeriodic() {
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

    if(m_controller.getYButton() && m_controller.getBButton()){
      m_mecanum.resetGyro();
      System.out.println("Resetting Gyro...");
    }
  }

  private void drive(double x, double y, double rot, boolean fieldRelative) {
    m_mecanum.drive(x, y, rot, fieldRelative, getPeriod());
  }
}
