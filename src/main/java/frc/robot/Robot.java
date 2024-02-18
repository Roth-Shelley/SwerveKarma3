// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//testing1

public class Robot extends TimedRobot {
  public static CTREConfigs ctreConfigs;

  XboxController controller = new XboxController(2);
  private Command m_autonomousCommand;
  
  private RobotContainer m_robotContainer;

  SendableChooser rampStateChooser;

  @Override
  public void robotInit() {
    CameraServer.startAutomaticCapture();
    
    ctreConfigs = new CTREConfigs();
    m_robotContainer = new RobotContainer();

    rampStateChooser = new SendableChooser();
    rampStateChooser.setDefaultOption("HOME", controller);
    rampStateChooser.addOption("SHOOTING", controller);
    rampStateChooser.addOption("AMPING", controller);
    rampStateChooser.addOption("HANDOFF_ARM", controller);
    SmartDashboard.putData("Ramp State", rampStateChooser);

  }

  @Override
  public void robotPeriodic() {
    
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
   // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
       m_autonomousCommand.cancel();
     }
     
  }

  @Override
  public void teleopPeriodic() {

    if(rampStateChooser != null) {
      // if(rampStateChooser.getSelected().equals(whichState(m_robotContainer.m_ShooterAndRamp.currentState))) {
       // m_robotContainer.getShooterAndRamp().executeRampState(rampStateChooser.getSelected().toString());
      // }
    }


  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

  }
}
