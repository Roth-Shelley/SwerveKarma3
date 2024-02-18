// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.commands.AligntoSpeaker;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Intake;
import frc.robot.commands.AlignIntake;
import frc.robot.commands.Climb;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.Climber;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.ShooterAndRamp;
//import frc.robot.commands.Auto.AutoPaths.autoChooser;
//import frc.robot.subsystems.Intake;
//import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Swerve;
//import frc.robot.subsystems.TelescopicArm;


public class RobotContainer {
  public TeleopSwerve tele;
  private final XboxController driver = new XboxController(0);
  private final XboxController coDriver = new XboxController(1);
  
 // private final Intake m_intake = new Intake();

//  private final LEDs leds = new LEDs(m_intake);
private final VisionSubsystem vision = new VisionSubsystem();
private final Swerve s_Swerve = new Swerve(vision);
//private final Intake s_intake = new Intake();
//private final ShooterAndRamp m_ShooterAndRamp = new ShooterAndRamp();
//private final Climb s_climb = new Climb(coDriver);
 
  
 // private final TelescopicArm m_arm = new TelescopicArm();
 // private final autoChooser chooser = new autoChooser(m_arm, m_intake, s_Swerve);
  public RobotContainer() {
    tele =  new TeleopSwerve(s_Swerve, driver, () -> !driver.getLeftBumper());


    s_Swerve.setDefaultCommand(tele);
    //Climber BOOMBOOM = new Climber();
    //BOOMBOOM.setDefaultCommand(new Climb(driver));
    
    configureBindings();
  }

  private void configureBindings() {
    AlignIntake aligner = new AlignIntake(s_Swerve, driver,  () -> !driver.getLeftBumper(), vision, new PIDController(1.3, 0, 0));
    AligntoSpeaker bumba= new AligntoSpeaker(s_Swerve, vision, new PIDController(0.03, 0, 0), coDriver, () -> !driver.getLeftBumper());


   new Trigger(driver::getXButton).whileTrue(bumba);
   new Trigger(driver::getAButton).onTrue(runOnce(s_Swerve::resetEveything));
    
    
  }

  //public ShooterAndRamp getShooterAndRamp() {
    //return m_ShooterAndRamp;
  //}


//    public Command getAutonomousCommand() {
//      PathPlannerPath path = PathPlannerPath.fromPathFile("trial path");

//         // Create a path following command using AutoBuilder. This will also trigger event markers.
//       return AutoBuilder.followPath(path);
//      return 
//    }
// }
}