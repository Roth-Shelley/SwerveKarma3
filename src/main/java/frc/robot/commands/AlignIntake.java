package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VisionSubsystem;



public class AlignIntake extends Command{
    double visionAngle;
    private final Swerve swervy;
    private final VisionSubsystem vision;
    private final PIDController rotationPID;
    private TeleopSwerve swervo;

   







     public AlignIntake(Swerve swerve, VisionSubsystem vision, PIDController rotationPID) {
        this.swervy = swerve;
        this.vision = vision;
        this.rotationPID  = rotationPID;
    
      
        addRequirements(vision);

        



      

            
        
        
    }

    public void execute() {
  
        if (vision.getDetection()[0] != 0 && vision.getDetection()[1] != 0) {
          
            double rotation = rotationPID.calculate(vision.getDetection()[0]);
          
            vision.setRotationPID(rotation);
          

        }
        else {
            
        }
        
        
            
             
        }

    public void initialize() {
       vision.setRotationPID(0);
    }

    

    public void end(boolean interrupted) {
        vision.setRotationPID(0);
    
    }
           


    }



    
