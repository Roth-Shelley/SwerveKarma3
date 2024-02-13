package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.math.CoordinateSystems;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VisionSubsystem;



public class AligntoSpeaker extends Command{
    double visionAngle;
    private final Swerve swervy;
    private final VisionSubsystem vision;
    private final PIDController rotationPID;
    
   







     public AligntoSpeaker(Swerve swerve, VisionSubsystem vision, PIDController rotationPID) {
        this.swervy = swerve;
        this.vision = vision;
        this.rotationPID  = rotationPID;
      
        addRequirements(vision);

        



      

            
        
        
    }

    public void execute() {
        Pose2d currentPose;
        Translation2d translationSpeaker;

        if (!vision.isBlue) {

         currentPose = CoordinateSystems.RightSide_RobToField(swervy.getPose());
        double degrees = currentPose.getRotation().getDegrees();
         translationSpeaker = new Translation2d(8.066, 1.4478);
        


        }
        else {
             currentPose = swervy.getPose();
            double degrees = currentPose.getRotation().getDegrees();
            translationSpeaker = new Translation2d(-8.066, 1.4478);
        }


        


        
        
        Translation2d relativeTranslation = translationSpeaker.minus(currentPose.getTranslation());
        double DegreesPerpendicularHeading = currentPose.getRotation().getDegrees() - Math.atan(relativeTranslation.getY()/relativeTranslation.getX());
        vision.setRotationPID(DegreesPerpendicularHeading);



        }
        
        

        
       
            
             
        



    public void end(boolean interrupted) {
        vision.setRotationPID(0);
    }
           


    }
