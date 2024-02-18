
package frc.robot.commands;

// import java.util.function.DoubleSupplier;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.PIDCommand;
// import frc.math.CoordinateSystems;
// import frc.robot.subsystems.Swerve;
// import frc.robot.subsystems.VisionSubsystem;



// public class AligntoSpeaker extends Command{
//     double visionAngle;
//     private final Swerve swervy;
//     private final VisionSubsystem vision;
//     private final PIDController rotationPID;
    
   







//      public AligntoSpeaker(Swerve swerve, VisionSubsystem vision, PIDController rotationPID) {
//         this.swervy = swerve;
//         this.vision = vision;
//         this.rotationPID  = rotationPID;
      
//         addRequirements(vision);

        



      

            
        
        
//     }

//     public void execute() {
//         Pose2d currentPose;
//         Translation2d translationSpeaker;

//         if (!vision.isBlue) {

//          currentPose = CoordinateSystems.RightSide_RobToField(swervy.getPose());
//         double degrees = currentPose.getRotation().getDegrees();
//          translationSpeaker = new Translation2d(8.066, 1.4478);
        


//         }
//         else {
//              currentPose = swervy.getPose();
//             double degrees = currentPose.getRotation().getDegrees();
//             translationSpeaker = new Translation2d(-8.066, 1.4478);
//         }


        


        
        
//         Translation2d relativeTranslation = translationSpeaker.minus(currentPose.getTranslation());
//         double DegreesPerpendicularHeading = currentPose.getRotation().getDegrees() - Math.atan(relativeTranslation.getY()/relativeTranslation.getX());
//         vision.setRotationPID(DegreesPerpendicularHeading);



//         }
        
        

        
       
            
             
    

    // public void end(boolean interrupted) {
    //     vision.setRotationPID(0);
    // }
           


    // } 
// package frc.robot.commands;

// import frc.math.CoordinateSystems;
// import frc.robot.Constants;
// import frc.robot.subsystems.Swerve;
// import frc.robot.subsystems.VisionSubsystem;
// import edu.wpi.first.wpilibj.DataLogManager;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import java.util.function.BooleanSupplier;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.filter.SlewRateLimiter;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj2.command.Command;


// public class AligntoSpeaker extends Command {

//     Pose2d currentPose;
//     Translation2d translationSpeaker;

    
//     private Translation2d translation;
//     private BooleanSupplier fieldRelative;
//     private VisionSubsystem Vision;
//     private double rotation;
  
    
//     private Swerve s_Swerve;
//     private XboxController controller;
//     SlewRateLimiter limiter = new SlewRateLimiter(3);
//     double maxSpeed = 1;
//     private PIDController PIDController;
//     boolean isFieldRelative;
    

//     public AligntoSpeaker(Swerve s_Swerve, XboxController controller, BooleanSupplier fieldRelative, VisionSubsystem vision, PIDController rotationPID) {
//         this.s_Swerve = s_Swerve;
//         addRequirements(s_Swerve);
//         this.fieldRelative = fieldRelative;
//         this.controller = controller;
//         this.Vision = vision;
//         this.PIDController = rotationPID;
//          isFieldRelative = fieldRelative.getAsBoolean();
       
//     }


   
//     @Override
//     public void execute() {
//                 if (!Vision.isBlue) {

//          currentPose = CoordinateSystems.RightSide_RobToField(s_Swerve.getPose());
//         double degrees = currentPose.getRotation().getDegrees();
//          translationSpeaker = new Translation2d(8.066, 1.4478);
        


//         }
//         else {
//              currentPose = s_Swerve.getPose();
//             double degrees = currentPose.getRotation().getDegrees();
//             translationSpeaker = new Translation2d(-8.066, 1.4478);
//         }


        


        
        
//         Translation2d relativeTranslation = translationSpeaker.minus(currentPose.getTranslation());
//         double DegreesPerpendicularHeading = currentPose.getRotation().getDegrees() - Math.atan(relativeTranslation.getY()/relativeTranslation.getX());
//         Vision.setRotationPID(DegreesPerpendicularHeading);

       

    
          
             

     
//          double yAxis = -controller.getLeftY();
//          double xAxis = -controller.getLeftX();
    
      

//         yAxis = Math.copySign(yAxis * yAxis, Math.signum(yAxis));
//         xAxis = Math.copySign(xAxis * xAxis, Math.signum(xAxis));


//         yAxis = (Math.abs(yAxis) < Constants.stickDeadband) ? 0 : yAxis;
//         xAxis = (Math.abs(xAxis) < Constants.stickDeadband) ? 0 : xAxis;
    
//         translation = new Translation2d(xAxis, yAxis);
//         if (rotation > Constants.Swerve.maxAngularVelocity) {
//             rotation = Constants.Swerve.maxAngularVelocity;
//         }
//         else if (rotation < -Constants.Swerve.maxAngularVelocity) {
//             rotation = -Constants.Swerve.maxAngularVelocity;
//         }
//       SmartDashboard.putNumber("rotation setting in alignintake", rotation);

  

//         s_Swerve.drive(translation.times(0.65), -rotation, isFieldRelative);
//     }
// }

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.PipelineConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VisionSubsystem;

public class AligntoSpeaker extends Command {
    double visionAngle;
    private final Swerve swervy;
    private final VisionSubsystem vision;
    private final PIDController rotationPID;
    private double tx;
    private double PidSetpoint;
    private final XboxController xbox;
    private final BooleanSupplier fieldrelative;
    private  boolean isfieldRelative;


public AligntoSpeaker(Swerve swerve, VisionSubsystem vision, PIDController rotationPID, XboxController xbox, BooleanSupplier fieldRelative) {
    this.swervy = swerve;
    this.vision = vision;
    this.rotationPID = rotationPID;
    this.xbox = xbox;
    this.fieldrelative = fieldRelative;

    addRequirements(swervy);

    tx = 0;
    
}

public void execute() {
     isfieldRelative = fieldrelative.getAsBoolean();
     tx = vision.getSpeakerDetection();
     SmartDashboard.putNumber("tx speaker", tx);
    if (tx != 10000) {

     
     PidSetpoint = rotationPID.calculate(tx);
     isfieldRelative = fieldrelative.getAsBoolean();
    SmartDashboard.putBoolean("is aligning to speaker", true);
     }   
     
     else  { 
        SmartDashboard.putBoolean("is aligning to speaker", false);

         PidSetpoint = 0;
        isfieldRelative = fieldrelative.getAsBoolean();
     }
         double yAxis = -xbox.getLeftY();
         double xAxis = -xbox.getLeftX();
    
      

        yAxis = Math.copySign(yAxis * yAxis, Math.signum(yAxis));
        xAxis = Math.copySign(xAxis * xAxis, Math.signum(xAxis));


        yAxis = (Math.abs(yAxis) < Constants.stickDeadband) ? 0 : yAxis;
        xAxis = (Math.abs(xAxis) < Constants.stickDeadband) ? 0 : xAxis;
    
        Translation2d translation = new Translation2d(xAxis, yAxis);
        if (PidSetpoint > Constants.Swerve.maxAngularVelocity) {
            PidSetpoint = Constants.Swerve.maxAngularVelocity;
        }
        else if (PidSetpoint < -Constants.Swerve.maxAngularVelocity) {
            PidSetpoint = -Constants.Swerve.maxAngularVelocity;
        }
     // SmartDashboard.putNumber("rotation setting in alignintake", rotation);
     // SmartDashboard.putString("State of ramp", shooterAndRamp.whichStateAreWeCurrentlyIn().toString());
  SmartDashboard.putBoolean("we are driving", true);
  SmartDashboard.putNumber("PID SETPOINT SPEAKER", PidSetpoint);
        swervy.drive(translation.times(0.65), -PidSetpoint, isfieldRelative);
     



}
}





    

