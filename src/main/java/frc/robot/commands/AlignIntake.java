// package frc.robot.commands;

// import java.util.function.DoubleSupplier;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.PIDCommand;
// import frc.robot.subsystems.Swerve;
// import frc.robot.subsystems.VisionSubsystem;



// public class AlignIntake extends Command{
//     double visionAngle;
//     private final Swerve swervy;
//     private final VisionSubsystem vision;
//     private final PIDController rotationPID;
//     private TeleopSwerve swervo;

   







//      public AlignIntake(Swerve swerve, VisionSubsystem vision, PIDController rotationPID) {
//         this.swervy = swerve;
//         this.vision = vision;
//         this.rotationPID  = rotationPID;
    
      
//         addRequirements(vision);

        



      

            
        
        
    // }

    // public void execute() {
  
    //     if (vision.getDetection()[0] != 0 && vision.getDetection()[1] != 0) {
          
    //         double rotation = rotationPID.calculate(vision.getDetection()[0]);
          
    //         vision.setRotationPID(rotation);
          

    //     }
    //     else {
            
    //     }
        
        
            
             
    //     }

    // public void initialize() {
    //    vision.setRotationPID(0);
    // }

    

    // public void end(boolean interrupted) {
    //     vision.setRotationPID(0);
    
    // }
           


    // }


    package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ShooterAndRamp;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class AlignIntake extends Command {

    
    private Translation2d translation;
    private BooleanSupplier fieldRelative;
    private VisionSubsystem Vision;
    private double rotation;
    private ShooterAndRamp shooterAndRamp;
  
    
    private Swerve s_Swerve;
    
    private XboxController controller;
    SlewRateLimiter limiter = new SlewRateLimiter(3);
    double maxSpeed = 1;
    private PIDController PIDController;
    

    public AlignIntake(Swerve s_Swerve, XboxController controller, BooleanSupplier fieldRelative, VisionSubsystem vision, PIDController rotationPID) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        this.fieldRelative = fieldRelative;
        this.controller = controller;
        this.Vision = vision;
        this.PIDController = rotationPID;
        this.shooterAndRamp = shooterAndRamp;

    }


   
    @Override
    public void execute() {
        if (Vision.getDetection()[0] != 0 && Vision.getDetection()[1] != 0) {
          
     rotation = PIDController.calculate(Vision.getDetection()[0]);
     SmartDashboard.putNumber("visionX", Vision.getDetection()[0]);
           SmartDashboard.putString("did the alignintake get a detection", "yessir");
           
     

    
          
             

         }
         else {
             rotation = 0;
     
            SmartDashboard.putString("did the alignintake get a detection", "nope lmao u suck");
         }

        boolean isFieldRelative = fieldRelative.getAsBoolean();
      

         double yAxis = -controller.getLeftY();
         double xAxis = -controller.getLeftX();
    
      

        yAxis = Math.copySign(yAxis * yAxis, Math.signum(yAxis));
        xAxis = Math.copySign(xAxis * xAxis, Math.signum(xAxis));


        yAxis = (Math.abs(yAxis) < Constants.stickDeadband) ? 0 : yAxis;
        xAxis = (Math.abs(xAxis) < Constants.stickDeadband) ? 0 : xAxis;
    
        translation = new Translation2d(xAxis, yAxis);
        if (rotation > Constants.Swerve.maxAngularVelocity) {
            rotation = Constants.Swerve.maxAngularVelocity;
        }
        else if (rotation < -Constants.Swerve.maxAngularVelocity) {
            rotation = -Constants.Swerve.maxAngularVelocity;
        }
      SmartDashboard.putNumber("rotation setting in alignintake", rotation);
      //SmartDashboard.putString("State of ramp", shooterAndRamp.whichState().toString());
  
        s_Swerve.drive(translation.times(0.65), -rotation, isFieldRelative);
    }
}



    
