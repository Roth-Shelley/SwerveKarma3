// package frc.robot.subsystems;

// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.ArmConstants;
// import frc.util.InterpolationArmFeedforward;

// import java.text.BreakIterator;

// import com.revrobotics.CANSparkLowLevel;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkPIDController;
// import com.revrobotics.CANSparkBase.ControlType;
// import com.revrobotics.CANSparkBase.IdleMode;
// import com.revrobotics.CANSparkLowLevel.MotorType;

// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.DutyCycleEncoder;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



// public class Arm extends SubsystemBase {


//     ArmFeedforward feedforward;
//     CANSparkMax PivotMotor;
//     CANSparkMax ExtensionMotor;
//     CANSparkMax ReleaseMotor;

//     double lengthBelowPivot;
//     double setpointlengthBelowPivot;

//     double currentAngleFromPosX;
//     double setpointAngleFromPosX;


//     double lastLength;
//     double lastAngle;


//     DutyCycleEncoder PivotEncoder;
//     DutyCycleEncoder ExtensionEncoder;
//     SparkPIDController PID_ExtensionMotor;
//     SparkPIDController PID_PivotMotor;

//     RelativeEncoder PivotMotor_internalEncoder;
//     RelativeEncoder ExtensionMotor_internalEncoder;

//     DigitalInput limitSwitchArmExtension;
//     DigitalInput limitSwitchIntake;
//     DigitalInput limitSwitchPivot;

//     Swerve swervo;
//     ShooterAndRamp shooter;

//     InterpolationArmFeedforward armFeedforward;
//     private final TrapezoidProfile.Constraints m_constraints =
//     new TrapezoidProfile.Constraints(ArmConstants.maxAngularV, ArmConstants.maxAngularA);



//     private final ProfiledPIDController ProfiledPID_PIVOT;


//     private state State;
//     private boolean readyforscoring;
//     private boolean hasScored;
//     private boolean timeUP;
//     private boolean hasIntaked;



   



  

    

//      public Arm(int id_Pivot, int id_Extension, int id_ReleaseMotor, ShooterAndRamp shooter, Swerve swervo) {

//         //counterclockwise --> positive direction for pivot
       
        
//         this.shooter = shooter;
//         this.swervo = swervo;

//         PivotMotor = new CANSparkMax(id_Pivot, CANSparkLowLevel.MotorType.kBrushless);
//         SparkPIDController PID_PivotMotor = PivotMotor.getPIDController();

       
        
//         PID_PivotMotor.setOutputRange(0, 12);


//         SparkPIDController PID_ExtensionMotor = ExtensionMotor.getPIDController();
//         PID_ExtensionMotor.setP(0);
//         PID_ExtensionMotor.setD(0);
//         PID_ExtensionMotor.setOutputRange(0, 12);




//         PivotMotor_internalEncoder = PivotMotor.getEncoder();
//         PivotEncoder = new DutyCycleEncoder(ArmConstants.DIO_pivot);
//         PivotMotor_internalEncoder.setPosition(PivotEncoder.getAbsolutePosition() * ArmConstants.PivotGearRatio);

//         ExtensionMotor = new CANSparkMax(id_Extension, CANSparkLowLevel.MotorType.kBrushless);
//         ExtensionMotor.setInverted(true);
        
//         ExtensionMotor_internalEncoder = ExtensionMotor.getEncoder();
//         ExtensionMotor_internalEncoder.setPosition(0);

        


        
//         ReleaseMotor = new CANSparkMax(id_ReleaseMotor, MotorType.kBrushless);
//         lengthBelowPivot = 0;

//         limitSwitchIntake = new DigitalInput(ArmConstants.DIO_Intake_LIMITSWITCH);
//         limitSwitchArmExtension = new DigitalInput(ArmConstants.DIO_Extension_LIMITSWITCH);
//         limitSwitchPivot = new DigitalInput(ArmConstants.DIO_Pivot_LIMITSWITCH);



//          armFeedforward = new InterpolationArmFeedforward();

        
        

//     }

//     @Override
//     public void periodic() {

//         stateUpdate();
        
//        if (State == state.HOME) {
//         setpointlengthBelowPivot = 0;
//         if (Math.abs(setpointlengthBelowPivot - lengthBelowPivot) < ArmConstants.LENGTHTOLERANCE) {
//         setpointAngleFromPosX = ArmConstants.HOMEANGLE;
//         }
        


//        } 

//        if (State == state.INTAKING) {

        
      
//         setpointAngleFromPosX = ArmConstants.INTAKEANGLE;

    

//      if (Math.abs(setpointAngleFromPosX - currentAngleFromPosX) < ArmConstants.ANGLETOLERANCE) {
//             setpointlengthBelowPivot = ArmConstants.LENGTHINTAKE;
//             if (!hasIntaked) {
//                 hasIntaked = true;    
//                 intakeObject();
                
//             }
            

//         }
//     }
//     if (State == state.SCORING) {
//         setpointAngleFromPosX = ArmConstants.SCORINGANGLE;
//          if (Math.abs(setpointAngleFromPosX - currentAngleFromPosX) < ArmConstants.ANGLETOLERANCE) {
//             setpointlengthBelowPivot = ArmConstants.LENGTHSCORING;
            

//         }
                


                




//     }

//     setAngle();
//     setLength();

//        }
     



//         // if (lastAngle != setpointAngleFromPosX) {
//         //     setAngle();
            

//         // }

//         // if (lastLength != setpointlengthBelowPivot) {
//         //     setLength();

//         // }

//         // lastAngle = setpointAngleFromPosX;
//         // lastLength = setpointlengthBelowPivot;


        


//         // PivotMotor.setVoltage(
//         //     ProfiledPID_PIVOT.calculate(setpointAngleFromPosX - currentAngleFromPosX)
//         //         + armFeedforward.calculate(ProfiledPID_PIVOT.getSetpoint().velocity));

      
    

        

       




        
        
        
    

//     enum state {
//         INTAKING,
//         HOME,
//         SCORING
        
//       }

    


//     public void stateUpdate() {
//         if (hasNote() && State != state.HOME) {
//             State = state.HOME;
//         }
//         else if (State == state.HOME && .State == state.AMP && !hasNote()) {
//             State = state.INTAKING;

//         }
//         else if (State == state.HOME && readyforscoring)  {
//             State = state.SCORING;
            
//         }

//         else if (State == state.SCORING && !hasNote()) {
//             State = state.HOME;
//         }

//         else if (State == state.INTAKING && timeUP) {
//             State = state.HOME;
//             timeUP = false;
//         }



      


//     }

        

            

         
    


//     public boolean hasNote() {
//         return limitSwitchIntake.get();
//     }

   

  
 

    

//     public void setLength() {

//         if (lastLength != setpointlengthBelowPivot) {
        
//         double RotationstoComplete  = ArmConstants.PIVOT_GEARING * (setpointlengthBelowPivot -  lengthBelowPivot) / (2 * Math.PI * ArmConstants.radius_sprocket_Extension);
//         PID_ExtensionMotor.setReference(RotationstoComplete, ControlType.kPosition);

//         lastLength = setpointlengthBelowPivot;
//     }
// }

//     public double getLength() {

//         return currentAngleFromPosX;
        

//     }

//     public void setAngle() {

//         if (lastAngle != setpointAngleFromPosX) {
//            ProfiledPID_PIVOT =
//             new ProfiledPIDController(lengthBelowPivot * ArmConstants.PEXTENSION, 0, ArmConstants.DEXTENSION, m_constraints, 0.02);
//         }

       
//         armFeedforward.updateFeedforwardRotation(getAngle(), ProfiledPID_PIVOT.getSetpoint().velocity);
//         armFeedforward.updateLengthbelowPivot(getLength());
//         PivotMotor.setVoltage(ProfiledPID_PIVOT.calculate(setpointAngleFromPosX - currentAngleFromPosX) + armFeedforward.CalculateFeedforward());

//         lastAngle = setpointAngleFromPosX;
//     }

//     public double getAngle() {
//         return lengthBelowPivot;

//     }

   
    
//     public void releaseObject() {
//         ReleaseMotor.set(0.5);
//         double initTime = Timer.getFPGATimestamp();
//         while (limitSwitchIntake.get() || Timer.getFPGATimestamp() - initTime > 3) {
//             wait(5);
           
//         }
        

//     }

//     public boolean intakeObject() {
//         ReleaseMotor.set(-0.5);
//         double initTime = Timer.getFPGATimestamp();
//         while (!limitSwitchIntake.get() || Timer.getFPGATimestamp() - initTime > 3) {
//             wait(5);
           
//         }
//         timeUP = true;
        

        


//     }

   

// }
    

