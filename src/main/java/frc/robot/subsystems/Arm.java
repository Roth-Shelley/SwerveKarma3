// package frc.robot.subsystems;

// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.OperatorConstants.ArmConstants;
// import frc.robot.util.InterpolationArmFeedforward;

// import java.text.BreakIterator;

// import com.revrobotics.CANSparkLowLevel;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkPIDController;
// import com.revrobotics.CANSparkBase.ControlType;
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

//     InterpolationArmFeedforward armFeedforward;
//     private final TrapezoidProfile.Constraints m_constraints =
//     new TrapezoidProfile.Constraints(ArmConstants.maxAngularV, ArmConstants.maxAngularA);



//     private final ProfiledPIDController ProfiledPID_PIVOT;


//     state State;



   



  

    

//      public arm(int id_Pivot, int id_Extension, int id_ReleaseMotor) {

//         //counterclockwise --> positive direction for pivot
       
        


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
//         ExtensionEncoder = new DutyCycleEncoder(ArmConstants.DIO_ExtensionEncoder);
//         ExtensionMotor_internalEncoder = ExtensionMotor.getEncoder();
//         ExtensionMotor_internalEncoder.setPosition(0);

        


        
//         ReleaseMotor = new CANSparkMax(id_ReleaseMotor, MotorType.kBrushless);
//         lengthBelowPivot = ArmConstants.lengthInitial;

//         limitSwitchIntake = new DigitalInput(ArmConstants.DIO_Intake_LIMITSWITCH);
//         limitSwitchArmExtension = new DigitalInput(ArmConstants.DIO_Extension_LIMITSWITCH);
//         limitSwitchPivot = new DigitalInput(ArmConstants.DIO_Pivot_LIMITSWITCH);



//          armFeedforward = new InterpolationArmFeedforward();

        
        

//     }

//     @Override
//     public void periodic() {

//         stateUpdate();
        
//         if (State == state.TRYNNAINTAKE) {
//             setpointlengthBelowPivot = 0.5;
//             if ((setpointlengthBelowPivot - lengthBelowPivot) < 0.08) {
//                 setpointAngleFromPosX  = ArmConstants.IntakeAngle;
//             }
//             if (setpointAngleFromPosX == ArmConstants.IntakeAngle && (setpointAngleFromPosX - currentAngleFromPosX) <  Math.PI / 14) {
//                 setpointlengthBelowPivot = ArmConstants.IntakeLength;

                
//             }

            
//         }

//         if (State ==state.READYTOINTAKE) {
//             //codetointake
//         }

//         if(State == state.HOME) {
//             setpointlengthBelowPivot = 0.5;

//             if ((setpointlengthBelowPivot - lengthBelowPivot) < 0.08) {
//                 setpointAngleFromPosX = ArmConstants.homeAngle;
                
                


//             }
//         }

//         if(State == state.SCORING && (setpointlengthBelowPivot - lengthBelowPivot) < 0.05) {
//             setpointAngleFromPosX = ArmConstants.ScoringAngle;
//             if (currentAngleFromPosX > ArmConstants.minAngle_scoringExtension) {
//                 setpointlengthBelowPivot = 1;
//                 if (setpointlengthBelowPivot - lengthBelowPivot < 0.01 && !limitSwitchArmExtension.get()) {
//                     //reset encoders
//                 }
                


//             }

            
//         }

//         if(State == state.SCORING && !hasNote()) {
             

//             State = state.HOME;
            
//         }



//         // if (lastAngle != setpointAngleFromPosX) {
//         //     setAngle();
            

//         // }

//         // if (lastLength != setpointlengthBelowPivot) {
//         //     setLength();

//         // }

//         // lastAngle = setpointAngleFromPosX;
//         // lastLength = setpointlengthBelowPivot;


//         setAngle();
//         setLength();


//         // PivotMotor.setVoltage(
//         //     ProfiledPID_PIVOT.calculate(setpointAngleFromPosX - currentAngleFromPosX)
//         //         + armFeedforward.calculate(ProfiledPID_PIVOT.getSetpoint().velocity));

//       }
    

        

       




        
        
        
    

//     enum state {
//         TRYNNAINTAKE,
//         READYTOINTAKE,
//         HOME,
//         SCORING
        
//       }

    


//     public void stateUpdate(Intake intake, Ramp ramp, Swerve swerve) {
      


//         if ((intake.holdingObject() || ramp.holdingObject() || hasNote()) && Ramp.State == Ramp.SCORINGINAMP) {
//             if (State == state.HOME ) {
//                 if (!limitSwitchIntake.get()) {
//                     State = state.TRYNNAINTAKE;

//                 }
//                 else if (swerve.State == Swerve.state.GOINGTOAMP) {
//                     State = state.SCORING;
                    
//                 }
            

               
                
//             }
//             else if (State == state.TRYNNAINTAKE && setpointAngleFromPosX - currentAngleFromPosX < Math.PI / 36 && setpointlengthBelowPivot - lengthBelowPivot < 0.025) {
//                 State = state.READYTOINTAKE;
//             }

//             else if (State == state.READYTOINTAKE && limitSwitchIntake.get()) {
//                 State = state.HOME;
//             }

//             else if (State == state.SCORING && !limitSwitchIntake.get()) {
//                 State = state.HOME;
//             }

//             else {
//                 State = state.HOME;

//                 SmartDashboard.putBoolean("None of the conditions were met in the stateUpdate() for Arm.java", true);
//             }
//         }

//         }

        

            

         
    


//     public boolean hasNote() {
//         return limitSwitchIntake.get();
//     }

   

  
 

//     public void resetEncoders() {


//     }

//     public void setLength() {
//         double totalLength = setpointlengthBelowPivot * ArmConstants.lengthTotal;
//         double RotationstoComplete  = (totalLength -  lengthBelowPivot) / (2 * Math.PI * ArmConstants.radius_sprocket_Extension * ArmConstants.gearRatio_extension);
//         PID_ExtensionMotor.setReference(RotationstoComplete, ControlType.kPosition);






    


//     }

//     public double getLength() {

//         return currentAngleFromPosX;
        

//     }

//     public void setAngle() {

//         if (lastAngle != setpointAngleFromPosX) {
//            ProfiledPID_PIVOT =
//             new ProfiledPIDController(0, 0, 0, m_constraints, 0.02);
//         }

       
//         armFeedforward.updateFeedforwardRotation(setpointAngleFromPosX, ProfiledPID_PIVOT.getSetpoint().velocity);
//         armFeedforward.updateLengthbelowPivot(lengthBelowPivot);
//         PivotMotor.setVoltage(ProfiledPID_PIVOT.calculate(setpointAngleFromPosX - currentAngleFromPosX) + armFeedforward.CalculateFeedforward());

//         lastAngle = setpointAngleFromPosX;

        


//     }

//     public double getAngle() {
//         return lengthBelowPivot;

//     }

   
    
//     public void releaseObject() {

//     }

//     public void intakeObject() {

//     }


//     public void home() {

//     }

//     public void IntakePosition() {

//     }

//     public void 
    

