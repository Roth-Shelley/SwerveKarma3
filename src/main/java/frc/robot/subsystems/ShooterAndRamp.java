// package frc.robot.subsystems;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
// import frc.robot.Constants;
// import com.revrobotics.CANPIDController;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkMaxPIDController;
// import com.revrobotics.SparkPIDController;
// import com.revrobotics.CANSparkBase.ControlType;
// import com.revrobotics.CANSparkMax;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import frc.robot.Constants.Shooter;

// public class ShooterAndRamp {

//     CANSparkMax leftMotor = new CANSparkMax(Constants.Shooter.leftMotorID, MotorType.kBrushless);
//     CANSparkMax rightMotor = new CANSparkMax(Constants.Shooter.rightMotorID, MotorType.kBrushless);
//     CanSparkMax pivotMotor1 = new CanSparkMax(Constants.Shooter.pivotMotor1ID, MotorType.kBrushless)
//     CANSparkMax pivotMotor2 = new CanSparkMax(Constants.Shooter.pivotMotor2ID, MotorType.kBrushless)


//     RelativeEncoder leftEncoder = leftMotor.getEncoder();
//     RelativeEncoder rightEncoder = rightMotor.getEncoder();
//     RelativeEncoder pivotMotor1 = pivotMotor1.getEncoder();
//     RelativeEncoder pivotMotor2 = pivotMotor2.getEncoder();

//     private SparkPIDController rightPIDController;
//     private SparkPIDController leftPIDController;
//     private SparkPIDController pivot1PIDController;
//     private SparkPIDController pivot2PIDController;



//     private double targetVelocity = 0; 
//     private int rollingAvg = 0;


//     enum States {
//         SHOOTINGAMP,
//         SHOOTINGSPEAKER,
//         HOMESTATE
//     }

//     public ShooterAndRamp() {


//         leftMotor.restoreFactoryDefaults();
//         rightMotor.restoreFactoryDefaults();
//         pivotMotor1.restoreFactoryDefaults();
//         privotMotor2.restoreFactoryDefaults();

        
//         // leftEncoder.setPosition(0);
//         // rightEncoder.setPosition(0);

//         leftMotor.follow(rightMotor);
//         // rightMotor.follow(rightMotor);

//         leftMotor.setInverted(false);
//         rightMotor.setInverted(false);

//         leftPIDController = leftMotor.getPIDController();
//         rightPIDController = rightMotor.getPIDController();

//         leftPIDController.setP(Shooter.proportialPIDConstant);
//         leftPIDController.setI(Shooter.integralPIDConstant);
//         leftPIDController.setD(Shooter.derivativePIDConstant);
//         leftPIDController.setIZone(Shooter.integralPIDConstant);
//         leftPIDController.setFF(Shooter.leftFeedForwardPIDConstant);
//         leftPIDController.setOutputRange(Shooter.minPIDOutput, Shooter.maxPIDOutput);
    
//         rightPIDController.setP(Shooter.proportialPIDConstant);
//         rightPIDController.setI(Shooter.integralPIDConstant);
//         rightPIDController.setD(Shooter.derivativePIDConstant);
//         rightPIDController.setIZone(Shooter.integralPIDConstant);
//         rightPIDController.setFF(Shooter.rightFeedForwardPIDConstant);
//         rightPIDController.setOutputRange(Shooter.minPIDOutput, Shooter.maxPIDOutput);



//         leftMotor.burnFlash();
//         rightMotor.burnFlash();

//         pivot1PIDController.setP(Shooter.placeholder);
//         pivot1PIDController.setI(Shooter.placeholder);
//         pivot1PIDController.setD(Shooter.placeholder);

//         pivot2PIDController.setP(Shooter.placeholder);
//         pivot2PIDController.setI(Shooter.placeholder);
//         pivot2PIDController.setD(Shooter.placeholder);


//         //set G
//         // set p
//         // set I
//         // set d

//         // PID myPID = new PIDController(rollingAvg, targetVelocity, rollingAvg)

        
//     }

//   public void periodic() {
// //     position = getEncoder()
// // feedforward = cos(position) * kG
// // effort = pid.calculate(position)
// // effort += feedforward
// // controller.set(effort)

//   }

//       public void movePivot(double speed){
//     if(Math.abs(speed) <= 0.1){
//       speed = 0;
//     }

//       pivotMotor1.set(speed);
//       pivotMotor2.set(speed);

//     }

//     public void switchState(States state) {

//     if(state.equals(States.SHOOTINGAMP)) {
//         movePivot(0.4);
//     } else if (state.equals(States.SHOOTINGSPEAKER)){
//         movePivot(0.5);
//     } else if(state.equals(States.HOMESTATE)){
//         movePivot(0.4);
//     }
//     }



// }