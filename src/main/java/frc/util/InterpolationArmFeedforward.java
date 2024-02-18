// package frc.util;

// import edu.wpi.first.math.controller.ArmFeedforward;
// import frc.robot.Constants.OperatorConstants.ArmConstants;

// public class InterpolationArmFeedforward {
//     private double lengthBelowPivot;
//     private ArmFeedforward armFeedforward;
//     private double currentAngle;
//     private double velocity;
   


//     public InterpolationArmFeedforward() {
//         armFeedforward = getFeedforward();
       



//     }


//     public ArmFeedforward getFeedforward() {
//         double ks = ArmConstants.ksm  * lengthBelowPivot + ArmConstants.ksb;
//         double kv = ArmConstants.kvm  * lengthBelowPivot + ArmConstants.kvb;
//         double kg = ArmConstants.kgm  * lengthBelowPivot + ArmConstants.kgb;
//         double kp = ArmConstants.kPm  * lengthBelowPivot + ArmConstants.kPb;
//         double kd = ArmConstants.kDm  * lengthBelowPivot + ArmConstants.kDb;


//         return new ArmFeedforward(ks, kg, kv);

        
        
        
//     }

//     public double CalculateFeedforward() {
//         if (currentAngle > Math.PI / 2 ) {
//             currentAngle = currentAngle - Math.PI/2;
//         }
//         else {
//             currentAngle = Math.PI/2 - currentAngle;
//         }

//         return armFeedforward.calculate(currentAngle, velocity);
//     }

//     public void updateFeedforwardRotation(double newAngle, double newVelocity) {
//         currentAngle = newAngle;
//         velocity = newVelocity;

//     }
//     public void updateLengthbelowPivot(double length) {
        
//         lengthBelowPivot = length;
//     }



    
    
// } {
    
// }
