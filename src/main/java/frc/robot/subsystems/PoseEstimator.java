// package frc.robot.subsystems;

// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class PoseEstimator extends SubsystemBase{
//     public static PoseEstimator instance;
//     NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("limelight");
//     double tv = networkTable.getEntry("tv").getDouble(0);
//     double tx = networkTable.getEntry("tx").getDouble(0);
//     double ty = networkTable.getEntry("ty").getDouble(0);
//     double ta = networkTable.getEntry("ta").getDouble(0);
//     double ts = networkTable.getEntry("ts").getDouble(0);
//     double tl = networkTable.getEntry("tl").getDouble(0);

//     public PoseEstimator() {

//     }
//     @Override
//     public void periodic() {
//     }
//      public static PoseEstimator getInstance(){
//         if(instance == null){
//         instance = new PoseEstimator();
//         }
//         return instance;
        
//     }
// }
// package frc.robot.subsystems;


// public class AprilTagPoseEstimator extends SubsystemBase {
//     private static AprilTagPoseEstimator instance;
//     private NetworkTable networkTable;
//     private double tv;
//     private double tx;
//     private double ty;
//     private double ta;
//     private double ts;
//     private double tl;

//     private AprilTagPoseEstimator() {
//         networkTable = NetworkTableInstance.getDefault().getTable("limelight");
//         tv = networkTable.getEntry("tv").getDouble(0);
//         tx = networkTable.getEntry("tx").getDouble(0);
//         ty = networkTable.getEntry("ty").getDouble(0);
//         ta = networkTable.getEntry("ta").getDouble(0);
//         ts = networkTable.getEntry("ts").getDouble(0);
//         tl = networkTable.getEntry("tl").getDouble(0);
//     }

//     public static AprilTagPoseEstimator getInstance() {
//         if (instance == null) {
//             instance = new AprilTagPoseEstimator();
//         }
//         return instance;
//     }

//     public double getTargetVertical() {
//         return tv;
//     }

//     public double getTargetHorizontal() {
//         return tx;
//     }

//     public double getTargetVerticalOffset() {
//         return ty;
//     }

//     public double getTargetArea() {
//         return ta;
//     }

//     public double getTargetSkew() {
//         return ts;
//     }

//     public double getTargetLatency() {
//         return tl;
//     }

//     @Override
//     public void periodic() {
//         // Update the values periodically
//         tv = networkTable.getEntry("tv").getDouble(0);
//         tx = networkTable.getEntry("tx").getDouble(0);
//         ty = networkTable.getEntry("ty").getDouble(0);
//         ta = networkTable.getEntry("ta").getDouble(0);
//         ts = networkTable.getEntry("ts").getDouble(0);
//         tl = networkTable.getEntry("tl").getDouble(0);
//     }
// }
