package frc.robot.subsystems;


import util.LimelightHelpers.LimelightHelpers;
import util.LimelightHelpers.LimelightHelpers.LimelightResults;
import util.LimelightHelpers.LimelightHelpers.Results;

import java.util.ArrayList;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.math.CoordinateSystems;
import frc.math.LimelightTargetCalculator;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase{

  //  public final Matrix
    

   // public  Pose2d pose_final;
   
   private Pose2d currentPose;
    public Results llresultsLL3;
    public Results llresultsLL2;

    public double rotation;
    public double timestamp;
    public boolean isNew; // is new pose
    
    public Pose2d currentPoseFIELDRELATIVE;
    private final Field2d field = new Field2d();

    public double maxAmbiguity;
    public double minDistance;
    public boolean hasInitialPose = false;

    public  Pose2d startingpos;
    

    public boolean hasTarget = false;
    public int singleIDUsed;
    public boolean hasObject;
    public boolean hasNoteTarget;

    public double translationToTargetX;
    public double translationToTargetY;
    public String LL2name = "limelight2";
    public String LL3name = "limelight";

    NetworkTableEntry botposeEntry;
    NetworkTableEntry json;

    DoubleArrayTopic botpose;
    double[] data;  
    PoseAndTimestamp poseAndTimestamp;
    SendableChooser<Boolean> AllianceColor = new SendableChooser<>();
    public boolean isBlue;

    ArrayList<Double> ta = new ArrayList<>();
    ArrayList<Double> tx = new ArrayList<>();
    ArrayList<Double> ty = new ArrayList<>();

    ArrayList<Double> txFiducial = new ArrayList<>();
    ArrayList<Double> IndexFiducial = new ArrayList<>();
     ArrayList<Double> AreaFiducial = new ArrayList<>();
    private double rotationPID = 0;
    private NetworkTableEntry botposeEntryLL3;
    private NetworkTableEntry botposeEntryLL2;


   
    
    
   
    


    public VisionSubsystem() {
        SmartDashboard.putNumber("setting val", 0);
        AllianceColor.addOption("BLUE ALLIANCE", true);
        AllianceColor.addOption("RED ALLIANCE", false );
        SmartDashboard.putData("Alliance Color", AllianceColor);


       SmartDashboard.putData("FieldVision", field);


       
       

        botposeEntry = NetworkTableInstance.getDefault().getTable(LL2name).getEntry("botpose");
  

        if (NetworkTableInstance.getDefault().getTable(LL2name).getEntry("tv").getInteger(0) == 1) {
            
        data = botposeEntry.getDoubleArray(new double[7]);
        Pose2d initPose = new Pose2d(data[0], data[1], new Rotation2d(data[4]));
        SmartDashboard.putBoolean("FIELD IS BEING SET", true);


         if (!isBlue) {
         startingpos = CoordinateSystems.RightSide_FieldToRob(initPose);
        }
 
        if (isBlue) {
        startingpos = initPose;

        }   




        //converts coordinate systems to controller set up to combine with odometry




        // if (DriverStation.getAlliance().isPresent()) {


        // if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
        //     startingpos = CoordinateSystems.LeftSide_FieldToRob(initPose);


            
        // } 
        // if (DriverStation.getAlliance().get() == (DriverStation.Alliance.Red)) {
        //     startingpos = CoordinateSystems.RightSide_FieldToRob(initPose);
        // }
        hasInitialPose = true;

        SmartDashboard.putNumber("initialPoseX", initPose.getX());
        SmartDashboard.putNumber("initialPoseY", initPose.getY());
        SmartDashboard.putNumber("initialPoseRotation", initPose.getRotation().getDegrees());
        
    }
     



        
        else {
            hasInitialPose = false;


        }
    }
            
        
    



    

 class PoseAndTimestamp {
    private Pose2d pose;
    private double timestamp;
   
    public PoseAndTimestamp(Pose2d pose, double timestamp) {
        this.pose = pose;
        this.timestamp = timestamp;
        
        // this.isNew = isNew;
    }

    public Pose2d getPose() {
        return pose;
    }

    public double getTimestamp() {
        return timestamp;
    }

  
    // public boolean getisNew() {
    //     return isNew;
    // }
}



 public void periodic() {
    
    if (RobotState.isEnabled()) {

    isBlue = AllianceColor.getSelected();
   
 
     
     botposeEntryLL3 = NetworkTableInstance.getDefault().getTable(LL3name).getEntry("botpose");


    SmartDashboard.putBoolean("Has InitPose", hasInitialPose);
    if (NetworkTableInstance.getDefault().getTable(LL3name).getEntry("tv").getInteger(0) == 1) {
        SmartDashboard.putBoolean("has target", true);

        isNew = true;
   
    data = botposeEntry.getDoubleArray(new double[7]);
     currentPoseFIELDRELATIVE = new Pose2d(data[0], data[1], new Rotation2d(data[4]));
     field.setRobotPose(CoordinateSystems.FieldMiddle_FieldBottomLeft(currentPoseFIELDRELATIVE));



    //converts coordinate systems to controller set up to combine with odometry

   
    if (!isBlue) {
         currentPose = CoordinateSystems.RightSide_FieldToRob(currentPoseFIELDRELATIVE);

    }

    if (isBlue) {
        currentPose = currentPoseFIELDRELATIVE;

    } 
   
    
    if (!hasInitialPose) {
        startingpos = currentPose;
        hasInitialPose = true;
    }

isNew = true;



    poseAndTimestamp = new PoseAndTimestamp(currentPose, data[6]);
 SmartDashboard.putBoolean("Has InitPose", hasInitialPose);
 SmartDashboard.putNumber("xLocalizationVision", currentPose.getX());
  SmartDashboard.putNumber("yLocalizationVision", currentPose.getY());
  


    }

 else {

    isNew = false;
    SmartDashboard.putBoolean("has target", false);
    SmartDashboard.putBoolean("Has InitPose", hasInitialPose);



 }
 //////////////////////////////////////////////////////////////////////////////////////////

 //object detection




 llresultsLL2 = LimelightHelpers.getLatestResults(LL2name).targetingResults;
 llresultsLL3 = LimelightHelpers.getLatestResults(LL3name).targetingResults;


 if (!tx.isEmpty()) {
 tx.clear();
 ta.clear();
 ty.clear();
 }

 if (!txFiducial.isEmpty()) {
    txFiducial.clear();
    IndexFiducial.clear();
    AreaFiducial.clear();
 }
 if (NetworkTableInstance.getDefault().getTable(LL2name).getEntry("tv").getInteger(0) == 1.0) {
SmartDashboard.putNumber("1 if limelight has detected a note", 1);
   for (int i = 0;  i < llresultsLL2.targets_Detector.length; i++) {
   tx.add( llresultsLL2.targets_Detector[i].tx);
   ta.add(llresultsLL2.targets_Detector[i].ta);
   ty.add(llresultsLL2.targets_Detector[i].ty);
  

   }
}
else {
    SmartDashboard.putNumber("1 if limelight has detected a note", 0);
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//fiducial detection
for (int i = 0; i < llresultsLL3.targets_Fiducials.length; i++) {
    txFiducial.add(llresultsLL3.targets_Fiducials[i].tx);
    IndexFiducial.add(llresultsLL3.targets_Fiducials[i].fiducialID);
    AreaFiducial.add(llresultsLL3.targets_Fiducials[i].ta);

  }
 

}  
 }
public double[] getDetection() {
    double[] xNy = {0,0};
    if (NetworkTableInstance.getDefault().getTable(LL2name).getEntry("tv").getInteger(0) == 1 && !tx.isEmpty()) {
        double best = 0;
    int bestIndex = 0;
    double AngleofRotationx = 0;
    double AngleofRotationy = 0;
    
    for(int i = 0; i < ta.size(); i++) {
        if (best > tx.get(i)) {
            best = tx.get(i);
            bestIndex = i;
        }
    }

    AngleofRotationx = tx.get(bestIndex) * Math.PI / 180;
    AngleofRotationy = ty.get(bestIndex) * Math.PI / 180;


     xNy[0] = AngleofRotationx;
     xNy[1] =  AngleofRotationy;




    }
    SmartDashboard.putNumber("x", xNy[0]);
    SmartDashboard.putNumber("y", xNy[1]);
   return xNy;

    




}



      





public PoseAndTimestamp getVision() {
    return poseAndTimestamp;
}

public void setRotationPID(double rotation) {
    SmartDashboard.putBoolean("pid has been set backend", true);
    SmartDashboard.putNumber("backend pid val setting", rotation);
    
    
    rotationPID = rotation;
}
public double getRotationPID() {
    SmartDashboard.putBoolean("pid has been gotten backend", true);
    SmartDashboard.putNumber("backend pid val", rotationPID);
    return rotationPID;
}


public void setPipelineLL3(int index) {
    LimelightHelpers.setPipelineIndex(LL3name, index);
    
}
public void setPipelineLL2(int index) {
    LimelightHelpers.setPipelineIndex(LL2name, index);
}

public double getSpeakerDetection() {
    double val = 0;
    boolean yep = false;
    for (int i = 0; i<txFiducial.size(); i++) {
        if(IndexFiducial.get(i) == Constants.PipelineConstants.SPEAKERBLUE) {
            val = txFiducial.get(i);
            yep = true;

        }

    }
    SmartDashboard.putBoolean("8 detected", yep);
    if (yep != true) {
        return 10000;
    }
    else {
    
    return val;
    }
}


public double getBestTargetArea() {
    double bestarea = 0;
    if (!AreaFiducial.isEmpty()) {
    for (double area: AreaFiducial) {
        if (area > bestarea) {
            bestarea = area;
        }

    }
}

return bestarea;
    
}

public int getNumberofAprilTags() {
    return txFiducial.size();
}

}