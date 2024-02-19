package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.OdometryThread;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import frc.math.CoordinateSystems;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public static SwerveModule[] mSwerveMods;
    public static AHRS gyro;
    private final Field2d field;;
    //static LEDs leds;
    public double maxSpeed = 1.2;
    double gyroThing;
    VisionSubsystem vision;
    boolean hasResetGyro;
   

    private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));

  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

    public SwerveDrivePoseEstimator odometry;
    public SwerveDriveOdometry odometry2;

    public Swerve(VisionSubsystem vision) {
        field = new Field2d();
        SmartDashboard.putData("Field", field);
        this.vision = vision;
        gyro = new AHRS(SPI.Port.kMXP);

        while (gyro.isCalibrating()) {
            Thread.yield();
        }
        resetEveything();

  

      
        
       
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(1, Constants.Swerve.Mod1.constants, Constants.Swerve.Mod1.invertedDrive, Constants.Swerve.Mod1.invertedSteer),
            new SwerveModule(0, Constants.Swerve.Mod0.constants, Constants.Swerve.Mod0.invertedDrive, Constants.Swerve.Mod0.invertedSteer),
            new SwerveModule(3, Constants.Swerve.Mod3.constants, Constants.Swerve.Mod3.invertedDrive, Constants.Swerve.Mod3.invertedSteer),
            new SwerveModule(2, Constants.Swerve.Mod2.constants, Constants.Swerve.Mod2.invertedDrive, Constants.Swerve.Mod2.invertedSteer),
            
        };
        
 
 
        SmartDashboard.putNumber("gyroInitReading", getGyro().getDegrees());
        odometry = new SwerveDrivePoseEstimator(Constants.Swerve.kinematics, getGyro(), getModulePositions(), new Pose2d(new Translation2d(-8.175+ 0.45, 1.4478), new Rotation2d(180)),
             stateStdDevs, visionMeasurementStdDevs);
        odometry2 = new SwerveDriveOdometry(Constants.Swerve.kinematics, getGyro(), getModulePositions());



            


                
    }

    public void resetEveything() {
        gyro.zeroYaw();
        
        
    }

    public void resetPose(Pose2d pose) {
        odometry.resetPosition(getGyro(), getModulePositions(), getPose());

    }

    public void robotInit(){
        gyroThing = gyro.getRotation2d().getDegrees();
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.kinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getGyro()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
        SmartDashboard.putNumber("Gyrothingiesies", getGyro().getDegrees());
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber]);
        }
    }   
    
    public void moveByChassisSpeeds(ChassisSpeeds speed){
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(speed, getGyro()));
        setModuleStates(swerveModuleStates);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredAutoState(desiredStates[mod.moduleNumber]);
        }
    }    


    public boolean isAtState(){
        return false;
    }
    public SwerveModuleState[] getStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public void zeroGyro(){
        gyro.reset();
    }

    
                                                                                                                                                                               
    public Rotation2d getGyro() {
        return Rotation2d.fromDegrees(Math.IEEEremainder(gyro.getYaw(), 360d) * -1d);
    }

    public void resetFieldPosition(){
        zeroGyro();
        odometry.resetPosition(getGyro(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Pose2d getPose() {
        return odometry.getEstimatedPosition();
        
    }

    public Rotation2d getRoll(){
        return Rotation2d.fromDegrees(gyro.getRoll());
    }

    public void resetGyro(){
        gyro.reset();
        
    }

    @Override
    public void periodic(){
         if (gyro.isConnected() && hasResetGyro == false && !gyro.isCalibrating()) {
             hasResetGyro = true;
             resetEveything();
         }
         if (hasResetGyro == true) {
            
        odometry.update(getGyro(), getModulePositions());
        odometry2.update(getGyro(), getModulePositions());
        updatePoseEstimatorwVision();
        SmartDashboard.putNumber("localizationX", odometry.getEstimatedPosition().getX());
        SmartDashboard.putNumber("localizationY", odometry.getEstimatedPosition().getY());
        SmartDashboard.putNumber("localizationR", getGyro().getDegrees());
        SmartDashboard.putNumber("Gyrothingy Displacement", gyro.getDisplacementY());



        SmartDashboard.putNumber("odometryX", odometry2.getPoseMeters().getX());
        SmartDashboard.putNumber("odometryY", odometry2.getPoseMeters().getY());
        
        }



       
       
       

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
        field.setRobotPose(CoordinateSystems.FieldMiddle_FieldBottomLeft(new Pose2d(odometry.getEstimatedPosition().getX(), odometry.getEstimatedPosition().getY(), getGyro())));



    }


    
    
    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }
    public Rotation2d getPitch(){
        return Rotation2d.fromDegrees(gyro.getPitch());
    }
    public void setModuleRoation(Rotation2d rotation){
        for(SwerveModule mod: mSwerveMods){
            mod.setDesiredState(new SwerveModuleState(0, rotation));
        }
    }
    public SwerveModule[] getModules(){
        return mSwerveMods;
    }

    public double getXGyro(){
        return gyro.getDisplacementX();
    }

    public double getYGyro(){
        return gyro.getDisplacementY();
    }

    public AHRS getAhrs(){
        return gyro;
    }
    public void stop(){        
        for(SwerveModule mod : mSwerveMods){
            mod.stop();
        }
    }
    
  public Command createCommandForTrajectory(Trajectory trajectory, Supplier<Pose2d> poseSupplier) {
    
      return null;
  }

public double getCurrentTotalVelocity() {
    return 0;
}

public Rotation2d getHeading() {
    return null;
}

public Rotation2d getYaw() {
    return null;
}

public void moveByChassisSpeeds(double forwardSpeed, double leftwardSpeed, double angSpeed, double currentAng) {
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            forwardSpeed,
            leftwardSpeed,
            angSpeed,
            Rotation2d.fromDegrees(Math.toDegrees(currentAng)));
    SwerveModuleState[] states = Constants.Swerve.kinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Swerve.maxSpeed);
    setModuleStates(states);
}

public void updatePoseEstimatorwVision() {
    if (!vision.hasInitialPose) {
        return;
    }
    else if (vision.getVision().getPose().getX() == 0) {
        return;
    }
    double posediff = getPose().getTranslation().getDistance(vision.getVision().getPose().getTranslation());
    if (vision.isNew && vision.hasInitialPose) {
        double xystd;
        double degstd;

        if (vision.getNumberofAprilTags() >= 2) {
            xystd = 0.5;
            degstd = 6;
        }
        else if (vision.getBestTargetArea() > 0.8 && posediff < 0.2) {
            xystd = 1;
            degstd = 12;
        }
        else  {
            return;
        }

        odometry.setVisionMeasurementStdDevs(VecBuilder.fill(xystd, xystd, Units.degreesToRadians(degstd)));
        odometry.addVisionMeasurement(vision.getVision().getPose(), vision.getVision().getTimestamp());
    }     
}


}