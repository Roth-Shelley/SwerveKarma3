package frc.robot.subsystems;

import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;

import frc.math.CoordinateSystems;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
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
    private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));

  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(10));

    SwerveDrivePoseEstimator odometry;

    public Swerve(VisionSubsystem vision) {
        field = new Field2d();
        SmartDashboard.putData("Field", field);
        this.vision = vision;
        gyro = new AHRS(SPI.Port.kMXP);
        zeroGyro();
       
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(1, Constants.Swerve.Mod1.constants, Constants.Swerve.Mod1.invertedDrive, Constants.Swerve.Mod1.invertedSteer),
            new SwerveModule(3, Constants.Swerve.Mod3.constants, Constants.Swerve.Mod3.invertedDrive, Constants.Swerve.Mod3.invertedSteer),
            new SwerveModule(0, Constants.Swerve.Mod0.constants, Constants.Swerve.Mod0.invertedDrive, Constants.Swerve.Mod0.invertedSteer),
            new SwerveModule(2, Constants.Swerve.Mod2.constants, Constants.Swerve.Mod2.invertedDrive, Constants.Swerve.Mod2.invertedSteer),
        };
        


        
        odometry = new SwerveDrivePoseEstimator(Constants.Swerve.kinematics, getGyro().unaryMinus(), getModulePositions(), new Pose2d(new Translation2d(-8.27+0.45, 1.4478), new Rotation2d(Math.PI)),
             stateStdDevs, visionMeasurementStdDevs);

             field.setRobotPose(CoordinateSystems.FieldMiddle_FieldBottomLeft(getPose()));
             SmartDashboard.putNumber("xInitial", CoordinateSystems.FieldMiddle_FieldBottomLeft(getPose()).getX());
              SmartDashboard.putNumber("yInitial", CoordinateSystems.FieldMiddle_FieldBottomLeft(getPose()).getY());
        

                
    }

    public void resetEveything() {
        gyro.reset();
        gyro.resetDisplacement();
    }

    public void robotInit(){
        gyroThing = gyro.getRotation2d().getDegrees();
    }
    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.kinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getY(), 
                                    translation.getX(), 
                                    rotation, 
                                    getGyro()
                                )
                                : new ChassisSpeeds(
                                    translation.getY(), 
                                    translation.getX(), 
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
        return Rotation2d.fromDegrees(360- gyro.getYaw());
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
        gyro.resetDisplacement();
    }

    @Override
    public void periodic(){

        odometry.update(getGyro(), getModulePositions());
        if(vision.isNew) {
            
             odometry.addVisionMeasurement(vision.getVision().getPose(), vision.getVision().getTimestamp());
       

        }
        
       

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
        //field.setRobotPose(CoordinateSystems.FieldMiddle_FieldBottomLeft(odometry.getEstimatedPosition()));



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


}