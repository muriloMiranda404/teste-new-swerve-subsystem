package frc.robot.subsystems;

import java.io.File;
import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.swerve;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.encoders.SwerveAbsoluteEncoder;
import swervelib.parser.SwerveParser;

public class SwerveSubsystem extends SubsystemBase{
    
    SwerveDrive swerveDrive;
    Pigeon2 pigeon2;
    SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    LimelightConfig limelightConfig;
    PIDController xPID, yPID;
    ProfiledPIDController rotationPID;
    HolonomicDriveController drivePID;
    SwerveModuleState[] state;
    SwerveModule[] modules;
    SwerveAbsoluteEncoder absoluteEncoder;

    public static SwerveSubsystem mInstance = null;

    private SwerveSubsystem(File directory){

        try{
            swerveDrive = new SwerveParser(directory).createSwerveDrive(swerve.MAX_SPEED);
        } catch(Exception e){
            System.out.println("erro ao criar swerve drive");
        } finally{
            limelightConfig = LimelightConfig.getInstance();
            pigeon2 = new Pigeon2(9);

            swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(swerveDrive.kinematics, pigeon2.getRotation2d(), 
                                                                    swerveDrive.getModulePositions(), 
                                                                    new Pose2d());

            xPID = new PIDController(0.01, 0, 0);//frente
            yPID = new PIDController(0.01, 0, 0);//lado
            rotationPID = new ProfiledPIDController(0.01, 0, 0, new TrapezoidProfile.Constraints(Math.PI, Math.PI));//rotação
            drivePID = new HolonomicDriveController(xPID, yPID, rotationPID);

            xPID.setTolerance(0.01);
            yPID.setTolerance(0.2);
            rotationPID.setTolerance(0.2);
            drivePID.setEnabled(true);

            setupPathPlanner();
        }
    }

    public static SwerveSubsystem getInstance(){
        if(mInstance == null){
            mInstance = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
        }
        return mInstance;
    }

    @Override
    public void periodic() {
        swerveDrive.updateOdometry();
        swerveDrivePoseEstimator.update(pigeon2.getRotation2d(), swerveDrive.getModulePositions());

        if(limelightConfig.getHasTarget()){
            Pose2d pose = limelightConfig.getGlobalPose();
            swerveDrivePoseEstimator.addVisionMeasurement(pose, Timer.getFPGATimestamp());
        }
    }

    public void drive(Translation2d translation, double rotation, boolean fieldOriented){
        swerveDrive.drive(translation, rotation, fieldOriented, false);
    }

    public void zeroGyro(){
        swerveDrive.zeroGyro();
    }

        public void setupPathPlanner(){

            RobotConfig config;
            try{
              config = RobotConfig.fromGUISettings();
              
              boolean feedforwards = true;
              
              AutoBuilder.configure(
                    this::getPose,
                    this::resetOdometry, 
                    this::getRobotRelativeSpeeds, 
                    (speeds, feedforward) -> {
                      if (feedforwards)
                      {
                        swerveDrive.drive(
                            speeds,
                            swerveDrive.kinematics.toSwerveModuleStates(speeds),
                            feedforward.linearForces()
                                         );
                      } else
                      {
                        swerveDrive.setChassisSpeeds(speeds);
                      }}, 
                      new PPHolonomicDriveController(
                        new PIDConstants(0.15, 0.0, 0.0), 
                        new PIDConstants(0.6, 0.0, 0.0)),
                      config, 
                      () -> {
                        
                      var alliance = DriverStation.getAlliance();
                      if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                      }
                      return false;
                    },
                    this 
                    );
                  } catch (Exception e) {
                    e.printStackTrace();
                  }
          }

    public void resetOdometry(Pose2d pose2d){
        swerveDrive.resetOdometry(pose2d);
    }

    public void resetDriveEncoder(){
        swerveDrive.resetDriveEncoders();
    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
        return swerveDrive.getRobotVelocity();
    }

    public Command driveCommand(DoubleSupplier X, DoubleSupplier Y, DoubleSupplier rotation){
        return driveCommand(X, Y, rotation, false);
    }
  
    // public Command driveCommand(DoubleSupplier X, DoubleSupplier Y, DoubleSupplier rotation){
    //     return run(() ->{
    //     double xController = Math.pow(X.getAsDouble(), 3);
    //     double yController = Math.pow(Y.getAsDouble(), 3);

    //     driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(xController * swerveDrive.getMaximumChassisVelocity(),
    //                                                                     yController * swerveDrive.getMaximumChassisVelocity(), 
    //                                                                     rotation.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
    //                                                                     getheading().getRadians(), 
    //                                                                     7.0));
    //     });
    // }

    /**
     * 
     * @param X
     * @param Y
     * @param rotation
     * @param closedLoop
     * @return
     */
    public Command driveCommand(DoubleSupplier X, DoubleSupplier Y, DoubleSupplier rotation, boolean closedLoop){
        return run(() ->{
        double xController = Math.pow(X.getAsDouble(), 3);
        double yController = Math.pow(Y.getAsDouble(), 3);
        double rotInput = rotation.getAsDouble();
        Rotation2d rotation2d = swerveDrivePoseEstimator.getEstimatedPosition().getRotation();

        ChassisSpeeds normalDrive = swerveDrive.swerveController.getTargetSpeeds(xController,
                                                                                 yController, 
                                                                                 rotInput,
                                                                                 getheading().getRadians(),
                                                                                  7.0);

            if(closedLoop == true){
            Pose2d poseAtual = getPose();
            double td = 0.02;

            Pose2d targetPose = new Pose2d(
                poseAtual.getX() + normalDrive.vxMetersPerSecond * td,//certo (frente e traz)
                poseAtual.getY() + normalDrive.vyMetersPerSecond * td,//errado (lados)
                poseAtual.getRotation().plus(new Rotation2d(normalDrive.omegaRadiansPerSecond * td)) //errado ( rotação)
            );

            ChassisSpeeds controllerDrive = drivePID.calculate(poseAtual, targetPose, normalDrive.vxMetersPerSecond, rotation2d);

            driveFieldOriented(controllerDrive);
        } else {
            swerveDrive.drive(new Translation2d(xController * swerveDrive.getMaximumChassisVelocity() * -1.0,
                                                yController *swerveDrive.getMaximumChassisVelocity() * 1.0),
                                                rotInput * swerveDrive.getMaximumChassisAngularVelocity() * -1.0,
                                                true,
                                                false);
        }
    });
    }

    public Command driveRobot(DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega, boolean fieldOriented){
        return run(() ->{
    
        ChassisSpeeds speed = fieldOriented == true ? ChassisSpeeds.fromFieldRelativeSpeeds(x.getAsDouble() * swerveDrive.getMaximumChassisVelocity(), 
                                                                                            y.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                                                                                            omega.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
                                                                                            pigeon2.getRotation2d())
                                                                                            :
                                                                                            new ChassisSpeeds(x.getAsDouble(),
                                                                                                              y.getAsDouble(),
                                                                                                              omega.getAsDouble());
    
        state = swerveDrive.kinematics.toSwerveModuleStates(speed);
        SwerveDriveKinematics.desaturateWheelSpeeds(state, swerve.MAX_SPEED);
    
        modules = swerveDrive.getModules();
    
        for(int i = 0; i < modules.length; i++){
          modules[i].setDesiredState(state[i], true, true);
        }
        });
      }

    public Pose2d getPose(){
        return swerveDrive.getPose();
    }

    public Rotation2d getheading(){
        return Rotation2d.fromDegrees(Math.IEEEremainder(pigeon2.getYaw().getValueAsDouble(), 360));
    }

    public void driveFieldOriented(ChassisSpeeds speed){
        swerveDrive.drive(speed);
    }

    public void setMotorBrake(boolean brake){
        swerveDrive.setMotorIdleMode(brake);
    }
    
    public Command getAutonomousCommand(String pathName, boolean setOdomToStart){ 
        if(setOdomToStart){
            return AutoBuilder.buildAuto(pathName);
        }
        return new PathPlannerAuto(pathName);
    }
}