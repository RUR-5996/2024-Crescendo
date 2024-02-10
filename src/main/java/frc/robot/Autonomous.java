package frc.robot;

import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;;

public class Autonomous{
    // Create config for trajectory
    public static double timetrajectoryStarted;
    public static String trajectoryStatus="";
    public static Autonomous SINGLE_INSTANCE = new Autonomous();    

    
    public static double elapsedTime;

    public static Autonomous getInstance(){
        return SINGLE_INSTANCE;
    }
    public static HolonomicDriveController HDC = new HolonomicDriveController(
        new PIDController(Constants.kP,0, 0), 
        new PIDController(Constants.kP, 0, 0), 
        new ProfiledPIDController(Constants.kP*Constants.MAX_SPEED_RADIANSperSECOND/Constants.MAX_SPEED_METERSperSECOND, 0, 0, 
        new Constraints(Constants.MAX_SPEED_RADIANSperSECOND, Constants.MAX_SPEED_RADIANSperSECOND)));
    

    /**This is WPILIBs Trajectory Runner (docs.wpilib.org), it pretends that your robot is NOT a swerve drive.  This will work, but there are better options for 2022
     * @param _trajectory Pass in a trajectory that's stored in TrajectoryContainer
     * @param _odometry Pass in the robots odometry from SwerveDrive.java
     * @param _rotation2d Pass in the current angle of the robot
     */



    //Overload this method to accomdate different starting points, this can be useful when playing with multiple paths
    /**
     * This is PathPlanner.  It's awesome :) open up pathplanner.exe on the driverstation laptop.  Point the application to the locaiton of your coding project (must contain build.gradle).  Draw the path.  It will autosave. If everything is characterized correctly and your odometry reflects reality, ie. when the robot goes 1 meter it says it goes one meter--it will work like a charm.
     * @param _pathTraj run Pathplanner.loadpath("name of file without extension") pass it here
     * @param _odometry SwerveDrive.java's odometry
     * @param _rotation2d Pass in the current angle of the robot
     */
    public static void PathPlannerRunner(PathPlannerTrajectory _pathTraj, SwerveDriveOdometry _odometry, Rotation2d _rotation2d){
        elapsedTime = Timer.getFPGATimestamp()-timetrajectoryStarted;
        switch (trajectoryStatus) {
            case "setup":    
            // PathPlannerState helloPath =((PathPlannerState)_pathTraj.getInitialState());

                timetrajectoryStarted = Timer.getFPGATimestamp();
                trajectoryStatus = "execute";
                break;
            case "execute":
                
                if (elapsedTime <  ((State) _pathTraj.getEndState()).timeSeconds){
                    ChassisSpeeds _speeds = HDC.calculate(
                        _odometry.getPoseMeters(), 
                        ((State) _pathTraj.sample(elapsedTime)),((State) _pathTraj.sample(elapsedTime)).targetHolonomicRotation);
                    SwerveDrive.drive(_speeds.vxMetersPerSecond,
                    _speeds.vyMetersPerSecond, 
                    _speeds.omegaRadiansPerSecond);
                    
                } else {
                    SwerveDrive.drive(0,0,0);
                    SwerveDrive.setHoldRobotAngleSetpoint(((State) _pathTraj.getEndState()).targetHolonomicRotation.getRadians());
                    trajectoryStatus = "done";

                }
                break;
            default:
                SwerveDrive.drive(0,0,0);
                break;
        }
    }

    public static void resetTrajectoryStatus(){
        trajectoryStatus = "setup";
    }
    
}