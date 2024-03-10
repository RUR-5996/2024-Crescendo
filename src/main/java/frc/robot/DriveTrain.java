package frc.robot;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveConstants;
import frc.robot.SwerveDef.DriveMotor;
import frc.robot.SwerveDef.SteerMotor;
import frc.robot.SwerveDef.SteerSensor;
import frc.robot.SwerveDef.SwerveModule;

public class DriveTrain {

    private static DriveTrain instance;
    
    SteerMotor flSteer;
    DriveMotor flDrive;
    SteerSensor flSensor;
    SwerveModule flModule;

    SteerMotor frSteer;
    DriveMotor frDrive;
    SteerSensor frSensor;
    SwerveModule frModule;

    SteerMotor rlSteer;
    DriveMotor rlDrive;
    SteerSensor rlSensor;
    SwerveModule rlModule;

    SteerMotor rrSteer;
    DriveMotor rrDrive;
    SteerSensor rrSensor;
    SwerveModule rrModule;

    SwerveDriveKinematics swerveKinematics;


    public DriveTrain() {
        flSteer = new SteerMotor(1, SwerveConstants.FL_STEER_INVERT_TYPE);
        flDrive = new DriveMotor(1, SwerveConstants.FL_DRIVE_INVERT_TYPE);
        flSensor = new SteerSensor(1, SwerveConstants.FL_STEER_OFFSET);
        flModule = new SwerveModule(flSteer, SwerveConstants.FL_STEER_PID_VALUES, flDrive, SwerveConstants.FL_DRIVE_PID_VALUES, flSensor);

        frSteer = new SteerMotor(2, SwerveConstants.FR_STEER_INVERT_TYPE);
        frDrive = new DriveMotor(2, SwerveConstants.FR_DRIVE_INVERT_TYPE);
        frSensor = new SteerSensor(3, SwerveConstants.FR_STEER_OFFSET);
        frModule = new SwerveModule(frSteer, SwerveConstants.FR_STEER_PID_VALUES, frDrive, SwerveConstants.FR_DRIVE_PID_VALUES, frSensor);

        rlSteer = new SteerMotor(3, SwerveConstants.RL_STEER_INVERT_TYPE);
        rlDrive = new DriveMotor(3, SwerveConstants.RL_DRIVE_INVERT_TYPE);
        rlSensor = new SteerSensor(0, SwerveConstants.RL_STEER_OFFSET);
        rlModule = new SwerveModule(rlSteer, SwerveConstants.RL_STEER_PID_VALUES, rlDrive, SwerveConstants.RL_DRIVE_PID_VALUES, rlSensor);

        rrSteer = new SteerMotor(4, SwerveConstants.RR_STEER_INVERT_TYPE);
        rrDrive = new DriveMotor(4, SwerveConstants.RR_DRIVE_INVERT_TYPE);
        rrSensor = new SteerSensor(2, SwerveConstants.RR_STEER_OFFSET);
        rrModule = new SwerveModule(rrSteer, SwerveConstants.RR_STEER_PID_VALUES, rrDrive, SwerveConstants.RR_DRIVE_PID_VALUES, rrSensor);

        flModule.moduleInit();
        frModule.moduleInit();
        rlModule.moduleInit();
        rrModule.moduleInit();

        swerveKinematics = new SwerveDriveKinematics(SwerveConstants.FL_LOC, SwerveConstants.FR_LOC, SwerveConstants.RL_LOC, SwerveConstants.RR_LOC);

        setToCoast();
    }

    public static DriveTrain getInstance() {
        if(instance == null) {
            instance = new DriveTrain();
        }
        return instance;
    }

    public void setModuleSpeeds(SwerveModuleState[] _swerveModuleSates) {
        flModule.setState(_swerveModuleSates[0]);
        frModule.setState(_swerveModuleSates[1]);
        rlModule.setState(_swerveModuleSates[2]);
        rrModule.setState(_swerveModuleSates[3]);
    }

    public void setToCoast() {
        flModule.setToCoast();
        frModule.setToCoast();
        rlModule.setToCoast();
        rrModule.setToCoast();
    }

    public void setToBrake() {
        flModule.setToBrake();
        frModule.setToBrake();
        rlModule.setToBrake();
        rrModule.setToBrake();
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            flModule.getState(),
            frModule.getState(),
            rlModule.getState(),
            rrModule.getState()
        };
    }
}
