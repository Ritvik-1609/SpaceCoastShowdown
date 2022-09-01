package frc.robot.subsystems;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.fasterxml.jackson.databind.node.TreeTraversingParser;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class DriveSubsystem extends SubsystemBase {

    private final Constants constants = new Constants();

    private final WPI_TalonSRX left_master;
    private final WPI_TalonSRX right_master;
    private final WPI_TalonSRX left_follower;
    private final WPI_TalonSRX right_follower;

    private final MotorControllerGroup leftMotors;
    private final MotorControllerGroup rightMotors;

    AHRS gyro = new AHRS(SPI.Port.kMXP);
    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(constants.kTrackWidth);
    DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
            constants.ksVolts,
            constants.kvVoltSecondsPerMeter,
            constants.kaVoltSecondsSquaredPerMeter);

    Field2d field;

    PIDController leftPidController = new PIDController(constants.drive_kp, 0, 0);
    PIDController righPidController = new PIDController(constants.drive_kp, 0, 0);
    Trajectory trajectory = new Trajectory();

    Pose2d pose;
    private static DriveSubsystem instance;

    

    double turnStartTime;
    double currentTime;
    double turnFinishTime;

    double driveStartTime;
    double driveFinishTime;

    private DifferentialDrivetrainSim robotDriveSim;

    private Pose2d BLUE_TOP_STARTPose2d;
    private Pose2d BLUE_MID_STARTPose2d;
    private Pose2d BLUE_BOTTOM_STARTPose2d;

    private Pose2d RED_TOP_STARTPose2d;
    private Pose2d RED_MID_STARTPose2d;
    private Pose2d RED_BOTTOM_STARTPose2d;

    SendableChooser<Pose2d> startingPose2D = new SendableChooser<Pose2d>();
    


    public DriveSubsystem() {
        left_master = new WPI_TalonSRX(constants.LEFT_MASTER_CAN_ID);
        right_master = new WPI_TalonSRX(constants.RIGHT_MASTER_CAN_ID);
        left_follower = new WPI_TalonSRX(constants.LEFT_FOLLOWER_CAN_ID);
        right_follower = new WPI_TalonSRX(constants.RIGHT_FOLLOWER_CAN_ID);

        left_follower.follow(left_master);
        right_follower.follow(right_master);

        left_master.setInverted(false);
        right_master.setInverted(true);

        leftMotors = new MotorControllerGroup(left_master, left_follower);
        rightMotors = new MotorControllerGroup(right_master, right_follower);

        left_master.setNeutralMode(constants.driveMode);
        right_master.setNeutralMode(constants.driveMode);

        left_follower.setNeutralMode(constants.driveMode);
        right_follower.setNeutralMode(constants.driveMode);

        gyro.zeroYaw();
        field = new Field2d();
        SmartDashboard.putData("Field", field);

    }

    public void getStartPose(){
        startingPose2D.addOption("Blue: Top Position", BLUE_TOP_STARTPose2d);
        startingPose2D.addOption("Blue: Mid Position", BLUE_MID_STARTPose2d);
        startingPose2D.addOption("Blue: Bottom Position", BLUE_BOTTOM_STARTPose2d);

        startingPose2D.addOption("Red: Top Position", RED_TOP_STARTPose2d);
        startingPose2D.addOption("Red: Mid Position", RED_MID_STARTPose2d);
        startingPose2D.addOption("Red: Bottom Position", RED_BOTTOM_STARTPose2d);

        SmartDashboard.putData(startingPose2D);
    }

    public void drive(double Speed, double rotation) {
        var wheelSpeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(Speed * 3, 0.0, rotation * 3));
    }

    public DifferentialDriveKinematics geKinematics() {
        return kinematics;
    }

    public PIDController getRightPidController() {
        return righPidController;
    }

    public PIDController getLeftPidController() {
        return leftPidController;
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-gyro.getAngle());
    }

    public SimpleMotorFeedforward gSimpleMotorFeedforward() {
        return feedforward;
    }

    public DifferentialDriveWheelSpeeds getSpeeds() {
        return new DifferentialDriveWheelSpeeds(
                getLeftRate(),
                getRightRate());
    }

    public double getLeftRate() {
        // number of ticks per 100 ms -> m/s
        return left_master.getSelectedSensorVelocity()
                * (2 * Math.PI * constants.kWheelRadius / constants.kEncoderResolution);
    }

    /**
     * 
     * Gets the speed in m/s of the right side of the robot
     * 
     * @return double
     */
    public double getRightRate() {
        // number of ticks per 100 ms -> m/s
        return -right_master.getSelectedSensorVelocity()
                * (2 * Math.PI * constants.kWheelRadius / constants.kEncoderResolution);
    }

    public Pose2d getPose() {
        return pose;
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(pose, gyro.getRotation2d());
    }

    public void resetEncoders() {
        this.left_master.setSelectedSensorPosition(0);
        this.right_master.setSelectedSensorPosition(0);

    }

    public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
        final double leftFeedForward = feedforward.calculate(speeds.leftMetersPerSecond);
        final double rightFeedForward = feedforward.calculate(speeds.rightMetersPerSecond);

        final double leftOutput = leftPidController.calculate(getLeftRate(), speeds.leftMetersPerSecond);
        final double rightOutput = righPidController.calculate(getRightRate(), speeds.rightMetersPerSecond);

        leftMotors.setVoltage(leftOutput + leftFeedForward);
        rightMotors.setVoltage(rightOutput + rightFeedForward);

    }

    public void setOutput(double leftVolts, double rightVolts) {
        left_master.setVoltage(leftVolts);
        right_master.setVoltage(rightVolts);
    }

    @Override
    public void periodic() {
        pose = odometry.update(getHeading(), getLeftRate(), getRightRate());
        field.setRobotPose(odometry.getPoseMeters());

    }

    
    public double getLeftDistance(){
        return (left_master.getSelectedSensorPosition()/constants.kEncoderResolution) * (2 * Math.PI *constants.kWheelRadius);
    }

    public double getRightDistance(){
        return (right_master.getSelectedSensorPosition()/constants.kEncoderResolution) * (2 * Math.PI * constants.kWheelRadius);
    }

    public static synchronized DriveSubsystem getInstance() {
        if (instance == null) {
            instance = new DriveSubsystem();
        }
        return instance;
    }

    public RamseteCommand autoPath(String filePath) {

        Field2d field = new Field2d();
        String trajectoryJSON = "paths/"+filePath+".wpilib.json";
        Trajectory trajectory = new Trajectory();
    
        //need to talk to pranav on what the max voltage should be currently 10
        var autoVoltageConstraint = 
            new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                    constants.ksVolts,
                    constants.kvVoltSecondsPerMeter,
                    constants.kaVoltSecondsSquaredPerMeter),
                    geKinematics(),
                    10);

        //add constraints
        TrajectoryConfig config = new TrajectoryConfig(
            constants.maxVelocityMetersPerSecond,
            constants.maxAccelerationMetersPerSecondSq);
    
        config.setKinematics(geKinematics()).addConstraint(autoVoltageConstraint);

        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                new Pose2d(3, 0, new Rotation2d(0)),
                config);
        
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }
              
        

        RamseteController rController = new RamseteController(constants.kRamseteB, constants.kRamseteZeta);
    
        RamseteCommand command = new RamseteCommand(
            exampleTrajectory,
            this::getPose,
            rController,
            this.gSimpleMotorFeedforward(),
            this.geKinematics(),
            this::getSpeeds,
            this.getLeftPidController(),
            this.getRightPidController(),
            this::setOutput,
            this);
    
        SmartDashboard.putData(field);
        field.getObject("traj").setTrajectory(exampleTrajectory);
    
        resetOdometry(exampleTrajectory.getInitialPose());
    
        return command;
    
      }

   
    

    // public void goDistance(double distance){

    // double invert = 1;
    // if (distance < 0){
    // invert = -invert;
    // }
    // switch(driveState){
    // case READY_TO_DRIVE:
    // if (Math.abs(distance) > 0){
    // Double totalDrivetime =
    // (Math.abs(distance)/DrivetrainConstants.autoLinearSpeed) * 1000;
    // this.driveStartTime = System.currentTimeMillis();
    // this.driveFinishTime = this.driveStartTime + totalDrivetime;
    // driveState = DriveMode.DRIVING;
    // this.drive(invert * DrivetrainConstants.autoLinearSpeed, 0);
    // }
    // break;
    // case DRIVING:
    // this.currentTime = System.currentTimeMillis();
    // if (currentTime < driveFinishTime){
    // this.drive(invert * DrivetrainConstants.autoLinearSpeed,0);
    // }
    // else{
    // driveStartTime = 0;
    // driveFinishTime = 0;
    // currentTime = 0;
    // this.drive(0, 0);
    // Robot.driveButtonPressed = false;
    // driveState = DriveMode.READY_TO_DRIVE;
    // }

    // }

    // }

    // public enum DriveMode{
    // READY_TO_DRIVE,
    // DRIVING,

    // }

}
