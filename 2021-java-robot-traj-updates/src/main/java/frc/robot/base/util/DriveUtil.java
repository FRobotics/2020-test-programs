package frc.robot.base.util;

import frc.robot.base.input.Axis;
import frc.robot.base.input.Controller;
import frc.robot.base.subsystem.StandardDriveTrain;
import edu.wpi.first.networktables.NetworkTableEntry;
//import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.base.Controls;
import frc.robot.base.NTHandler;
import frc.robot.hailfire.subsystem.DriveTrain;

public class DriveUtil {

    // this stuff shouldn't be static but I don't think it'll ever matter and it's
    // easiest for now

    private static double controllerDeadBand = 0.2;
    private static int controllerPower = 2;

    //=============================================================================================================
    // -------- logic for standard teleop drive.
    public static void standardDrive(StandardDriveTrain driveTrain, Controller controller, boolean reverse) {
        int r = reverse ? -1 : 1;
        double fb = -r * Util.adjustInput(controller.getAxis(Controls.DriveTrain.DRIVE_FORWARD_BACKWARD),
                controllerDeadBand, controllerPower);
        double lr = -r * Util.adjustInput(controller.getAxis(Controls.DriveTrain.TURN_LEFT_RIGHT), controllerDeadBand,
                controllerPower);

        double left = fb - lr;
        double right = fb + lr;

        if (driveTrain.isClosedLoop()) {
            driveTrain.setLeftVelocity(left * driveTrain.getAbsoluteMaxSpeed());
            driveTrain.setRightVelocity(right * driveTrain.getAbsoluteMaxSpeed());
        } else {
            driveTrain.setLeftPercentOutput(left);
            driveTrain.setRightPercentOutput(right);
        }

        if (controller.buttonPressed(Controls.DriveTrain.USE_CLOSED_LOOP)) {
            driveTrain.setClosedLoop(true);
        }

        if (controller.buttonPressed(Controls.DriveTrain.DONT_USE_CLOSED_LOOP)) {
            driveTrain.setClosedLoop(false);
        }

        if (controller.getAxis(Axis.LEFT_TRIGGER) > 0.5) {
            driveTrain.resetDistance();
        }
    }

    private static Trajectory trajectory = new Trajectory();

    private static DifferentialDriveOdometry trajOdom;
    private static DifferentialDriveKinematics trajKine;
    private static RamseteController trajRamsete;

    private static double trajInitialGyro;
    private static double trajInitialLeft = 0.d;
    private static double trajInitialRight = 0.d;

    private static long pathStartTime = 0;

    private static double trajXErrorFt, trajYErrorFt, trajGyroErrorDeg = 0.d;

    private static boolean trajOnTarget = false;

    private static DriveTrain trajDriveTrain;

    //JAS added local storage to eliminate repeated lookups...
    private static NetworkTableEntry nte_trajAngleDelta; 
    private static NetworkTableEntry nte_trajLeftDistance;
    private static NetworkTableEntry nte_trajRightDistance;
    private static NetworkTableEntry nte_trajSampTime;

    private static NetworkTableEntry nte_trajOnTarget;
    private static NetworkTableEntry nte_trajXErrorFt;
    private static NetworkTableEntry nte_trajYErrorFt;
    private static NetworkTableEntry nte_trajGyroErrorDeg;

    private static NetworkTableEntry nte_trajRobotPosX;
    private static NetworkTableEntry nte_trajRobotPosY;
    private static NetworkTableEntry nte_trajRobotAngle;

    private static NetworkTableEntry nte_trajDesiredPosX;
    private static NetworkTableEntry nte_trajDesiredPosY;
    private static NetworkTableEntry nte_trajDesiredAngle;

    private static NetworkTableEntry nte_trajTotalTime;
    private static NetworkTableEntry nte_trajCurrentTime;



    //=============================================================================================================
    // -------- follow a path.  call every 20 ms until path is complete or driver wants to stop.
    public static void followPath() {

        //--------read sensor information
        double angleDelta = -(trajDriveTrain.getGyroAngle() - trajInitialGyro);
        double leftDist = trajDriveTrain.getLeftDistance() - trajInitialLeft;
        double rightDist = trajDriveTrain.getRightDistance() - trajInitialRight;
        double sampTime = (double)(System.currentTimeMillis() - pathStartTime) * 0.001d;
        double maxSpeed = trajDriveTrain.getCurrentMaxSpeed();

        //--------sample trajectory
        Trajectory.State currentState = trajectory.sample( sampTime );

        //--------update odometry -- robot's position on the field.
        Pose2d trajCurrentPosition = trajOdom.update(
                Rotation2d.fromDegrees(angleDelta),
                Units.feetToMeters(leftDist), 
                Units.feetToMeters(rightDist));

        //--------execute ramsete controller to calculate new robot speed demand.
        //trajRamsete.setEnabled(false);
        ChassisSpeeds trajChassisDmd = trajRamsete.calculate(trajCurrentPosition, currentState);
        DifferentialDriveWheelSpeeds trajWheelDmds = trajKine.toWheelSpeeds(trajChassisDmd);

        //--------normalize to ensure not going faster than possible.
        trajWheelDmds.normalize(Units.feetToMeters(maxSpeed));

        //--------output speed demand to wheels.
        trajDriveTrain.setLeftVelocity(Units.metersToFeet(trajWheelDmds.leftMetersPerSecond));
        trajDriveTrain.setRightVelocity(Units.metersToFeet(trajWheelDmds.rightMetersPerSecond));

        //--------calculate position error.
        Transform2d trajErrorPose = trajCurrentPosition.minus(currentState.poseMeters);
        trajXErrorFt = Units.metersToFeet(trajErrorPose.getX());
        trajYErrorFt = Units.metersToFeet(trajErrorPose.getY());
        trajGyroErrorDeg = trajErrorPose.getRotation().getDegrees();

        trajOnTarget = trajRamsete.atReference();

        //JAS added
        nte_trajAngleDelta.setDouble(angleDelta);
        nte_trajLeftDistance.setDouble(leftDist);
        nte_trajRightDistance.setDouble(rightDist);
        nte_trajSampTime.setDouble(sampTime);

        nte_trajOnTarget.setBoolean(trajOnTarget);
        nte_trajXErrorFt.setDouble(trajXErrorFt);
        nte_trajYErrorFt.setDouble(trajYErrorFt);
        nte_trajGyroErrorDeg.setDouble(trajGyroErrorDeg);

        nte_trajRobotPosX.setDouble(Units.metersToFeet(trajCurrentPosition.getX()));
        nte_trajRobotPosY.setDouble(Units.metersToFeet(trajCurrentPosition.getY()));
        nte_trajRobotAngle.setDouble(trajCurrentPosition.getRotation().getDegrees());

        nte_trajDesiredPosX.setDouble(Units.metersToFeet(currentState.poseMeters.getX()));
        nte_trajDesiredPosY.setDouble(Units.metersToFeet(currentState.poseMeters.getY()));
        nte_trajDesiredAngle.setDouble(currentState.poseMeters.getRotation().getDegrees());
    }

    //=============================================================================================================
    // -------- See if path is complete.
    public static boolean finishedPath() {

        double currentTimeSec = (double)( System.currentTimeMillis() - pathStartTime ) * 0.001d;
        boolean trajOnTime = currentTimeSec >= trajectory.getTotalTimeSeconds();
        boolean trajOutTime = currentTimeSec >= (trajectory.getTotalTimeSeconds() + 10.d);

        nte_trajTotalTime.setDouble(trajectory.getTotalTimeSeconds());
        nte_trajCurrentTime.setDouble(currentTimeSec);

        return (trajOnTime && trajOnTarget) || trajOutTime;
    }

    private static final double trackWidth = 24.d;

    //=============================================================================================================
    // -------- Initialize to start trajectory.
        //public static void startTrajectory(Trajectory t, double gyroAngle, double left, double right) {
    public static void startTrajectory(Trajectory t, DriveTrain myDriveTrain ) {

        trajDriveTrain = myDriveTrain;

        trajOdom = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0));
        trajOdom.resetPosition(new Pose2d(), Rotation2d.fromDegrees(0) );
        trajKine = new DifferentialDriveKinematics(Units.feetToMeters(trackWidth));
        trajRamsete = new RamseteController();

        trajInitialGyro = trajDriveTrain.getGyroAngle();
        trajInitialLeft = trajDriveTrain.getLeftDistance();
        trajInitialRight = trajDriveTrain.getRightDistance();

        trajectory = t;
        pathStartTime = System.currentTimeMillis();

        //JAS added save entry variable so repeated string lookup not required...
        nte_trajAngleDelta = NTHandler.getRobotEntry("traj/trajAngleDelta");
        nte_trajLeftDistance = NTHandler.getRobotEntry("traj/trajLeftDistance");
        nte_trajRightDistance = NTHandler.getRobotEntry("traj/trajRightDistance");
        nte_trajSampTime = NTHandler.getRobotEntry("traj/trajSampTime");

        nte_trajOnTarget = NTHandler.getRobotEntry("traj/trajOnTarget");
        nte_trajXErrorFt = NTHandler.getRobotEntry("traj/trajXErrorFt");
        nte_trajYErrorFt = NTHandler.getRobotEntry("traj/trajYErrorFt");
        nte_trajGyroErrorDeg = NTHandler.getRobotEntry("traj/trajGyroErrorDeg");

        nte_trajRobotPosX = NTHandler.getRobotEntry("traj/trajRobotPosX"); 
        nte_trajRobotPosY = NTHandler.getRobotEntry("traj/trajRobotPosY"); 
        nte_trajRobotAngle = NTHandler.getRobotEntry("traj/trajRobotAngle"); 

        nte_trajDesiredPosX = NTHandler.getRobotEntry("traj/trajDesiredPosX"); 
        nte_trajDesiredPosY = NTHandler.getRobotEntry("traj/trajDesiredPosY"); 
        nte_trajDesiredAngle = NTHandler.getRobotEntry("traj/trajDesiredAngle"); 

        nte_trajTotalTime = NTHandler.getRobotEntry("traj/trajTotalTime"); 
        nte_trajCurrentTime = NTHandler.getRobotEntry("traj/trajCurrentTime"); 


        //JAS added debug
        DriverStation.reportWarning(
            "DriveUtil.startTrajectory called - "+
            "gyro="+Double.toString(trajInitialGyro)+
            ", left="+Double.toString(trajInitialLeft)+
            ", right="+Double.toString(trajInitialRight)+
            ", time="+Long.toString(pathStartTime), false );

        //HAL.sendConsoleLine("DriveUtil.startTrajectory called - "+
        //    "gyro="+Double.toString(trajInitialGyro)+
        //    ", left="+Double.toString(trajInitialLeft)+
        //    ", right="+Double.toString(trajInitialRight)+
        //    ", time="+Long.toString(pathStartTime) );
    }
}
