package frc.robot.base.util;

import frc.robot.base.input.Axis;
import frc.robot.base.input.Controller;
import frc.robot.base.subsystem.StandardDriveTrain;
import edu.wpi.first.hal.HAL;
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
        NTHandler.getRobotEntry("trajAngleDelta").setDouble(angleDelta);
        NTHandler.getRobotEntry("trajLeftDistance").setDouble(leftDist);
        NTHandler.getRobotEntry("trajRightDistance").setDouble(rightDist);
        NTHandler.getRobotEntry("trajSampTime").setDouble(sampTime);
        NTHandler.getRobotEntry("trajOnTarget").setBoolean(trajOnTarget);


        NTHandler.getRobotEntry("trajXErrorFt").setDouble(trajXErrorFt);
        NTHandler.getRobotEntry("trajYErrorFt").setDouble(trajYErrorFt);
        NTHandler.getRobotEntry("trajGyroErrorDeg").setDouble(trajGyroErrorDeg);

        NTHandler.getRobotEntry("trajRobotPosX").setDouble(Units.metersToFeet(trajCurrentPosition.getX()));
        NTHandler.getRobotEntry("trajRobotPosY").setDouble(Units.metersToFeet(trajCurrentPosition.getY()));
        NTHandler.getRobotEntry("trajRobotAngle").setDouble(trajCurrentPosition.getRotation().getDegrees());

        NTHandler.getRobotEntry("trajDesiredPosX").setDouble(Units.metersToFeet(currentState.poseMeters.getX()));
        NTHandler.getRobotEntry("trajDesiredPosY").setDouble(Units.metersToFeet(currentState.poseMeters.getY()));
        NTHandler.getRobotEntry("trajDesiredAngle").setDouble(currentState.poseMeters.getRotation().getDegrees());
    }

    public static boolean finishedPath() {

        double currentTimeSec = (double)( System.currentTimeMillis() - pathStartTime ) * 0.001d;
        boolean trajOnTime = currentTimeSec >= trajectory.getTotalTimeSeconds();
        boolean trajOutTime = currentTimeSec >= (trajectory.getTotalTimeSeconds() + 10.d);
        NTHandler.getRobotEntry("trajTotalTime").setDouble(trajectory.getTotalTimeSeconds());
        NTHandler.getRobotEntry("trajCurrentTime").setDouble(currentTimeSec);

        return (trajOnTime && trajOnTarget) || trajOutTime;
    }

    private static final double trackWidth = 24.d;

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

        //JAS added debug
        HAL.sendConsoleLine("DriveUtil.startTrajectory called - "+
            "gyro="+Double.toString(trajInitialGyro)+
            ", left="+Double.toString(trajInitialLeft)+
            ", right="+Double.toString(trajInitialRight)+
            ", time="+Long.toString(pathStartTime) );
    }
}
