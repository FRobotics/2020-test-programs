package frc.robot.base.subsystem;

import frc.robot.base.device.motor.EncoderMotor;
import frc.robot.base.device.motor.EncoderMotorConfig;

import java.util.Map;
import java.util.function.Supplier;

/**
 * A drive train with two encoder motors and a rate limiter for each motor that is controlled with a controller
 * It features two driving modes, closed loop and open loop in case something goes wrong
 * NOTE: rate limiters are currently disabled
 */
public class StandardDriveTrain extends Subsystem {

    private EncoderMotor leftMotor;
    private EncoderMotor rightMotor;
    private double absoluteMaxSpeed;
    private double currentMaxSpeed;

    private boolean useClosedLoop = true;

    public StandardDriveTrain(
            EncoderMotor leftMotor, EncoderMotor rightMotor,
            double maxAcceleration, double maxSpeed, double startMaxSpeed) {
        super("driveTrain");
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.currentMaxSpeed = startMaxSpeed;
        this.absoluteMaxSpeed = maxSpeed;
    }

    private double leftDemand = 0;
    private double leftOutputRaw = 0;

    private double rightDemand = 0;
    private double rightOutputRaw = 0;

    public void setLeftVelocity(double velocity) {
        velocity = safeVelocity(velocity);
        this.leftDemand = velocity;
        this.leftOutputRaw = this.leftMotor.setVelocity(velocity);
    }

    public void setRightVelocity(double velocity) {
        velocity = safeVelocity(velocity);
        this.rightDemand = velocity;
        this.rightOutputRaw = this.rightMotor.setVelocity(velocity);
    }

    public void setVelocity(double velocity) {
        setLeftVelocity(velocity);
        setRightVelocity(velocity);
    }

    public void setLeftPercentOutput(double percent) {
        percent = safePercent(percent);
        this.leftDemand = percent;
        this.leftOutputRaw = percent;
        this.leftMotor.setPercentOutput(percent);
    }

    public void setRightPercentOutput(double percent) {
        percent = safePercent(percent);
        this.rightDemand = percent;
        this.rightOutputRaw = percent;
        this.rightMotor.setPercentOutput(percent);
    }

    public void setPercentOutput(double percent) {
        this.setLeftPercentOutput(percent);
        this.setRightPercentOutput(percent);
    }

    public void setLeftVelOrPercent(double percent) {
        if(useClosedLoop) {
            this.setLeftVelocity(percent * getCurrentMaxSpeed());
        } else {
            this.setLeftPercentOutput(percent);
        }
    }

    public void setRightVelOrPercent(double percent) {
        if(useClosedLoop) {
            this.setRightVelocity(percent * getCurrentMaxSpeed());
        } else {
            this.setRightPercentOutput(percent);
        }
    }

    public void setVelOrPercent(double percent) {
        if(useClosedLoop) {
            this.setVelocity(percent * getCurrentMaxSpeed());
        } else {
            this.setPercentOutput(percent);
        }
    }

    public double safeVelocity(double velocity) {
        return Math.max(Math.min(velocity, currentMaxSpeed), -currentMaxSpeed);
    }

    public double safePercent(double percent) {
        return Math.max(Math.min(percent, 1), -1);
    }

    private double sensorRightDistanceFt = 0.0d;
    private double sensorRightVelocityFps = 0.0d;
    private double sensorRightVelocityRaw = 0.0d;
    private double sensorRightOutputPct = 0.0d;
    private double sensorLeftDistanceFt = 0.0d;
    private double sensorLeftVelocityFps = 0.0d;
    private double sensorLeftVelocityRaw = 0.0d;
    private double sensorLeftOutputPct = 0.0d;


    @Override
    public void acquire() {
        // --------read right drive motor
        sensorRightDistanceFt = rightMotor.getDistance();
        sensorRightVelocityFps = rightMotor.getVelocity();
        sensorRightVelocityRaw = rightMotor.getVelocityRaw();
        sensorRightOutputPct = rightMotor.getOutputPercent();
        // --------read left drive motor
        sensorLeftDistanceFt = leftMotor.getDistance();
        sensorLeftVelocityFps = leftMotor.getVelocity();
        sensorLeftVelocityRaw = leftMotor.getVelocityRaw();
        sensorLeftOutputPct = leftMotor.getOutputPercent();
    }


    @Override
    public void stop() {
        setPercentOutput(0);
        this.leftMotor.resetDistance();
        this.rightMotor.resetDistance();
    }

    //JAS made changes to not talk with hardware.
    @Override
    public Map<String, Supplier<Object>> NTSets() {
        return Map.ofEntries(
                Map.entry("left/velocity", this::getLeftVelocity), //leftMotor::getVelocity),
                Map.entry("left/distance", this::getLeftDistance), //leftMotor::getDistance),
                Map.entry("left/demand", () -> leftDemand),
                Map.entry("left/demandRaw", () -> leftOutputRaw),
                Map.entry("left/received/velocity", this::getLeftVelocityRaw), //leftMotor::getVelocityRaw),
                Map.entry("left/received/outputPercent", this::getLeftOutputPct), //eftMotor::getOutputPercent),

                Map.entry("right/velocity", this::getRightVelocity), //rightMotor::getVelocity),
                Map.entry("right/distance", this::getRightDistance), //rightMotor::getDistance),
                Map.entry("right/demand", () -> rightDemand),
                Map.entry("right/demandRaw", () -> rightOutputRaw),
                Map.entry("right/received/velocity", this::getRightVelocityRaw), //rightMotor::getVelocityRaw),
                Map.entry("right/received/outputPercent", this::getRightOutputPct), //rightMotor::getOutputPercent),

                Map.entry("closedLoopControl", () -> useClosedLoop)
        );
    }

    public void setCurrentMaxSpeed(double maxSpeed) {
        this.currentMaxSpeed = maxSpeed;
    }

    public double getCurrentMaxSpeed() {
        return this.currentMaxSpeed;
    }

    public void setClosedLoop(boolean useClosedLoop) {
        this.useClosedLoop = useClosedLoop;
    }

    public boolean isClosedLoop() { // added 3/7 KW
        return this.useClosedLoop;
    }

    public double getLeftDemand() {
        return this.leftDemand;
    }

    public double getRightDemand() {
        return this.rightDemand;
    }

    public double getAverageDemand() {
        return (this.leftDemand + this.rightDemand) / 2;
    }

    public double getLeftVelocity() {
        //JAS
        //return this.leftMotor.getVelocity();
        return sensorLeftVelocityFps;
    }

    public double getRightVelocity() {
        //JAS
        //return this.leftMotor.getVelocity();
        return sensorRightVelocityFps;
    }

    public double getAverageVelocity() {
        //JAS
        //return (this.leftMotor.getVelocity() + this.rightMotor.getVelocity()) / 2;
        return ( getLeftVelocity() + getRightVelocity() ) * 0.5d ;
    }
    
    public double getLeftDistance() {
        //JAS
        //return leftMotor.getDistance();
        return sensorLeftDistanceFt;
    }
    
    public double getRightDistance() {
        //JAS
        //return rightMotor.getDistance();
        return sensorRightDistanceFt;
    }

    public double getAverageDistance() {
        //JAS
        //return (leftMotor.getDistance() + rightMotor.getDistance()) / 2;
        return ( getRightDistance() + getLeftDistance() ) * 0.5d;
    }

    //JAS added
    public double getLeftOutputPct() {
        return sensorLeftOutputPct;
    }

    //JAS added
    public double getRightOutputPct() {
        return sensorRightOutputPct;
    }

    //JAS ADDED
    public double getLeftVelocityRaw() {
        return sensorLeftVelocityRaw;
    }

    //JAS ADDED
    public double getRightVelocityRaw() {
        return sensorRightVelocityRaw;
    }
    
    public double getAbsoluteMaxSpeed() {
        return this.absoluteMaxSpeed;
    }
    public void setLeftMotorConfig(EncoderMotorConfig config) {
        this.leftMotor.setConfig(config);
    }

    public void setRightMotorConfig(EncoderMotorConfig config) {
        this.rightMotor.setConfig(config);
    }

    public void setMotorConfigs(EncoderMotorConfig config) {
        setLeftMotorConfig(config);
        setRightMotorConfig(config);
    }

    public void resetDistance() {
        this.leftMotor.resetDistance();
        this.rightMotor.resetDistance();
    }
}
