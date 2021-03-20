package frc.robot.hailfire.subsystem;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.analog.adis16448.frc.ADIS16448_IMU;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

//import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.base.subsystem.StandardDriveTrain;
//JAS uncomment if using alt gyro code.
//import frc.robot.base.util.ALT_ADIS16448_IMU;
import frc.robot.base.util.DriveUtil;
import frc.robot.base.util.PosControl;
import frc.robot.base.util.Util;
import frc.robot.hailfire.Controls;
import frc.robot.hailfire.IDs;
import frc.robot.base.device.motor.PhoenixMotorPair;
import frc.robot.base.device.DoubleSolenoid4150;
import frc.robot.base.device.Pixy;
import frc.robot.hailfire.MotorConfig;
import frc.robot.hailfire.Vision;
import edu.wpi.first.wpilibj.SPI;

public class DriveTrain extends StandardDriveTrain {

    private boolean reverseControl = false;
    private static ADIS16448_IMU gyro; //JAS moved init, removed final, made static = new ADIS16448_IMU( );
    //JAS switch definition if using alt Gyro code.
    //private static ALT_ADIS16448_IMU gyro; //JAS moved init, removed final, made static = new ADIS16448_IMU( );

    private static final double LOW_MAX_SPEED = 5.5;

    private DoubleSolenoid4150 evoShifter = new DoubleSolenoid4150(
            IDs.DriveTrain.LEFT_EVO_SHIFTER_FORWARD,
            IDs.DriveTrain.LEFT_EVO_SHIFTER_REVERSE
    );

    private final Pixy pixy = new Pixy(3);

    private boolean autoShift = false;
    private boolean autoAim = false;
    
    public final Trajectory STRAIGHT = Util.loadTrajectory("/home/lvuser/Trajectory/test01_straight.json");
    public final Trajectory TURN_LEFT = Util.loadTrajectory("/home/lvuser/Trajectory/test02_turnLeft.json");
    public final Trajectory TURN_RIGHT = Util.loadTrajectory("/home/lvuser/Trajectory/test03_turnRight.json");
    public final Trajectory BACK_TO_START = Util.loadTrajectory("/home/lvuser/Trajectory/test04_BackToStart.json");
    
    public static PhoenixMotorPair createMotor(int master, int follower) {
        var motor = new PhoenixMotorPair(
            new TalonSRX(master),
            new VictorSPX(follower),
            MotorConfig.DriveTrain.LOW_CONFIG
        );
        motor.setNeutralMode(NeutralMode.Brake);
        motor.setRampTime(0.5);
        return motor;
    }

    public DriveTrain() {
        super(
                new PhoenixMotorPair(
                        new TalonSRX(IDs.DriveTrain.LEFT_MOTOR_MASTER),
                        new VictorSPX(IDs.DriveTrain.LEFT_MOTOR_FOLLOWER),
                        MotorConfig.DriveTrain.LOW_CONFIG
                ).invert(),
                new PhoenixMotorPair(
                        new TalonSRX(IDs.DriveTrain.RIGHT_MOTOR_MASTER),
                        new VictorSPX(IDs.DriveTrain.RIGHT_MOTOR_FOLLOWER),
                        MotorConfig.DriveTrain.LOW_CONFIG
                ),
                10, 19, LOW_MAX_SPEED);

        //JAS moved gyro init to constructor.  Used different call to set longer cal time.
        //--------calibrate the gyro....
        //                        yaw axis, port, cal time
        gyro = new ADIS16448_IMU( ADIS16448_IMU.IMUAxis.kZ, SPI.Port.kMXP, 8 ); // 8 second cal time
        //ALT gyro with potentially more accurate cal routine.
        //gyro = new ALT_ADIS16448_IMU( ADIS16448_IMU.IMUAxis.kZ, SPI.Port.kMXP, 8 ); // 8 second cal time
    }
    
    private PosControl posControl;
    private double angleX = 0;

    //JAS added local storage for sensor data
    private double sensorGyroAngle = 0.0d;
    private double sensorGyroAngleX = 0.0d;
    private double sensorGyroAngleY = 0.0d;
    private double sensorGyroAngleZ = 0.0d;
    private double sensorGyroTemp = 0.0d;
    private int sensorAcquireCalled = 0;

    //JAS added common sensor acquire routine
    @Override
    public void acquire() {
        // --------read gyro
        sensorGyroAngle = gyro.getAngle();
        sensorGyroAngleX = gyro.getGyroAngleX();
        sensorGyroAngleY = gyro.getGyroAngleY();
        sensorGyroAngleZ = gyro.getGyroAngleZ();
        sensorGyroTemp = gyro.getTemperature() * 1.8d + 32.0d;
        // --------read sensors from base class.
        super.acquire();
        sensorAcquireCalled = (sensorAcquireCalled+1) % 2048;
    }

    @Override
    public void control() {

        double turnSpeed = 0.2;

        if (Controls.DriveTrain.TURN_RIGHT()){
            setLeftVelOrPercent(-turnSpeed);
            setRightVelOrPercent(turnSpeed);
            this.autoAim = false;
        } else if (Controls.DriveTrain.TURN_LEFT()){
            setLeftVelOrPercent(turnSpeed);
            setRightVelOrPercent(-turnSpeed);
            this.autoAim = false;
        } else {
            if (Controls.DriveTrain.AUTO_AIM()) {
                // TODO: these are complete guesses
                posControl = new PosControl(0, 0.1, 0.2, 0.5, 2);
                this.autoAim = true;
            }
            if (this.autoAim) {
                // TODO: I have no idea if these units are compatible or if the direction is correct lmao
                double calculatedSpeed = posControl.getSpeed(angleX);
                System.out.println("angle: " + angleX + " / calcspeed: " + calculatedSpeed);
                this.setLeftVelOrPercent(calculatedSpeed);
                this.setRightVelOrPercent(-calculatedSpeed);
            } else {
                DriveUtil.standardDrive(this, Controls.drive, reverseControl);
                if (Controls.DriveTrain.TOGGLE_REVERSE()) {
                    this.toggleReversed();
                }
            }
        }

        // shift gears

        if(Controls.DriveTrain.LOW_GEAR()){
            shiftToLowGear();
            autoShift = false;
        }

        if (Controls.DriveTrain.HIGH_GEAR()) {
            shiftToHighGear();
            autoShift = false;
        }

        if(Controls.DriveTrain.AUTO_SHIFT()) {
            autoShift = true;
        }

        if(autoShift) {
            if (
                Math.abs(getAverageDemand()) > 5
                && Math.abs(getAverageVelocity()) > 5
            ) {
                shiftToHighGear();
            }

            if (
                Math.abs(getAverageDemand()) < 4.5
                && Math.abs(getAverageVelocity()) < 4.5
            ) {
                shiftToLowGear();
            }
        }
    }

    public void shiftToHighGear() {
        if(evoShifter.extend()) {
            setMotorConfigs(MotorConfig.DriveTrain.HIGH_CONFIG);
            setCurrentMaxSpeed(getAbsoluteMaxSpeed());
        }
    }

    public void shiftToLowGear() {
        if(evoShifter.retract()) {
            setMotorConfigs(MotorConfig.DriveTrain.LOW_CONFIG);
            setCurrentMaxSpeed(LOW_MAX_SPEED);
        }
    }

    //JAS added
    public double getGyroAngle() {
        return sensorGyroAngle;
    }
    //JAS added
    public double getGyroAngleX() {
        return sensorGyroAngleX;
    }
    //JAS added
    public double getGyroAngleY() {
        return sensorGyroAngleY;
    }
    //JAS added
    public double getGyroAngleZ() {
        return sensorGyroAngleZ;
    }
    //JAS added
    public double getGyroTemp() {
        return sensorGyroTemp;
    }
    //JAS added
    public double getAcquireCalled() {
        return (double)sensorAcquireCalled;
    }

    PosControl aimPosControl = new PosControl(0, 1, 0.5, 0.2, 0.5);;

    public void autoAim() {
        /* 
         * TODO: either use this code or discard it;
         * this constantly updates the target and uses a watchdog for safety in the vision class
         * I don't think we'll need this though + we'd have to change vision to work this way again
         */
        if(!Vision.isStale()) {
            double calculatedSpeed = aimPosControl.getSpeed(Vision.getYawOffset());
            this.setLeftVelOrPercent(-calculatedSpeed);
            this.setRightVelOrPercent(calculatedSpeed);
        }
    }

    @Override
    public Map<String, Consumer<Object>> NTGets() {
        return Map.ofEntries(Util.<Double>setter("/vision/data/OffsetX", 
            a -> this.angleX = a
        ));
    }
    
    @Override
    public Map<String, Supplier<Object>> NTSets() {
        Map<String, Supplier<Object>> sets = new HashMap<>();
        sets.putAll(super.NTSets());
        sets.putAll(Map.of(
            "pixyReading", pixy::read,
            "gyroAngle", this::getGyroAngle,
            "gyroAngleX", this::getGyroAngleX,
            "gyroAngleY", this::getGyroAngleY,
            "gyroAngleZ", this::getGyroAngleZ,
            "gyroTemp", this::getGyroTemp,
            "acquireCalled", this::getAcquireCalled
        ));
        return sets;
    }

    public void setReversed(boolean reverse) {
        this.reverseControl = reverse;
    }

    public void toggleReversed() {
        this.reverseControl = !this.reverseControl;
    }
}
