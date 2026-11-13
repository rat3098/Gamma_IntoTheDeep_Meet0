package org.firstinspires.ftc.teamcode.common.hardware;


import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


public class RobotHardware {
    /* Declare OpMode members. */
    HardwareMap hwMap =  null;

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    public DcMotor motorfl = null;
    public DcMotor motorfr = null;
    public DcMotor motorbr = null;

    public DcMotor motorbl = null;


    public DcMotor hogback = null;

    public DcMotor intake1 = null;

    public DcMotor intake2 = null;


    public Servo gate = null;

    //public CRServo pusher0 = null;

    public IMU imu;

    // Initial robot orientation
    public YawPitchRollAngles orientation0;
    public AngularVelocity angularVelocity0;
    public double yaw0;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware() {}

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     *
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init(HardwareMap ahwMap)    {
        // save reference to hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        motorfl = hwMap.get(DcMotor.class, "motorfl");
        motorfr = hwMap.get(DcMotor.class, "motorfr");
        motorbl = hwMap.get(DcMotor.class, "motorbl");
        motorbr = hwMap.get(DcMotor.class, "motorbr");
        hogback = hwMap.get(DcMotor.class, "hogback");
        intake1 = hwMap.get(DcMotor.class, "intake1");
        intake2 = hwMap.get(DcMotor.class, "intake2");
        gate = hwMap.get(Servo.class, "gate");
       // limelight = hwMap.get(Limelight3A.class, "limelight");
      //  pusher0 = hwMap.get(CRServo.class, "pusher0");


        // set Brake zero power behavior
        motorfr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorfl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorbr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorbl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hogback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorfl.setDirection(DcMotor.Direction.REVERSE);
        motorbl.setDirection(DcMotor.Direction.REVERSE);

      //  pusher0.setPower(0);
        gate.setPosition(0.0);
       // limelight.stop();

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        setDrivetrainMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Initialize IMU in the control hub
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu = hwMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        // Retrieve the very initial Rotational Angles and Velocities
        orientation0 = imu.getRobotYawPitchRollAngles();
        angularVelocity0 = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        yaw0 = orientation0.getYaw(AngleUnit.DEGREES);
    }

    public void setAutoDriveMotorMode() {
        motorfr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorfl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorbr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorbl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorfr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorfl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorbr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorbl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorfr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorfl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorbr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorbl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public double getCurrentYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void setDrivetrainMode(DcMotor.RunMode mode) {
        motorfl.setMode(mode);
        motorfr.setMode(mode);
        motorbl.setMode(mode);
        motorbr.setMode(mode);
    }
    public void setDriveForward(double fl, double fr, double bl, double br){
        if (fl > 1.0)
            fl = 1.0;
        else if (fl < -1.0)
            fl = -1.0;

        if (fr > 1.0)
            fr = 1.0;
        else if (fr < -1.0)
            fr = -1.0;

        if (bl > 1.0)
            bl = 1.0;
        else if (bl < -1.0)
            bl = -1.0;

        if (br > 1.0)
            br = 1.0;
        else if (br < -1.0)
            br = -1.0;

        motorfl.setPower((-1) * fl);
        motorfr.setPower(fr);
        motorbl.setPower(bl);
        motorbr.setPower(br);
    }


    public void setDrivePower(double fl, double fr, double bl, double br) {
        if (fl > 1.0)
            fl = 1.0;
        else if (fl < -1.0)
            fl = -1.0;

        if (fr > 1.0)
            fr = 1.0;
        else if (fr < -1.0)
            fr = -1.0;

        if (bl > 1.0)
            bl = 1.0;
        else if (bl < -1.0)
            bl = -1.0;

        if (br > 1.0)
            br = 1.0;
        else if (br < -1.0)
            br = -1.0;

        motorfl.setPower(fl);
        motorfr.setPower(fr);
        motorbl.setPower(bl);
        motorbr.setPower(br);
    }

    public void setAllDrivePower(double p){ setDrivePower(p,p,p,p);}
    // public void setArmPower(double armPower){
    //   linearSlider.setPower(armPower);
    //}






}
