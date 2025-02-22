package org.firstinspires.ftc.teamcode.opmode.OpMode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

//
@TeleOp(name = "OpMode3")

public class OpMode3 extends LinearOpMode {
    public ElapsedTime mRunTime = new ElapsedTime();

    RobotHardware robot = new RobotHardware();

    private int sleepMs1 = 0;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

//        robot.liftHex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.liftHex.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        robot.liftHex.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Put initialization blocks here.
        waitForStart();
        while (opModeIsActive()) {
            double horizontal = gamepad1.left_stick_x * -0.5;
            double vertical = -gamepad1.left_stick_y * 0.5;
            double turn = gamepad1.right_stick_x * 0.5;

            robot.setDrivePower(vertical + turn + horizontal, vertical - turn - horizontal, vertical + turn - horizontal, vertical - turn + horizontal);

            telemetry.addLine(String.format("FL: %d \nBL %d \nFR: %d \nBR: %d ",
                    robot.motorfl.getCurrentPosition(),
                    robot.motorbl.getCurrentPosition(),
                    robot.motorfr.getCurrentPosition(),
                    robot.motorbr.getCurrentPosition()
            ));


//make sure one of the directions is correct/reversed


            //lift arm start
/*
            if (gamepad2.b) { //if button a pressed
                robot.liftHex.setPower(0.8);
                //tilt the lift to be upright
                sleep(1000);
                robot.liftHex.setPower(0);
            }
           robot.liftHex.setPower(0);

 */

            while (gamepad2.b) { //if button a pressed
                // robot.liftHex.setPower(1.0);
                robot.liftHex.setPower(1.0);
                //tilt the lift to be upright
            }
            robot.liftHex.setPower(0);


            while (gamepad2.x) {
                //robot.liftHex.setPower(-1.0);
                robot.liftHex.setPower(-1.0);
            }
            robot.liftHex.setPower(0);


            while (gamepad2.a) { //if button a pressed
                robot.liftArm.setPower(0.5);
                //tilt the lift to be upright
            }
            robot.liftArm.setPower(0);


            while (gamepad2.y) {
                robot.liftArm.setPower(-0.5);
            }
            robot.liftArm.setPower(0);



            //if(gamepad1.right_trigger > 0.7){
            //robot.airplaneLauncher.setPosition(1.0);


//grabber
            if (gamepad2.left_trigger > 0.5) {
                robot.grabServoLeft.setPosition(1.0); // open
            } else if (gamepad2.left_bumper) {
                robot.grabServoLeft.setPosition(0.0); // close
            }


//tilt servo
            if (gamepad2.right_stick_y > 0.7) {
                robot.tiltServoLeft.setPosition(0.0);

            } else if (gamepad2.right_stick_y < -0.7) {
                robot.tiltServoLeft.setPosition(0.8);
            }

//tilt servo #2
            if (gamepad2.left_stick_y > 0.7) {
                robot.tiltServoLeft.setPosition(0.0);

            } else if (gamepad2.left_stick_y < -0.7) {
                robot.tiltServoLeft.setPosition(0.6);
            }


        }


        //emergency releases
    }


    void liftHexArm(int ticks, double power, long timeOutMills) {
        long timeCurrent, timeBegin;
        timeBegin = timeCurrent = System.currentTimeMillis();
        {

        }



     /*

        robot.liftHex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftHex.setTargetPosition(ticks);
        robot.liftHex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftHex.setPower(power);
        while(opModeIsActive()
                && robot.liftHex.isBusy()
                && (timeCurrent - timeBegin) < timeOutMills) {
            timeCurrent = System.currentTimeMillis();





    private void TiltLiftOne ( double crankPowerBegin, int crankTimeMs, double crankPowerEnd,
        double liftPowerBegin, int liftTimeMs, double liftPowerEnd){
            //tilt the lift to be upright
            robot.liftHex.setPower(crankPowerBegin);   //set motor power
            sleep(crankTimeMs);          // let motor run for some time seconds.
            robot.liftHex.setPower(crankPowerEnd);   //set lower motor power to maintain the position

            // Extend liftArm
            robot.liftArm.setPower(liftPowerBegin);
            sleep(liftTimeMs);             // let motor run for some time seconds.
            robot.liftArm.setPower(liftPowerEnd);

        }

        }


      */
    }}