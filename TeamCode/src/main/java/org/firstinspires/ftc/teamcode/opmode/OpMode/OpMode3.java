package org.firstinspires.ftc.teamcode.opmode.OpMode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.common.hardware.BotCoefficients;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

//a
@TeleOp(name = "OpMode3")

public class OpMode3 extends LinearOpMode {
    public ElapsedTime mRunTime = new ElapsedTime();

    RobotHardware robot = new RobotHardware();

    private int sleepMs1 = 0;
    private boolean bMoveUpHogback = false;
    private boolean wasPusherMoving = false;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        // Put initialization blocks here.

        waitForStart();
        while (opModeIsActive()) {
            double horizontal = -1.0 * gamepad1.right_stick_x * 0.6;
            double vertical = gamepad1.right_stick_y * 0.6;
            double turn = -1.0 * gamepad1.left_stick_x * 0.6;

            double flPower = vertical - turn - horizontal;
            double frPower = vertical + turn - horizontal;
            double blPower = vertical - turn + horizontal;
            double brPower = vertical + turn + horizontal;
            double scaling = Math.max(1.0,
                    Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)),
                            Math.max(Math.abs(blPower), Math.abs(brPower))));
            flPower = flPower / scaling;
            frPower = frPower / scaling;
            blPower = blPower / scaling;
            brPower = brPower / scaling;
            robot.setDrivePower(flPower, frPower, blPower, brPower);

            telemetry.addLine(String.format("FL: %d \nBL %d \nFR: %d \nBR: %d ",
                    robot.motorfl.getCurrentPosition(),
                    robot.motorbl.getCurrentPosition(),
                    robot.motorfr.getCurrentPosition(),
                    robot.motorbr.getCurrentPosition()
            ));


//make sure one of the directions is correct/reversed

//            if (gamepad2.left_trigger_pressed) {
//                robot.pusher0.setPower(1.0);
//                wasPusherMoving = true;
//            } else if (gamepad2.left_bumper) {
//                robot.pusher0.setPower(-1.0);
//                wasPusherMoving = true;
//            } else {
//                // fire a tiny counter-pulse to stop movement
//                if (wasPusherMoving) {
//                    double brakePower;
//                    if (robot.pusher0.getPower() < 0) { // 2 conditions: forward or backward
//                        brakePower = 0.4;   // pulse backward
//                    } else {
//                        brakePower = -0.4;  // pulse forward
//                    }
//                    robot.pusher0.setPower(brakePower);
//                    sleep(30); // 30ms hardware pulse to stop completely
//                    wasPusherMoving = false;
//                }
//
//                // Hard stop signal
//                robot.pusher0.setPower(0.0);
//            }



            // 1. Moving Forward
            if (gamepad2.right_stick_y > 0.5 ) {
                robot.hogback.setPower(BotCoefficients.HOGBACK_POWERUP);
            }
// 2. Moving Backward
            else if (gamepad2.right_stick_y < -0.5) {
                robot.hogback.setPower(BotCoefficients.HOGBACK_POWERDOWN);
            }
// 3. Joystick is back in the neutral/original position
            else {
                robot.hogback.setPower(BotCoefficients.HOGBACK_STOP); // Sets power to 0.0
            }
        }


    }
}