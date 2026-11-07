
package org.firstinspires.ftc.teamcode.opmode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.hardware.BotCoefficients;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

@TeleOp(name = "OpMode3")
public class OpMode3 extends LinearOpMode {

    private final RobotHardware robot = new RobotHardware();

    private static final double DRIVE_SPEED = 0.6;
    private static final double INTAKE_FORWARD_VELOCITY = -0.8;

    private final ElapsedTime pusherBrakeTimer = new ElapsedTime();

    private double lastPusherDirection = 0.0;
    private boolean pusherBraking = false;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        telemetry.addLine("OpMode3 Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // =========================
            // MECANUM DRIVE - GAMEPAD 1
            // =========================

            double y = -gamepad1.left_stick_y * DRIVE_SPEED;
            double x = gamepad1.left_stick_x * DRIVE_SPEED;
            double rx = gamepad1.right_stick_x * DRIVE_SPEED;

            double denominator = Math.max(
                    Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

            double flPower = (y + x + rx) / denominator;
            double blPower = (y - x + rx) / denominator;
            double frPower = (y - x - rx) / denominator;
            double brPower = (y + x - rx) / denominator;

            robot.setDrivePower(
                    flPower, frPower, blPower, brPower);

            // =========================
            // INTAKE - GAMEPAD 2
            // A = START
            // B = STOP
            // =========================

            if (gamepad2.y) {
                robot.intake.setPower(0.5);
            }else if(gamepad2.a){
                robot.intake.setPower(-0.5);
            }
            if (gamepad2.b) {
                robot.intake.setPower(0.0);
            }

            // =========================
            // PUSHER - GAMEPAD 2
            // LEFT TRIGGER = FORWARD
            // LEFT BUMPER = REVERSE
            // =========================

            if (gamepad2.left_trigger > 0.5) {

             //   robot.pusher0.setPower(1.0);
                robot.pusher1.setPower(-1.0);
                lastPusherDirection = 1.0;
                pusherBraking = false;

            } else if (gamepad2.left_bumper) {

              //  robot.pusher0.setPower(-1.0);
                robot.pusher1.setPower(1.0);
                lastPusherDirection = -1.0;
                pusherBraking = false;

            } else if (!pusherBraking && lastPusherDirection != 0.0) {

                // Brief counterpulse when the pusher control is released
              //  robot.pusher0.setPower(-lastPusherDirection);
                robot.pusher1.setPower(lastPusherDirection);
                pusherBrakeTimer.reset();
                pusherBraking = true;

            } else if (pusherBraking
                    && pusherBrakeTimer.milliseconds() < 30) {

              //  robot.pusher0.setPower(-lastPusherDirection);
                robot.pusher1.setPower(lastPusherDirection);

            } else {

             //   robot.pusher0.setPower(0.0);
                robot.pusher1.setPower(0.0);
                lastPusherDirection = 0.0;
                pusherBraking = false;
            }

            // =========================
            // HOGBACK - GAMEPAD 2
            // RIGHT STICK Y
            // =========================

            if (gamepad2.right_stick_y < -0.5) {

                robot.hogback.setPower(
                        BotCoefficients.HOGBACK_POWERUP);

            } else if (gamepad2.right_stick_y > 0.5) {

                robot.hogback.setPower(
                        BotCoefficients.HOGBACK_POWERDOWN);

            } else {

                robot.hogback.setPower(
                        BotCoefficients.HOGBACK_STOP);
            }

            // =========================
            // ENCODER TELEMETRY
            // =========================

            telemetry.addData(
                    "FL Encoder", robot.motorfl.getCurrentPosition());
            telemetry.addData(
                    "BL Encoder", robot.motorbl.getCurrentPosition());
            telemetry.addData(
                    "FR Encoder", robot.motorfr.getCurrentPosition());
            telemetry.addData(
                    "BR Encoder", robot.motorbr.getCurrentPosition());

            telemetry.addData(
                    "Intake", gamepad2.b ? "Stopping" :
                            gamepad2.a ? "Running" : "Holding State");

            telemetry.update();
        }

        // Stop motors when TeleOp ends
        robot.setAllDrivePower(0.0);
        robot.intake.setPower(0.0);
        robot.hogback.setPower(0.0);
      //  robot.pusher0.setPower(0.0);
    }
}
