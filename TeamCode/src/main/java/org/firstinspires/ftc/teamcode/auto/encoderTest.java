package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;


@Autonomous(name="encoderTest")
public class encoderTest extends AutoHardware {
    @Override
    public void runOpMode() {

        initAll();
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        sleep(1000);
        extender.setTargetPosition(-1);

        sleep(1000);

        //rotator.setTargetPosition(1);
        //rotator.setTargetPosition(0);

        //sleep(2000);

    }
    }