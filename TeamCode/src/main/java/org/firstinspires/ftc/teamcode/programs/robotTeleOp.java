package org.firstinspires.ftc.teamcode.programs;
import static org.firstinspires.ftc.teamcode.robotconfigs.robotConfig.imu;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.robotconfigs.robotConfig.*;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.base.Commands;
import org.firstinspires.ftc.teamcode.base.Components;
import org.firstinspires.ftc.teamcode.robotconfigs.robotConfig;
@TeleOp
public class robotTeleOp extends LinearOpMode {
    robotConfig robot = new robotConfig();
    double target = TARGET_PICKUP;
    boolean buttonAPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Components.initialize(this, robot, false, true);
        Components.activateActuatorControl();
        waitForStart();

            if (isStopRequested()) return;

            int iterations = 0;
            while (opModeIsActive()) {



                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x;
                double rx = -gamepad1.right_stick_x;

                if (gamepad1.options) {
                    imu.resetYaw();
                }

                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                rotX = rotX * 1.1;

                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                double frontLeftPower = (rotY + rotX + rx) / denominator;
                double backLeftPower = (rotY - rotX + rx) / denominator;
                double frontRightPower = (rotY - rotX - rx) / denominator;
                double backRightPower = (rotY + rotX - rx) / denominator;


                if (gamepad1.a && !buttonAPressed) {
                    Commands.executor.setCommands(
                            new Commands.SequentialCommand(
                                    new Commands.InstantCommand(() ->
                                            wrist.setTarget(WRIST_PICKUP)
                                    ),
                                    new Commands.SleepCommand(.7),
                                    new Commands.InstantCommand(() ->
                                            target = 0
                                    )
                            )
                    );
                }

                else if (gamepad1.left_trigger>0.5) {
                    frontLeftPower = 0.5 * frontLeftPower;
                    backLeftPower = 0.5 * backLeftPower;
                    frontRightPower = 0.5 * frontRightPower;
                    backRightPower = 0.5 * backRightPower;
                } else if (gamepad1.b){ // Low junction
                    target = TARGET_LOW;

                }else if (gamepad1.x){ // Mid junction
                    target = TARGET_MID;
                    arm1.setTarget(ARM_SCORE);
                    arm2.setTarget(ARM_SCORE);
                    wrist.setTarget(WRIST_SCORE);

                }else if (gamepad1.y){ // High junction
                    target = TARGET_HIGH;
                    arm1.setTarget(ARM_SCORE);
                    arm2.setTarget(ARM_SCORE);
                    wrist.setTarget(WRIST_SCORE);
                }
                if (gamepad1.left_bumper){
                    if (target < TARGET_MAX){
                        target += 15;
                    }

                }else if (gamepad1.right_bumper){
                    if (target > TARGET_PICKUP){
                        target -= 15;
                    }
                }

                buttonAPressed = gamepad1.a;

                Commands.executor.runOnce();

                claw.setPosition(1-gamepad1.right_trigger);

                slide1.setTarget(target);
                slide2.setTarget(target);
                telemetry.addData("slide1 pos", slide1.getCurrentPosition());
                telemetry.addData("slide2 pos", slide2.getCurrentPosition());
                telemetry.addData("iterations:",++iterations);
                telemetry.addData("Slide1:",slide1.getCurrentPosition());
                telemetry.addData("Slide2:",slide2.getCurrentPosition());
                telemetry.addData("Target:",target);
                telemetry.update();

                leftFront.setPower(frontLeftPower);
                leftBack.setPower(backLeftPower);
                rightFront.setPower(frontRightPower);
                rightBack.setPower(backRightPower);
            }
    }
}

