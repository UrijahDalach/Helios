package org.firstinspires.ftc.teamcode.programs;
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
    double frontLeftPower = 0;
    double backLeftPower = 0;
    double frontRightPower = 0;
    double backRightPower = 0;
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
            frontLeftPower = (rotY + rotX + rx) / denominator;
            backLeftPower = (rotY - rotX + rx) / denominator;
            frontRightPower = (rotY - rotX - rx) / denominator;
            backRightPower = (rotY + rotX - rx) / denominator;

            Commands.executor.setCommands(
                new Commands.SequentialCommand(
                    new Commands.ConditionalCommand(
                            new Commands.IfThen(() -> gamepad1.a && !buttonAPressed,
                                new Commands.SequentialCommand(
                                        new Commands.InstantCommand(() ->
                                                wrist.setTarget(WRIST_PICKUP)
                                        ),
                                        new Commands.SleepCommand(.7),
                                        new Commands.InstantCommand(() ->
                                                target = 0
                                        )
                                )
                            )
                    ),
                    new Commands.ConditionalCommand(
                            new Commands.IfThen(() -> gamepad1.left_trigger>0.5,
                                    new Commands.InstantCommand(() -> {
                                            frontLeftPower = 0.5 * frontLeftPower;
                                            backLeftPower = 0.5 * backLeftPower;
                                            frontRightPower = 0.5 * frontRightPower;
                                            backRightPower = 0.5 * backRightPower;
                                    })
                            )
                    ),
                    new Commands.ConditionalCommand(
                        new Commands.IfThen(() -> gamepad1.b,
                            new Commands.InstantCommand(() -> {
                                target = TARGET_LOW;
                            })
                        )
                    ),
                    new Commands.ConditionalCommand(
                            new Commands.IfThen(() -> gamepad1.x,
                                    new Commands.InstantCommand(() -> {
                                        target = TARGET_MID;
                                        arm1.setTarget(ARM_SCORE);
                                        arm2.setTarget(ARM_SCORE);
                                        wrist.setTarget(WRIST_SCORE);
                                    })
                            )
                    ),
                    new Commands.ConditionalCommand(
                            new Commands.IfThen(() -> gamepad1.y,
                                    new Commands.InstantCommand(() -> {
                                        target = TARGET_HIGH;
                                        arm1.setTarget(ARM_SCORE);
                                        arm2.setTarget(ARM_SCORE);
                                        wrist.setTarget(WRIST_SCORE);
                                    })
                            )
                    ),
                    new Commands.ConditionalCommand(
                            new Commands.IfThen(() -> gamepad1.left_bumper && target < TARGET_MAX,
                                    new Commands.InstantCommand(() -> {
                                        target += 15;
                                    })
                            )
                    ),
                    new Commands.ConditionalCommand(
                            new Commands.IfThen(() -> gamepad1.right_bumper && target > TARGET_PICKUP,
                                    new Commands.InstantCommand(() -> {
                                        target -= 15;
                                    })
                            )
                    )
                )

            );

            buttonAPressed = gamepad1.a;

            Commands.executor.runOnce();

            claw.setTarget(1-gamepad1.right_trigger);

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

