package org.firstinspires.ftc.teamcode.programs;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.base.Commands.executor;
import static org.firstinspires.ftc.teamcode.robotconfigs.RobotConfig.*;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.base.Commands;
import org.firstinspires.ftc.teamcode.base.Components;
import org.firstinspires.ftc.teamcode.robotconfigs.RobotConfig;
@TeleOp
public class RobotTeleOp extends LinearOpMode {
    RobotConfig robot = new RobotConfig();
    double target = TARGET_PICKUP;
    double frontLeftPower = 0;
    double backLeftPower = 0;
    double frontRightPower = 0;
    double backRightPower = 0;
    int iterations = 0;

    public void telemetryFunction() {
        telemetry.addData("slide1 pos", slide1.getCurrentPosition());
        telemetry.addData("slide2 pos", slide2.getCurrentPosition());
        telemetry.addData("iterations:",++iterations);
        telemetry.addData("Slide1:",slide1.getCurrentPosition());
        telemetry.addData("Slide2:",slide2.getCurrentPosition());
        telemetry.addData("Target:",target);
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Components.initialize(this, robot, false, true);
        Components.activateActuatorControl();
        slide1.setTarget(TARGET_PICKUP);
        slide2.setTarget(TARGET_PICKUP);

        Commands.executor.setCommands(
            new Commands.RunResettingLoop(
                new Commands.FieldCentricMecanumCommand(
                    new Components.BotMotor[] {
                                leftFront,
                                leftBack,
                                rightFront,
                                rightBack
                        },
                        () -> imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS),
                        1,
                        () -> (double) gamepad1.left_stick_x,
                        () -> (double) -gamepad1.left_stick_y,
                        () -> (double) gamepad1.right_stick_x
                ),
                new Commands.PressCommand(
                    new Commands.IfThen(() ->
                        gamepad1.a,
                        new Commands.SequentialCommand(
                            new Commands.InstantCommand(() ->
                                wrist.setTarget(WRIST_PICKUP)
                            ),
                            new Commands.SleepCommand(.7),
                            new Commands.InstantCommand(() -> {
                                target = 0;
                            })
                        )
                    )
                ),
                new Commands.PressCommand(
                    new Commands.IfThen(() -> gamepad1.left_trigger>0.5,
                        new Commands.InstantCommand(() -> {
                            frontLeftPower = 0.5 * frontLeftPower;
                            backLeftPower = 0.5 * backLeftPower;
                            frontRightPower = 0.5 * frontRightPower;
                            backRightPower = 0.5 * backRightPower;
                        })
                    )
                ),
                new Commands.PressCommand(
                    new Commands.IfThen(() -> gamepad1.b,
                        new Commands.InstantCommand(() -> {
                            slide1.setTarget(TARGET_LOW);
                            slide2.setTarget(TARGET_LOW);
                        })
                    )
                ),
                new Commands.PressCommand(
                    new Commands.IfThen(() -> gamepad1.x,
                        new Commands.InstantCommand(() -> {
                            slide1.setTarget(TARGET_MID);
                            slide2.setTarget(TARGET_MID);
                            arm1.setTarget(ARM_SCORE);
                            arm2.setTarget(ARM_SCORE);
                            wrist.setTarget(WRIST_SCORE);
                        })
                    )
                ),
                new Commands.PressCommand(
                    new Commands.IfThen(() -> gamepad1.y,
                        new Commands.InstantCommand(() -> {
                            slide1.setTarget(TARGET_HIGH);
                            slide2.setTarget(TARGET_HIGH);
                            arm1.setTarget(ARM_SCORE);
                            arm2.setTarget(ARM_SCORE);
                            wrist.setTarget(WRIST_SCORE);
                        })
                    )
                ),
                Commands.triggeredDynamicCommand(
                        () -> gamepad1.left_bumper,
                        () -> gamepad1.right_bumper,
                        new Commands.InstantCommand(() -> target += 15),
                        new Commands.InstantCommand(() -> target -= 15)
                ),
                new Commands.PressCommand(
                    new Commands.IfThen(() -> gamepad1.options,
                        new Commands.InstantCommand(() -> {
                            imu.resetYaw();
                        })
                    )
                ),
                new Commands.InstantCommand(() -> {
                    telemetryFunction();
                    claw.setTarget(1-gamepad1.right_trigger);
                })
                )
            );
        waitForStart();

        executor.runLoop(this::opModeIsActive);
    }
};


