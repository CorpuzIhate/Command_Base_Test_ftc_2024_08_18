package org.firstinspires.ftc.teamcode.Command_Based_TeleOp_2024_08_17.auto;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Command_Based_TeleOp_2024_08_17.Commands.TeleOpJoystickRobotCentricCMD;
import org.firstinspires.ftc.teamcode.Command_Based_TeleOp_2024_08_17.Commands.TelemetryManagerCMD;
import org.firstinspires.ftc.teamcode.Command_Based_TeleOp_2024_08_17.Subsystems.MecanumDriveBaseSubsystem;
import org.firstinspires.ftc.teamcode.Command_Based_TeleOp_2024_08_17.Subsystems.TelemetryManagerSubsystem;


@Autonomous(name = "Command Auto")
public class basicAutoOpMode extends LinearOpMode {


    private BNO055IMU imu;


    double fwdPwr;
    double strafePwr;
    double rotationPwr;

    private Motor frontLeft;
    Motor frontRight;
    Motor backLeft;
    Motor backRight;

    public GamepadEx driverOP;
    public double dpp = (3 * Math.PI) / 560;

    @Override
    public void runOpMode() {


        // the current position of the motor

        frontLeft = new Motor(hardwareMap, "front_left");
        frontRight = new Motor(hardwareMap, "front_right");
        backLeft = new Motor(hardwareMap, "back_left");
        backRight = new Motor(hardwareMap, "back_right");

        // reset the encoder
        frontLeft.resetEncoder();
        frontRight.resetEncoder();
        backLeft.resetEncoder();
        backRight.resetEncoder();

        frontLeft.setRunMode(Motor.RunMode.PositionControl);
        frontRight.setRunMode(Motor.RunMode.PositionControl);
        backLeft.setRunMode(Motor.RunMode.PositionControl);
        backRight.setRunMode(Motor.RunMode.PositionControl);

        frontLeft.setPositionCoefficient(0.05);
        frontRight.setPositionCoefficient(0.05);
        backLeft.setPositionCoefficient(0.05);
        backRight.setPositionCoefficient(0.05);

        frontLeft.setPositionTolerance(13.6);
        frontRight.setPositionTolerance(13.6);
        backLeft.setPositionTolerance(13.6);
        backRight.setPositionTolerance(13.6);


        driverOP = new GamepadEx(gamepad1);



        frontLeft.setTargetPosition(1200);
        frontRight.setTargetPosition(1200);
        backLeft.setTargetPosition(1200);
        backRight.setTargetPosition(1200);

        frontLeft.set(0);
        frontRight.set(0);
        backRight.set(0);
        backLeft.set(0);
        while(!frontLeft.atTargetPosition() ){
            frontLeft.set(0.5);
            frontRight.set(0.5);
            backRight.set(-0.5);
            backLeft.set(-0.5);




        }
        frontLeft.stopMotor();
        frontRight.stopMotor();
        backRight.stopMotor();
        backLeft.stopMotor();

    }
}
