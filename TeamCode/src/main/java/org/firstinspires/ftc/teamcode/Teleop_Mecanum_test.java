package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.HardwareDevice;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name = "Mecanum Test")
public class Teleop_Mecanum_test extends OpMode {

    Motor frontLeft, frontRight, backLeft, backRight;
    GamepadEx driverOp;
    MecanumDrive mecanum;


    @Override public void  init(){
        frontLeft = new Motor(hardwareMap, "front_left");
        frontRight = new Motor(hardwareMap, "front_right");
        backLeft = new Motor(hardwareMap, "back_left");
        backRight = new Motor(hardwareMap, "back_right");

        driverOp = new GamepadEx(gamepad1);
        mecanum = new MecanumDrive(frontLeft, frontRight,
                backLeft, backRight);
    }
    @Override public void loop(){

        mecanum.driveRobotCentric(
                driverOp.getLeftY(),
                driverOp.getLeftX(),
                driverOp.getRightY()
        );
    }

}
