package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.HardwareDevice;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name = "Mecanum Test")
public class Teleop_Mecanum_test extends OpMode {

    Motor frontLeft, frontRight, backLeft, backRight;
    GamepadEx driverOp;
    MecanumDrive mecanum;
    private BNO055IMU imu;
    double heading;


    @Override public void  init(){
        frontLeft = new Motor(hardwareMap, "front_left");
        frontRight = new Motor(hardwareMap, "front_right");
        backLeft = new Motor(hardwareMap, "back_left");
        backRight = new Motor(hardwareMap, "back_right");

        BNO055IMU.Parameters myIMUparameters;

        myIMUparameters = new BNO055IMU.Parameters();

        myIMUparameters.angleUnit = myIMUparameters.angleUnit.DEGREES;

        myIMUparameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(myIMUparameters);
        driverOp = new GamepadEx(gamepad1);
        mecanum = new MecanumDrive(frontLeft, frontRight,
                backLeft, backRight);
        frontRight.setInverted(true);
        backLeft.setInverted(true);



    }
    @Override public void loop(){
        heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYX, DEGREES).firstAngle;
        mecanum.driveFieldCentric(
                driverOp.getLeftX(),
                driverOp.getLeftY(),
                driverOp.getRightX(),
                heading
        );
    }

}
