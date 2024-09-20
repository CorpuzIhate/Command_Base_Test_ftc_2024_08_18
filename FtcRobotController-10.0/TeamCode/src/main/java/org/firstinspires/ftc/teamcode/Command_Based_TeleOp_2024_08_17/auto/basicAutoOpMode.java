package org.firstinspires.ftc.teamcode.Command_Based_TeleOp_2024_08_17.auto;


import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Autonomous(name = "Command Auto")
public class basicAutoOpMode extends LinearOpMode {

    FtcDashboard dashboard;
    Telemetry main_dashboardTelemetry;


    private DcMotor frontLeftDriveMotor;
    private DcMotor frontRightDriveMotor;
    private DcMotor backLeftDriveMotor;
    private DcMotor backRightDriveMotor;


    public int ticksToDist;
    private ElapsedTime runtime = new ElapsedTime();
    //TODO check if ticks to distance calculation is correct
    double WHEEL_CIRCUMFERENCE_METERS = (3 * Math.PI) * 0.0254;

    @Override
    public void runOpMode() {
        waitForStart();

        frontLeftDriveMotor = hardwareMap.get(DcMotor.class, "front_left");
        frontRightDriveMotor = hardwareMap.get(DcMotor.class, "front_right");
        backLeftDriveMotor = hardwareMap.get(DcMotor.class, "back_left");
        backRightDriveMotor = hardwareMap.get(DcMotor.class, "back_right");

        //TODO check if you need to reverse the motors

        backLeftDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);



        frontLeftDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDriveMotor.setTargetPosition(12000);
        frontRightDriveMotor.setTargetPosition(12000);
        backLeftDriveMotor.setTargetPosition(12000);
        backRightDriveMotor.setTargetPosition(12000);


        frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        frontLeftDriveMotor.setPower( 0.5);
        frontRightDriveMotor.setPower(0.5);
        backRightDriveMotor.setPower(0.5);
        backLeftDriveMotor.setPower(0.5);






    }



    private void driveForwardDistance(double power, double distanceMeters) {//test me

        frontLeftDriveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        frontLeftDriveMotor.setTargetPosition(1200);



        frontLeftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftDriveMotor.setPower(0.3);
        while(frontLeftDriveMotor.isBusy() && opModeIsActive()){
           main_dashboardTelemetry.addData("Front_left_pos", frontLeftDriveMotor.getCurrentPosition());

        }


    }




    private int metersToEncoderTicks(double distMeters, int ticksToRotation, double wheelCircumferenceMeters) {
        double numRevolutions = distMeters / wheelCircumferenceMeters;
        int encoderTicks = (int) (numRevolutions * ticksToRotation);
        return encoderTicks;
    }


    public void setMotorSpeeds(double forwardPower, double strafePower,
                                   double rotationPower){
        forwardPower *= -1;
        strafePower *= -1;
        rotationPower *= -1;


        double frontLeftSpeed = forwardPower - strafePower - rotationPower;
        double backLeftSpeed = forwardPower + strafePower - rotationPower;
        double frontRightSpeed = forwardPower + strafePower + rotationPower;
        double backRightSpeed = forwardPower - strafePower + rotationPower;

        //math.max tale 2 doubles and figure out which one is higher
        // This is used to determine the current max speed as different sides of the robot
        // may have their motors moving faster


        double max = Math.max(Math.abs(frontLeftSpeed), Math.abs(frontRightSpeed));

        //first we compare the front motors. then we compare that with the back motors to find
        // the fastest motor
        max = Math.max(max, Math.abs(backLeftSpeed));
        max = Math.max(max, Math.abs(backRightSpeed));


        // if the faster motor at the moment has a power over 1, we divide all motors by the max
        if (max > 1.0) {
            frontLeftSpeed /= max;
            frontRightSpeed /= max;
            backLeftSpeed /= max;
            backRightSpeed /= max;
        }


        frontLeftDriveMotor.setPower(frontLeftSpeed);
        frontRightDriveMotor.setPower(frontLeftSpeed);
        backLeftDriveMotor.setPower(backLeftSpeed);
        backRightDriveMotor.setPower(backRightSpeed);


    }
}