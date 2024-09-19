package org.firstinspires.ftc.teamcode.Command_Based_TeleOp_2024_08_17.auto;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class riverDaleAuto extends LinearOpMode {


    DcMotor leftMotor = null;
    DcMotor rightMotor = null;
    DcMotor armMotor = null;
    DcMotor clawPivotMotor = null;

    IMU imu = null;


    private double headingError;
    private double turnSpeed;
    private double driveSpeed;
    private double leftSpeed;
    private double rightSpeed;
    private double targetHeading;


    //CONSTANTS//

    static final double P_DRIVE_GAIN = 0.3;
    static final double P_TURN_GAIN = 0.2;
    static final double HEADING_THRESHOLD = 1.0;




    static RevHubOrientationOnRobot.LogoFacingDirection[] logoFacingDirections
            = RevHubOrientationOnRobot.LogoFacingDirection.values();
    static RevHubOrientationOnRobot.UsbFacingDirection[] usbFacingDirections
            = RevHubOrientationOnRobot.UsbFacingDirection.values();
    int logoFacingDirectionPosition;
    int usbFacingDirectionPosition;


    @Override
    public void runOpMode() throws InterruptedException {

        imu = hardwareMap.get(IMU.class, "imu");
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        clawPivotMotor = hardwareMap.get(DcMotor.class, "pivotMotor");


        //CHECK THESE
        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usb = RevHubOrientationOnRobot.UsbFacingDirection.DOWN;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
        imu.initialize(new IMU.Parameters(orientationOnRobot));





        waitForStart();

        while(opModeIsActive()) {

        }

    }



    private double encoderToMeters(int encVal, int ticksToRotation, double wheelCircumference) {
        double numRevolutions = encVal / ticksToRotation;
        double metersTravelled = numRevolutions * wheelCircumference;
        return metersTravelled;
    }

    //This method is run when the robot isn't moving

    private int metersToEncoderTicks(double distMeters, int ticksToRotation, double wheelCircumference) {
        double numRevolutions = distMeters/wheelCircumference;
        int encoderTicks = (int)numRevolutions * ticksToRotation;
        return encoderTicks;
    }


    private void driveForwardDistance(double power, double distanceMeters) {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int ticksToDist = metersToEncoderTicks(distanceMeters, 1440, 0);
        leftMotor.setTargetPosition(ticksToDist);
        rightMotor.setTargetPosition(ticksToDist);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        moveRobot(power, 0);

    }




    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_TURN_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    private void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        leftSpeed  = drive - turn;
        rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0)
        {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        leftMotor.setPower(leftSpeed);
        rightMotor.setPower(rightSpeed);

    }


    /**
     * Use a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}