package org.firstinspires.ftc.teamcode.Command_Based_TeleOp_2024_08_17;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Command_Based_TeleOp_2024_08_17.Commands.TeleOpJoystickRobotCentricCMD;
import org.firstinspires.ftc.teamcode.Command_Based_TeleOp_2024_08_17.Commands.TelemetryManagerCMD;
import org.firstinspires.ftc.teamcode.Command_Based_TeleOp_2024_08_17.Commands.VacuumIntakeCMD;
import org.firstinspires.ftc.teamcode.Command_Based_TeleOp_2024_08_17.Commands.VacuumSetPositionCMD;
import org.firstinspires.ftc.teamcode.Command_Based_TeleOp_2024_08_17.Subsystems.MecanumDriveBaseSubsystem;
import org.firstinspires.ftc.teamcode.Command_Based_TeleOp_2024_08_17.Subsystems.TelemetryManagerSubsystem;
import org.firstinspires.ftc.teamcode.Command_Based_TeleOp_2024_08_17.Subsystems.VacuumSubsystem;


@TeleOp(name = "Command Base Test")
public class RobotContainer extends CommandOpMode {


    private final MecanumDriveBaseSubsystem mecanumDriveBaseSub = new MecanumDriveBaseSubsystem();
    private final TelemetryManagerSubsystem telemetryManagerSub = new TelemetryManagerSubsystem();
    private final VacuumSubsystem vacuumSubsystem = new VacuumSubsystem();

    private BNO055IMU imu;


    double fwdPwr;
    double strafePwr;
    double rotationPwr;

    Motor frontLeft;
    Motor frontRight;
    Motor backLeft;
    Motor backRight;

    ServoEx vacuumTargetServo;
    CRServo vacuumPeriodicServo;

    public GamepadEx driverOP;
    double minAngle = 0;
    double maxAngle =360;

    Button intakeButton;
    Button setPositionButton;
    @Override
    public void initialize() {
        fwdPwr = -gamepad1.left_stick_y;
        strafePwr = -gamepad1.left_stick_x;
        rotationPwr = -gamepad1.right_stick_x;

        vacuumTargetServo = new SimpleServo(
                hardwareMap, "Intake_Servo", minAngle, maxAngle);

        //TODO because were setting the intake servo to 2 different objects it might give us an error
        vacuumTargetServo = new SimpleServo(
                hardwareMap, "Intake_Servo", minAngle, maxAngle);

        frontLeft = new Motor(hardwareMap, "front_left");
        frontRight = new Motor(hardwareMap, "front_right");
        backLeft = new Motor(hardwareMap, "back_left");
        backRight = new Motor(hardwareMap, "back_right");

        frontLeft.setRunMode(Motor.RunMode.RawPower);
        frontRight.setRunMode(Motor.RunMode.RawPower);
        backLeft.setRunMode(Motor.RunMode.RawPower);
        backRight.setRunMode(Motor.RunMode.RawPower);

        backLeft.setInverted(true);
        backRight.setInverted(true);


        driverOP = new GamepadEx(gamepad1);
        intakeButton = driverOP.getGamepadButton(GamepadKeys.Button.A);
        setPositionButton = driverOP.getGamepadButton(GamepadKeys.Button.B);


        BNO055IMU.Parameters myIMUparameters;

        myIMUparameters = new BNO055IMU.Parameters();

        myIMUparameters.angleUnit = myIMUparameters.angleUnit.RADIANS;

        myIMUparameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(myIMUparameters);

        telemetryManagerSub.setDefaultCommand(new PerpetualCommand(new TelemetryManagerCMD(telemetryManagerSub)));


        mecanumDriveBaseSub.setDefaultCommand(new TeleOpJoystickRobotCentricCMD(mecanumDriveBaseSub,
                telemetryManagerSub.getTelemetryObject(), driverOP::getLeftY, driverOP::getLeftX, driverOP::getRightX,
                frontLeft, frontRight, backLeft, backRight));

        intakeButton.whenHeld(new VacuumIntakeCMD(vacuumSubsystem, vacuumPeriodicServo,
                vacuumTargetServo));
        setPositionButton.whenHeld(new VacuumSetPositionCMD(vacuumSubsystem,vacuumPeriodicServo,
                vacuumTargetServo));

    }




}


