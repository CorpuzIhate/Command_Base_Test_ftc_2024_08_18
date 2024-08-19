package org.firstinspires.ftc.teamcode.Command_Based_TeleOp_2024_08_17;



import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandScheduler;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Command_Based_TeleOp_2024_08_17.Commands.TeleOpJoystickCMD;
import org.firstinspires.ftc.teamcode.Command_Based_TeleOp_2024_08_17.Subsystems.MecanumDriveBaseSubsystem;


@TeleOp(name = "Command Base Test")
public class RobotContainer extends OpMode {

    private final MecanumDriveBaseSubsystem mecanumDriveBaseSub = new MecanumDriveBaseSubsystem();

    //delete ftc dashboard during competition to prevent errors.
    //FTC Dashboard is PROHIBITED during games but allowed during pits
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry main_dashboardTelemetry = dashboard.getTelemetry();


    double fwdPwr;
    double strafePwr;
    double rotationPwr;
    Motor frontLeft;
    Motor frontRight;
    Motor backLeft;
    Motor backRight;


    public void init(){
        fwdPwr = -gamepad1.left_stick_y;
        strafePwr = -gamepad1.left_stick_x;
        rotationPwr = -gamepad1.right_stick_x;

        frontLeft = new Motor(hardwareMap, "front_left");
        frontRight = new Motor(hardwareMap, "front_right");
        backLeft = new Motor(hardwareMap, "back_left");
        backRight = new Motor(hardwareMap, "back_right");

        frontLeft.setRunMode(Motor.RunMode.RawPower);
        frontRight.setRunMode(Motor.RunMode.RawPower);
        backLeft.setRunMode(Motor.RunMode.RawPower);
        backRight.setRunMode(Motor.RunMode.RawPower);



       mecanumDriveBaseSub.setDefaultCommand(new TeleOpJoystickCMD(mecanumDriveBaseSub,
                main_dashboardTelemetry, fwdPwr, strafePwr,rotationPwr,
               frontLeft, frontRight, backLeft, backRight));
    }

    public void loop(){
        fwdPwr = -gamepad1.left_stick_y;
        strafePwr = -gamepad1.left_stick_x;
        rotationPwr = -gamepad1.right_stick_x;

        main_dashboardTelemetry.addData("m_forwardPower (RobotContainer)", fwdPwr);
        main_dashboardTelemetry.addData("m_strafePower (RobotContainer)", strafePwr);
        main_dashboardTelemetry.addData("m_rotationPower (RobotContainer)", rotationPwr);

        CommandScheduler.getInstance().run();
        main_dashboardTelemetry.update();
    }




}
