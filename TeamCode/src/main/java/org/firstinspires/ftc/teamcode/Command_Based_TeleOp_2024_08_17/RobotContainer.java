package org.firstinspires.ftc.teamcode.Command_Based_TeleOp_2024_08_17;



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

//    GamepadEx driverOp;

    double fwdPwr;
    double strafePwr;
    double rotationPwr;

    public void init(){
        fwdPwr = -gamepad1.left_stick_y;
        strafePwr = -gamepad1.left_stick_x;
        rotationPwr = -gamepad1.right_stick_x;

//        Constants.Motors.frontLeft.setRunMode(Motor.RunMode.RawPower);
//        Constants.Motors.frontRight.setRunMode(Motor.RunMode.RawPower);
//        Constants.Motors.backLeft.setRunMode(Motor.RunMode.RawPower);
//        Constants.Motors.backRight.setRunMode(Motor.RunMode.RawPower);


//        mecanumDriveBaseSub.setDefaultCommand(new TeleOpJoystickCMD(mecanumDriveBaseSub,
//                main_dashboardTelemetry, fwdPwr, strafePwr,rotationPwr ));
    }

    public void loop(){
        fwdPwr = -gamepad1.left_stick_y;
        strafePwr = -gamepad1.left_stick_x;
        rotationPwr = -gamepad1.right_stick_x;



        CommandScheduler.getInstance().run();
        main_dashboardTelemetry.update();
    }




}
