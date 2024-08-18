package org.firstinspires.ftc.teamcode.Command_Based_TeleOp_2024_08_17;



import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandScheduler;

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




    public void init(){
        mecanumDriveBaseSub.setDefaultCommand(new TeleOpJoystickCMD(mecanumDriveBaseSub,main_dashboardTelemetry));
    }

    public void loop(){
        CommandScheduler.getInstance().run();
        main_dashboardTelemetry.update();
    }




}
