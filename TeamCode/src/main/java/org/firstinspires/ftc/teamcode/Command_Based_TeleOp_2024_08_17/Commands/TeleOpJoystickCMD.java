package org.firstinspires.ftc.teamcode.Command_Based_TeleOp_2024_08_17.Commands;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Command_Based_TeleOp_2024_08_17.Subsystems.MecanumDriveBaseSubsystem;

public class TeleOpJoystickCMD extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final MecanumDriveBaseSubsystem m_MecanumSub;
    private final Telemetry m_dashboardTelemetry;
//    private final GamepadEx m_joystick;


    public TeleOpJoystickCMD(MecanumDriveBaseSubsystem mecanumDriveBaseSubsystem,
                             Telemetry dashboardTelemetry
                             /*GamepadEx joystick*/
    ) {

        m_dashboardTelemetry = dashboardTelemetry;
        m_MecanumSub = mecanumDriveBaseSubsystem;
//        m_joystick = joystick;
        addRequirements(mecanumDriveBaseSubsystem);
    }
    @Override
    public  void execute(){
//        double joystickAngle = Math.atan(m_joystick.getLeftY() / m_joystick.getLeftX());
//        double joystickMagnitude = Math.hypot( m_joystick.getLeftY() , m_joystick.getLeftX());
//
//        double frontLeftOutput = joystickMagnitude * Math.sin( joystickAngle + (Math.PI / 4) ) + m_joystick.getLeftX();
//
//        double frontRightOutput = joystickMagnitude * Math.sin( joystickAngle - (Math.PI / 4) ) - m_joystick.getLeftX();
//
//        double backLeftOutput = joystickMagnitude * Math.sin( joystickAngle - (Math.PI / 4) ) + m_joystick.getLeftX();
//
//        double backRightOutput = joystickMagnitude * Math.sin( joystickAngle + (Math.PI / 4) ) - m_joystick.getLeftX();

//        m_MecanumSub.FL.set(frontLeftOutput);
//        m_MecanumSub.FR.set(frontRightOutput);
//        m_MecanumSub.BL.set(backLeftOutput);
//        m_MecanumSub.BR.set(backRightOutput);

//        m_dashboardTelemetry.addData("frontLeftOutput", frontLeftOutput);
//        m_dashboardTelemetry.addData("frontRightOutput", frontRightOutput);
//        m_dashboardTelemetry.addData("backLeftOutput", backLeftOutput);
//        m_dashboardTelemetry.addData("backRightOutput", backRightOutput);
//
//        m_dashboardTelemetry.addData("joystickAngle", joystickAngle);



        m_dashboardTelemetry.addData("hello urmom", m_MecanumSub.urmom);

    }

}
