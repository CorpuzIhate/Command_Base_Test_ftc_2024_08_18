package org.firstinspires.ftc.teamcode.Command_Based_TeleOp_2024_08_17.Commands;




import com.arcrobotics.ftclib.command.CommandBase;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Command_Based_TeleOp_2024_08_17.Subsystems.MecanumDriveBaseSubsystem;

public class TeleOpJoystickCMD extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final MecanumDriveBaseSubsystem m_MecanumSub;
    private final Telemetry m_dashboardTelemetry;


    public TeleOpJoystickCMD(MecanumDriveBaseSubsystem mecanumDriveBaseSubsystem,
                             Telemetry dashboardTelemetry
    ) {

        m_dashboardTelemetry = dashboardTelemetry;
        m_MecanumSub = mecanumDriveBaseSubsystem;

        addRequirements(mecanumDriveBaseSubsystem);
    }
    @Override
    public  void execute(){




        m_dashboardTelemetry.addData("hello urmom", m_MecanumSub.urmom);

    }

}
