package org.firstinspires.ftc.teamcode.Command_Based_TeleOp_2024_08_17;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.hardware.motors.Motor;

public final class Constants{
    public static final class Motors{

        public static final Motor frontLeft = new Motor(hardwareMap, "front_left");
        public static final Motor frontRight = new Motor(hardwareMap, "front_right");
        public static final Motor backLeft = new Motor(hardwareMap, "back_left");
        public static final Motor backRight = new Motor(hardwareMap, "back_right");


    }

}
