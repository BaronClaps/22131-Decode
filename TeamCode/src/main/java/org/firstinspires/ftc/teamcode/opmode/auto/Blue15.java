package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.config.util.Alliance;

@Autonomous(name = "Blue 15", group = "Interesting", preselectTeleOp = "Tele")
public class Blue15 extends Auto15 {
    public Blue15() {
        super(Alliance.BLUE);
    }
}

