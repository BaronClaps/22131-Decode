package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.config.util.Alliance;

@Autonomous(name = "Red 15", group = "Interesting", preselectTeleOp = "Tele")
public class Red15 extends Auto15 {
    public Red15() {
        super(Alliance.RED);
    }
}

