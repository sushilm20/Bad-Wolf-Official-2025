package org.riverdell.robotics.autonomous.impl.tests

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import io.liftgate.robotics.mono.pipeline.single
import org.riverdell.robotics.autonomous.HypnoticAuto
import org.riverdell.robotics.autonomous.movement.geometry.Pose
import org.riverdell.robotics.autonomous.movement.degrees
import org.riverdell.robotics.autonomous.movement.navigateTo

@Autonomous(name = "Test | Turn", group = "Test")
class TestTurn : HypnoticAuto({ opMode ->
    single("turn") {
        navigateTo(Pose(0.0, 0.0, TestConfig.turn.degrees))
        navigateTo(Pose(TestConfig.x, TestConfig.y, TestConfig.turn.degrees))
    }
})