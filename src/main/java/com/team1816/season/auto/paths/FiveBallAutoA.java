package com.team1816.season.auto.paths;

import com.team1816.lib.auto.paths.PathContainer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;

public class FiveBallAutoA implements PathContainer {

    @Override
    public List<Pose2d> buildWaypoints() {
        return List.of(
            new Pose2d(237, 207, Rotation2d.fromDegrees(135)),
            new Pose2d(208, 239, Rotation2d.fromDegrees(135))
        );
    }

    @Override
    public List<Rotation2d> buildHeadings() {
        return List.of(Rotation2d.fromDegrees(130), Rotation2d.fromDegrees(200));
    }

    @Override
    public boolean usingApp() {
        return true;
    }
}
