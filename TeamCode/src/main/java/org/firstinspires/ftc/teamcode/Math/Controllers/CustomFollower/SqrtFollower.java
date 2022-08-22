package org.firstinspires.ftc.teamcode.Math.Controllers.CustomFollower;

import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

public class SqrtFollower extends TrajectoryFollower {

	protected Pose2d lastError;

	@Override
	public @NotNull Pose2d getLastError() {
		return lastError;
	}

	@Override
	protected void setLastError(@NotNull Pose2d pose2d) {
		this.lastError = pose2d;
	}

	@Override
	protected @NotNull DriveSignal internalUpdate(@NotNull Pose2d pose2d, @Nullable Pose2d pose2d1) {
		return null;
	}
}
