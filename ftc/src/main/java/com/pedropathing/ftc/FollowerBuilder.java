package com.pedropathing.ftc;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.*;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.ftc.localization.constants.OctoQuadConstants;
import com.pedropathing.ftc.localization.constants.OTOSConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.ftc.localization.localizers.DriveEncoderLocalizer;
import com.pedropathing.ftc.localization.localizers.OctoQuadLocalizer;
import com.pedropathing.ftc.localization.localizers.OTOSLocalizer;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.ftc.localization.localizers.ThreeWheelIMULocalizer;
import com.pedropathing.ftc.localization.localizers.ThreeWheelLocalizer;
import com.pedropathing.ftc.localization.localizers.TwoWheelLocalizer;
import com.pedropathing.localization.Localizer;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.function.Supplier;

/** This is the FollowerBuilder.
 * It is used to create Followers with a specific drivetrain + localizer without having to use a full constructor
 *
 * @author Baron Henderson - 20077 The Indubitables
 */
public class FollowerBuilder {
    private final FollowerConstants constants;
    private PathConstraints constraints;
    private final HardwareMap hardwareMap;
    private Supplier<Localizer> localizerSupplier;
    private Drivetrain drivetrain;
    private boolean resetIMUOnInit = true;

    public FollowerBuilder(FollowerConstants constants, HardwareMap hardwareMap) {
        this.constants = constants;
        this.hardwareMap = hardwareMap;
        constraints = PathConstraints.defaultConstraints;
    }

    public FollowerBuilder setLocalizer(Localizer localizer) {
        this.localizerSupplier = () -> localizer;
        return this;
    }

    public FollowerBuilder driveEncoderLocalizer(DriveEncoderConstants lConstants) {
        this.localizerSupplier = () -> new DriveEncoderLocalizer(hardwareMap, lConstants);
        return this;
    }

    public FollowerBuilder octoQuadLocalizer(OctoQuadConstants lConstants, OctoQuadLocalizer.InitMode initMode) {
        this.localizerSupplier = () -> new OctoQuadLocalizer(hardwareMap, lConstants, initMode);
        return this;
    }

    public FollowerBuilder OTOSLocalizer(OTOSConstants lConstants) {
        this.localizerSupplier = () -> new OTOSLocalizer(hardwareMap, lConstants);
        return this;
    }

    public FollowerBuilder pinpointLocalizer(PinpointConstants lConstants) {
        this.localizerSupplier = () -> new PinpointLocalizer(hardwareMap, lConstants, resetIMUOnInit);
        return this;
    }

    public FollowerBuilder threeWheelIMULocalizer(ThreeWheelIMUConstants lConstants) {
        this.localizerSupplier = () -> new ThreeWheelIMULocalizer(hardwareMap, lConstants);
        return this;
    }

    public FollowerBuilder threeWheelLocalizer(ThreeWheelConstants lConstants) {
        this.localizerSupplier = () -> new ThreeWheelLocalizer(hardwareMap, lConstants);
        return this;
    }

    public FollowerBuilder twoWheelLocalizer(TwoWheelConstants lConstants) {
        this.localizerSupplier = () -> new TwoWheelLocalizer(hardwareMap, lConstants);
        return this;
    }

    public FollowerBuilder setDrivetrain(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        return this;
    }

    public FollowerBuilder mecanumDrivetrain(MecanumConstants mecanumConstants) {
        return setDrivetrain(new Mecanum(hardwareMap, mecanumConstants));
    }

    @Deprecated
    public FollowerBuilder mecanumExDrivetrain(MecanumConstants mecanumConstants) {
        return setDrivetrain(new MecanumEx(hardwareMap, mecanumConstants));
    }

    public FollowerBuilder swerveDrivetrain(SwerveConstants swerveConstants, SwervePod... pods) {
        return setDrivetrain(new Swerve(hardwareMap, swerveConstants, pods));
    }

    public FollowerBuilder pathConstraints(PathConstraints pathConstraints) {
        this.constraints = pathConstraints;
        PathConstraints.setDefaultConstraints(pathConstraints);
        return this;
    }

    public FollowerBuilder resetIMUOnInit(boolean resetIMUOnInit) {
        this.resetIMUOnInit = resetIMUOnInit;
        return this;
    }

    public Follower build() {
        return new Follower(constants, localizerSupplier.get(), drivetrain, constraints, resetIMUOnInit);
    }
}
