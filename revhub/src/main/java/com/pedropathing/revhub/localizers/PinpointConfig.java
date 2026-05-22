package com.pedropathing.revhub.localizers;

import com.pedropathing.config.ConfigVar;
import com.pedropathing.config.Configuration;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class PinpointConfig {
    public final ConfigVar<String> name = ConfigVar.required();
    public final ConfigVar<GoBildaPinpointDriver.EncoderDirection> xPodDirection = ConfigVar.of(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    public final ConfigVar<GoBildaPinpointDriver.EncoderDirection> yPodDirection = ConfigVar.of(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    public final ConfigVar<Double> xPodOffset = ConfigVar.required();
    public final ConfigVar<Double> yPodOffset = ConfigVar.required();
    public final ConfigVar<DistanceUnit> offsetUnits = ConfigVar.of(DistanceUnit.INCH);
    public final ConfigVar<DistanceUnit> distanceUnit = ConfigVar.of(DistanceUnit.INCH);
    ;
    public final ConfigVar<GoBildaPinpointDriver.GoBildaOdometryPods> podType = ConfigVar.of(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD); // TODO: it cannot just be pod type because custom resolutions also exist!!

    public PinpointConfig(Configuration<PinpointConfig> config) {
        config.configure(this);
    }
}