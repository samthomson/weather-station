# Firmware variants: which sensor drivers get compiled in.
#
# Each attribute is a variant name -> full feature-flag set. Booleans map to
# ENABLE_<FLAG>=1/0 compile definitions; `pms` is null or a model string
# (ENABLE_PMS derives from it, PMS_MODEL is only defined when non-null).
# The mapping to defines happens in nix/firmware.nix.
let
  defaults = {
    dht = false;
    bme280 = false;
    bh1750 = false;
    rain = false;
    pms = null;
    mq = false;
    oled = false;
    sps30 = false;
    sds011 = false;
    bmp280 = false;
  };
in
{
  # Reference MVP build (parity target; see docs/nixify-plan.md).
  mvp = defaults // { bme280 = true; bh1750 = true; rain = true; pms = "PMS5003"; };

  # Air-quality stations: OLED + particulate sensor, nothing else.
  airquality-sps30 = defaults // { sps30 = true; oled = true; };
  airquality-sds011 = defaults // { sds011 = true; oled = true; };
}
