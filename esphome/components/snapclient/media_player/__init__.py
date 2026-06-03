from esphome import pins
import esphome.codegen as cg
from esphome.components import audio_dac, media_player, socket, wifi
from esphome.components.esp32 import add_idf_component, add_idf_sdkconfig_option
from esphome.components.ethernet import SPI_ETHERNET_TYPES
from esphome.components.i2s_audio import (
    CONF_I2S_DOUT_PIN,
    CONF_STEREO,
    I2SAudioOut,
    i2s_audio_component_schema,
    register_i2s_audio_component,
)
import esphome.config_validation as cv
from esphome.const import (
    CONF_AUDIO_DAC,
    CONF_DISABLED_BY_DEFAULT,
    CONF_ENTITY_CATEGORY,
    CONF_NAME,
    CONF_PORT,
    CONF_TYPE,
    ENTITY_CATEGORY_DIAGNOSTIC,
)
from esphome.core import CORE, ID
from esphome.core.entity_helpers import setup_entity

from .. import snapclient_ns

CODEOWNERS = ["@luar123"]

DEPENDENCIES = ["esp32", "i2s_audio"]
AUTO_LOAD = ["mdns", "socket"]

CONF_HOSTNAME = "hostname"
CONF_MUTE_PIN = "mute_pin"
CONF_CONTROL_PORT = "control_port"
CONF_VOLUME_CURVE_DB_RANGE = "volume_curve_db_range"

SNAPCLIENT_GIT_VERSION = "5a1128290d61615c49dcb8d9b6202c8a2dcd09c6"
SNAPCLIENT_GIT_REPO = "https://github.com/W-Floyd/snapclient.git"

SnapClientComponent = snapclient_ns.class_(
    "SnapClientComponent", cg.Component, media_player.MediaPlayer, I2SAudioOut
)

# Number component for volume curve dB range
VolumeCurveDbRange = snapclient_ns.class_(
    "VolumeCurveDbRange",
    cg.Component,
    cg.EntityBase,
)


def _consume_sockets(config):
    """Register socket needs for this component."""
    # upstream uses 10 sockets, but 7 are used for http server
    socket.consume_sockets(3, "snapclient")(config)
    return config


CONFIG_SCHEMA = cv.All(
    media_player.media_player_schema(SnapClientComponent)
    .extend(
        i2s_audio_component_schema(
            SnapClientComponent,
            default_sample_rate=44100,
            default_channel=CONF_STEREO,
            default_bits_per_sample="16bit",
        )
    )
    .extend(
        {
            cv.GenerateID(): cv.declare_id(SnapClientComponent),
            cv.Optional(CONF_NAME): cv.string,
            # Empty hostname means "discover via mDNS".
            cv.Optional(CONF_HOSTNAME): cv.domain,
            cv.Optional(CONF_PORT, default=1704): cv.port,
            cv.Optional(CONF_CONTROL_PORT, default=1705): cv.port,
            cv.Required(CONF_I2S_DOUT_PIN): pins.internal_gpio_output_pin_number,
            cv.Optional(CONF_MUTE_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_AUDIO_DAC): cv.use_id(audio_dac.AudioDac),
            cv.Optional(CONF_VOLUME_CURVE_DB_RANGE, default=60): cv.int_range(
                min=0, max=90
            ),
        }
    )
    .extend(cv.COMPONENT_SCHEMA),
    _consume_sockets,  # Register socket usage during validation
)


async def to_code(config):
    add_idf_component(name="espressif/esp-dsp", ref=">1.5.0")
    for component in [
        "dsp_processor",
        "dsp_processor_settings",
        "flac",
        "libbuffer",
        "libmedian",
        "lightsnapcast",
        "opus",
        "snapclient",
        "timefilter",
    ]:
        add_idf_component(
            name=component,
            ref=SNAPCLIENT_GIT_VERSION,
            repo=SNAPCLIENT_GIT_REPO,
        )
    if CONF_AUDIO_DAC not in config:
        add_idf_sdkconfig_option("CONFIG_USE_DSP_PROCESSOR", True)
        add_idf_sdkconfig_option("CONFIG_SNAPCLIENT_USE_SOFT_VOL", True)
    if CONF_NAME not in config:
        config[CONF_NAME] = CORE.name or ""

    use_mdns = config.get(CONF_HOSTNAME) is None
    if not use_mdns:
        add_idf_sdkconfig_option("CONFIG_SNAPSERVER_HOST", str(config[CONF_HOSTNAME]))
    add_idf_sdkconfig_option("CONFIG_SNAPSERVER_PORT", int(config[CONF_PORT]))
    add_idf_sdkconfig_option(
        "CONFIG_SNAPSERVER_CONTROL_PORT", int(config[CONF_CONTROL_PORT])
    )
    add_idf_sdkconfig_option("CONFIG_SNAPSERVER_USE_MDNS", use_mdns)
    add_idf_sdkconfig_option(
        "CONFIG_SNAPCLIENT_VOLUME_CURVE_DB_RANGE",
        config[CONF_VOLUME_CURVE_DB_RANGE],
    )
    add_idf_sdkconfig_option("CONFIG_SNAPCLIENT_NAME", config[CONF_NAME])
    add_idf_sdkconfig_option("CONFIG_FREERTOS_TASK_NOTIFICATION_ARRAY_ENTRIES", 2)
    ethernet = CORE.config.get("ethernet")
    if ethernet:
        if ethernet.get(CONF_TYPE) in SPI_ETHERNET_TYPES:
            cg.add_build_flag("-DCONFIG_SNAPCLIENT_USE_SPI_ETHERNET=1")
        else:
            cg.add_build_flag("-DCONFIG_SNAPCLIENT_USE_INTERNAL_ETHERNET=1")
    wifi.enable_runtime_power_save_control()

    var = await media_player.new_media_player(config)
    await cg.register_component(var, config)
    await register_i2s_audio_component(var, config)
    cg.add(var.set_dout_pin(config[CONF_I2S_DOUT_PIN]))
    cg.add(var.set_snapserver_hostname(config.get(CONF_HOSTNAME, "")))
    cg.add(var.set_snapserver_port(config[CONF_PORT]))
    cg.add(var.set_snapserver_control_port(config[CONF_CONTROL_PORT]))
    cg.add(var.set_snapserver_use_mdns(use_mdns))
    if CONF_MUTE_PIN in config:
        pin = await cg.gpio_pin_expression(config[CONF_MUTE_PIN])
        cg.add(var.set_mute_pin(pin))
    if audio_dac_config := config.get(CONF_AUDIO_DAC):
        aud_dac = await cg.get_variable(audio_dac_config)
        cg.add(var.set_audio_dac(aud_dac))

    # Register the volume curve NumberEntity - disabled by default
    vol_curve_id = ID("volume_curve_db_range", type=VolumeCurveDbRange)
    vol_curve_var = cg.new_Pvariable(vol_curve_id)
    vol_curve_config = {
        CONF_NAME: "Volume Curve dB Range",
        CONF_DISABLED_BY_DEFAULT: True,
        CONF_ENTITY_CATEGORY: ENTITY_CATEGORY_DIAGNOSTIC,
    }
    await setup_entity(vol_curve_var, vol_curve_config, "number")
    cg.add(vol_curve_var.traits.set_min_value(0))
    cg.add(vol_curve_var.traits.set_max_value(90))
    cg.add(vol_curve_var.traits.set_step(1))
    cg.add(
        cg.RawStatement(
            "id(volume_curve_db_range).traits.set_mode(esphome::number::NUMBER_MODE_SLIDER);"
        )
    )
    cg.add(cg.App.register_number(vol_curve_var))

    # Set initial value from config
    cg.add(vol_curve_var.publish_state(config[CONF_VOLUME_CURVE_DB_RANGE]))
