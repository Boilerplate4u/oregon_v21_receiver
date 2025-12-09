# esphome/components/oregon_v21_receiver/__init__.py

import esphome.codegen as cg
from esphome.components import remote_receiver
import esphome.config_validation as cv
from esphome.const import CONF_ID

CODEOWNERS = ["@Boilerplate4U"]
DEPENDENCIES = ["remote_receiver"]
CONF_REMOTE_RECEIVER = "remote_receiver_id"

oregon_v21_ns = cg.esphome_ns.namespace("oregon_v21_receiver")
OregonV21Receiver = oregon_v21_ns.class_("OregonV21Receiver", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(OregonV21Receiver),
        cv.GenerateID(CONF_REMOTE_RECEIVER): cv.use_id(
            remote_receiver.RemoteReceiverBase
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[cv.GenerateID()])
    await cg.register_component(var, config)

    # Link to the remote_receiver instance
    rx_var = await cg.get_variable(config[cv.GenerateID(CONF_REMOTE_RECEIVER)])
    cg.add(var.set_remote_receiver(rx_var))
