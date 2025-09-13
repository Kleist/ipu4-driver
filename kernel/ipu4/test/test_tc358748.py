# SPDX-License-Identifier: GPL-2.0
import actions
import ipu4
import tc358748

def test_load_unload_module(target):
    # IPU4 driver needs to be unloaded first because it will
    # create tc358748 devices
    ipu4.unload_all(target)

    # Ensure we reload the module when testing during development
    actions.unmodprobe(target, module=tc358748.MODULE)
    assert not tc358748.MODULE in actions.loaded_modules(target)

    actions.modprobe(target, module=tc358748.MODULE)
    assert tc358748.MODULE in actions.loaded_modules(target)

def test_probe_devices(target):
    tc358748.clean_slate(target)

    for adapter in tc358748.I2C_ADAPTERS:
        assert actions.i2c_new_device(target, compatible=tc358748.COMPATIBLE, adapter=adapter, address=tc358748.I2C_ADDRESS)
        # NB: probe fails, because there is no fwnode, so it can only be registered correctly by using ipu-bridge which intel-ipu4 initializes

    for adapter in tc358748.I2C_ADAPTERS:
        assert actions.i2c_delete_device(target, adapter=adapter, address=tc358748.I2C_ADDRESS)
