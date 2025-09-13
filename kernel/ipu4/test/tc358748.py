# SPDX-License-Identifier: GPL-2.0
import actions

import ipu4 

MODULE="ambu_tc358748"
COMPATIBLE="ambu,tc358748"

I2C_ADAPTERS=[0, 3]
I2C_ADDRESS=0x0e #

def clean_slate(target, address=I2C_ADDRESS):
    # The IPU4 driver includes ambu-ipu-bridge, which instantiates two tc358748 devices
    ipu4.unload_all(target)
    actions.load_and_delete_i2c_devices(target, module=MODULE, adapters=I2C_ADAPTERS, address=address)

