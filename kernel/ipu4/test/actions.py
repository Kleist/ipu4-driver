GREP_NO_MATCH_STATUS=1
import logging
import time
import collections
import os

import helpers
import agent_actions

from ambu_test_library.connection.sshpass import TestLibraryError

try:
    from testbox import reboot_netv2
except:
    def reboot_netv2():
        logging.info("No testbox import found - not rebooting netv2 FPGA")
        pass

# Scope info from sysfs
ScopeInfo = collections.namedtuple("ScopeInfo", ['sysfs_id', 'type', 'pipeline', 'status'])

def is_ecp5_scope(scope_type):
    # This dict should cover all scopes used in tests (but doesn't need to cover all Ambu scopes)
    IS_ECP5_SCOPE = {0x0C: True, 0x0D: True, 0x12: False, 0x13: False}

    assert scope_type in IS_ECP5_SCOPE, \
        f"Unsupported scope_type 0x{scope_type:02X}, please add to IS_ECP5_SCOPE dict"
    return IS_ECP5_SCOPE[scope_type]

def reboot_test_agent_fpga_if_ecp5_scope(scope_types):
    if any(is_ecp5_scope(st) for st in scope_types):
        # reboot fpga to help stabilize ecp5 scope instability - see TAC-23
        logging.info("Rebooting test agent FPGA(netv2)")
        reboot_netv2()
        logging.info("Test agent FPGA(netv2) rebooted")

def sysfs_read_scope_info(target, sysfs_id):
    return ScopeInfo(
        sysfs_id = sysfs_id,
        type = int(target.cmd(f"cat /sys/class/scope/scope{sysfs_id}/type").strip()),
        pipeline = int(target.cmd(f"cat /sys/class/scope/scope{sysfs_id}/pipeline").strip()),
        status = target.cmd(f"cat /sys/class/scope/scope{sysfs_id}/status").strip()
    )

def delay_until(action, timeout=20, resolution=0.1, message=None):
    logging.info(f"delay_until({action},{timeout},{message})")
    start = time.time()
    if message is None:
        message = "Timeout {}s waiting for {}".format(timeout, action)
    while 1:
        now = time.time()
        if action():
            return now-start
        if now > start + timeout:
            raise RuntimeError(message)
        time.sleep(resolution)

def _cmd_succeeds(target, cmd):
    try:
        logging.info(f"Running {cmd}")
        target.cmd(cmd)
        logging.info(f"{cmd} succeeded")
        return True
    except TestLibraryError as error:
        logging.info(f"{cmd} failed")
        return False

def module_enable_debug_logs(target, module):
    target.cmd(f'echo "module {module} +p" >/proc/dynamic_debug/control')

def modprobe_with_dmesg(target, module, grep_for, args=""):
    write_dmesg(target, f"modprobe {module}")
    print(f'modprobing {module}, waiting until dmesg contains "{grep_for}"')
    filename = f"/mnt/storage/amp/log/modprobe-{module}-dmesg.txt"
    output = target.cmd(f'dmesg -W > {filename} & PID=$! && modprobe {module} {args} && until grep "{grep_for}" {filename}; do sleep .1; done && kill $PID && wait')
    print(f'modprobing {module} done')

def modprobe(target, module, args=""):
    write_dmesg(target, f"modprobe {module} {args}")
    target.cmd(f'modprobe {module} {args}')

def unmodprobe(target, module):
    write_dmesg(target, f"unmodprobe {module}")
    target.cmd(f'modprobe -r {module}')
    target.cmd(f'test ! lsmod|grep "^{module} "', required_exit_status=GREP_NO_MATCH_STATUS)

def loaded_modules(target):
    lsmod = target.cmd('lsmod')
    # Skip the header line
    lines = lsmod.split('\n')[1:]
    return [line.split(' ')[0] for line in lines]

def clear_dmesg(target, msg=None):
    target.cmd('dmesg -C')
    if msg is not None:
        write_dmesg(target, msg)

def write_dmesg(target, msg, prefix="ambutest: "):
    print(f"{prefix}{msg}")
    target.cmd(f'echo \"{prefix}{msg}\" > /dev/kmsg')

def clear_var_log_messages(target):
    """
    Clears var log messages and forces syslog-ng to re-open the file

    This is used to later be able to only grep through messages that
    happened after this point in time.
    """
    target.cmd('rm /var/log/messages;syslog-ng-ctl reopen')

def kill_pid_and_wait_for_stopped(target, pid):
    target.cmd(f'kill {pid}; tail --pid="{pid}" -f /dev/null')

def i2c_new_device(target, compatible, adapter, address):
    write_dmesg(target, f"i2c_new_device(compatible={compatible},adapter={adapter},address=0x{address:x})")
    return _cmd_succeeds(target, f'echo {compatible} 0x{address:x} > /sys/class/i2c-adapter/i2c-{adapter}/new_device')

def i2c_delete_device(target, adapter, address):
    write_dmesg(target, f"i2c_delete_device(adapter={adapter},address=0x{address:x})")
    return _cmd_succeeds(target, f'echo 0x{address:x} > /sys/class/i2c-adapter/i2c-{adapter}/delete_device')

def load_and_delete_i2c_devices(target, module, adapters, address):
    """
    Used to ensure we have a clean slate when starting a test
    """
    write_dmesg(target, f"load_and_delete_i2c_devices")
    if not module in loaded_modules(target):
        modprobe(target, module)
    for adapter in adapters:
        if i2c_address_in_use(target, adapter=adapter, address=address):
            i2c_delete_device(target, adapter=adapter, address=address)

def device_has_regmap(target, adapter, address):
    return _cmd_succeeds(target, f'cat /sys/kernel/debug/regmap/{adapter}-{address:04x}/name')

def i2c_address_in_use(target, adapter, address):
    return _cmd_succeeds(target, f'cat /sys/class/i2c-adapter/i2c-{adapter}/{adapter}-{address:04x}/name')

def count_v4l2_subdevs(target):
    return int(target.cmd('ls /dev/v4l-subdev* 2>/dev/null | wc -l').strip())

def unbind_pci_device(target, pci_device):
    write_dmesg(target, f"{pci_device} > unbind")
    target.cmd(f"echo -n {pci_device} > /sys/bus/pci/devices/{pci_device}/driver/unbind")

def pci_device_bound(target, pci_device):
    return "found" == target.cmd(f"if [ -e /sys/bus/pci/devices/{pci_device}/driver ]; then echo found; fi").strip()

def v4l2_ports(pipeline):
    assert pipeline in [1, 2]
    if pipeline == 1:
        i2c_id = 0
        csi2_id = 0
        video_id = 12 # /dev/video12
    else:
        i2c_id = 3
        csi2_id = 4
        video_id = 13 # /dev/video13

    return {
        'tc358748': f'"tc358748 {i2c_id}-000e" :0',
        'csi2_0': f'"Intel IPU4 CSI2 {csi2_id}" :0',
        'csi2_1': f'"Intel IPU4 CSI2 {csi2_id}" :1',
        'capture': f'"Intel IPU4 ISYS Capture {video_id}" :0',
    }

def setup_ipu4_formats(target, pipeline, resolution):
    ports = v4l2_ports(pipeline)
    formats = [
        f'{value} [fmt:RGB888_1X24/{resolution}]'
        for key, value in ports.items()
        if key != 'capture' # capture doesn't have set_fmt callback
    ]

    format_string = ",\n".join(f.replace('"','\\"') for f in formats)
    cmd = f'media-ctl -v -V "{format_string}"'
    print(f"Running: {cmd}")
    target.cmd(cmd)

def setup_ipu4_links(target, pipeline):
    ports = v4l2_ports(pipeline)
    links = [
        f'{ports["tc358748"]} -> {ports["csi2_0"]} [1]',
        f'{ports["csi2_1"]} -> {ports["capture"]} [5]',
    ]
    links_string = ", ".join(l.replace('"','\\"') for l in links)
    cmd = f'media-ctl -v -l "{links_string}"'
    print(f"Running: {cmd}")
    target.cmd(cmd)

def scope_sysfs_nodes(target):
    sysfs_dirs = target.cmd('ls /sys/class/scope').strip().split()
    return [int(sysfs_dir[5:]) for sysfs_dir in sysfs_dirs]

def scopes_inserted(target):
    """
    Returns dict of {sysfs_scope_file_number: scope_id},
    e.g. { 1: 4 } if /sys/class/scope/scope1/type is 4, and there are no other scopes inserted.
    """
    sysfs_ids = scope_sysfs_nodes(target)

    scopes = []
    for sysfs_id in sysfs_ids:
        scopes.append(sysfs_read_scope_info(target, sysfs_id))
    return scopes

def insert_scope_and_wait_for_ready(target, test_agent, scope_info):
    agent_actions.insert_scope(test_agent, scope_info.type)
    scope = agent_actions.get_scope_by_id(scope_info.type)
    helpers.assert_wait_until_scope_is_ready(scope_info.sysfs_id, scope, target)

def remove_scope_and_wait_for_gone(target, test_agent, scope_info):
    agent_actions.remove_scope(test_agent, scope_info.type)
    helpers.assert_wait_until_scope_sysfs_is_gone(scope_info.sysfs_id, target)

def prepare_scopes(target, test_agent, scope_types):
    """
    Prepare device with one or more scopes inserted.
    The scopes are inserted in the order they are provided in `scope_types`.

    * On aBox2, this assigns pipelines in order, so first scope has pipeline 1,
      second scope has pipeline 2.
    * On aView2, the pipeline assignment depends on what scopes was inserted
    before.
    """
    assert len(scope_types) in [0,1,2], "Only 0, 1 or 2 scopes supported"

    current_scopes = scopes_inserted(target)
    if [scope.type for scope in current_scopes] == scope_types:
        logging.info(f"Current scopes: {current_scopes}")
        return current_scopes

    logging.info(f"Current scopes not as expected, re-inserting: {current_scopes}")

    agent_actions.remove_all_scopes(test_agent)

    # Workaround for misbehaving ECP5 streams from test agent
    reboot_test_agent_fpga_if_ecp5_scope(scope_types)

    assert delay_until(lambda: scope_sysfs_nodes(target) == []) < 10


    for sysfs_id, scope_type in enumerate(scope_types):
        agent_actions.insert_scope(test_agent, scope_type)
        helpers.assert_wait_until_scope_is_ready(sysfs_id, agent_actions.get_scope_by_id(scope_type), target)

    current_scopes = scopes_inserted(target)
    logging.info(f"Current scopes after re-insert: {current_scopes}")
    assert [scope.type for scope in current_scopes] == scope_types
    return current_scopes

def file_size(target, file_path):
    return int(target.cmd(f'stat -c %s {file_path}'))
