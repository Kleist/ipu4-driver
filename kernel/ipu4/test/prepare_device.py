#!/usr/bin/env python
import subprocess

import logging

import helpers

LOG_MESSAGE_AFTER_PREPARE = "prepare_device.py done - ready to start tests"

logger = logging.getLogger("prepare_device.py")

def test_hwclock(target):
    def hwclock_hctosys():
        return target.cmd("/sbin/hwclock --utc --hctosys")

    # If this fails, it is likely that the DUT is bad (EMB-845), and boot will be delayed significantly
    time, result = helpers.measure_time(hwclock_hctosys)
    assert time < 2


def test_set_time_and_timezone(target):
    def get_system_epoch_time():
        return subprocess.check_output("date -u +%s".split()).decode("utf-8").rstrip()

    def set_target_system_time(epoch_time):
        target.cmd(f"date -s @'{epoch_time}'")

    def update_target_hardware_clock():
        target.cmd("/sbin/hwclock --utc --systohc")

    def set_target_timezone_to_europe_copenhagen():
        target.cmd(f"ln -sf /usr/share/zoneinfo/Europe/Copenhagen /etc/localtime")

    def get_target_epoch_time():
        return target.cmd("date -u +%s")

    test_agent_epoch_time = get_system_epoch_time()
    set_target_system_time(test_agent_epoch_time)    
    update_target_hardware_clock()
    set_target_timezone_to_europe_copenhagen()

    test_agent_epoch_time = int(get_system_epoch_time())
    target_epoch_time = int(get_target_epoch_time())
    assert abs(test_agent_epoch_time - target_epoch_time) < 10

def prepare_firmware_upgrade(target):
    def firmware_upgrade_needed():
        check_firmware_output = target.cmd('firmware-upgrade --check-firmware-version; echo $?', -1)
        result = check_firmware_output.splitlines()[-1] != '0'
        logger.info(f"firmware_upgrade_needed {result}:_{check_firmware_output}")
        return result

    def wait_for_reboot():
        assert helpers.delay_until(target.is_not_booted, 10 * 60) < 5 * 60
        assert helpers.delay_until(target.is_booted, 15 * 60) < 10 * 60

    if not firmware_upgrade_needed():
        return

    target.firmware_upgrade()
    wait_for_reboot()

def prepare_scope_settings(target):
    # Scope settings will be uploaded on boot, so this shouldn't write any parameters
    # but if write for some reason fails, we detect it here and stop the tests,
    # instead of in some random test.
    target.cmd('scope-settings-upload', timeout=240)

def prepare_log_done(request, target):
    # Add a log message to indicate when the real test starts. This is used in 
    target.cmd(
        f"logger -s -t ambutest {request.node.nodeid}: -p 6 {LOG_MESSAGE_AFTER_PREPARE}"
    )
