import time
import logging

logger = logging.getLogger(__name__)

MAX_NO_SCOPES_DETECTED_SEC = 10
MAX_SCOPE_READY_TIME = 10
MAX_WAIT_FOR_CIB_VOLTAGE = 10
MAX_NUMBER_OF_POWER_RETRY = 10

def assert_wait_until_no_scopes_detected(target, max_wait=MAX_NO_SCOPES_DETECTED_SEC):
    start = time.perf_counter()
    while max_wait > time.perf_counter() - start:
        if int(target.cmd("ls /sys/class/scope | wc -l")) == 0:
            return
    assert False, f"One or more scopes not disconnected from target"


def assert_wait_until_scope_is_ready(scope_idx, scope, target, max_wait=MAX_SCOPE_READY_TIME):
    start = time.perf_counter()
    while max_wait > time.perf_counter() - start:
        if target.scope.is_ready(scope_idx, scope.id) is True:
            return
    assert False, f"scope{scope_idx} with scope {scope.__dict__} is not ready within {max_wait} seconds!"


def assert_wait_until_scope_sysfs_is_gone(scope_idx, target, max_wait=MAX_NO_SCOPES_DETECTED_SEC):
    start = time.perf_counter()
    while max_wait > time.perf_counter() - start:
        result = target.cmd(f"if [ -d /sys/class/scope/scope{scope_idx} ]; then echo found; else echo not found; fi ", required_exit_status=-1).strip()
        if "not found" == result:
            return
    assert False, f"scope{scope_idx} not disconnected in time"


def measure_time(func):
    start = time.perf_counter()
    result = func()
    return time.perf_counter() - start, result


def delay_until(func, max_time_out=100, interval=1):
    start = time.perf_counter()
    while time.perf_counter() - start < max_time_out:
        if func() is True:
            return time.perf_counter() - start
        time.sleep(float(interval))
    return time.perf_counter() - start


def cib_voltage_detected(test_agent):
    """
    This function returns True if voltage is detected on ov426 CIB
    - requires that simulated ov426 scope is detected to ov426 CIB.
    """
    return test_agent.cib.get_measurement(2).is_voltage_detected() is True


def no_cib_voltage_detected(test_agent):
    return cib_voltage_detected(test_agent) is False


def press_power_button_and_retry(test_agent):
    """
    This function use CIB voltage to detect if target is booting after
    pressing power button.

    If no voltage is detected within 10 seconds, then retry pressing the power button
    - this sequence is repeated 10 times.

    """

    for _ in range(MAX_NUMBER_OF_POWER_RETRY):
        logger.info(f"press power button")
        test_agent.power_button.press()
        start = time.perf_counter()
        while time.perf_counter() - start < MAX_WAIT_FOR_CIB_VOLTAGE:
            if cib_voltage_detected(test_agent):
                logger.info(f"device booting after {_} retries!")
                return
            time.sleep(1)
        logger.info("no voltage detected on any cib - let's retry press power button")
    assert False, f"Device NOT booted after {MAX_NUMBER_OF_POWER_RETRY} retries!\n"




