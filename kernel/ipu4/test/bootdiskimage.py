from logger import init as logger_init # Avoid name clash
import sys

import argparse
import logging
import time

import ambu_test_library as atl

import helpers

logger_init()

logger = logging.getLogger('bootdiskimage')
# Scope parameters are written on first boot, and it takes ~5s per scope,
# so worst case we spend 13*5 seconds writing scope parameters
# This is only the case when the previous job used different scope parameters,
# for example when switching between release branch and master.
DEFAULT_MAX_BOOT_TIME_SEC = 180


def boot_image_on_abox2(image, max_boot_time):
    test_agent = atl.test_agent.get()
    target = atl.target.get_abox2()
    logger.info("disable power")
    test_agent.power.disable()
    time.sleep(30)

    logger.info("disable all scopes and outputs")
    test_agent.init()

    logger.info("enable usb image")
    test_agent.usb.enable_image([image])
    logger.info(f"usb image: {test_agent.usb.image}")

    logger.info("enable power")
    test_agent.power.enable()
    time.sleep(3)

    logger.info("press power button and wait for boot..")
    start = time.perf_counter()
    helpers.press_power_button_and_retry(test_agent)
    elapsed_time = time.perf_counter() - start

    boot_time = helpers.delay_until(target.is_booted, 3 * 60)
    assert target.is_booted(), f"abox2 not booted within {boot_time} seconds!"
    target.cmd("sync")  # sync files on target in case boot time is above expected threshold to have an updated syslog
    assert (
        boot_time < max_boot_time
    ), f"abox2 not booted after {max_boot_time} seconds, it took {boot_time} seconds!"
    logger.info(f"Abox2 booted after {boot_time} seconds")
    logger.info(f"Total boot session time: {elapsed_time + boot_time} seconds")

def boot_image_on_aview2(image, max_boot_time):
    def press_power_button_and_reset_signal_and_retry(test_agent):
        def is_cib_voltage_stable(test_agent):
            for _ in range(5):
                if helpers.no_cib_voltage_detected(test_agent):
                    return False
                time.sleep(0.5)
            return True

        for _ in range(helpers.MAX_NUMBER_OF_POWER_RETRY):
            logger.info("press power button")
            test_agent.power_button.press()
            time.sleep(1)
            logger.info("press hw reset signal")
            test_agent.hw_reset_signal.press()
            start = time.perf_counter()
            while time.perf_counter() - start < helpers.MAX_WAIT_FOR_CIB_VOLTAGE:
                if is_cib_voltage_stable(test_agent):
                    logger.info(f"device booting after {_} retries!")
                    return
                time.sleep(1)
            logger.debug("no voltage detected on any cib - let's retry")
        assert False, f"Device NOT booted after {helpers.MAX_NUMBER_OF_POWER_RETRY} retries!\n"

    test_agent = atl.test_agent.get()
    target = atl.target.get_aview2()

    logger.info("disable power")
    test_agent.power.disable()
    logger.info("enable hw reset signal")
    test_agent.hw_reset_signal.enable()  # press reset btn
    time.sleep(30)

    logger.info("disable all scopes and outputs")
    test_agent.init()

    logger.info("enable usb image")
    test_agent.usb.enable_image([image])
    logger.info(f"usb image: {test_agent.usb.image}")

    logger.info("enable power")
    test_agent.power.enable()
    logger.info("disable hw reset signal")
    test_agent.hw_reset_signal.disable()  # release reset btn

    print("press power button and reset signal and wait for boot..")
    start = time.perf_counter()
    press_power_button_and_reset_signal_and_retry(test_agent)
    elapsed_time = time.perf_counter() - start

    boot_time = helpers.delay_until(target.is_booted, max_boot_time + 60)
    assert target.is_booted(), f"aview2 not booted within {boot_time} seconds!"
    target.cmd("sync")  # sync files on target in case boot time is above expected threshold to have an updated syslog
    assert (
        boot_time < max_boot_time
    ), f"aview2 booted after {boot_time} seconds, but the threshold was set to {max_boot_time} seconds!"
    logger.info(f"Aview2 booted after {boot_time} seconds")
    logger.info(f"Total boot session time: {elapsed_time + boot_time} seconds")


def boot_image(target, image, max_boot_time):
    if target == "abox2":
        boot_image_on_abox2(image, max_boot_time)
    elif target == "aview2":
        boot_image_on_aview2(image, max_boot_time)

def log_to_stdout(level):
    logger.setLevel(level)
    sh = logging.StreamHandler(sys.stdout)
    formatter = logging.Formatter('[%(asctime)s %(levelname)s %(filename)s.%(funcName)s:%(lineno)d] %(message)s')
    sh.setFormatter(formatter)
    logger.addHandler(sh)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('image', help="")
    parser.add_argument('target', choices=['abox2', 'aview2'])
    parser.add_argument('-v','--verbose', default=0, action='count', help='Add -v for INFO output, -vv for DEBUG output')
    parser.add_argument("--test-log", default="./bootdiskimage.log", action="store", type=str, help="Specifies where to put the log file (default: ./bootdiskimage.log)")
    parser.add_argument("--boottime", default=DEFAULT_MAX_BOOT_TIME_SEC, type=int, help="Max time to wait for image to boot")
    args = parser.parse_args()

    if args.verbose == 0:
        log_to_stdout(logging.WARNING)
    if args.verbose == 1:
        log_to_stdout(logging.INFO)
    else:
        log_to_stdout(logging.DEBUG)

    boot_image(args.target, args.image, args.boottime)
