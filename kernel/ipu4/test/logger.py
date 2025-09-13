import argparse
import logging
import os

def init():
    parser = argparse.ArgumentParser()
    parser.add_argument("--test-log", default="./ambu_test.log", action="store", type=str, help="Specifies where to put the log file (default: ./ambu_test.log)")
    parser.add_argument("--log-level", default="info", choices=["debug", "info"], action="store", type=str, help="Set log level severity and format for logging (default: info)")
    args, unknown_args = parser.parse_known_args()

    log_path = f"{os.path.abspath(args.test_log)}"

    def set_logging_basic_config(logging_level, logging_format):
        os.makedirs(os.path.dirname(log_path), exist_ok=True)
        logging.basicConfig(filename=log_path, level=logging_level, format=logging_format, datefmt="%Y-%m-%dT%H:%M:%S")


    if args.log_level == "debug":
        formatter = '%(asctime)s.%(msecs)03d - %(filename)s:%(name)s:%(funcName)s:%(lineno)d: - %(levelname)s - %(message)s'
        set_logging_basic_config(logging.DEBUG, formatter)
    elif args.log_level == "info":
        formatter = '%(asctime)s.%(msecs)03d %(name)s - %(levelname)s - %(message)s'
        set_logging_basic_config(logging.INFO, formatter)