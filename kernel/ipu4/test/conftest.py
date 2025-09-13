import logger

import pytest

import ambu_test_library

import manual_agent

_target = ambu_test_library.target.get()
_test_agent = ambu_test_library.test_agent.get()
if _test_agent is None:
    _test_agent = manual_agent.ManualAgent(_target)

logger.init()

def pytest_addoption(parser):
    # Method to add the option to ini
    parser.addoption(
        "--test-log",
        action="store",
        default="./ambu_test.log",
        help="Specifies the where to put the test log file (default: ./ambu_test.log)",
    )

@pytest.fixture
def test_agent():
    return _test_agent


@pytest.fixture
def target():
    return _target

class FixtureError(Exception):
    pass

def raise_fixture_error(message):
    raise FixtureError(message)

def write_dmesg(message):
    _target.cmd(f'echo "{message}" > /dev/kmsg && sync /mnt/storage/amp/log/*', log_output=False)

@pytest.fixture(scope='function', autouse=True)
def fixture_function(request):
    if _target.is_not_booted():
        raise_fixture_error(f"Test function {request.node.nodeid} not started since target is not booted!")
    # NB: the duplicated ambutest: is needed because it is for some reason stripped from syslog
    # Without this, split_logs.py from emb-tools cannot split the logs per test
    write_dmesg(f"ambutest: ambutest: {request.node.nodeid}: Starting test")

    yield

    if _target.is_not_booted():
        raise_fixture_error(f"Target not booted after test function {request.node.nodeid}!")
    # NB: the duplicated ambutest: is needed because it is for some reason stripped from syslog
    # Without this, split_logs.py from emb-tools cannot split the logs per test
    write_dmesg(f"ambutest: ambutest: {request.node.nodeid}: Test ended")
