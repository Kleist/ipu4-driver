import pytest

from prepare_device import LOG_MESSAGE_AFTER_PREPARE

@pytest.mark.order(-1)
def test_kernel_oops(target):
    """
    Ensure that no kernel oops's like null-pointer dereference, etc. happened
    during the other tests
    """
    kernel_oopses = target.cmd(
        # Use sed to print lines after LOG_MESSAGE_AFTER_PREPARE
        f'sed -n "/{LOG_MESSAGE_AFTER_PREPARE}/,\\$p" /mnt/storage/amp/log/messages*'
        # Use grep to find any Oops-like messages
        + ' | grep -EC 5 "kernel: .* (RIP|BUG|Oops):"'
        # Grep finding nothing is not a failure
        + '|| true'
        ).strip()
    assert "" == kernel_oopses

@pytest.mark.order(-1)
def test_for_sysrq_help(target):
    """
    Target seems to get keypresses for sysrq commands, most of them result
    in HELP message printed, but some actually do stuff (like reboot the device)

    The intention with this test is to detect if we accidentally trigger sysrq
    commands in tests.

    [Ref]: EMB-2181
    """
    assert "" == target.cmd('grep -a "sysrq: HELP" /mnt/storage/amp/log/messages* 2>/dev/null|head')
