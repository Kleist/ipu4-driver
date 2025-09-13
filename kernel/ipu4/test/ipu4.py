import actions

MODULE='intel_ipu4'
MODULE_ISYS='intel_ipu4_isys'
PCI_DEVICE="0000:00:03.0"

def unload_all(target):
    if actions.pci_device_bound(target, PCI_DEVICE):
        actions.unbind_pci_device(target, PCI_DEVICE)
    actions.unmodprobe(target, module=MODULE_ISYS)
    actions.unmodprobe(target, module=MODULE)
