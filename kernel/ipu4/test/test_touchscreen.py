def test_touchscreen(target):
    input_devices = target.cmd('ls /dev/input').strip().split()
    print(f"Found input devices: {input_devices}")
    assert 'touchscreen0' in input_devices
