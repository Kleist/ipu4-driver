import time
import re
import contextlib

import pytest

import frame_count
import actions
import ipu4
import tc358748

TOSHIBA_BRIDGE='ambu_tc358748'
BYTES_PER_PIXEL=4 # BGRx format

CULLINAN_SCOPE_ID=0xC # Grey scope supported on both platforms

LION_SCOPE_ID=0x12 # Green scope supported on both platforms

SCOPE_IS_HD = {
    0xC: True,
    0xD: True,
    0x12: False,
    0x13: False,
}

def scope_resolution(scope_type):
    assert scope_type in SCOPE_IS_HD, f"scope type not found in SCOPE_IS_HD {scope_type}"
    if SCOPE_IS_HD[scope_type]:
        return 800
    else:
        return 400

def scope_resolution_string(scope_type):
    res = scope_resolution(scope_type)
    return f"{res}x{res}"

def scope_type_string(scope):
    return f"scope=0x{scope:02X}"

def scopes_type_string(scopes):
    return "scopes=[" + ",".join([f'0x{scope:02X}' for scope in scopes]) + "]"

def video_device_path(pipeline):
    assert pipeline in [1, 2]
    if pipeline == 1:
        return "video12"
    else:
        return "video13"

def log_dir(request):
    return "/mnt/storage/amp/log/test_intel_ipu4/" + request.node.nodeid

@contextlib.contextmanager
def capture_setup(request, target, test_agent, scope_types=[CULLINAN_SCOPE_ID], force_reload=False):
    """
    Prepare the IPU4 driver, media-ctl and test agent for a test.
    """
    load_modules = force_reload or not 'intel_ipu4_isys' in actions.loaded_modules(target)

    if force_reload:
        # Unload everything, to make sure the test starts from scratch
        # This also ensures that during development, newly overwritten .ko-files are used
        tc358748.clean_slate(target)
        ipu4.unload_all(target)
        actions.unmodprobe(target, module=TOSHIBA_BRIDGE)

    scope_infos = actions.prepare_scopes(target, test_agent, scope_types)

    try:
        if load_modules:
            actions.modprobe_with_dmesg(target, module=ipu4.MODULE, grep_for='IPU4 PCI driver ready')
            for adapter in tc358748.I2C_ADAPTERS:
                assert actions.i2c_address_in_use(target, adapter, tc358748.I2C_ADDRESS)
                assert actions.device_has_regmap(target, adapter=adapter, address=tc358748.I2C_ADDRESS)

            assert 0 == actions.count_v4l2_subdevs(target)
            actions.modprobe(target, module=ipu4.MODULE_ISYS)

        assert actions.count_v4l2_subdevs(target) > 0

        # Setup similar to ieimediactl
        for scope in scope_infos:
            resolution = scope_resolution_string(scope.type)
            actions.write_dmesg(target, f"Setting up media-ctl for {resolution} on channel {scope.pipeline}")
            actions.setup_ipu4_formats(target, pipeline=scope.pipeline, resolution=resolution)
            actions.setup_ipu4_links(target, pipeline=scope.pipeline)

        # Make sure we have clean log dir
        test_log_dir = log_dir(request)
        target.cmd(f'rm -rf "{test_log_dir}" && mkdir -p "{test_log_dir}"')

        yield scope_infos

    finally:
        if force_reload:
            ipu4.unload_all(target)
            assert 0 == actions.count_v4l2_subdevs(target)

            for adapter in tc358748.I2C_ADAPTERS:
                assert not actions.i2c_address_in_use(target, adapter, tc358748.I2C_ADDRESS)
                assert not actions.device_has_regmap(target, adapter=adapter, address=tc358748.I2C_ADDRESS)

def build_gst_pipeline(scope_info, v4l2_args="", crop_rect = None, sink="fakesink"):
    resolution = scope_resolution(scope_info.type)
    vdev = video_device_path(scope_info.pipeline)
    filters = ""
    if crop_rect is not None:
        assert len(crop_rect) == 4, "Expected crop rect = [left, top, width, height]"
        left, top, width, height = crop_rect
        filters = f" ! videocrop left={left} top={top} right={resolution-width} bottom={resolution-height} "

    return f"""v4l2src device=/dev/{vdev} {v4l2_args} io-mode=mmap
            ! video/x-raw,width={resolution},height={resolution},format=BGRx,framerate=30/1
            {filters}
            ! {sink}"""

def build_gst_command(*args, **kwargs):
    return "gst-launch-1.0 -v " + build_gst_pipeline(*args, **kwargs)


@contextlib.contextmanager
def gst_pipeline_in_background(target, scope_info, **kwargs):
    gst_cmd = build_gst_command(scope_info, **kwargs)
    actions.write_dmesg(target, f"Running {gst_cmd} in the background")
    def gst_launch_pids():
        GST_PIDOF_CMD = 'pidof gst-launch-1.0 2>/dev/null||echo "No gst-launch-1.0"'
        # Saves last value on "function".last, to be able to access it outside lambda below
        gst_launch_pids.last = target.cmd(GST_PIDOF_CMD).strip()
        return gst_launch_pids.last

    pids_before = gst_launch_pids()
    print(f"pidof(s) gst-launch-1.0 before: {pids_before}")
    result = target.cmd(f'nohup {gst_cmd} >/dev/kmsg 2>&1 &')

    # Since we're starting gst in the background, we won't notice if it fails early. 
    # This is a sanity check to catch that (if it fails fast enough)
    assert actions.delay_until(lambda: pids_before != gst_launch_pids()) < 5, "gst-launch-1.0 seems to have failed, please check dmesg"
    pids_during = gst_launch_pids.last
    print(f"pidof(s) gst-launch-1.0 during: {pids_during}")

    current_pid = sorted(map(int, pids_during.split(' ')))[-1]
    try:
        yield
    finally:
        actions.write_dmesg(target, f"Killing background gst for pipeline {scope_info.pipeline} with pid {current_pid}")
        actions.kill_pid_and_wait_for_stopped(target, current_pid)
        print(f"pidof(s) gst-launch-1.0 after kill: {gst_launch_pids()}")


def test_unbind_while_streaming(request, target, test_agent):
    """
    Verify that we don't crash when unbinding while gstreamer is running.
    This is critical for the development workflow, where we need to reload
    the driver 100s of times per day. Without this working, reload can only
    be done through reboot, which takes significantly longer.

    NB: v4l2 and the media ctl framework does not support unbinding while streaming,
    so this is not guaranteed to continue working.
    See https://lore.kernel.org/all/Zc3F7On4kghB_PWW@kekkonen.localdomain/
    """
    scope = CULLINAN_SCOPE_ID
    with capture_setup(request, target, test_agent, [scope], force_reload=True) as scope_infos:
        resolution = scope_resolution(scope_infos[0].type)
        vdev = video_device_path(scope_infos[0].pipeline)
        gstreamer_cmd = "gst-launch-1.0 -v " + build_gst_pipeline(scope_infos[0])
        actions.write_dmesg(target, f"Running {gstreamer_cmd}")
        target.cmd(gstreamer_cmd, timeout=5, required_exit_status=None) # None == expect timeout
        actions.write_dmesg(target, f"gstreamer call timed out (still running)")
        gst_pid = target.cmd('pidof gst-launch-1.0').strip()

    # After unbind (handled by the `with capture_setup`, gst-launch-1.0 should be stopped
    target.cmd('pidof gst-launch-1.0', required_exit_status=1).strip()

@pytest.mark.parametrize('scope', [CULLINAN_SCOPE_ID, LION_SCOPE_ID], ids=scope_type_string)
def test_gstreamer_single(request, target, test_agent, scope):
    output_path = f"{log_dir(request)}/gstreamer_frame.raw"
    with capture_setup(request, target, test_agent, [scope]) as scope_infos:
        gstreamer_cmd = "gst-launch-1.0 -v " + build_gst_pipeline(scope_infos[0],
                                                                  v4l2_args="num-buffers=1",
                                                                  sink=f"filesink location={output_path}")
        actions.write_dmesg(target, f"Running {gstreamer_cmd}")
        target.cmd(gstreamer_cmd)
        actions.write_dmesg(target, f"gstreamer done")

        print(f"Converting frame to PNG")
        resolution = scope_resolution(scope_infos[0].type)
        target.cmd(f'bgr0_to_image -W {resolution} -H {resolution} {output_path} {log_dir(request)}/gstreamer_frame.png')

def test_gstreamer_no_lost_frames(request, target, test_agent):
    scope = CULLINAN_SCOPE_ID # This is resolution independent, so no need to test for multiple resolutions
    NUM_FRAMES=30
    MAX_LOST_FRAMES_LOGS=1 # We sometimes see frames lost during startup
    with capture_setup(request, target, test_agent, [scope]) as scope_infos:
        # lost frames is logged as a warning, which is shown with GST_DEBUG=3
        gstreamer_cmd = "GST_DEBUG=3 gst-launch-1.0 -v " + build_gst_pipeline(scope_infos[0],
                                                                  v4l2_args=f"num-buffers={NUM_FRAMES}")
        actions.write_dmesg(target, f"Running {gstreamer_cmd}")
        gstreamer_output = target.cmd(gstreamer_cmd)
        actions.write_dmesg(target, f"gstreamer done")

        assert gstreamer_output.count('lost frames') <= MAX_LOST_FRAMES_LOGS

@pytest.mark.parametrize('scope', [LION_SCOPE_ID, CULLINAN_SCOPE_ID], ids=scope_type_string)
def test_yavta_single(request, target, test_agent, scope):
    frames=2
    with capture_setup(request, target, test_agent, [scope]) as scope_infos:
        resolution = scope_resolution(scope_infos[0].type)
        vdev = video_device_path(scope_infos[0].pipeline)

        # Clear dmesg, so the checks later only find new messages
        actions.clear_dmesg(target, "Cleared dmesg, calling yavta next")
        yavta_cmd = f'yavta -c{frames} -n5 -s {resolution}x{resolution} --file={log_dir(request)}/frame-#.bin -f XBGR32 /dev/{vdev}'
        actions.write_dmesg(target, f"Running {yavta_cmd}")
        target.cmd(yavta_cmd, timeout=5)
        actions.write_dmesg(target, f"yavta done")

        dmesg = target.cmd('dmesg')

        # Errors that should no longer happen at current state
        assert not re.search("isys port open ready failed 0\n", dmesg)

        assert not re.search("intel_ipu4.isys.24: isys fw com open failed -5\n", dmesg)

        assert not re.search("intel_ipu4.isys.24: stream open time out\n", dmesg)

        assert not re.search ("intel_ipu4.isys.24: start stream of firmware failed\n", dmesg)

        assert f'{frames}' == target.cmd(f'ls {log_dir(request)}/|wc -l').strip(), f"Expected {frames} .bin files"

        # Because of EMB-2083, we don't look at the first frame
        for frame in range(1, frames):
            print(f"Converting frame {frame+1}/{frames}")
            target.cmd(f'bgr0_to_image -W {resolution} -H {resolution} {log_dir(request)}/frame-{frame:06d}.bin {log_dir(request)}/frame-{frame:06d}.png')

@pytest.mark.parametrize('scope', [CULLINAN_SCOPE_ID, LION_SCOPE_ID], ids=scope_type_string)
def test_yavta_many(request, target, test_agent, scope):
    """
    Testing that capturing more than 20 frames works.
    Early version of the new IPU4 driver didn't call ipu6_put_fw_msg_buf to
    release msg buffers, so it ran out of buffers after 20 frame captures.
    """
    frames=100
    with capture_setup(request, target, test_agent, [scope]) as scope_infos:
        resolution = scope_resolution(scope_infos[0].type)
        vdev = video_device_path(scope_infos[0].pipeline)
        yavta_cmd = f'yavta -c{frames} -n5 -s {resolution}x{resolution} --file={log_dir(request)}/frame-#.bin -f XBGR32 /dev/{vdev}'


        actions.write_dmesg(target, f"Running {yavta_cmd}")
        target.cmd(yavta_cmd)
        actions.write_dmesg(target, f"yavta done")

        # Only converting second and last frame, since it takes a while per frame
        # Because of EMB-2083, we don't look at the first frame
        for frame in [1, frames-1]:
            print(f"Converting frame {frame+1}/{frames}")
            target.cmd(f'bgr0_to_image -W {resolution} -H {resolution} {log_dir(request)}/frame-{frame:06d}.bin {log_dir(request)}/frame-{frame:06d}.png')

        # If the test passed - throw away the bin files so they don't take up space on gitlab
        target.cmd(f'rm -rf "{log_dir(request)}"/*.bin')

@pytest.mark.parametrize('scopes', (
        # Insert two scopes, to ensure we get one of them on pipeline 2
        # On aBox2, the second scope is always on pipeline2
        # On aView2, it depends on previous scopes inserted
        [CULLINAN_SCOPE_ID, LION_SCOPE_ID],
        [LION_SCOPE_ID,     CULLINAN_SCOPE_ID],
), ids=scopes_type_string)
def test_yavta_pipeline2(request, target, test_agent, scopes):
    """
    Tests pipeline 2 works. In order to get the scope to use pipeline two, we must insert another scope first.
    """
    frames=2
    with capture_setup(request, target, test_agent, scopes) as scope_infos:
        for scope in scope_infos:
            if scope.pipeline == 2:
                break

        assert scope.pipeline == 2, "No scope with pipeline 2 found"
        print(f"Testing with scope type {scope.type} (because it got pipeline 2)")

        resolution = scope_resolution(scope.type)
        vdev = video_device_path(scope.pipeline)

        # Clear dmesg, so the checks later only find new messages
        actions.clear_dmesg(target, "Cleared dmesg, calling yavta next")
        yavta_cmd = f'yavta -c{frames} -n5 -s {resolution}x{resolution} --file={log_dir(request)}/frame-#.bin -f XBGR32 /dev/{vdev}'
        actions.write_dmesg(target, f"Running {yavta_cmd}")
        target.cmd(yavta_cmd, timeout=5)
        actions.write_dmesg(target, f"yavta done")

        assert f'{frames}' == target.cmd(f'ls {log_dir(request)}/|wc -l').strip(), f"Expected {frames} .bin files"
        # Because of EMB-2083, we don't look at the first frame
        for frame in range(1, frames):
            print(f"Converting frame {frame+1}/{frames}")
            target.cmd(f'bgr0_to_image -W {resolution} -H {resolution} {log_dir(request)}/frame-{frame:06d}.bin {log_dir(request)}/frame-{frame:06d}.png')


@pytest.mark.parametrize('scopes', (
        [LION_SCOPE_ID,     CULLINAN_SCOPE_ID],
        [CULLINAN_SCOPE_ID, LION_SCOPE_ID],
), ids=scopes_type_string)
def test_gstreamer_dual(request, target, test_agent, scopes):
    """
    Tests both pipelines in parallel
    """
    num_frames=100
    with capture_setup(request, target, test_agent, scopes) as scope_infos:
        assert scope_infos[0].type in scopes, f"Unknown scope {scope_infos[0]}"
        assert scope_infos[1].type in scopes, f"Unknown scope {scope_infos[1]}"
        assert scope_infos[0].type != scope_infos[1].type
        def raw_filesink_pipeline(scope_info):
            res = scope_resolution(scope_info.type)
            name = f"0x{scope_info.type:x}_pipeline{scope_info.pipeline}"
            output_file = f"{log_dir(request)}/frame_{name}.raw"

            return output_file, res, build_gst_pipeline(scope_info,
                                                        v4l2_args=f"num-buffers={num_frames}",
                                                        sink=f"filesink location={output_file}")

        output1, res1, pipeline1 = raw_filesink_pipeline(scope_infos[0])
        output2, res2, pipeline2 = raw_filesink_pipeline(scope_infos[1])
        gstreamer_cmd = f"""
            gst-launch-1.0 -v
              {pipeline1}
              {pipeline2}
        """
        actions.write_dmesg(target, f"Running {gstreamer_cmd}")
        target.cmd(gstreamer_cmd)
        actions.write_dmesg(target, f"gstreamer done")

        frame_size_1 = res1*res1*BYTES_PER_PIXEL
        assert frame_size_1*num_frames == actions.file_size(target, output1)

        frame_size_2 = res2*res2*BYTES_PER_PIXEL
        assert frame_size_2*num_frames == actions.file_size(target, output2)

        target.cmd(f'rm -rf "{log_dir(request)}"')

@pytest.mark.parametrize('scope', [CULLINAN_SCOPE_ID, LION_SCOPE_ID], ids=scope_type_string)
def test_frame_count(request, target, test_agent, scope):
    output_folder=f"{log_dir(request)}"
    output_file = f"{output_folder}/gstreamer_frame.raw"
    FRAMES=4
    with capture_setup(request, target, test_agent, [scope]) as scope_infos:
        gstreamer_cmd = "gst-launch-1.0 -v " + build_gst_pipeline(scope_infos[0],
                                                                  v4l2_args=f"num-buffers={FRAMES}",
                                                                  sink=f"filesink location={output_file}")
    target.cmd(gstreamer_cmd)
    target.copy_from_target(f"{output_file}", "/tmp/")
    frames = frame_count.extract("/tmp/gstreamer_frame.raw", scope_resolution(scope_infos[0].type))
    
    second = frames[1]
    # First frame often includes garbage data in the first lines which 
    # ruins the frame counter so skip it

    def wrap_255(val):
        return val if val < 256 else val-255

    assert list(map(wrap_255, range(second, second+FRAMES-1))) == frames[1:]

@pytest.mark.parametrize('scopes', (
        [CULLINAN_SCOPE_ID, LION_SCOPE_ID],
        [LION_SCOPE_ID,     CULLINAN_SCOPE_ID],
), ids=scopes_type_string)
def test_restart_isys(request, target, test_agent, scopes):
    """
    The test reproduces the IPU4 FW behavior, where we were not
    able to re-open new streams. The error results in SERIOUS_PIPELINE_ERROR in AMP,
    and 'isys power cycle required' logged in dmesg. Note that not all 
    SERIOUS_PIPELINE_ERROR's are 'isys power cycle required'.

    The error is triggered by removing a scope while the IPU4 is streaming from it,
    while another stream is also running.

    Test sequence:

    1. Insert 2 scopes
    2. Starting gstreamer pipeline on the first scope in background
    3. Starting gstreamer pipeline on the second scope in background
    4. Disconnecting the second scope
    5. Stopping the second scope pipeline
    6. Verify that the first scope pipeline is still streaming
    7. Inserting the second scope (only need for next iteration)
    8. Check for 'isys power cycle required' in the logs
    9. Repeat from 3
    """
    REINSERTIONS=3
    CROP_SIZE = 4 # Cropping to 4x4 frame to only keep frame count and keep file size low
    actions.clear_var_log_messages(target)
    log_dir_path = log_dir(request)
    first_output = f"{log_dir_path}/gstreamer0.raw"

    def first_output_size():
        try:
            return actions.file_size(target, first_output)
        except:
            return -1

    with capture_setup(request, target, test_agent, scopes) as scope_infos:
        actions.write_dmesg(target, "Before gst: " + target.cmd(f'ls -lh {log_dir_path}'))
        with gst_pipeline_in_background(target,
                                        scope_infos[0],
                                        crop_rect=[0,0,CROP_SIZE,CROP_SIZE],
                                        sink = f'filesink location={first_output} buffer-mode=unbuffered'):
            assert actions.delay_until(lambda: first_output_size() > 0) < 5
            target.cmd('echo 1 > /sys/module/intel_ipu4_isys/parameters/force_need_reset')
            for i in range(REINSERTIONS):
                actions.write_dmesg(target, "1 gst running: " + target.cmd(f'ls -lh {log_dir_path}'))
                before_size = first_output_size()
                print(f"Size of {first_output} before 2nd scope is inserted: {before_size}")
                print(f"Reinsertion {i+1}/{REINSERTIONS}")

                with gst_pipeline_in_background(target, scope_infos[1]):
                    actions.write_dmesg(target, "2nd gst started: " + target.cmd(f'ls -lh {log_dir_path}'))
                    time.sleep(1)
                    # Toggle scope and restart pipeline on second scope
                    actions.remove_scope_and_wait_for_gone(target, test_agent, scope_infos[1])
                    time.sleep(1)
                actions.write_dmesg(target, "Back to 1 gst running: " + target.cmd(f'ls -lh {log_dir_path}'))

                after_size = first_output_size()
                print(f"Size of {first_output} after second scope is removed: {after_size} (+{after_size-before_size})")
                assert before_size < after_size, f"Outer gstreamer stopped writing to {first_output}"

                actions.insert_scope_and_wait_for_ready(target, test_agent, scope_infos[1])

                # Check we haven't hit the error we're trying to reproduce
                isys_power_cycle_msgs = target.cmd('grep -B 10 "isys power cycle required" /var/log/messages||true', log_output=False)
                assert "" == isys_power_cycle_msgs, f"isys power cycle required happened in reinsertion {i+1}"
