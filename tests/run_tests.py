#!/usr/bin/env python3
"""Run the existing host suite plus CBoard/auto-aim cases on Linux or Windows."""
from pathlib import Path
import argparse
import shlex
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument('--compiler', default='gcc')
parser.add_argument('--output', default='build/host-tests')
args = parser.parse_args()
root = Path(__file__).resolve().parent.parent
out = Path(args.output).resolve()
out.mkdir(parents=True, exist_ok=True)
common = ['-std=c11', '-O2', '-Wall', '-Wextra', '-Werror',
          '-Itests/stubs', '-Imodule/remote', '-Iapplication/gimbal',
          '-Iapplication/cmd', '-Iapplication/auto_aim', '-Imodule/cboard',
          '-Imodule/vision_comm', '-Ibsp/usb']
auto_sources = ['application/auto_aim/auto_aim.c',
                'application/auto_aim/auto_aim_coordinates.c', 'module/cboard/cboard_protocol.c',
                'module/vision_comm/vision_comm.c', 'bsp/usb/vision_usb_port.c']
runtime_sources = ['application/gimbal/gimbal_runtime.c', 'application/gimbal/2yaw_gimbal.c',
                   'application/gimbal/gimbal_control.c', 'application/gimbal/vofa_protocol.c',
                   'application/gimbal/gimbal_imu_text.c', *auto_sources]
cases = []
for line in (root / 'tests/run_gimbal_tests.ps1').read_text().splitlines():
    if not line.strip().startswith('& $Compiler '):
        continue
    flags = shlex.split(line.strip()[len('& $Compiler '):])
    index = flags.index('-o')
    name = Path(flags[index + 1]).stem
    flags[index + 1] = str(out / name)
    cases.append((name, flags))
cases.append(('binary_remote', common + ['-DAUTO_AIM_USB_MODE=1', '-Wno-unused-function', '-Ibsp/usart', 'tests/remote_control_test.c', 'module/remote/remote_control.c', 'application/cmd/dt7_cmd.c', '-o', str(out / 'binary_remote')]))
cases.append(('cboard_protocol', common + ['tests/cboard_protocol_test.c',
    'module/cboard/cboard_protocol.c', 'module/vision_comm/vision_comm.c',
    'application/auto_aim/auto_aim_coordinates.c',
    'application/gimbal/gimbal_control.c', '-o', str(out / 'cboard_protocol')]))
for confirmed in (0, 1):
    name = f'auto_aim_runtime_cal{confirmed}'
    cases.append((name, common + ['-DAUTO_AIM_USB_MODE=1',
        f'-DAUTO_AIM_COORDINATES_CONFIRMED={confirmed}',
        'tests/auto_aim_runtime_test.c', *runtime_sources, '-o', str(out / name)]))
cases.append(('auto_aim_observe', common + ['-DAUTO_AIM_USB_MODE=1',
    '-DGIMBAL_MOTOR_OUTPUT_ENABLED=0', '-DAUTO_AIM_COORDINATES_CONFIRMED=1',
    'tests/auto_aim_observe_test.c', *runtime_sources, '-o', str(out / 'auto_aim_observe')]))
cases.append(('vision_comm', common + ['tests/vision_comm_test.c',
    'module/vision_comm/vision_comm.c', '-o', str(out / 'vision_comm')]))
cases.append(('vision_usb_port', common + ['tests/vision_usb_port_test.c',
    'bsp/usb/vision_usb_port.c', '-o', str(out / 'vision_usb_port')]))
for name, flags in cases:
    subprocess.run([args.compiler, *flags, '-lm'], cwd=root, check=True)
    exe = flags[flags.index('-o') + 1]
    subprocess.run([exe], cwd=root, check=True)
    print(f'{name}: PASS', flush=True)
print(f'All {len(cases)} host suites passed.')
