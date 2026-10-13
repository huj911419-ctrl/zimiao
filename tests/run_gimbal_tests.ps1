param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot
Push-Location $repoRoot
try {
    New-Item -ItemType Directory -Force -Path build | Out-Null
    & $Compiler -std=c11 -O2 -Wall -Wextra -Werror -Iapplication/gimbal tests/gimbal_control_test.c application/gimbal/gimbal_control.c -o build/gimbal_control_test.exe
    if ($LASTEXITCODE) { throw 'Control test compilation failed' }
    & ./build/gimbal_control_test.exe
    if ($LASTEXITCODE) { throw 'Control tests failed' }
    & $Compiler -std=c11 -O2 -Wall -Wextra -Werror -Wno-unused-function -Itests/stubs -Imodule/remote -Ibsp/usart -Iapplication/gimbal -Iapplication/cmd tests/remote_control_test.c module/remote/remote_control.c application/cmd/dt7_cmd.c -o build/remote_control_test.exe
    if ($LASTEXITCODE) { throw 'RC test compilation failed' }
    & ./build/remote_control_test.exe
    if ($LASTEXITCODE) { throw 'RC tests failed' }
    & $Compiler -std=c11 -O2 -Wall -Wextra -Werror -DGIMBAL_KEY_MOUSE_ENABLED=1 -Itests/stubs -Imodule/remote -Ibsp/usart -Iapplication/gimbal -Iapplication/cmd tests/key_mouse_control_test.c module/remote/remote_control.c application/cmd/dt7_cmd.c application/cmd/gimbal_key_mouse.c application/gimbal/gimbal_control.c -o build/key_mouse_control_test.exe
    if ($LASTEXITCODE) { throw 'Keyboard/mouse test compilation failed' }
    & ./build/key_mouse_control_test.exe
    if ($LASTEXITCODE) { throw 'Keyboard/mouse tests failed' }
    & $Compiler -std=c11 -O2 -Wall -Wextra -Werror -Itests/stubs -Ibsp/usart tests/usart_receive_test.c bsp/usart/bsp_usart.c -o build/usart_receive_test.exe
    if ($LASTEXITCODE) { throw 'UART receive test compilation failed' }
    & ./build/usart_receive_test.exe
    if ($LASTEXITCODE) { throw 'UART receive tests failed' }
    & $Compiler -std=c11 -O2 -Wall -Wextra -Werror -Iapplication/gimbal tests/vofa_protocol_test.c application/gimbal/vofa_protocol.c -o build/vofa_protocol_test.exe
    if ($LASTEXITCODE) { throw 'VOFA protocol test compilation failed' }
    & ./build/vofa_protocol_test.exe
    if ($LASTEXITCODE) { throw 'VOFA protocol tests failed' }
    & $Compiler -std=c11 -O2 -Wall -Wextra -Werror -Itests/stubs -Imodule/remote -Iapplication/gimbal -Iapplication/cmd tests/vofa_runtime_test.c application/gimbal/vofa_protocol.c application/gimbal/gimbal_control.c application/gimbal/2yaw_gimbal.c application/gimbal/gimbal_runtime.c application/gimbal/gimbal_imu_text.c -o build/vofa_runtime_test.exe
    if ($LASTEXITCODE) { throw 'VOFA runtime test compilation failed' }
    & ./build/vofa_runtime_test.exe
    if ($LASTEXITCODE) { throw 'VOFA runtime tests failed' }
    & $Compiler -std=c11 -O2 -Wall -Wextra -Werror -Itests/stubs -Iapplication/gimbal tests/imu_uart_log_test.c application/gimbal/gimbal_imu_uart.c application/gimbal/gimbal_imu_text.c -o build/imu_uart_log_test.exe
    if ($LASTEXITCODE) { throw 'USART10 logger test compilation failed' }
    & ./build/imu_uart_log_test.exe
    if ($LASTEXITCODE) { throw 'USART10 logger tests failed' }
    & $Compiler -std=c11 -O2 -Wall -Wextra -Werror -Itests/stubs -Iapplication/sentry -Iapplication/gimbal -Ibsp/can tests/can_scan_test.c application/sentry/can_scan.c -o build/can_scan_test.exe
    if ($LASTEXITCODE) { throw 'CAN scan test compilation failed' }
    & ./build/can_scan_test.exe
    if ($LASTEXITCODE) { throw 'CAN scan tests failed' }
} finally { Pop-Location }
