@set RTT_ROOT=E:\0.SourceCode\rt-thread
@set BSP_ROOT=%cd%\.\..\imx6ull

@set RTT_TOOL_PATH=%cd%\..\..\..\.\tools\imx

:: only support rt-smart and rt-thread
@set RTT_PROJ=rt-thread

@set RTT_CC=gcc
@set RTT_CC_PREFIX=arm-none-eabi-

@set RTT_EXEC_PATH=%ENV_ROOT%\tools\gnu_gcc\arm_gcc\mingw\bin
@set PATH=%RTT_EXEC_PATH%;%RTT_TOOL_PATH%;%ENV_ROOT%\tools\gnu_gcc\arm_gcc\mingw\bin;%PATH%

@echo config finished.