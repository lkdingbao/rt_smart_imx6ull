@set RTT_ROOT=E:\0.SourceCode\rtthread\rt-smart\rt-thread
@set BSP_ROOT=%cd%\..

@set RTT_TOOL_PATH=%cd%\..\scripts

:: only support rt-smart and rt-thread
@set RTT_PROJ=rt-smart

@set RTT_CC=gcc
@set RTT_CC_PREFIX=arm-linux-musleabi-

@set RTT_EXEC_PATH=%ENV_ROOT%\tools\gnu_gcc\arm_gcc\musleabi\bin
@set PATH=%RTT_EXEC_PATH%;%RTT_TOOL_PATH%;%ENV_ROOT%\tools\gnu_gcc\arm_gcc\mingw\bin;%PATH%

@echo config finished.