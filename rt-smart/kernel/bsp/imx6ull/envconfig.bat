@set RTT_ROOT=%cd%\..\..\..\.\kernel

@set RTT_CC=gcc
@set RTT_CC_PREFIX=arm-linux-musleabi-

@set RTT_EXEC_PATH=%ENV_ROOT%\tools\gnu_gcc\arm_gcc\musleabi\bin
@set PATH=%RTT_EXEC_PATH%;%ENV_ROOT%\tools\gnu_gcc\arm_gcc\mingw\bin;%PATH%

@echo config finished.