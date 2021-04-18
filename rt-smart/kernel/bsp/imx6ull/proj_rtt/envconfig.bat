@set RTT_ROOT=E:\0.SourceCode\rtthread\rt-thread
@set BSP_ROOT=%cd%\..

@set RTT_TOOL_PATH=%cd%\..\scripts

:: only support rt-smart and rt-thread
@set RTT_PROJ=rt-thread

@set RTT_CC=gcc
@set RTT_CC_PREFIX=arm-none-eabi-

@set RTT_EXEC_PATH=%ENV_ROOT%\tools\gnu_gcc\arm_gcc\mingw\bin
@set PATH=%RTT_EXEC_PATH%;%RTT_TOOL_PATH%;%ENV_ROOT%\tools\gnu_gcc\arm_gcc\mingw\bin;%PATH%

@echo config finished.

@echo off

if "%1"=="clean" (
    :: clear scons scripts
    rm -rf .sconsign.dblite .config.old *.pyc *.elf output.*
    rm -rf cconfig.h
    :: clear building outputs
    make clean
) else if "%1"=="debug" (
    @echo create disassembly file ...
    %RTT_CC_PREFIX%objdump -D -S %RTT_PROJ:-=%.elf > output.DEBUG
    @echo create disassembly file success.
) else (
    :: clear last building objects
    scons --clean
)