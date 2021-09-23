target extended-remote | openocd -f openocd.cfg
set print asm-demangle on
monitor arm semihosting enable
monitor arm semihosting_fileio enable
load
monitor reset halt
step
