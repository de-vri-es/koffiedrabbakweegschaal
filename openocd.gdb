target extended-remote | openocd -f openocd.cfg
set print asm-demangle on
load
monitor reset halt
step
