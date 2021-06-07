# GDB (GNU Project Debugger) cheat sheet
```sh
sudo apt install gdb
```

## Table of contents
* [Run gdb](#run-gdb)
* [Cheatsheet](#cheatsheet)

### RUN gdb
```sh
# Compile in debug mode
gdb --args <executable_path> <args...>
```

### Cheatsheet
`(gdb)`
```sh
# Run the program to be debugged
run
# Kill the running program
kill
# List code with lines
list <filename>:<function/line>
# Set a breakpoint
break <line>
# Get information about the break- and watchpoints
info breakpoints
# Delete specific breakpoint
delete <breakpoint#>
# Delete all breakpoints
clear
# Enable breakpoint
enable <breakpoint#>
# Disable breakpoint
disable <breakpoint#>
# Go to next instruction diving into functions
step
# Go to next instruction without diving into functions
next
# Continue until the current function returns
finish
# Continue normal execution
continue
# Get the variables
info locals
# Get variable value
print <variable_name>
# Print in each stepping instruction
display <variable_name>
# Get information about the displays
info display
# Remove display
undisplay <display#>
# Enable display
enable <display#>
# Disable display
disable <display#>
# Get threads
info threads
# Choose thread to operate on
thread <thread#>
```