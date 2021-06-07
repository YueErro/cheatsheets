# Valgrind cheat sheet
```sh
sudo apt install valgrind
```

## Table of contents
* [When to use it](#when-to-use-it)
* [Run valgrind](#run-valgrind)
* [Important output messages](#important-output-messages)

### When to use it
Segmentation fault it a memory access outside of what you should access to, outside your segment. This could happen in different situations:
* Index out of bounds
* nullptr
* Uninitialized variables
* ...
In such cases it is recommended to use valgrind.

### Run valgrind
```sh
valgrind <executable_path>
# Usefull flags
valgrind --leak-check=full --show-leak-kinds=all --track-origins=yes --log-file=<save_path> <executable_path>
```

### Important output messages
Indentation is important when looking at the output of valgrind. These are some of the important messages that will give you a clue:
* Invalid ... --> 0 bytes means just the next index
* Conditional ...
* Address ... --> 0xfew numbers less than 5 digits means nullptr
* --> Is not stack: ins not a local variable of your function
* --> Not malloc: there is no call to new
* --> Not (recently) free'd: there is not delete nor release of a pointer