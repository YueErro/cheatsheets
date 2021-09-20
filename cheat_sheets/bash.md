# Bash cheat sheet

## Table of contents
* [Getting started](#Getting-started)
* [Redirection](#Redirection)
* [Control structure](#Control-structure)
* [Arrays](#Arrays)
* [Find](#Find)
* [Sort](#Sort)
* [Color script output](#Color-script-output)
* [Set](#Set)

### Getting started
User input:
```sh
read <varname>
```
Variable's value:
```sh
echo "$<varname>"
# Appending characters
echo "&{<varname>}"
```
There are two type of quoting:
* Weak: uses double quotes `""`.
* Strong: uses single quotes `''`.

Run script in debug mode:
```sh
bash -x <file>.sh
```

### Redirections
Two ways of redirection standard output (STDOUT):
* `>`: Truncate
* `>>`: Append

Standard streams are defined as follows:
* 0 = Standard in (STDIN)
* 1 = Standard out (STDOUT) --> By default
* 2 = Standard error (STDERR)

Redirect STDOUT to STDERR:
```sh
> /dev/stderr
```

Redirect STDERR to STDOUT:
```sh
2>&1
```

Additionally, redirect to the black hole:
```sh
/dev/null 2>&1
```

### Control structure
File operators:
* `-e`: File existance.
* `-d`: Directory existance.
* `-f`: Regular file existance.
* `-h`: Symbolic link existance.
* `-r`: Readable.
* `-w`: Writable.
* `-x`: Executable.

String comparators:
* `-z`: Length of string is zero.
* `-n`: Legth of string is non-zero.
* `=`: Strings are equals.
* `!=`: Strings are not equals.

Integer comparators:
* `-eq`: Integers are equals.
* `-ne`: Integers are not equals.
* `-gt`: Int1 is greater than int2.
* `-ge`: Int1 is greater than or equals to int2.
* `-lt`: Int1 is less than int2.
* `-le`: Int1 is less than or equals to int2.

For conditions use `[[ ]]` with the white space between.

`<` and `>` operators inside `[[ ]]` compare strings, not numbers.

### Arrays
Dealing with arrays:
* `"${#array[@]}"`: Length of the array.
* `"${array[@]}"`: Item values of the array.
* `"${!array[@]}"`: Indexes of the array.

Use `*` instead of `@` if you want to split the value of the array items if they have white space or similar.

Append item to the array:
* `array+=('new item at the end')`: Add to the end.
* `array=('new item to the beginning')`: Add at the beginning.

Delete array:
```sh
# Index
unset array[0]
# whole
unset array
```

Merge arrays:
```sh
array3=("$array1[@]" "$array2[@]")
```

### Find
Matching of many extensions:
```sh
find . -name "*.txt" -o -name "*.sh"
```

More find flags:
* `-type`: Filter by file, `f`, or directory, `d`, or block devices, `b`, or symlinks, `l`.
* `-size n[cwbkMG]`: Filter by size [bytes, 2bytes, 512bytes(default), 1KB, 1MB, 1GB].
* `-path`: Filter by path.
* `-mmin n`: Modified `n` minutes ago.
* `-mmin -n`: Modified less than `n` minutes ago.
* `-mmin +n`: Modified more than `n` minutes ago.

Similarly access time, `-amin`, or changed time, `-cmin` or `time` instead of `min` for day units intead of minutes.

### Sort
Available flags:
* `-n`: Sort numerically.
* `-r`: Reverse the sort order.
* `-k`: Sort by key.
* `-t`: To specify the delimeter.

### Color script output
Basic colors:

| Color   | Value |
|:--------|:------|
| Black   | 0     |
| Red     | 1     |
| Green   | 2     |
| Yellow  | 3     |
| Blue    | 4     |
| Magenta | 5     |
| Cyan    | 6     |
| White   | 7     |

E.g:
```sh
GREEN=$(tput setaf 2)
echo "${GREEN}This output is in green"
```

### set
* `set -x`: Debug mode.
* `set -e`: Exist the script immediately if any command returns a non-zero status.
* `set -x`: Debug mode
* `set -u`: Treat `unset` variables as an error when substituting.