# Packages format cheat sheet

## Table of contents
* [Installation](#Installation)
* [Compression](#Compression)

### Installation
```sh
# .rpm pkg
rpm -i <pkg>.rpm
# from source
cd <pkg>
./configure
make
make install
```

# Compression
```sh
# compress into .tar
tar cf <name>.tar <file1> <file2>
# extract from .tar
tar xf <name>.tar
# compress into .tar.gz
tar czf <name>.tar.gz <file1> <file2>
# extract from .tar.gz
tar xzf <name>.tar.gz
# compress into .tar.bz2
tar cjf <name>.tar.bz2 <file1> <file2>
# extract from .tar.bz2
tar xjf <name>.tar.bz2
```
