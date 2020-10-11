# Dockerfile cheat sheet
```sh
vi Dockerfile
```

```rb
# Can be used it in FROM, $<key>
ARG <key>=<value>
# Image to be used
FROM <image>
# Use it to add metadata such as maintainer
LABEL <key>=<value>
# For now on docker will be in that path
WORDKDIR <containerpath>
#  It can be used below, $<key> to refer to <containerpath>
ENV <key>=<containerpath>
# Executed in a new layer on top of the image. Use backslash at the end to jump of line.
RUN <command>
# Unique or only last taken into account. To concatenate use semicolons. User can overwrite it
CMD <command>
# User can not overwrite it
ENTRYPOINT <command>
# In containerpath include the name to be copied
COPY <localpath> <containerpath>
# In containerpath do not include the name to be copied
ADD <localzippath> <containerpath>
# 1 something went wrong. use NONE to disable healthcheck inheritance. Unique or last
HEALTHCHECK --interval=<sec>s --timeout=<sec>s --retries=<num> CMD <command> || exit 1
# By default SIGTERM, with SIGKILL it will be removed after docker stop <docker>
STOPSIGNAL SIGKILL
# It listens to that port
EXPOSE <port>
```

**It should specify at least one `CDM` or `ENTRYPOINT` command.**
