# docker cheat sheet
```sh
sudo apt install docker docker.io
```

## Table of contents
* [Configuration](#Configuration)
* [Build and Run](#Build-and-Run)
* [Management](#Management)
* [Network](#Network)
* [Docker Hub](#Docker-Hub)

### Configuration
```sh
# docker on start
sudo systemctl enable docker
# docker not on start
sudo systemctl disable docker
# docker without sudo
sudo groupadd docker
sudo usermod -aG docker <user>
# Log out and log in or use newgrp
newgrp docker
```

### Build and Run
```sh
# Create container from image
docker create <image>
# Start container
docker start <container>
# Build image from the Dockerfile
docker build -t <image> <path>
# Create and start the docker
docker run --rm -it -p <port>:<EXPOSEport>  -v <localpath>:<containerpath> --name <container> <image>
# --rm remove container automatically after it exits
# -it connect container to terminal
# -p port
# -v volume share data between them, localparh is usually `pwd`
```

### Management
```sh
# List docker configuration
docker info
# List running containers, use -a to include stopped images
docker ps
# Execute a command in the container
docker exec <name> <command>
# Stop container through SIGTERM
docker stop <name/id>
# Stop container through SiGKILL
docker kill <name/id>
# Remove container, use -v to remove associated volumes
docker rm -f <name/id>
# List images
docker images
# Remove image
docker rmi -f <name/id>
```

### Network
```sh
# List networks
docker network ps
# Host to container
docker network create --subnet <ip> <network>
# Assign IP to container
docker run -it --name <container> --ip <ip> --net <network> <image>
```

### Docker Hub
```sh
# Log in
docker login
# Download the image
docker pull <user>/<image>:<tag>
# Upload the image
docker push <user>/<image>:<tag>
# Add tag to the image
docker tag <user>/<image>:<tag> <user>/<image>:<newtag>
# List history of the image
docker history <user>/<image>:<tag>
# Display logs from the container
docker logs <container>
# Display EXPOSE port of the container
docker port <containter>
# List changes made to a container
docker diff <container>
# Show running processes in the container
docker top <container>
```
