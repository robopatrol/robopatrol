# Robo-Patrol

ROS (Robot OS) nodes for Turtlebot.

Robo-Patrol: [![Travis Build Status](https://travis-ci.org/robopatrol/robopatrol.svg?branch=master)](https://travis-ci.org/robopatrol/robopatrol)
[![Coverage Status](https://coveralls.io/repos/github/robopatrol/robopatrol/badge.svg?branch=master)](https://coveralls.io/github/robopatrol/robopatrol?branch=master)
[![](https://imagelayers.io/badge/robopatrol/robopatrol:latest.svg)](https://imagelayers.io/?images=robopatrol/robopatrol:latest)

[REST Server](https://github.com/robopatrol/robopatrol-server): [![Build Status](https://travis-ci.org/robopatrol/robopatrol-server.svg?branch=master)](https://travis-ci.org/robopatrol/robopatrol-server) [![codecov.io](https://codecov.io/github/robopatrol/robopatrol-server/coverage.svg?branch=master)](https://codecov.io/github/robopatrol/robopatrol-server?branch=master)

[Client Web Application](https://github.com/robopatrol/robopatrol-webapp): [![Travis Build Status](https://travis-ci.org/robopatrol/robopatrol-webapp.svg?branch=master)](https://travis-ci.org/robopatrol/robopatrol-webapp)
[![Coverage Status](https://coveralls.io/repos/github/robopatrol/robopatrol-webapp/badge.svg?branch=master)](https://coveralls.io/github/robopatrol/robopatrol-webapp?branch=master)

Main project documentation: **[Project Wiki](https://github.com/robopatrol/robopatrol/wiki)**

More technical documentation for developers can be found in [/docs](robopatrol/docs/)

## Setup ROS and Robopatrol with Docker

### Get Docker

[Install Docker](https://docs.docker.com/engine/installation/) (or use Homebrew on OS X: `brew install docker docker-machine docker-compose`).

Linux can run Docker without a `docker-machine` VM. So Linux users may skip this next part.

On OS X and Windows a docker-machine needs to be created **once**:
Virtualbox example: `docker create default --driver virtualbox` 

### Run Robopatrol inside a Docker container

Tell Docker to use the docker-machine: `eval $(docker-machine env)`

```shell
docker-compose build
docker-compose up
```

Interactive launch (e.g. to run tests or try something out):

```shell
docker-compose run robopatrol /bin/bash
```

## Release

Releases are published as Docker containers: [Robopatrol on Docker Hub](https://hub.docker.com/u/robopatrol/)

### Publish a Release

Build and upload a container image:

```shell
docker build -t robopatrol/robopatrol .
docker push robopatrol/robopatrol
```
