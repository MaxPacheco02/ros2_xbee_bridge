#!/bin/bash


IMAGE_NAME=ros2_xbee_bridge_img
IMAGE_TAG=humble

export DOCKER_BUILDKIT=1
docker build \
  --ssh default \
  -t $IMAGE_NAME:$IMAGE_TAG \
  -f Dockerfile .

