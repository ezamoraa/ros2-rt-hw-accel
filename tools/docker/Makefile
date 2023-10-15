DOCKER_IMAGE=ezamoraa/ros2-rt-hw-accel-dev
REGISTRY=docker.io
IMAGE_DIR=$(dir $(abspath $(lastword $(MAKEFILE_LIST))))
REPO_DIR=$(abspath ${IMAGE_DIR}/../../../)
IMAGE_DIR_REL=$(shell realpath --relative-to ${REPO_DIR} ${IMAGE_DIR})

VERSION=$(shell cat ${IMAGE_DIR}/version)
ifneq ($(IMAGE_VERSION),)
	VERSION=$(IMAGE_VERSION)
endif
DOCKERFILE=${IMAGE_DIR}/Dockerfile

export BUILDKIT_PROGRESS=plain
export DOCKER_BUILDKIT=1

.PHONY: image publish

image:
	docker build --build-arg IMAGE_DIR=${IMAGE_DIR_REL} --ssh default -t ${DOCKER_IMAGE}:${VERSION} -f ${DOCKERFILE} ${REPO_DIR}
	docker tag ${DOCKER_IMAGE}:$(VERSION) ${REGISTRY}/${DOCKER_IMAGE}:$(VERSION)
	docker tag ${DOCKER_IMAGE}:$(VERSION) ${DOCKER_IMAGE}:latest

publish:
	docker push ${REGISTRY}/${DOCKER_IMAGE}:$(VERSION)