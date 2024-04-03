build:
ifdef  foxy
	docker build -f Dockerfile -t mclfoxy .
endif
ifdef  humble
	docker build -f HumbleDockerfile -t mclhumble .
endif

run:
ifdef  foxy
	docker run --rm -it --init   --gpus=all  --network host --pid host --ipc host -e "DISPLAY" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
 -v $(CURDIR)/ros_ws:/ros_ws mclfoxy
endif
ifdef  humble
	docker run --rm -it --init   --gpus=all --privileged --network host --pid host --ipc host -e "DISPLAY" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
 -v $(CURDIR)/ros_ws:/ros_ws mclhumble
endif
