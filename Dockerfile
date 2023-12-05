FROM ubuntu:latest

RUN apt-get update -y && \
    apt-get install -y g++ cmake git gdb && \
    apt-get install -y libeigen3-dev

RUN git clone https://github.com/osqp/osqp
RUN cd osqp && mkdir build
RUN cd osqp/build && cmake -G "Unix Makefiles" ..
RUN cd osqp/build && cmake --build . --target install

RUN apt-get install -y libglfw3 libglfw3-dev xorg-dev libx11-dev

RUN git clone https://github.com/google-deepmind/mujoco
RUN cd mujoco && mkdir build
RUN cd mujoco/build && cmake ..
RUN cd mujoco/build && cmake --build . --target install

# ADD https://api.github.com/repos/JTylerBoylan/orlqp/git/refs/heads/main version.json
# RUN git clone https://github.com/JTylerBoylan/orlqp
# RUN cd orlqp && mkdir build
# RUN cd orlqp/build && cmake ..
# RUN cd orlqp/build && cmake --build . --target install