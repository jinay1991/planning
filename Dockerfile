FROM ubuntu:18.04

RUN apt-get update && apt-get upgrade -y && apt-get autoremove -y

# Installation of general dependencies
RUN apt-get install -y build-essential gcc g++ lcov make cmake
RUN apt-get install -y openjdk-11-jdk openjdk-11-jre
RUN apt-get install -y libtool clang-format-6.0
RUN apt-get install -y git curl

# Installation of Bazel Package
RUN echo "deb [arch=amd64] https://storage.googleapis.com/bazel-apt stable jdk1.8" | tee /etc/apt/sources.list.d/bazel.list
RUN curl https://bazel.build/bazel-release.pub.gpg | apt-key add -
RUN apt-get update && apt-get install -y bazel

# Installation of uWebSockets (Udacity Simulation Connectivity Protocol)
RUN apt-get install -y libuv1-dev libssl-dev
RUN git clone https://github.com/uWebSockets/uWebSockets
RUN cd uWebSockets && git checkout e94b6e1
RUN mkdir -p uWebSockets/build
RUN cd uWebSockets/build && cmake ..
RUN cd uWebSockets/build && make -j4 && make install
RUN ln -s /usr/lib64/libuWS.so /usr/lib/libuWS.so
RUN rm -r uWebSockets

# Installation of Docker (used to compile image with scheduled job)
RUN apt-get install -y apt-transport-https ca-certificates software-properties-common
RUN curl -fsSL https://download.docker.com/linux/ubuntu/gpg | apt-key add -
RUN add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu bionic stable"
RUN apt-get update && apt-cache policy docker-ce
RUN apt-get install -y docker-ce docker-ce-cli
