FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

# Updates
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get autoremove -y

# Installation of general dependencies
RUN apt-get install -y \
    g++ gcc clang-format clang-tidy git git-lfs \
    wget curl gnupg \
    openjdk-11-jdk openjdk-11-jre lcov \
    vim gdb

# Installation of Bazel Package
RUN curl -fsSL https://bazel.build/bazel-release.pub.gpg | gpg --dearmor > bazel.gpg && \
    mv bazel.gpg /etc/apt/trusted.gpg.d/ && \
    echo "deb [arch=amd64] https://storage.googleapis.com/bazel-apt stable jdk1.8" | tee /etc/apt/sources.list.d/bazel.list && \
    apt-get update && apt-get install -y bazel && \
    echo "source /etc/bash_completion.d/bazel" >> ~/.bashrc

# Installation of Bazel Tools
RUN wget https://github.com/bazelbuild/buildtools/releases/download/4.0.1/buildifier && \
    chmod +x buildifier && \
    mv buildifier /usr/bin

# Installation of Utilities useful for connecting to Udacity simulator
RUN apt-get install -y \
    libssl-dev libuv1-dev

# cleanup
RUN apt-get clean && rm -rf /var/lib/apt/lists/* && \
    apt-get autoremove && apt-get autoclean
