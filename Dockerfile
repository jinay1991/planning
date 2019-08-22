FROM ubuntu:18.04

RUN apt-get update && apt-get upgrade -y && apt-get autoremove -y

# Installation of general dependencies
RUN apt-get install -y build-essential gcc g++ make cmake unzip zip zlib1g-dev pkg-config
RUN apt-get install -y openjdk-11-jdk openjdk-11-jre
RUN apt-get install -y python python-setuptools python-wheel python-pip
RUN apt-get install -y libgtest-dev lcov libtool clang-format-6.0
RUN apt-get install -y git curl

# Installation of Python Package dependencies
RUN pip install -U pip
RUN pip install -U numpy

# Installation of Bazel Package
RUN echo "deb [arch=amd64] https://storage.googleapis.com/bazel-apt stable jdk1.8" | tee /etc/apt/sources.list.d/bazel.list
RUN curl https://bazel.build/bazel-release.pub.gpg | apt-key add -
RUN apt-get update && apt-get install -y bazel