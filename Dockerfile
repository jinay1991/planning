FROM ubuntu:18.04

RUN apt-get update && apt-get upgrade -y && apt-get autoremove -y

# Installation of general dependencies
RUN apt-get install -y build-essential gcc g++ lcov make cmake
RUN apt-get install -y openjdk-11-jdk openjdk-11-jre
RUN apt-get install -y libssl-dev libuv1-dev
RUN apt-get install -y libtool clang-format-6.0
RUN apt-get install -y git curl
RUN apt-get install -y wget

# Installation of python3
RUN apt-get install -y python3 python3-pip python3-pygments
RUN python3 -m pip install -U pip 
RUN python3 -m pip install -U future

# Installation of dependencies to Doxygen
RUN apt-get install -y doxygen graphviz plantuml

# Installation of static code analysis
RUN apt-get install -y cppcheck

# Installation of Bazel Package
RUN echo "deb [arch=amd64] https://storage.googleapis.com/bazel-apt stable jdk1.8" | tee /etc/apt/sources.list.d/bazel.list
RUN curl https://bazel.build/bazel-release.pub.gpg | apt-key add -
RUN apt-get update && apt-get install -y bazel

# Installation of Bazel Tools
RUN wget https://github.com/bazelbuild/buildtools/releases/download/0.29.0/buildifier
RUN chmod +x buildifier
RUN mv buildifier /usr/bin

# cleanup
RUN apt-get clean && rm -rf /var/lib/apt/lists/*
RUN apt-get autoremove && apt-get autoclean
