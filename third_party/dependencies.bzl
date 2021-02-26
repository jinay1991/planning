load("@planning//third_party/benchmark:benchmark.bzl", "benchmark")
load("@planning//third_party/eigen:eigen.bzl", "eigen")
load("@planning//third_party/glog:glog.bzl", "glog")
load("@planning//third_party/googletest:googletest.bzl", "googletest")
load("@planning//third_party/llvm_toolchain:llvm_toolchain.bzl", "llvm_toolchain")
load("@planning//third_party/nholthaus:nholthaus.bzl", "nholthaus")
load("@planning//third_party/nlohmann:nlohmann.bzl", "nlohmann")
load("@planning//third_party/openssl:openssl.bzl", "openssl")
load("@planning//third_party/spline:spline.bzl", "spline")
load("@planning//third_party/sysroot:sysroot.bzl", "sysroot")
load("@planning//third_party/uv:uv.bzl", "uv")
load("@planning//third_party/uwebsocket:uwebsocket.bzl", "uwebsocket")
load("@planning//third_party/zlib:zlib.bzl", "zlib")

def planning_dependencies():
    """ Load 3rd party dependencies """
    benchmark()
    eigen()
    glog()
    googletest()
    llvm_toolchain()
    nholthaus()
    nlohmann()
    openssl()
    spline()
    sysroot()
    uv()
    uwebsocket()
    zlib()
