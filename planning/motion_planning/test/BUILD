load("@rules_cc//cc:defs.bzl", "cc_test")

cc_test(
    name = "unit_tests",
    srcs = [
        "data_source_tests.cpp",
        "lane_evaluator_tests.cpp",
        "maneuver_generator_tests.cpp",
        "maneuver_tests.cpp",
        "motion_planning_tests.cpp",
        "trajectory_evaluator_tests.cpp",
        "trajectory_optimizer_tests.cpp",
        "trajectory_planner_tests.cpp",
        "trajectory_prioritizer_tests.cpp",
        "trajectory_selector_tests.cpp",
        "velocity_planner_tests.cpp",
    ],
    tags = ["unit"],
    deps = [
        "//planning/motion_planning",
        "//planning/motion_planning/test/support",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "node_tests",
    srcs = ["motion_planning_node_tests.cpp"],
    tags = ["component"],
    deps = [
        "//planning/motion_planning/node",
        "//planning/motion_planning/test/support",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "fuzzer_tests",
    srcs = ["motion_planning_fuzzer_tests.cpp"],
    tags = [
        "fuzzer",
        "manual",
    ],
    deps = [
        "//planning/motion_planning",
    ],
)

cc_test(
    name = "benchmark_tests",
    srcs = [
        "motion_planning_benchmark_tests.cpp",
    ],
    copts = [
        "-Wall",
        "-Werror",
    ],
    tags = [
        "benchmark",
        "manual",
    ],
    deps = [
        "//planning/motion_planning/node",
        "//planning/motion_planning/test/support",
        "@benchmark//:benchmark_main",
    ],
)
