///
/// @file argument_parser_test.cpp
/// @brief Contains unit tests for Argument Parser APIs
/// @copyright Copyright (c) 2020. All Rights Reserved.
///
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <string>

#include "argument_parser/argument_parser.h"

namespace planning
{
namespace
{
TEST(ArgumentParserTest, DefaultConstructor)
{
    auto unit = ArgumentParser();
    auto actual = unit.GetParsedArgs();

    EXPECT_EQ(actual.map_name, "data/highway_map.csv");
    EXPECT_FALSE(actual.verbose);
}
TEST(ArgumentParserTest, WhenHelpArgument)
{
    char* argv[] = {(char*)"dummy", (char*)"-h"};
    int argc = sizeof(argv) / sizeof(char*);
    EXPECT_EXIT(ArgumentParser(argc, argv), ::testing::ExitedWithCode(1), "");
}

TEST(ArgumentParserTest, ParameterizedConstructor)
{
    char* argv[] = {(char*)"dummy", (char*)"--map_data", (char*)"path/to/highway_map.csv", (char*)"-v", (char*)"1"};
    int argc = sizeof(argv) / sizeof(char*);
    auto unit = ArgumentParser(argc, argv);
    auto actual = unit.GetParsedArgs();

    EXPECT_EQ(actual.map_name, "path/to/highway_map.csv");
    EXPECT_TRUE(actual.verbose);
}
}  // namespace
}  // namespace planning