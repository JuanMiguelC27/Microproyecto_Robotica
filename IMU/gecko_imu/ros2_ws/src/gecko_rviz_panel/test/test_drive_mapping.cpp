#include <gtest/gtest.h>

#include "gecko_rviz_panel/drive_mapping.hpp"

using gecko_rviz_panel::driveFromVector;

TEST(Drive, DeadZone)
{
  EXPECT_EQ(driveFromVector(0.0, 0.0), 0);
  EXPECT_EQ(driveFromVector(0.1, 0.1), 0);
}

TEST(Drive, Cardinals)
{
  EXPECT_EQ(driveFromVector(0.0, 1.0), 1);
  EXPECT_EQ(driveFromVector(0.0, -1.0), 2);
  EXPECT_EQ(driveFromVector(1.0, 0.0), 3);
  EXPECT_EQ(driveFromVector(-1.0, 0.0), 4);
}

TEST(Drive, DominantAxis)
{
  EXPECT_EQ(driveFromVector(0.7, 0.6), 3);
  EXPECT_EQ(driveFromVector(0.6, -0.7), 2);
}
