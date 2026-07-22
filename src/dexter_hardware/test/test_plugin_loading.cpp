#include <memory>

#include <gtest/gtest.h>

#include "hardware_interface/system_interface.hpp"
#include "pluginlib/class_loader.hpp"

TEST(DexterSystemPlugin, IsDiscoverableAndConstructibleWithoutOpeningCan)
{
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
    "hardware_interface", "hardware_interface::SystemInterface");
  auto instance = loader.createUniqueInstance("dexter_hardware/DexterSystem");
  EXPECT_NE(instance, nullptr);
}
