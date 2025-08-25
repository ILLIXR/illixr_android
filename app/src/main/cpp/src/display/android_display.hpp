#pragma once

#include "display_backend.hpp"

namespace ILLIXR::display {

class android_display : public display_backend {
public:
    android_display();

    void setup_display(const std::shared_ptr<switchboard> sb, VkInstance vk_instance,
                       VkPhysicalDevice vk_physical_device) override;

    VkSurfaceKHR create_surface() override;
    void cleanup() override;
    display_backend_type get_type() override;
    std::set<const char*> get_required_instance_extensions() override;
    std::set<const char*> get_required_device_extensions() override;

private:
    VkPhysicalDevice                                            vk_physical_device_;
};

}
