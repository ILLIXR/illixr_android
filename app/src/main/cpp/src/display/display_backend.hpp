#pragma once

#include "illixr/switchboard.hpp"
#include "illixr/vk/display_provider.hpp"

namespace ILLIXR::display {

class display_backend {
public:
    enum display_backend_type { GLFW, X11_DIRECT, HEADLESS, ANDROID_DISPLAY };

    virtual void         setup_display(const std::shared_ptr<switchboard> sb, VkInstance vk_instance,
                                       VkPhysicalDevice vk_physical_device) = 0;

#ifdef ILLIXR_ANDROID_BUILD
    virtual VkSurfaceKHR create_surface(ANativeWindow* window)              = 0;
#else
    virtual VkSurfaceKHR create_surface()                                   = 0;
#endif
    virtual void         cleanup()                                          = 0;

    virtual std::set<const char*> get_required_instance_extensions() = 0;
    virtual std::set<const char*> get_required_device_extensions()   = 0;

    virtual display_backend_type get_type() = 0;

    virtual ~display_backend() {}
protected:
    VkInstance vk_instance_;
};

} // namespace ILLIXR::display
