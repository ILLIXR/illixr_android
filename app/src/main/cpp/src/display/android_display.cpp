#include "android_display.hpp"

using namespace ILLIXR::display;

android_display::android_display() {}

void android_display::setup_display(const std::shared_ptr<switchboard> sb, VkInstance vk_instance,
                                    VkPhysicalDevice vk_physical_device) {
    this->vk_instance_ = vk_instance;
    this->vk_physical_device_ = vk_physical_device;
}

VkSurfaceKHR android_display::create_surface() {

    return VK_NULL_HANDLE;
}

void android_display::cleanup() { }

std::set<const char*> android_display::get_required_instance_extensions() {
    std::set<const char*> extensions{VK_KHR_SURFACE_EXTENSION_NAME};
    extensions.insert(VK_KHR_ANDROID_SURFACE_EXTENSION_NAME);

    /*
     	bool has_debug_utils = false;
	for (const auto &ext : available_instance_extensions)
	{
		if (strncmp(ext.extensionName, VK_EXT_DEBUG_UTILS_EXTENSION_NAME, strlen(VK_EXT_DEBUG_UTILS_EXTENSION_NAME)) == 0)
		{
			has_debug_utils = true;
			break;
		}
	}
	if (has_debug_utils)
	{
		required_instance_extensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
	}
	else
	{
		LOGW("{} is not available; disabling debug utils messenger", VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
	}

     */

    return extensions;
}

std::set<const char*> android_display::get_required_device_extensions() {
    return {};
}

display_backend::display_backend_type android_display::get_type() {
    return ANDROID_DISPLAY;
}