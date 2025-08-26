#pragma once

#ifdef ILLIXR_ANDROID_BUILD
#include "display/android_display.hpp"
#else
#include "display/glfw_extended.hpp"
#include "display/x11_direct.hpp"
#include "display/headless.hpp"
#endif

#include "illixr/phonebook.hpp"
#include "illixr/switchboard.hpp"

#include <thread>
#include <vulkan/vulkan.h>

using namespace ILLIXR;
class display_vk : public vulkan::display_provider {
public:
    explicit display_vk(const phonebook* const pb
#ifdef ILLIXR_ANDROID_BUILD
    , ANativeWindow* window
#endif
    )
            : switchboard_{pb->lookup_impl<switchboard>()}
            , clock_{pb->lookup_impl<relative_clock>()}
#ifdef ILLIXR_ANDROID_BUILD
            , window_{window}
#endif
        {
#ifdef ILLIXR_ANDROID_BUILD
        backend_type_ = display::display_backend::ANDROID_DISPLAY;
#else
        // ILLIXR_DISPLAY_MODE defaults to GLFW if not specified.
        const char* env_var = switchboard_->get_env_char("ILLIXR_DISPLAY_MODE");
        if (!strcmp(env_var, "glfw")) {
            spdlog::get("illixr")->info("[vulkan_display] Selected GLFW for display backend");
            backend_type_ = display::display_backend::GLFW;
        } else if (!strcmp(env_var, "headless")) {
            spdlog::get("illixr")->info("[vulkan_display] Selected headless for display backend");
            backend_type_ = display::display_backend::HEADLESS;
        } else if (!strcmp(env_var, "x11_direct")) {
            spdlog::get("illixr")->info("[vulkan_display] Selected X11 direct mode for display backend");
            backend_type_ = display::display_backend::X11_DIRECT;
            direct_mode_  = true;
        } else {
            throw std::runtime_error("Invalid display mode: " + std::string(env_var));
        }
#endif

    }

    ~display_vk() override {
        running_ = false;
        if (main_thread_.joinable()) {
            main_thread_.join();
        }
        cleanup();
    }

    void start(std::set<const char*> instance_extensions, std::set<const char*> device_extensions);

    /**
     * @brief This function sets up the GLFW and Vulkan environments. See display_provider::setup().
     */
    void setup(std::set<const char*> instance_extensions, std::set<const char*> device_extensions);

    /**
     * @brief This function recreates the Vulkan swapchain. See display_provider::recreate_swapchain().
     */
    void recreate_swapchain() override;

    /**
     * @brief This function polls GLFW events. See display_provider::poll_window_events().
     */
    void poll_window_events() override {
        should_poll_ = true;
    }

private:
    void create_vk_instance(const std::set<const char*>& instance_extensions);

    bool is_physical_device_suitable(VkPhysicalDevice const& physical_device);

    void select_physical_device();

    void create_logical_device(const std::set<const char*>& device_extensions);

    /**
     * @brief Sets up the Vulkan environment.
     *
     * This function initializes the Vulkan instance, selects the physical device, creates the Vulkan device,
     * gets the graphics and present queues_, creates the swapchain, and sets up the VMA allocator.
     *
     * @throws runtime_error If any of the Vulkan setup steps fail.
     */
    void create_swapchain();

    void destroy_swapchain();
    void cleanup();

#ifndef ILLIXR_ANDROID_BUILD
    void main_loop();
#endif

    std::vector<const char*> required_device_extensions_ = {VK_KHR_SYNCHRONIZATION_2_EXTENSION_NAME};

    std::thread       main_thread_;
    std::atomic<bool> ready_{false};
    std::atomic<bool> running_{true};

    display::display_backend::display_backend_type backend_type_;
    bool                                           direct_mode_{false};
    int                                            selected_gpu_{-1};

    std::shared_ptr<display::display_backend> backend_;

    const std::shared_ptr<switchboard> switchboard_;

    std::atomic<bool> should_poll_{true};

    std::shared_ptr<relative_clock> clock_;

#ifdef ILLIXR_ANDROID_BUILD
    ANativeWindow* window_;
#endif
};
