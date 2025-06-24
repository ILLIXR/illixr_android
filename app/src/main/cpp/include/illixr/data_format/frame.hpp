#pragma once

#ifdef Success
#undef Success // For 'Success' conflict
#endif

#include "illixr/data_format/pose.hpp"
#include "illixr/switchboard.hpp"

#include <GLES/gl.h>

namespace ILLIXR::data_format {
    // Using arrays as a swapchain
    // Array of left eyes, array of right eyes
    // This more closely matches the format used by Monado
    struct rendered_frame : public switchboard::event {
        //    std::array<GLuint, 2> texture_handles; // Does not change between swaps in swapchain
        std::array<GLuint, 2>    swapchain_indices; // Index of image rendered for left and right swapchain
        std::array<GLuint, 2> swap_indices;    // Which element of the swapchain
        fast_pose_type        render_pose;     // The pose used when rendering this frame.
        time_point            sample_time;
        time_point            render_time;

        rendered_frame() { }

        rendered_frame(std::array<GLuint, 2>&& swapchain_indices_, std::array<GLuint, 2>&& swap_indices_,
                       fast_pose_type render_pose_, time_point sample_time_, time_point render_time_)
                : swapchain_indices{std::move(swapchain_indices_)}
                , swap_indices{std::move(swap_indices_)}
                , render_pose(render_pose_)
                , sample_time(sample_time_)
                , render_time(render_time_) { }
    };

}