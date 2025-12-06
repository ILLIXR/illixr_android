#pragma once

#include "illixr/data_format/frame.hpp"

#include <EGL/egl.h>
#include <GLES3/gl3.h>
#include <vector>

namespace ILLIXR {

struct stereo_texture {
    GLuint y_texture;   // Y plane texture
    GLuint uv_texture;  // UV plane texture
};

struct cached_nv12_frame {
    std::vector<uint8_t> data;
    int width;
    int height;
    time_point timestamp;

    [[nodiscard]] const uint8_t* get_y_plane() const {
        return data.data();
    }

    [[nodiscard]] const uint8_t* get_uv_plane() const {
        return data.data() + (width * height);
    }
};

class stereo_renderer {
public:
    stereo_renderer(int width, int height);
    ~stereo_renderer();
    void receive_dual_frame(const std::shared_ptr<const data_format::dual_frames>& frame);
    bool render_eye(int eye, const float* mvp_matrix);
    bool has_frame(int eye) const;
    bool has_both_frames() const;
    int get_width() const { return width_; }
    int get_height() const { return height_; }
private:
    void create_nv12_textures(stereo_texture& tex);
    void upload_nv12_frame(stereo_texture& tex, const cached_nv12_frame& frame);
    void render_with_shader(stereo_texture& tex, const float* mvp_matrix);
    void create_shader_program();
    void create_quad_geometry();
    void check_shader_compile(GLuint shader, const char* type);
    void check_program_link(GLuint program);

    int width_;
    int height_;

    // Cached frames from dual_frames (NV12 format)
    cached_nv12_frame left_frame_;
    cached_nv12_frame right_frame_;
    std::mutex frame_mutex_;

    // Stereo textures - one set per eye
    std::array<stereo_texture, 2> eye_textures_;  // [0] = left, [1] = right

    // Shader program for NV12 to RGB conversion
    GLuint shader_program_;
    GLint y_texture_loc_;
    GLint uv_texture_loc_;
    GLint mvp_loc_;

    // Quad geometry for rendering
    GLuint vao_;
    GLuint vbo_;

    // Frame tracking
    std::atomic<bool> left_frame_ready_;
    std::atomic<bool> right_frame_ready_;
};

} // ILLIXR
