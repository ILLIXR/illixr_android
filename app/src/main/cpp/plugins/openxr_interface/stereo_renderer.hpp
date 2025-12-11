#pragma once

#include "illixr/data_format/frame.hpp"
#include "illixr/quest3_params.hpp"

namespace ILLIXR {

// Stereo video renderer for Quest 3.
// Accepts dual_frames from the switchboard and renders to OpenXR swapchains.
// 
// Supports two frame formats:
// 1. external_oes: Zero-copy path using GL_TEXTURE_EXTERNAL_OES from SurfaceTexture
// 2. nv12: CPU-provided NV12 data, uploaded to textures each frame
// 
// Usage:
// 1. Call initialize() once with GL context current
// 2. Each frame, call receive_frame() with the dual_frames from switchboard
// 3. Call render_eye() for each eye with the appropriate MVP matrix
class stereo_renderer {
public:
    stereo_renderer();
    ~stereo_renderer();

    // Non-copyable
    stereo_renderer(const stereo_renderer&) = delete;
    stereo_renderer& operator=(const stereo_renderer&) = delete;

    // Initialize the renderer.
    // Must be called with GL context current.
    bool initialize();

    // Receive a frame from the switchboard.
    // Call this before render_eye() each frame.
    // 
    // For external_oes format: Just stores the texture handles
    // For nv12 format: Uploads pixel data to internal textures
    void receive_frame(const data_format::dual_frames& frame);

    // Render one eye to the currently bound framebuffer.
    // Uses the viewport that was already set via glViewport().
    bool render_eye(int eye, const float* mvp);

    // Render one eye to the currently bound framebuffer.
    // Call glBindFramebuffer() and glViewport() before this.
    bool render_eye(int eye, const float* mvp, int viewport_width, int viewport_height);

    // Check if renderer is initialized
    [[nodiscard]] bool is_initialized() const { return initialized_; }

    // Check if we have a valid frame to render
    [[nodiscard]] bool has_frame() const { return has_valid_frame_; }

    // Get the current frame's format
    [[nodiscard]] data_format::frame_format get_current_format() const { return current_format_; }

    // Release all GL resources
    void cleanup();

    // Set the crop dimensions (original size before padding)
    // Call this before initialize() or after receiving frame metadata
    void set_crop_region(int original_width, int original_height, int padded_width, int padded_height);

private:
    bool compile_shaders();
    void create_nv12_textures();
    void create_fullscreen_quad();
    void upload_nv12_data(int eye, const data_format::eye_frame& frame, int width, int height);

    bool initialized_{false};
    bool has_valid_frame_{false};

    // Crop/padding dimensions
    int original_width_{0};
    int original_height_{0};
    int padded_width_{0};
    int padded_height_{0};
    float crop_scale_x_{1.0f};
    float crop_scale_y_{1.0f};

    // Current frame data
    data_format::frame_format current_format_{data_format::frame_format::nv12};
    int frame_width_{0};
    int frame_height_{0};

    // External OES texture handles (from decoder, not owned)
    std::array<GLuint, 2> external_textures_{0, 0};
    std::array<std::array<float, 16>, 2> texture_transforms_;

    // NV12 textures (owned, for CPU-provided data)
    std::array<GLuint, 2> y_textures_{0, 0};   // Y plane (GL_R8)
    std::array<GLuint, 2> uv_textures_{0, 0};  // UV plane (GL_RG8)

    // Shader programs
    GLuint external_program_{0};  // For external_oes format
    GLuint nv12_program_{0};      // For nv12 format

    // Geometry
    GLuint vao_{0};
    GLuint vbo_{0};

    static constexpr float identity_matrix_[16] = {
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1
    };
};

// Shader source code
namespace shader_source {

// Vertex shader with texture coordinate adjustment for cropping
static const char* vertex_shader =
        "#version 300 es\n"
        "precision highp float;\n"
        "\n"
        "layout(location = 0) in vec3 a_position;\n"
        "layout(location = 1) in vec2 a_texcoord;\n"
        "\n"
        "out vec2 v_texcoord;\n"
        "\n"
        "uniform mat4 u_mvp;\n"
        "uniform mat4 u_tex_transform;\n"
        "uniform vec2 u_crop_scale;\n"
        "\n"
        "void main() {\n"
        "    gl_Position = u_mvp * vec4(a_position, 1.0);\n"
        "    vec4 transformed = u_tex_transform * vec4(a_texcoord, 0.0, 1.0);\n"
        "    v_texcoord = transformed.xy * u_crop_scale;\n"
        "}\n";

// Fragment shader for external OES textures (SurfaceTexture output)
static const char* external_fragment =
        "#version 300 es\n"
        "#extension GL_OES_EGL_image_external_essl3 : require\n"
        "precision highp float;\n"
        "\n"
        "uniform samplerExternalOES u_texture;\n"
        "\n"
        "in vec2 v_texcoord;\n"
        "out vec4 frag_color;\n"
        "\n"
        "void main() {\n"
        "    frag_color = texture(u_texture, v_texcoord);\n"
        "}\n";

// Fragment shader for NV12 textures with YUV to RGB conversion (BT.709)
static const char* nv12_fragment =
        "#version 300 es\n"
        "precision highp float;\n"
        "\n"
        "uniform sampler2D u_y_texture;\n"
        "uniform sampler2D u_uv_texture;\n"
        "\n"
        "in vec2 v_texcoord;\n"
        "out vec4 frag_color;\n"
        "\n"
        "void main() {\n"
        "    float y = texture(u_y_texture, v_texcoord).r;\n"
        "    vec2 uv = texture(u_uv_texture, v_texcoord).rg;\n"
        "    \n"
        "    y = 1.1643 * (y - 0.0625);\n"
        "    float u = uv.r - 0.5;\n"
        "    float v = uv.g - 0.5;\n"
        "    \n"
        "    float r = y + 1.7927 * v;\n"
        "    float g = y - 0.2132 * u - 0.5329 * v;\n"
        "    float b = y + 2.1124 * u;\n"
        "    \n"
        "    frag_color = vec4(clamp(r, 0.0, 1.0),\n"
        "                      clamp(g, 0.0, 1.0),\n"
        "                      clamp(b, 0.0, 1.0),\n"
        "                      1.0);\n"
        "}\n";

} // namespace shader_source

} // namespace ILLIXR
