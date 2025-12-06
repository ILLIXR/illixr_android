#include "stereo_renderer.hpp"

#include <spdlog/spdlog.h>

using namespace ILLIXR;
using namespace ILLIXR::data_format;

// Quest 3 optimized NV12 shaders (OpenGL ES 3.0)
const char* nv12_vertex_shader = R"(
#version 300 es
layout(location = 0) in vec3 position;
layout(location = 1) in vec2 texcoord;

out vec2 frag_tex_coord;

uniform mat4 mvp;

void main() {
    gl_Position = mvp * vec4(position, 1.0);
    frag_tex_coord = texcoord;
}
)";

const char* nv12_fragment_shader = R"(
#version 300 es
precision highp float;

uniform sampler2D y_texture;
uniform sampler2D uv_texture;

in vec2 frag_tex_coord;
out vec4 frag_color;

void main() {
    // Sample Y and UV planes
    float y = texture(y_texture, frag_tex_coord).r;
    vec2 uv = texture(uv_texture, frag_tex_coord).rg;

    // Convert from [0,1] range with BT.709 coefficients (better for HD content)
    y = 1.1643 * (y - 0.0625);
    float u = uv.r - 0.5;
    float v = uv.g - 0.5;

    // YUV to RGB conversion (BT.709 for HD video)
    float r = y + 1.7927 * v;
    float g = y - 0.2132 * u - 0.5329 * v;
    float b = y + 2.1124 * u;

    frag_color = vec4(clamp(r, 0.0, 1.0),
                      clamp(g, 0.0, 1.0),
                      clamp(b, 0.0, 1.0),
                      1.0);
}
)";


stereo_renderer::stereo_renderer(int w, int h)
        : width_{w}
        , height_{h}
        , left_frame_ready_(false)
        , right_frame_ready_(false) {
    // Create textures for both eyes
    for (int eye = 0; eye < 2; eye++) {
        create_nv12_textures(eye_textures_[eye]);
    }

    // Create shader program
    create_shader_program();

    // Create fullscreen quad
    create_quad_geometry();

    spdlog::get("illixr")->info("Quest 3 stereo video renderer initialized ({}x{} per eye)",
                                w, h);
}

stereo_renderer::~stereo_renderer() {
    // Cleanup textures
    for (auto& tex : eye_textures_) {
        glDeleteTextures(1, &tex.y_texture);
        glDeleteTextures(1, &tex.uv_texture);
    }

    glDeleteProgram(shader_program_);
    glDeleteVertexArrays(1, &vao_);
    glDeleteBuffers(1, &vbo_);
}

void stereo_renderer::receive_dual_frame(const std::shared_ptr<const dual_frames>& frame) {
    std::lock_guard<std::mutex> lock(frame_mutex_);

    // Validate frame dimensions
    if (frame->width != width_ || frame->height != height_) {
        spdlog::get("illixr")->warn("Frame dimensions mismatch: expected {}x{}, got {}x{}",
                                    width_, height_, frame->width, frame->height);
    }

    // Expected NV12 size: width * height * 3/2
    size_t expected_size = frame->width * frame->height * 3 / 2;

    // Cache left eye frame
    if (!frame->left_eye.empty()) {
        if (frame->left_eye.size() == expected_size) {
            left_frame_.data = frame->left_eye;
            left_frame_.width = frame->width;
            left_frame_.height = frame->height;
            left_frame_.timestamp = frame->render_time;
            left_frame_ready_ = true;
        } else {
            spdlog::get("illixr")->error("Left eye NV12 size mismatch: expected {}, got {}",
                                         expected_size, frame->left_eye.size());
        }
    }

    // Cache right eye frame
    if (!frame->right_eye.empty()) {
        if (frame->right_eye.size() == expected_size) {
            right_frame_.data = frame->right_eye;
            right_frame_.width = frame->width;
            right_frame_.height = frame->height;
            right_frame_.timestamp = frame->render_time;
            right_frame_ready_ = true;
        } else {
            spdlog::get("illixr")->error("Right eye NV12 size mismatch: expected {}, got {}",
                                         expected_size, frame->right_eye.size());
        }
    }
}

// Render frame for a specific eye
// eye: 0 = left, 1 = right
// mvp: Model-View-Projection matrix from OpenXR
bool stereo_renderer::render_eye(int eye, const float* mvp_matrix) {
    if (eye < 0 || eye > 1) {
        spdlog::get("illixr")->error("Invalid eye index: {}", eye);
        return false;
    }

    // Check if frame is ready for this eye
    if (eye == 0 && !left_frame_ready_) {
        return false;
    }
    if (eye == 1 && !right_frame_ready_) {
        return false;
    }

    // Get frame for this eye
    cached_nv12_frame frame;
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        frame = (eye == 0) ? left_frame_ : right_frame_;
    }

    // Upload NV12 data to textures
    upload_nv12_frame(eye_textures_[eye], frame);

    // Render with shader
    render_with_shader(eye_textures_[eye], mvp_matrix);

    return true;
}

// Check if frames are ready
bool stereo_renderer::has_frame(int eye) const {
    return (eye == 0) ? left_frame_ready_.load() : right_frame_ready_.load();
}

bool stereo_renderer::has_both_frames() const {
    return left_frame_ready_ && right_frame_ready_;
}

void stereo_renderer::create_nv12_textures(stereo_texture& tex) {
    // Create Y plane texture (full resolution, single channel)
    glGenTextures(1, &tex.y_texture);
    glBindTexture(GL_TEXTURE_2D, tex.y_texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    // Create UV plane texture (half resolution, two channels)
    glGenTextures(1, &tex.uv_texture);
    glBindTexture(GL_TEXTURE_2D, tex.uv_texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
}

void stereo_renderer::upload_nv12_frame(stereo_texture& tex, const cached_nv12_frame& frame) {
    // Upload Y plane (luminance) - GL_R8 for Quest 3 OpenGL ES 3.0
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, tex.y_texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R8,
                 frame.width, frame.height, 0,
                 GL_RED, GL_UNSIGNED_BYTE, frame.get_y_plane());

    // Upload UV plane (chrominance, half resolution, interleaved)
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, tex.uv_texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RG8,
                 frame.width / 2, frame.height / 2, 0,
                 GL_RG, GL_UNSIGNED_BYTE, frame.get_uv_plane());
}

void stereo_renderer::render_with_shader(stereo_texture& tex, const float* mvp_matrix) {
    // Use shader program
    glUseProgram(shader_program_);

    // Bind textures
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, tex.y_texture);
    glUniform1i(y_texture_loc_, 0);

    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, tex.uv_texture);
    glUniform1i(uv_texture_loc_, 1);

    // Set MVP matrix
    glUniformMatrix4fv(mvp_loc_, 1, GL_FALSE, mvp_matrix);

    // Draw quad
    glBindVertexArray(vao_);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    glBindVertexArray(0);
}

void stereo_renderer::create_shader_program() {
    // Compile vertex shader
    GLuint vertex_shader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertex_shader, 1, &nv12_vertex_shader, nullptr);
    glCompileShader(vertex_shader);
    check_shader_compile(vertex_shader, "vertex");

    // Compile fragment shader
    GLuint fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragment_shader, 1, &nv12_fragment_shader, nullptr);
    glCompileShader(fragment_shader);
    check_shader_compile(fragment_shader, "fragment");

    // Link program
    shader_program_ = glCreateProgram();
    glAttachShader(shader_program_, vertex_shader);
    glAttachShader(shader_program_, fragment_shader);
    glLinkProgram(shader_program_);
    check_program_link(shader_program_);

    // Get uniform locations
    y_texture_loc_ = glGetUniformLocation(shader_program_, "y_texture");
    uv_texture_loc_ = glGetUniformLocation(shader_program_, "uv_texture");
    mvp_loc_ = glGetUniformLocation(shader_program_, "mvp");

    // Cleanup
    glDeleteShader(vertex_shader);
    glDeleteShader(fragment_shader);

    spdlog::get("illixr")->info("NV12 shader program created successfully");
}

void stereo_renderer::create_quad_geometry() {
    // Fullscreen quad vertices (position + texcoord)
    float quad_vertices[] = {
            // Position (x, y, z)   // TexCoord (u, v)
            -1.0f, -1.0f, 0.0f,     0.0f, 1.0f,  // Bottom-left
            1.0f, -1.0f, 0.0f,     1.0f, 1.0f,  // Bottom-right
            -1.0f,  1.0f, 0.0f,     0.0f, 0.0f,  // Top-left
            1.0f,  1.0f, 0.0f,     1.0f, 0.0f   // Top-right
    };

    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);

    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quad_vertices), quad_vertices, GL_STATIC_DRAW);

    // Position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // TexCoord attribute
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float),
                          (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);
}

void stereo_renderer::check_shader_compile(GLuint shader, const char* type) {
    GLint success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char info_log[512];
        glGetShaderInfoLog(shader, 512, nullptr, info_log);
        spdlog::get("illixr")->error("Shader compilation failed ({}): {}", type, info_log);
    }
}

void stereo_renderer::check_program_link(GLuint program) {
    GLint success;
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success) {
        char info_log[512];
        glGetProgramInfoLog(program, 512, nullptr, info_log);
        spdlog::get("illixr")->error("Program linking failed: {}", info_log);
    }
}
