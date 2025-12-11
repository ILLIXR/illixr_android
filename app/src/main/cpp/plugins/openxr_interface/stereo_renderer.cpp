#include "stereo_renderer.hpp"

#include <EGL/egl.h>
#include <GLES3/gl3.h>
#include <GLES2/gl2ext.h>

#include <spdlog/spdlog.h>

using namespace ILLIXR;
using namespace ILLIXR::data_format;

constexpr float stereo_renderer::identity_matrix_[16];

// Identity matrix
static const float identity_matrix[16] = {
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f
};

stereo_renderer::stereo_renderer() {}

stereo_renderer::~stereo_renderer() {
    cleanup();
}

void stereo_renderer::set_crop_region(int original_width, int original_height,
                                      int padded_width, int padded_height) {
    original_width_ = original_width;
    original_height_ = original_height;
    padded_width_ = padded_width;
    padded_height_ = padded_height;

    // Calculate crop scale factors
    crop_scale_x_ = static_cast<float>(original_width) / static_cast<float>(padded_width);
    crop_scale_y_ = static_cast<float>(original_height) / static_cast<float>(padded_height);

    spdlog::get("illixr")->info("stereo_video_renderer: Crop {}x{} -> {}x{} (scale: {:.4f}, {:.4f})",
                                padded_width, padded_height,
                                original_width, original_height,
                                crop_scale_x_, crop_scale_y_);
}

bool stereo_renderer::initialize() {
    if (initialized_) {
        spdlog::get("illixr")->warn("stereo_renderer already initialized");
        return true;
    }

    EGLContext ctx;
    if ((ctx = eglGetCurrentContext()) == EGL_NO_CONTEXT) {
        spdlog::get("illixr")->error("stereo_renderer: No EGL context during initialization");
        return false;
    }
    spdlog::get("illixr")->info("stereo_video_renderer: GL context OK: {}", (void*)ctx);

    // Clear any prior errors
    while (glGetError() != GL_NO_ERROR) {}

    if (!compile_shaders()) {
        spdlog::get("illixr")->error("stereo_renderer: Failed to compile shaders");
        cleanup();
        return false;
    }
    spdlog::get("illixr")->info("stereo_video_renderer: Shaders compiled OK");

    while (glGetError() != GL_NO_ERROR) {}

    spdlog::get("illixr")->info("stereo_video_renderer: Shaders compiled OK");

    create_fullscreen_quad();

    GLenum err = glGetError();
    if (err != GL_NO_ERROR) {
        spdlog::get("illixr")->error("stereo_video_renderer: GL error after quad: 0x{:X}", err);
        return false;
    }

    create_nv12_textures();

    err = glGetError();
    if (err != GL_NO_ERROR) {
        spdlog::get("illixr")->error("stereo_renderer: GL error during init: 0x{:X}", err);
        cleanup();
        return false;
    }

    initialized_ = true;
    spdlog::get("illixr")->info("stereo_renderer: Initialized");
    return true;
}

bool stereo_renderer::compile_shaders() {
    auto compile_shader = [](GLenum type, const char* source) -> GLuint {
        GLuint shader = glCreateShader(type);
        if (shader == 0) {
            spdlog::get("illixr")->error("glCreateShader failed");
            return 0;
        }
        glShaderSource(shader, 1, &source, nullptr);
        glCompileShader(shader);

        GLint status = 0;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &status);
        if (status != GL_TRUE) {
            GLint len = 0;
            glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &len);
            if (len > 0) {
                std::string log(len, '\0');
                glGetShaderInfoLog(shader, len, nullptr, log.data());
                spdlog::get("illixr")->error("Shader compile error: {}", log);
            }
            glDeleteShader(shader);
            return 0;
        }
        return shader;
    };

    auto link_program = [](GLuint vs, GLuint fs) -> GLuint {
        GLuint program = glCreateProgram();
        if (program == 0) {
            spdlog::get("illixr")->error("glCreateProgram failed");
            return 0;
        }
        glAttachShader(program, vs);
        glAttachShader(program, fs);
        glLinkProgram(program);

        GLint status = 0;
        glGetProgramiv(program, GL_LINK_STATUS, &status);
        if (status != GL_TRUE) {
            GLint len = 0;
            glGetProgramiv(program, GL_INFO_LOG_LENGTH, &len);
            if (len > 0) {
                std::string log(len, '\0');
                glGetProgramInfoLog(program, len, nullptr, log.data());
                spdlog::get("illixr")->error("Program link error: {}", log);
            }
            glDeleteProgram(program);
            return 0;
        }

        glDetachShader(program, vs);
        glDetachShader(program, fs);
        return program;
    };

    // Compile vertex shader (shared by both fragment shaders)
    GLuint vs = compile_shader(GL_VERTEX_SHADER, shader_source::vertex_shader);
    if (vs == 0) {
        spdlog::get("illixr")->error("stereo_renderer: Vertex shader failed");
        return false;  // Vertex shader is required
    }

    // Compile external OES fragment shader (REQUIRED for SurfaceTexture)
    GLuint fs_ext = compile_shader(GL_FRAGMENT_SHADER, shader_source::external_fragment);
    if (fs_ext == 0) {
        spdlog::get("illixr")->error("stereo_renderer: External OES shader failed");
        glDeleteShader(vs);
        return false;  // This is required!
    }

    // Link external program
    external_program_ = link_program(vs, fs_ext);
    glDeleteShader(fs_ext);  // Can delete after linking

    if (external_program_ == 0) {
        spdlog::get("illixr")->error("stereo_renderer: External program link failed");
        glDeleteShader(vs);
        return false;
    }

    // Try to compile NV12 shader (OPTIONAL - fallback format)
    GLuint fs_nv12 = compile_shader(GL_FRAGMENT_SHADER, shader_source::nv12_fragment);
    if (fs_nv12 != 0) {
        nv12_program_ = link_program(vs, fs_nv12);
        glDeleteShader(fs_nv12);

        if (nv12_program_ == 0) {
            spdlog::get("illixr")->warn("stereo_renderer: NV12 program link failed (non-fatal)");
        }
    } else {
        spdlog::get("illixr")->warn("stereo_renderer: NV12 shader failed (non-fatal)");
    }

    glDeleteShader(vs);  // Can delete after all linking done

    // Clear any errors from optional NV12 shader
    while (glGetError() != GL_NO_ERROR) {}

    // Success as long as external_program_ is valid
    return external_program_ != 0;
}

void stereo_renderer::create_nv12_textures() {
    // Create Y and UV textures for both eyes
    for (int eye = 0; eye < 2; eye++) {
        // Y texture (luminance, full resolution)
        glGenTextures(1, &y_textures_[eye]);
        glBindTexture(GL_TEXTURE_2D, y_textures_[eye]);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

        // UV texture (chrominance, half resolution)
        glGenTextures(1, &uv_textures_[eye]);
        glBindTexture(GL_TEXTURE_2D, uv_textures_[eye]);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    }

    glBindTexture(GL_TEXTURE_2D, 0);
}

void stereo_renderer::create_fullscreen_quad() {
    // Position (x,y,z) + texcoord (u,v)
    static const float vertices[] = {
            -1.0f, -1.0f, 0.0f,  0.0f, 0.0f,
            1.0f, -1.0f, 0.0f,  1.0f, 0.0f,
            -1.0f,  1.0f, 0.0f,  0.0f, 1.0f,
            1.0f,  1.0f, 0.0f,  1.0f, 1.0f,
    };

    // Generate VAO first
    glGenVertexArrays(1, &vao_);
    if (vao_ == 0) {
        spdlog::get("illixr")->error("create_fullscreen_quad: glGenVertexArrays failed");
        return;
    }
    glBindVertexArray(vao_);

    // Generate and fill VBO
    glGenBuffers(1, &vbo_);
    if (vbo_ == 0) {
        spdlog::get("illixr")->error("create_fullscreen_quad: glGenBuffers failed");
        glBindVertexArray(0);
        return;
    }
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    // Check for errors after buffer upload
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) {
        spdlog::get("illixr")->error("create_fullscreen_quad: GL error after buffer: 0x{:X}", err);
    }

    // Position attribute (location = 0)
    // 3 floats, stride = 5 floats (20 bytes), offset = 0
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);

    // TexCoord attribute (location = 1)
    // 2 floats, stride = 5 floats (20 bytes), offset = 3 floats (12 bytes)
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));

    // Check for errors after attribute setup
    err = glGetError();
    if (err != GL_NO_ERROR) {
        spdlog::get("illixr")->error("create_fullscreen_quad: GL error after attribs: 0x{:X}", err);
    }

    // Unbind VAO (but keep VBO bound is fine, or unbind it too)
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    spdlog::get("illixr")->debug("create_fullscreen_quad: Created VAO={}, VBO={}", vao_, vbo_);

}

void stereo_renderer::receive_frame(const dual_frames& frame) {
    if (!initialized_) {
        spdlog::get("illixr")->warn("stereo_renderer: receive_frame called before initialize");
        return;
    }

    if (!frame.is_valid()) {
        spdlog::get("illixr")->warn("stereo_renderer: Received invalid frame");
        has_valid_frame_ = false;
        return;
    }

    current_format_ = frame.format;
    frame_width_ = frame.width;
    frame_height_ = frame.height;

    if (frame.format == frame_format::external_oes) {
        // Store texture handles (owned by decoder)
        external_textures_[0] = frame.left_eye.texture_id;
        external_textures_[1] = frame.right_eye.texture_id;

        // Copy transform matrices
        std::copy(frame.left_eye.texture_transform.begin(),
                  frame.left_eye.texture_transform.end(),
                  texture_transforms_[0].begin());
        std::copy(frame.right_eye.texture_transform.begin(),
                  frame.right_eye.texture_transform.end(),
                  texture_transforms_[1].begin());

    } else if (frame.format == frame_format::nv12) {
        // Upload NV12 data to our textures
        upload_nv12_data(0, frame.left_eye, frame.width, frame.height);
        upload_nv12_data(1, frame.right_eye, frame.width, frame.height);
    }

    has_valid_frame_ = true;
}

void stereo_renderer::upload_nv12_data(int eye, const eye_frame& frame,
                                             int width, int height) {
    if (frame.data.empty())
        return;

    // Expected NV12 size: width * height * 1.5
    //size_t expected_size = static_cast<size_t>(width * height * 3 / 2);
    //if (frame.data.size() < expected_size) {
    //    spdlog::get("illixr")->warn("stereo_renderer: NV12 data too small ({} < {})",
    //                 frame.data.size(), expected_size);
    //    return;
    //}

    // Upload Y plane
    glBindTexture(GL_TEXTURE_2D, y_textures_[eye]);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R8, width, height, 0,
                 GL_RED, GL_UNSIGNED_BYTE, frame.get_y_plane());

    // Upload UV plane (interleaved, half resolution)
    glBindTexture(GL_TEXTURE_2D, uv_textures_[eye]);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RG8, width / 2, height / 2, 0,
                 GL_RG, GL_UNSIGNED_BYTE, frame.get_uv_plane(width, height));

    glBindTexture(GL_TEXTURE_2D, 0);
}

bool stereo_renderer::render_eye(int eye, const float* mvp) {
    // Use stored frame dimensions for viewport
    int viewport_width = frame_width_ > 0 ? frame_width_ : HEADSET_WIDTH;
    int viewport_height = frame_height_ > 0 ? frame_height_ : HEADSET_HEIGHT;
    return render_eye(eye, mvp, viewport_width, viewport_height);
}

bool stereo_renderer::render_eye(int eye, const float* mvp,
                                       int viewport_width, int viewport_height) {
    if (!initialized_) {
        spdlog::get("illixr")->error("stereo_renderer: Not initialized");
        return false;
    }

    glViewport(0, 0, viewport_width, viewport_height);
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDisable(GL_DEPTH_TEST);

    if (!has_valid_frame_) {
        return true;
    }

    if (eye < 0 || eye > 1) {
        spdlog::get("illixr")->error("stereo_renderer: Invalid eye index {}", eye);
        return false;
    }

    const float* mvp_to_use = mvp ? mvp : identity_matrix;
    GLuint program = 0;

    if (current_format_ == frame_format::external_oes && external_program_) {
        program = external_program_;
        // Render using external OES texture
        glUseProgram(program);

        // Set MVP
        GLint mvp_loc = glGetUniformLocation(program, "u_mvp");
        if (mvp_loc >= 0)
            glUniformMatrix4fv(mvp_loc, 1, GL_FALSE, mvp_to_use);

        // Set texture transform
        GLint transform_loc = glGetUniformLocation(program, "u_tex_transform");
        if (transform_loc >= 0)
            glUniformMatrix4fv(transform_loc, 1, GL_FALSE, texture_transforms_[eye].data());

        // Set crop scale
        GLint crop_loc = glGetUniformLocation(program, "u_crop_scale");
        if (crop_loc >= 0)
            glUniform2f(crop_loc, crop_scale_x_, crop_scale_y_);

        // Bind texture
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_EXTERNAL_OES, external_textures_[eye]);
        GLint tex_loc = glGetUniformLocation(program, "u_texture");
        if (tex_loc >= 0)
            glUniform1i(tex_loc, 0);

    } else if (nv12_program_) {
        // Render using NV12 textures
        program = nv12_program_;
        glUseProgram(program);

        GLint mvp_loc = glGetUniformLocation(program, "u_mvp");
        if (mvp_loc >= 0)
            glUniformMatrix4fv(mvp_loc, 1, GL_FALSE, mvp_to_use);

        GLint transform_loc = glGetUniformLocation(program, "u_tex_transform");
        if (transform_loc >= 0)
            glUniformMatrix4fv(transform_loc, 1, GL_FALSE, identity_matrix_);

        GLint crop_loc = glGetUniformLocation(program, "u_crop_scale");
        if (crop_loc >= 0)
            glUniform2f(crop_loc, crop_scale_x_, crop_scale_y_);

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, y_textures_[eye]);
        GLint y_loc = glGetUniformLocation(program, "u_y_texture");
        if (y_loc >= 0)
            glUniform1i(y_loc, 0);

        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, uv_textures_[eye]);
        GLint uv_loc = glGetUniformLocation(program, "u_uv_texture");
        if (uv_loc >= 0)
            glUniform1i(uv_loc, 1);

    } else {
        spdlog::get("illixr")->error("stereo_renderer: No shader available for format");
        return false;
    }

    // Draw fullscreen quad
    glBindVertexArray(vao_);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    glBindVertexArray(0);

    // Cleanup
    glBindTexture(GL_TEXTURE_2D, 0);
    glBindTexture(GL_TEXTURE_EXTERNAL_OES, 0);
    glUseProgram(0);

    return true;
}

void stereo_renderer::cleanup() {
    if (vao_) {
        glDeleteVertexArrays(1, &vao_);
        vao_ = 0;
    }
    if (vbo_) {
        glDeleteBuffers(1, &vbo_);
        vbo_ = 0;
    }

    for (int i = 0; i < 2; i++) {
        if (y_textures_[i]) {
            glDeleteTextures(1, &y_textures_[i]);
            y_textures_[i] = 0;
        }
        if (uv_textures_[i]) {
            glDeleteTextures(1, &uv_textures_[i]);
            uv_textures_[i] = 0;
        }
    }

    if (external_program_) {
        glDeleteProgram(external_program_);
        external_program_ = 0;
    }
    if (nv12_program_) {
        glDeleteProgram(nv12_program_);
        nv12_program_ = 0;
    }

    initialized_ = false;
    has_valid_frame_ = false;
}
