#pragma once

#include "error_util.hpp"
#include "EGL/egl.h"

#include <cassert>
#include <cstring>
#include <spdlog/spdlog.h>
#include <string>
#include <vector>

using namespace ILLIXR;

static constexpr std::size_t GL_MAX_LOG_LENGTH = 4096U;

static GLuint init_and_link(const char* vertex_shader, const char* fragment_shader) {
    // GL handles for intermediary objects.
    GLint result;
    GLuint fragment_shader_handle, vertex_shader_handle, shader_program;
    vertex_shader_handle = glCreateShader(GL_VERTEX_SHADER);
    auto vert_shader_len = static_cast<GLint>(strlen(vertex_shader));
    glShaderSource(vertex_shader_handle, 1, &vertex_shader, &vert_shader_len);
    glCompileShader(vertex_shader_handle);
    glGetShaderiv(vertex_shader_handle, GL_COMPILE_STATUS, &result);
    if (result == GL_FALSE) {
        GLsizei             length = 0;
        std::vector<GLchar> gl_buf_log;
        gl_buf_log.resize(GL_MAX_LOG_LENGTH);

        glGetShaderInfoLog(vertex_shader_handle, GL_MAX_LOG_LENGTH * sizeof(GLchar), &length, gl_buf_log.data());
        const std::string msg{gl_buf_log.begin(), gl_buf_log.end()};
        //assert(length == static_cast<GLsizei>(msg.size()) && "Length of log should match GLchar vector contents");
        ILLIXR::abort("[shader_util] Failed to get vertex_shader_handle: " + msg);
    }

    GLint frag_result      = GL_FALSE;
    fragment_shader_handle = glCreateShader(GL_FRAGMENT_SHADER);
    auto frag_shader_len   = static_cast<GLint>(strlen(fragment_shader));
    glShaderSource(fragment_shader_handle, 1, &fragment_shader, &frag_shader_len);
    glCompileShader(fragment_shader_handle);
    glGetShaderiv(fragment_shader_handle, GL_COMPILE_STATUS, &frag_result);
    if (frag_result == GL_FALSE) {
        GLsizei             length = 0;
        std::vector<GLchar> gl_buf_log;
        gl_buf_log.resize(GL_MAX_LOG_LENGTH);

        glGetShaderInfoLog(fragment_shader_handle, GL_MAX_LOG_LENGTH * sizeof(GLchar), &length, gl_buf_log.data());
        const std::string msg{gl_buf_log.begin(), gl_buf_log.end()};
        assert(length == static_cast<GLsizei>(msg.size()) && "Length of log should match GLchar vector contents");
        ILLIXR::abort("[shader_util] Failed to get fragment_shader_handle: " + msg);
    }

    // Create program and link shaders
    shader_program = glCreateProgram();
    glAttachShader(shader_program, vertex_shader_handle);
    glAttachShader(shader_program, fragment_shader_handle);
    const GLenum gl_err_attach = glGetError();
    if (gl_err_attach != GL_NO_ERROR) {
        ILLIXR::abort("[shader_util] AttachShader or createProgram failed");
    }

    ///////////////////
    // Link and verify
    //glBindAttribLocation(shader_program, 0, "in_position");
    //glBindAttribLocation(shader_program, 1, "in_uv");
    glLinkProgram(shader_program);

    const GLenum gl_err_link = glGetError();
    if (gl_err_link != GL_NO_ERROR) {
        ILLIXR::abort("[shader_util] Linking failed");
    }

    glGetProgramiv(shader_program, GL_LINK_STATUS, &result);
    if (result == GL_FALSE) {
        GLsizei             length = 0;
        std::vector<GLchar> gl_buf_log;
        gl_buf_log.resize(GL_MAX_LOG_LENGTH);

        glGetProgramInfoLog(shader_program, GL_MAX_LOG_LENGTH * sizeof(GLchar), &length, gl_buf_log.data());
        const std::string msg{gl_buf_log.begin(), gl_buf_log.end()};
        assert(length == static_cast<GLsizei>(msg.size()) && "Length of log should match GLchar vector contents");
        ILLIXR::abort("[shader_util] Failed to get shader program: " + msg);
    }

    // After successful link, detach shaders from shader program
    glDetachShader(shader_program, vertex_shader_handle);
    glDetachShader(shader_program, fragment_shader_handle);

    return shader_program;
}
