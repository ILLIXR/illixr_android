#pragma once
#include <EGL/egl.h>
#include <GLES3/gl3.h>

static const char* const demo_vertex_shader =
    "precision mediump float;"
    "attribute vec3 in_position;\n"
    "attribute vec2 in_uv;\n"
    "uniform mat4 u_modelview;\n"
    "uniform mat4 u_projection;\n"
    "varying vec2 uv;\n"
    "void main() {\n"
    "    gl_Position = u_projection * u_modelview * vec4(in_position,1.0);\n"
    "    uv = in_uv;\n"
    "}\n";



static const char* const demo_fragment_shader =
        "precision mediump float;\n"
        "uniform sampler2D main_tex;\n"
        "varying vec2 uv;\n"
        "varying vec4 outcolor;\n"
        "\n "
        "void main()\n"
        "{\n"
        "       gl_FragColor = texture2D(main_tex, uv);\n"
        "}\n";
