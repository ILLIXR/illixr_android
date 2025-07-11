#pragma once

const char* const time_warp_chromatic_vertex_program_GLSL =
        "   precision mediump float;"
        "\n "
        "\n // Model-view-projection transformation matrix"
        "\n uniform mat4 TimeWarpStartTransform;"
        "\n uniform mat4 TimeWarpEndTransform;"
        "\n "
        "\n // Incoming"
        "\n attribute vec3 vertexPosition;"
        "\n attribute vec2 vertexUv0;"
        "\n attribute vec2 vertexUv1;"
        "\n attribute vec2 vertexUv2;"
        "\n varying vec2 fragmentUv0;"
        "\n varying vec2 fragmentUv1;"
        "\n varying vec2 fragmentUv2;"
        "\n "
        "\n // Outgoing interpolated values for the fragment shader"
        "\n varying vec3 color;"
        "\n "
        "\n void main() {"
        "\n gl_Position = vec4(vertexPosition, 1.0 );"
        "\n float displayFraction = vertexPosition.x * 0.5 + 0.5;"
        "\n vec3 startUv0 = (TimeWarpStartTransform * vec4( vertexUv0, -1, 1 )).xyz;"
        "\n vec3 startUv1 = (TimeWarpStartTransform * vec4( vertexUv1, -1, 1 )).xyz;"
        "\n vec3 startUv2 = (TimeWarpStartTransform * vec4( vertexUv2, -1, 1 )).xyz;"
        "\n vec3 endUv0 = (TimeWarpEndTransform * vec4( vertexUv0, -1, 1 )).xyz;"
        "\n vec3 endUv1 = (TimeWarpEndTransform * vec4( vertexUv1, -1, 1 )).xyz;"
        "\n vec3 endUv2 = (TimeWarpEndTransform * vec4( vertexUv2, -1, 1 )).xyz;"
        "\n"
        "   vec3 curUv0 = mix( startUv0, endUv0, displayFraction );\n"
        "   vec3 curUv1 = mix( startUv1, endUv1, displayFraction );\n"
        "   vec3 curUv2 = mix( startUv2, endUv2, displayFraction );\n"
        "\n"
        "   fragmentUv0 = curUv0.xy * ( 1.0 / max( curUv0.z, 0.00001 ) );\n"
        "   fragmentUv1 = curUv1.xy * ( 1.0 / max( curUv1.z, 0.00001 ) );\n"
        "   fragmentUv2 = curUv2.xy * ( 1.0 / max( curUv2.z, 0.00001 ) );\n"
        "\n }";

const char* const time_warp_chromatic_fragment_program_GLSL =
    "   precision mediump float;"
    "uniform int ArrayLayer;\n"
    "uniform  sampler2DArray Texture;\n"
    "attribute vec2 fragmentUv0;\n"
    "attribute vec2 fragmentUv1;\n"
    "attribute vec2 fragmentUv2;\n"
    "varying vec4 outColor;\n"
    "void main()\n"
    "{\n"
    "   outColor.r = texture( Texture, vec3( fragmentUv0, ArrayLayer ) ).r;\n"
    "   outColor.g = texture( Texture, vec3( fragmentUv1, ArrayLayer ) ).g;\n"
    "   outColor.b = texture( Texture, vec3( fragmentUv2, ArrayLayer ) ).b;\n"
    "   outColor.a = 1.0;\n"
    "}\n";

const char* const time_warp_chromatic_fragment_program_GLSL_alternative = 
        "precision mediump float;\n"
        "uniform  sampler2D Texture;\n"
        "varying vec2 fragmentUv0;\n"
        "varying vec2 fragmentUv1;\n"
        "varying vec2 fragmentUv2;\n"
        "varying vec4 outColor;\n"
        "\n "
        "\n varying vec3 color;"
        "\n "
        "\n void main() {"
        "\n   vec4 res_r = texture2D( Texture, fragmentUv0);\n"
        "\n   vec4 res_g = texture2D( Texture, fragmentUv1 );\n"
        "\n   vec4 res_b = texture2D( Texture, fragmentUv2 );\n"
        "\n   gl_FragColor = vec4(res_r.r, res_g.g, res_b.b, 1.0);"
        "\n }";
