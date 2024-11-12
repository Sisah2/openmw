#version 120
#pragma import_defines(DECODE)

varying vec2 uv;

#include "lib/util/packcolors.glsl"
#include "lib/core/fragment.h.glsl"

void main()
{
    vec4 encoded = samplerLastShader(uv);
    gl_FragColor = encoded;

#if defined(DECODE)
    vec4 scene, normals;
    decode(encoded, scene, normals);

#if defined(DECODE) && DECODE
    gl_FragColor = normals;
#else
    gl_FragColor = scene;
#endif

#endif
}
