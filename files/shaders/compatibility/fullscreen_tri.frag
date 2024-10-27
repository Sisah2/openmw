#version 120
#pragma import_defines(WRITE_NORMALS)

varying vec2 uv;

#include "lib/util/packcolors.glsl"
#include "lib/core/fragment.h.glsl"

uniform sampler2D externalNormals;
void main()
{
    gl_FragColor = samplerLastShader(uv);

#if defined(WRITE_NORMALS)
    if (gl_FragColor.a == 1.0)
    {
        vec4 scene, normals;
        decode(gl_FragColor, scene, normals);

        gl_FragColor.rgb = normals.rgb;
    }
    else
    {
        gl_FragColor.rgb = texture2D(externalNormals, uv).rgb * 2.0 - 1.0;
        gl_FragColor.a = 1.0;
    }
#endif

}
