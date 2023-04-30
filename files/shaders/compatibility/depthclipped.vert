#version 120

varying vec2 diffuseMapUV;
varying float alphaPassthrough;

#include "lib/core/vertex.h.glsl"
#include "vertexcolors.glsl"

varying vec3 passViewPos;
varying float depth;

void main()
{
    gl_Position = modelToClip(gl_Vertex);

    vec4 viewPos = modelToView(gl_Vertex);
    gl_ClipVertex = viewPos;

    if (colorMode == 2)
        alphaPassthrough = gl_Color.a;
    else
        alphaPassthrough = gl_FrontMaterial.diffuse.a;

    diffuseMapUV = (gl_TextureMatrix[0] * gl_MultiTexCoord0).xy;

    passViewPos = viewPos.xyz;

    depth = -(gl_ModelViewMatrix * gl_Vertex).z;;

}
