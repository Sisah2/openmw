#version 120

varying vec2 uv;
varying float depth;

#define PER_PIXEL_LIGHTING (@normalMap || @forcePPL)

#if !PER_PIXEL_LIGHTING
uniform int colorMode;
centroid varying vec4 lighting;
#include "lighting.glsl"
#endif

#if PER_PIXEL_LIGHTING
centroid varying vec4 passColor;
#endif

#if PER_PIXEL_LIGHTING || @specularMap
varying vec3 passViewPos;
varying vec3 passNormal;
#endif

void main(void)
{
    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;

    vec4 viewPos = (gl_ModelViewMatrix * gl_Vertex);
    gl_ClipVertex = viewPos;

#if @radialFog
    depth = length(viewPos.xyz);
#else
    depth = gl_Position.z;
#endif

#if (!PER_PIXEL_LIGHTING || @shadows_enabled)
    vec3 viewNormal = normalize((gl_NormalMatrix * gl_Normal).xyz);
#endif

#if !PER_PIXEL_LIGHTING
    lighting = doLighting(viewPos.xyz, viewNormal, gl_Color, false);
#else
    passColor = gl_Color;
#endif

#if PER_PIXEL_LIGHTING || @specularMap
    passNormal = gl_Normal.xyz;
    passViewPos = viewPos.xyz;
#endif

    uv = gl_MultiTexCoord0.xy;
}
