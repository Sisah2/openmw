#version 120

#define PER_PIXEL_LIGHTING 0

#define PARTICLE

#if @diffuseMap
varying vec2 diffuseMapUV;
#endif

varying float depth;
centroid varying vec3 passLighting;

#include "helpsettings.glsl"
#include "vertexcolors.glsl"
#include "lighting_util.glsl"

#ifdef LINEAR_LIGHTING
  #include "linear_lighting.glsl"
#else
  #include "lighting.glsl"
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

#if @diffuseMap
    diffuseMapUV = (gl_TextureMatrix[@diffuseMapUV] * gl_MultiTexCoord@diffuseMapUV).xy;
#endif

passColor = gl_Color;

vec3 viewNormal = normalize((gl_NormalMatrix * gl_Normal).xyz);
vec3 shadowDiffuseLighting;

#ifdef LINEAR_LIGHTING
    passLighting = doLighting(viewPos.xyz, viewNormal, gl_Color);
#else
    vec3 diffuseLight, ambientLight;
    doLighting(viewPos.xyz, viewNormal, diffuseLight, ambientLight, shadowDiffuseLighting);
    passLighting = getDiffuseColor().xyz * diffuseLight + getAmbientColor().xyz * ambientLight + getEmissionColor().xyz;
#endif
    clampLightingResult(passLighting);
    shadowDiffuseLighting *= getDiffuseColor().xyz;
    passLighting += shadowDiffuseLighting;
}
