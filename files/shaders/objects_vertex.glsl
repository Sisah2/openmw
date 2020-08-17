#version 120

#if @diffuseMap
varying vec2 diffuseMapUV;
#endif

#if @darkMap
varying vec2 darkMapUV;
#endif

#if @detailMap
varying vec2 detailMapUV;
#endif

#if @decalMap
varying vec2 decalMapUV;
#endif

#if @emissiveMap
varying vec2 emissiveMapUV;
#endif

#if @normalMap
varying vec4 passTangent;
#endif

#if @envMap
varying vec2 envMapUV;
#endif

varying float depth;

#define PER_PIXEL_LIGHTING (@normalMap || (@forcePPL && (@particleHandling <= 2)))

#if !PER_PIXEL_LIGHTING
uniform int colorMode;
centroid varying vec4 lighting;
#include "lighting.glsl"
#endif

centroid varying vec4 passColor;
varying vec3 passViewPos;
varying vec3 passNormal;

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

#if (@envMap || !PER_PIXEL_LIGHTING)
    vec3 viewNormal = normalize((gl_NormalMatrix * gl_Normal).xyz);
#endif

#if @envMap
    vec3 viewVec = normalize(viewPos.xyz);
    vec3 r = reflect( viewVec, viewNormal );
    float m = 2.0 * sqrt( r.x*r.x + r.y*r.y + (r.z+1.0)*(r.z+1.0) );
    envMapUV = vec2(r.x/m + 0.5, r.y/m + 0.5);
#endif

#if @diffuseMap
    diffuseMapUV = (gl_TextureMatrix[@diffuseMapUV] * gl_MultiTexCoord@diffuseMapUV).xy;
#endif

#if @darkMap
    darkMapUV = (gl_TextureMatrix[@darkMapUV] * gl_MultiTexCoord@darkMapUV).xy;
#endif

#if @detailMap
    detailMapUV = (gl_TextureMatrix[@detailMapUV] * gl_MultiTexCoord@detailMapUV).xy;
#endif

#if @decalMap
    decalMapUV = (gl_TextureMatrix[@decalMapUV] * gl_MultiTexCoord@decalMapUV).xy;
#endif

#if @emissiveMap
    emissiveMapUV = (gl_TextureMatrix[@emissiveMapUV] * gl_MultiTexCoord@emissiveMapUV).xy;
#endif

#if @normalMap
    passTangent = gl_MultiTexCoord7.xyzw;
#endif

#if !PER_PIXEL_LIGHTING
    lighting = doLighting(viewPos.xyz, viewNormal, gl_Color, false);
#endif

    passColor = gl_Color;
    passViewPos = viewPos.xyz;
    passNormal = gl_Normal.xyz;
}
