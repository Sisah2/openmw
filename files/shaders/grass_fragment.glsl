#version 120

#if @diffuseMap
uniform sampler2D diffuseMap;
varying vec2 diffuseMapUV;
#endif

#define PER_PIXEL_LIGHTING @forcePPL

varying float euclideanDepth;

#if !@radialFog
varying float linearDepth;
#endif

#if !PER_PIXEL_LIGHTING
centroid varying vec4 lighting;
#endif

#if PER_PIXEL_LIGHTING
uniform int colorMode;
centroid varying vec4 passColor;
varying vec3 passViewPos;
varying vec3 passNormal;
#include "lighting.glsl"
#endif

void main()
{
#if @diffuseMap
    vec2 adjustedDiffuseUV = diffuseMapUV;
#endif

#if PER_PIXEL_LIGHTING
    vec3 viewNormal = gl_NormalMatrix * normalize(passNormal);
#endif

#if @diffuseMap
    gl_FragData[0] = texture2D(diffuseMap, adjustedDiffuseUV);
#else
    gl_FragData[0] = vec4(1.0);
#endif

    if (euclideanDepth > @grassFadeStart)
        gl_FragData[0].a *= 1.0-smoothstep(@grassFadeStart, @grassFadeEnd, euclideanDepth);

#if !PER_PIXEL_LIGHTING
    gl_FragData[0] *= lighting;
#else
    if(gl_FragData[0].a != 0.0)
        gl_FragData[0] *= doLighting(passViewPos, normalize(viewNormal), passColor, true);
#endif

#if @radialFog
    float fogValue = clamp((euclideanDepth - gl_Fog.start) * gl_Fog.scale, 0.0, 1.0);
#else
    float fogValue = clamp((linearDepth - gl_Fog.start) * gl_Fog.scale, 0.0, 1.0);
#endif
    gl_FragData[0].xyz = mix(gl_FragData[0].xyz, gl_Fog.color.xyz, fogValue);
}
