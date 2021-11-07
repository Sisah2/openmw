#version 120

#define GRASS

#define PER_PIXEL_LIGHTING @normalMap

#if @diffuseMap
uniform sampler2D diffuseMap;
varying vec2 diffuseMapUV;
#endif

#if @normalMap
uniform sampler2D normalMap;
varying vec4 passTangent;
#endif

#include "vertexcolors.glsl"

varying float depth;

#if PER_PIXEL_LIGHTING
varying vec3 passNormal;
varying vec3 passViewPos;
#endif

#if PER_PIXEL_LIGHTING
    #include "lighting.glsl"
#else
     centroid varying vec3 passLighting;
#endif

uniform mat3 grassData;

#include "alpha.glsl"

void main()
{

if(grassData[2].y != grassData[2].x)
    if (depth > grassData[2].y)
        discard;

#if @normalMap
vec4 normalTex = texture2D(normalMap, diffuseMapUV);
vec3 normalizedNormal = normalize(passNormal);
vec3 normalizedTangent = normalize(passTangent.xyz);
vec3 binormal = cross(normalizedTangent, normalizedNormal) * passTangent.w;
mat3 tbnTranspose = mat3(normalizedTangent, binormal, normalizedNormal);
vec3 viewNormal = gl_NormalMatrix * normalize(tbnTranspose * (normalTex.xyz * 2.0 - 1.0));
#endif

#if @diffuseMap
    gl_FragData[0] = texture2D(diffuseMap, diffuseMapUV);
#else
    gl_FragData[0] = vec4(1.0);
#endif

    if (depth > grassData[2].x)
        gl_FragData[0].a *= 1.0-smoothstep(grassData[2].x, grassData[2].y, depth);

    alphaTest();

    vec3 lighting;
#if !PER_PIXEL_LIGHTING
    lighting = passLighting;
#else
    vec3 diffuseLight, ambientLight;
    doLighting(passViewPos, normalize(viewNormal), 1.0, diffuseLight, ambientLight);
    lighting = diffuseLight + ambientLight;
    clampLightingResult(lighting);
#endif

gl_FragData[0].xyz *= lighting;

    highp float fogValue = clamp((depth - gl_Fog.start) * gl_Fog.scale, 0.0, 1.0);

    gl_FragData[0].xyz = mix(gl_FragData[0].xyz, gl_Fog.color.xyz, fogValue);

gl_FragData[0].xyz = pow(gl_FragData[0].xyz, vec3(1.0/@gamma));

}
