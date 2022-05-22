#version 120

#if @useUBO
    #extension GL_ARB_uniform_buffer_object : require
#endif

#if @useGPUShader4
    #extension GL_EXT_gpu_shader4: require
#endif

#define GROUNDCOVER

uniform mat3 grassData;

#if @diffuseMap
uniform sampler2D diffuseMap;
varying vec2 diffuseMapUV;
#endif

#if @normalMap
uniform sampler2D normalMap;
varying vec2 normalMapUV;
varying vec4 passTangent;
#endif

// Other shaders respect forcePPL, but legacy groundcover mods were designed to work with vertex lighting.
// They may do not look as intended with per-pixel lighting, so ignore this setting for now.
#define PER_PIXEL_LIGHTING 1//@normalMap

varying float depth;
#define euclideanDepth depth
#define linearDepth depth

#if PER_PIXEL_LIGHTING
varying vec3 passViewPos;
varying vec3 passNormal;
#else
centroid varying vec3 passLighting;
centroid varying vec3 shadowDiffuseLighting;
#endif

uniform mat4 osg_ViewMatrixInverse;
uniform mat4 osg_ModelViewMatrix;
uniform mat4 osg_ViewMatrix;

#include "shadows_fragment.glsl"
#include "lighting.glsl"
#include "alpha.glsl"

void main()
{
#if !@grassDebugBatches
if(grassData[2].y != grassData[2].x)
    if (depth > grassData[2].y)
        discard;
#endif

#if @normalMap
    vec4 normalTex = texture2D(normalMap, normalMapUV);

    vec3 normalizedNormal = normalize(passNormal);
    vec3 normalizedTangent = normalize(passTangent.xyz);
    vec3 binormal = cross(normalizedTangent, normalizedNormal) * passTangent.w;
    mat3 tbnTranspose = mat3(normalizedTangent, binormal, normalizedNormal);

    vec3 viewNormal = gl_NormalMatrix * normalize(tbnTranspose * (normalTex.xyz * 2.0 - 1.0));
#else
    vec3 viewNormal = gl_NormalMatrix * normalize(passNormal);
#endif

#if @diffuseMap
    gl_FragData[0] = texture2D(diffuseMap, diffuseMapUV);
#else
    gl_FragData[0] = vec4(1.0);
#endif

    vec3 wPos = (osg_ViewMatrixInverse * osg_ModelViewMatrix * vec4(passViewPos, 1)).xyz;
    float waterDepth = max(-wPos.z, 0);

#if !@grassDebugBatches
    if (depth > grassData[2].x)
        gl_FragData[0].a *= 1.0-smoothstep(grassData[2].x, grassData[2].y, depth);
#endif


    alphaTest();

    float shadowing = unshadowedLightRatio(linearDepth);

    vec3 lighting;
#if !PER_PIXEL_LIGHTING
    lighting = passLighting + gl_LightModel.ambient.xyz + shadowDiffuseLighting * shadowing;
    gl_FragData[0].xyz *= lighting;
#else
    /*
    vec3 diffuseLight, ambientLight;
    doLighting(passViewPos, normalize(viewNormal), shadowing, diffuseLight, ambientLight, 1, false);
    lighting = diffuseLight + ambientLight;
    clampLightingResult(lighting);
    */

    vec3 color = gl_FragData[0].rgb;

    vec3 albedo; float ao;
    colorToPbr(color, albedo, ao);

    gl_FragData[0].xyz = getPbr(
        osg_ViewMatrixInverse,
        passViewPos,
        viewNormal,
        albedo,
        0.75, // roughness
        0.5, // base reflectance
        0.0, // metalness
        shadowing,
        #if SHADOWS
            shadowFadeStart,
        #else
            3500.0,
        #endif
        ao,
        vec3(0.0),
        0.5,
        waterDepth,
        MAT_DEFAULT
    );
#endif

    clampLightingResult(gl_FragData[0].xyz);

#if @radialFog
    float fogValue = clamp((euclideanDepth - gl_Fog.start) * gl_Fog.scale, 0.0, 1.0);
#else
    float fogValue = clamp((linearDepth - gl_Fog.start) * gl_Fog.scale, 0.0, 1.0);
#endif
    gl_FragData[0].xyz = mix(gl_FragData[0].xyz, gl_Fog.color.xyz, fogValue);

    applyShadowDebugOverlay();

#if @grassDebugBatches
    gl_FragData[0].xyz = debugColor;
#endif
}
