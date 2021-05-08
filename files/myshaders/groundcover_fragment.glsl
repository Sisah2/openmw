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

#include "helpsettings.glsl"
#include "vertexcolors.glsl"
#include "lighting_util.glsl"

varying float depth;

#if !@radialFog
varying float linearDepth;
#endif

#if @underwaterFog
uniform mat4 osg_ViewMatrixInverse;
#endif

#ifdef ANIMATED_HEIGHT_FOG
uniform float osg_SimulationTime;
#endif

#if PER_PIXEL_LIGHTING || @underwaterFog
varying vec3 passViewPos;
#endif

#if PER_PIXEL_LIGHTING
varying vec3 passNormal;
#endif

#if PER_PIXEL_LIGHTING
  #ifdef LINEAR_LIGHTING
    #include "linear_lighting.glsl"
  #else
    #include "lighting.glsl"
  #endif
#else
  centroid varying vec3 passLighting;
#endif

#include "effects.glsl"
#include "fog.glsl"
#include "alpha.glsl"

void main()
{

#if @underwaterFog
    bool isUnderwater = (osg_ViewMatrixInverse * vec4(passViewPos, 1.0)).z < -1.0 && osg_ViewMatrixInverse[3].z > -1.0;
    float underwaterFogValue = (isUnderwater) ? getUnderwaterFogValue(depth) : 0.0;
#endif
    float fogValue = getFogValue(depth);

#if @underwaterFog
if(underwaterFogValue != 1.0 && fogValue != 1.0)
#else
if(fogValue != 1.0)
#endif
{

if(@groundcoverFadeEnd != @groundcoverFadeStart)
    if (depth > @groundcoverFadeEnd)
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

    if (depth > @groundcoverFadeStart)
        gl_FragData[0].a *= 1.0-smoothstep(@groundcoverFadeStart, @groundcoverFadeEnd, depth);

    alphaTest();

//gl_FragData[0].xyz *= vec3(1.0+smoothstep(0.0, @groundcoverFadeEnd, depth));

#ifdef LINEAR_LIGHTING
    gl_FragData[0].xyz = pow(gl_FragData[0].xyz, vec3(2.2));
#endif

    vec3 lighting;
#if !PER_PIXEL_LIGHTING
    lighting = passLighting;
#else
#ifdef LINEAR_LIGHTING
    lighting = doLighting(passViewPos, normalize(viewNormal), passColor, 1.0);
#else
    vec3 diffuseLight, ambientLight;
    doLighting(passViewPos, normalize(viewNormal), 1.0, diffuseLight, ambientLight);
    lighting = diffuseLight + ambientLight;
#endif
    clampLightingResult(lighting);
#endif

gl_FragData[0].xyz *= lighting;

#ifdef LINEAR_LIGHTING
        gl_FragData[0].xyz = Uncharted2ToneMapping(gl_FragData[0].xyz);
        gl_FragData[0].xyz = pow(gl_FragData[0].xyz, vec3(1.0/(2.2+(@gamma.0/1000.0)-1.0)));
        gl_FragData[0].xyz = SpecialContrast(gl_FragData[0].xyz, mix(connight, conday, lcalcDiffuse(0).x));
#endif

}

#if @underwaterFog
    gl_FragData[0].xyz = mix(gl_FragData[0].xyz, uwfogcolor, underwaterFogValue);
#endif
    gl_FragData[0].xyz = mix(gl_FragData[0].xyz, gl_Fog.color.xyz, fogValue);

#if (@gamma != 1000) && !defined(LINEAR_LIGHTING)
    gl_FragData[0].xyz = pow(gl_FragData[0].xyz, vec3(1.0/(@gamma.0/1000.0)));
#endif

}
