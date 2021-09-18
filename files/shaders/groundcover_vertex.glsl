#version 120

#define PER_PIXEL_LIGHTING @normalMap

#define GRASS

#include "vertexcolors.glsl"

#if @diffuseMap
varying vec2 diffuseMapUV;
#endif

#if @normalMap
varying vec4 passTangent;
#endif

varying float depth;
#if !@radialFog
varying float linearDepth;
#endif

#if PER_PIXEL_LIGHTING
varying vec3 passNormal;
varying vec3 passViewPos;
#endif

#if !PER_PIXEL_LIGHTING
    centroid varying vec3 passLighting;
    #include "lighting.glsl"
#endif

uniform highp mat4 osg_ViewMatrixInverse;
uniform float osg_SimulationTime;

uniform vec3 windData;
uniform highp vec3 playerPos;
attribute float originalCoords;

#if @groundcoverStompMode == 0
#else
    #define STOMP 1
    #if @groundcoverStompMode == 2
        #define STOMP_HEIGHT_SENSITIVE 1
    #endif
    #define STOMP_INTENSITY_LEVEL @groundcoverStompIntensity
#endif

highp vec4 grassDisplacement(vec3 viewPos, vec4 vertex)
{
    highp float h = originalCoords;

    highp vec4 worldPos = osg_ViewMatrixInverse * vec4(viewPos, 1.0);

    highp vec2 WindVec = vec2(windData.x);

    highp float v = length(WindVec);
    highp vec2 displace = vec2(2.0 * WindVec + 0.1);

    highp vec2 harmonics = vec2(0.0);

    harmonics.xy += vec2((1.0 - 0.10*v) * sin(1.0*osg_SimulationTime +  worldPos.xy / 1100.0));
    harmonics.xy += vec2((1.0 - 0.04*v) * cos(2.0*osg_SimulationTime +  worldPos.xy / 750.0));
    harmonics.xy += vec2((1.0 + 0.14*v) * sin(3.0*osg_SimulationTime +  worldPos.xy / 500.0));
    harmonics.xy += vec2((1.0 + 0.28*v) * sin(5.0*osg_SimulationTime  +  worldPos.xy / 200.0));

    highp vec2 stomp = vec2(0.0);
#if STOMP
    highp float d = length(worldPos.xy - playerPos.xy);
#if STOMP_INTENSITY_LEVEL == 0
    // Gentle intensity
    const float STOMP_RANGE = 50.0; // maximum distance from player that grass is affected by stomping
    const float STOMP_DISTANCE = 20.0; // maximum distance stomping can move grass
#elif STOMP_INTENSITY_LEVEL == 1
    // Reduced intensity
    const float STOMP_RANGE = 80.0;
    const float STOMP_DISTANCE = 40.0;
#elif STOMP_INTENSITY_LEVEL == 2
    // MGE XE intensity
    const float STOMP_RANGE = 150.0;
    const float STOMP_DISTANCE = 60.0;
#endif
    if (d < STOMP_RANGE && d > 0.0)
        stomp = (STOMP_DISTANCE / d - STOMP_DISTANCE / STOMP_RANGE) * (worldPos.xy - playerPos.xy);

#ifdef STOMP_HEIGHT_SENSITIVE
    stomp *= clamp((worldPos.z - playerPos.z) / h, 0.0, 1.0);
#endif
#endif

    highp vec4 ret = vec4(0.0);
    ret.xy += clamp(0.02 * h, 0.0, 1.0) * (harmonics * displace + stomp);

#ifdef STORM_MODE
    highp vec2 stormDir = vec2(windData.y, windData.z);

    if(stormDir != vec2(0.0) && h > 0.0) {
        ret.xy += h*stormDir;
        ret.z -= length(ret.xy)/3.14;
        ret.z -= sin(osg_SimulationTime * min(h, 150.0) / 10.0) * length(stormDir);
     }
#endif

    return vertex + ret;
}

void main(void)
{
    highp vec4 viewPos = (gl_ModelViewMatrix * gl_Vertex);
    gl_Position = gl_ModelViewProjectionMatrix * grassDisplacement(viewPos.xyz, gl_Vertex);

    gl_ClipVertex = viewPos;
    depth = length(viewPos.xyz);

#if !@radialFog
    linearDepth = gl_Position.z;
#endif

#if @diffuseMap
    diffuseMapUV = (gl_TextureMatrix[@diffuseMapUV] * gl_MultiTexCoord@diffuseMapUV).xy;
#endif

#if @normalMap
    passTangent = gl_MultiTexCoord7.xyzw;
#endif

vec3 viewNormal = normalize((gl_NormalMatrix * gl_Normal).xyz);


#if !PER_PIXEL_LIGHTING
    vec3 shadowDiffuseLighting;
    vec3 diffuseLight, ambientLight;
    doLighting(viewPos.xyz, viewNormal, diffuseLight, ambientLight, shadowDiffuseLighting);
    passLighting = diffuseLight + ambientLight;
    clampLightingResult(passLighting);
    passLighting += shadowDiffuseLighting;
#endif

#if PER_PIXEL_LIGHTING
    passViewPos = viewPos.xyz;
    passNormal = gl_Normal.xyz;
#endif
}
