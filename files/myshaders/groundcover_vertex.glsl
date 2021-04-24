#version 120

#define PER_PIXEL_LIGHTING 0

#define GRASS

#include "helpsettings.glsl"
#include "vertexcolors.glsl"
#include "lighting_util.glsl"

#if @diffuseMap
varying vec2 diffuseMapUV;
#endif

varying float depth;
#if !@radialFog
varying float linearDepth;
#endif

#ifdef HEIGHT_FOG
varying vec3 fogH;
#endif

#if @underwaterFog
varying vec3 passViewPos;
#endif

centroid varying vec3 passLighting;

#ifdef LINEAR_LIGHTING
   #include "linear_lighting.glsl"
#else
   #include "lighting.glsl"
#endif

uniform mat4 osg_ViewMatrixInverse;
uniform float osg_SimulationTime;

uniform vec3 windData;
uniform vec3 playerPos;
attribute float originalCoords;

#if @groundcoverStompMode == 0
#else
    #define STOMP 1
    #if @groundcoverStompMode == 2
        #define STOMP_HEIGHT_SENSITIVE 1
    #endif
    #define STOMP_INTENSITY_LEVEL @groundcoverStompIntensity
#endif

vec4 grassDisplacement(vec3 viewPos, vec4 vertex)
{
    float h = originalCoords;

    vec4 worldPos = osg_ViewMatrixInverse * vec4(viewPos, 1.0);

    vec2 WindVec = vec2(windData.x);

    float v = length(WindVec);
    vec2 displace = vec2(2.0 * WindVec + 0.1);

    vec2 harmonics = vec2(0.0);

    harmonics.xy += vec2((1.0 - 0.10*v) * sin(1.0*osg_SimulationTime +  worldPos.xy / 1100.0));
    harmonics.xy += vec2((1.0 - 0.04*v) * cos(2.0*osg_SimulationTime +  worldPos.xy / 750.0));
    harmonics.xy += vec2((1.0 + 0.14*v) * sin(3.0*osg_SimulationTime +  worldPos.xy / 500.0));
    harmonics.xy += vec2((1.0 + 0.28*v) * sin(5.0*osg_SimulationTime  +  worldPos.xy / 200.0));

    vec2 stomp = vec2(0.0);
#if STOMP
    float d = length(worldPos.xy - playerPos.xy);
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

    vec4 ret = vec4(0.0);
    ret.xy += clamp(0.02 * h, 0.0, 1.0) * (harmonics * displace + stomp);

#ifdef STORM_MODE
    vec2 stormDir = vec2(windData.y, windData.z);

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
    vec4 viewPos = (gl_ModelViewMatrix * gl_Vertex);
    gl_Position = gl_ModelViewProjectionMatrix * grassDisplacement(viewPos.xyz, gl_Vertex);

    gl_ClipVertex = viewPos;
    depth = length(viewPos.xyz);

#if !@radialFog
    linearDepth = gl_Position.z;
#endif

#if @diffuseMap
    diffuseMapUV = (gl_TextureMatrix[@diffuseMapUV] * gl_MultiTexCoord@diffuseMapUV).xy;
#endif

vec3 viewNormal = normalize((gl_NormalMatrix * gl_Normal).xyz);


vec3 shadowDiffuseLighting;
#ifdef LINEAR_LIGHTING
    passLighting = doLighting(viewPos.xyz, viewNormal, gl_Color);
#else
    vec3 diffuseLight, ambientLight;
    doLighting(viewPos.xyz, viewNormal, diffuseLight, ambientLight, shadowDiffuseLighting);
    passLighting = diffuseLight + ambientLight;
#endif

    clampLightingResult(passLighting);
    passLighting += shadowDiffuseLighting;

#if @underwaterFog
    passViewPos = viewPos.xyz;
#endif

#ifdef HEIGHT_FOG
    fogH = (osg_ViewMatrixInverse * viewPos).xyz;
#endif

#ifdef UNDERWATER_DISTORTION
if(osg_ViewMatrixInverse[3].z < -1.0 && gl_LightSource[0].diffuse.x != 0.0)
{
    vec2 harmonics;
    vec4 wP = osg_ViewMatrixInverse * vec4(viewPos.xyz, 1.0);
    harmonics += vec2(sin(1.0*osg_SimulationTime + wP.xy / 1100.0));
    harmonics += vec2(cos(2.0*osg_SimulationTime + wP.xy / 750.0));
    harmonics += vec2(sin(3.0*osg_SimulationTime + wP.xy / 500.0));
    harmonics += vec2(sin(5.0*osg_SimulationTime + wP.xy / 200.0));
    gl_Position.xy += (depth * 0.003) * harmonics;
}
#endif
}
