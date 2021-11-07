#version 120

varying vec2 uv;
varying highp float depth;

#define PER_PIXEL_LIGHTING (@normalMap || @forcePPL)

uniform mat3 shaderSettings;
#include "helpsettings.glsl"
#include "vertexcolors.glsl"

#ifdef HEIGHT_FOG
varying vec3 fogH;
#endif

#if defined(UNDERWATER_DISTORTION) || defined(HEIGHT_FOG)
uniform mat4 osg_ViewMatrixInverse;
#endif

#ifdef UNDERWATER_DISTORTION
uniform float osg_SimulationTime;
#endif

varying vec3 passViewPos;

#if (PER_PIXEL_LIGHTING || @specularMap || defined(HEIGHT_FOG))
varying vec3 passNormal;
#endif

#if !PER_PIXEL_LIGHTING
    uniform highp mat4 osg_ViewMatrixInverse;
    centroid varying vec3 passLighting;
    #include "lighting_util.glsl"
    #include "lighting.glsl"
#endif

uniform bool radialFog;

void main(void)
{
    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;

    highp vec4 viewPos = (gl_ModelViewMatrix * gl_Vertex);
    gl_ClipVertex = viewPos;

if (radialFog)
    depth = length(viewPos.xyz);
else
    depth = gl_Position.z;

    passColor = gl_Color;
    passViewPos = viewPos.xyz;

#if (PER_PIXEL_LIGHTING || @specularMap || defined(HEIGHT_FOG))
    passNormal = gl_Normal.xyz;
#endif

#ifdef HEIGHT_FOG
    fogH = (osg_ViewMatrixInverse * viewPos).xyz;
#endif
    uv = gl_MultiTexCoord0.xy;

#ifdef UNDERWATER_DISTORTION
if(osg_ViewMatrixInverse[3].z < -1.0)
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

#if !PER_PIXEL_LIGHTING
    vec3 shadowDiffuseLighting, diffuseLight, ambientLight;
    vec3 viewNormal = normalize((gl_NormalMatrix * gl_Normal).xyz);
    doLighting(viewPos.xyz, viewNormal, diffuseLight, ambientLight, shadowDiffuseLighting);
    passLighting = colLoad(getDiffuseColor().xyz) * diffuseLight + vcolLoad(getAmbientColor().xyz) * ambientLight + colLoad(getEmissionColor().xyz);
    clampLightingResult(passLighting);
    shadowDiffuseLighting *= colLoad(getDiffuseColor().xyz);
    passLighting += shadowDiffuseLighting;
#endif
}
