#version 120

varying vec2 uv;
varying float depth;

#define PER_PIXEL_LIGHTING (@normalMap || @forcePPL)

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

#if (PER_PIXEL_LIGHTING || @specularMap || defined(HEIGHT_FOG) || @underwaterFog)
varying vec3 passViewPos;
#endif

#if (PER_PIXEL_LIGHTING || @specularMap || defined(HEIGHT_FOG))
varying vec3 passNormal;
#endif

#if !PER_PIXEL_LIGHTING
#include "lighting_util.glsl"
centroid varying vec3 passLighting;
  #ifdef LINEAR_LIGHTING
    #include "linear_lighting.glsl"
  #else
    #include "lighting.glsl"
  #endif
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

passColor = gl_Color;

#if (PER_PIXEL_LIGHTING || @specularMap || defined(HEIGHT_FOG) || @underwaterFog)
    passViewPos = viewPos.xyz;
#endif

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
    vec3 shadowDiffuseLighting;
    vec3 viewNormal = normalize((gl_NormalMatrix * gl_Normal).xyz);
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
#endif
}
