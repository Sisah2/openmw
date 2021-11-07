#version 120

#define OBJECT
#define PER_PIXEL_LIGHTING (@normalMap || (@forcePPL))

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

#if @normalMap
varying vec4 passTangent;
#endif

#if @envMap
varying vec2 envMapUV;
#endif

#if PER_PIXEL_LIGHTING || @specularMap
varying vec3 passNormal;
#endif

#ifdef HEIGHT_FOG
varying vec3 fogH;
#endif

#if defined(HEIGHT_FOG) || defined(UNDERWATER_DISTORTION)
uniform mat4 osg_ViewMatrixInverse;
#endif

#if defined(UNDERWATER_DISTORTION)
uniform bool isInterior;
#endif

#ifdef UNDERWATER_DISTORTION
uniform float osg_SimulationTime;
uniform bool isPlayer;
#endif

varying highp float depth;
varying vec3 passViewPos;

uniform bool radialFog;

#include "helpsettings.glsl"
#include "vertexcolors.glsl"

#if !PER_PIXEL_LIGHTING
    uniform highp mat4 osg_ViewMatrixInverse;
    centroid varying vec3 passLighting;
    #include "lighting_util.glsl"
    #include "lighting.glsl"
#endif

void main(void)
{
    highp vec4 viewPos = (gl_ModelViewMatrix * gl_Vertex);
    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
    gl_ClipVertex = viewPos;

if(radialFog)
    depth = length(viewPos.xyz);
else
    depth = gl_Position.z;

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
    decalMapUV = (gl_TextureMatrix[@decalMapUV] * gl_MultiTexCoord@decalMapUV).xy;;
#endif

#if @normalMap
    passTangent = gl_MultiTexCoord7.xyzw;
#endif

    passColor = gl_Color;
    passViewPos = viewPos.xyz;

#if PER_PIXEL_LIGHTING || @specularMap
    passNormal = gl_Normal.xyz;
#endif

#ifdef HEIGHT_FOG
    fogH = (osg_ViewMatrixInverse * viewPos).xyz;
#endif

#ifdef UNDERWATER_DISTORTION
if(osg_ViewMatrixInverse[3].z < -1.0 && !isInterior && !isPlayer)
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
    doLighting(viewPos.xyz, viewNormal, diffuseLight, ambientLight, shadowDiffuseLighting);
    passLighting = colLoad(getDiffuseColor().xyz) * diffuseLight + vcolLoad(getAmbientColor().xyz) * ambientLight + colLoad(getEmissionColor().xyz);
    clampLightingResult(passLighting);
    shadowDiffuseLighting *= colLoad(getDiffuseColor().xyz);
    passLighting += shadowDiffuseLighting;
#endif
}