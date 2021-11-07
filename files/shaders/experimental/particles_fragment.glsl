#version 120

#define PARTICLE

#if @diffuseMap
uniform sampler2D diffuseMap;
varying vec2 diffuseMapUV;
#endif

centroid varying vec3 passLighting;
varying float depth;
uniform float gamma;

#include "helpsettings.glsl"
#include "vertexcolors.glsl"
#include "lighting_util.glsl"

void main()
{

float fogValue = clamp((depth - gl_Fog.start) * gl_Fog.scale, 0.0, 1.0);

if(fogValue != 1.0)
{

#if @diffuseMap
    gl_FragData[0] = texture2D(diffuseMap, diffuseMapUV);
#else
    gl_FragData[0] = vec4(1.0);
#endif

    vec4 diffuseColor = getDiffuseColor();
    gl_FragData[0].a *= diffuseColor.a;

    if(gl_FragData[0].a == 0.0)
        discard;

    gl_FragData[0].xyz *= passLighting;

}
    gl_FragData[0].xyz = mix(gl_FragData[0].xyz, gl_Fog.color.xyz, fogValue);

    gl_FragData[0].xyz = pow(gl_FragData[0].xyz, vec3(1.0/ (@gamma + gamma - 1.0)));

}
