#version 120

#define PARTICLE

#if @diffuseMap
uniform sampler2D diffuseMap;
varying vec2 diffuseMapUV;
#endif

centroid varying vec3 passLighting;
varying float depth;

#include "helpsettings.glsl"
#include "vertexcolors.glsl"
#include "lighting_util.glsl"
#include "fog.glsl"

vec3 SpecialContrast(vec3 x, float suncon) 
{
	vec3 contrasted = x*x*x*(x*(x*6.0 - 15.0) + 10.0);
	x.rgb = mix(x.rgb, contrasted, suncon);
	return x;
}

void main()
{

float fogValue = getFogValue(depth);
if(fogValue != 1.0)
{

#if @diffuseMap
    gl_FragData[0] = texture2D(diffuseMap, diffuseMapUV);
#else
    gl_FragData[0] = vec4(1.0);
#endif

    if(gl_FragData[0].a == 0.0)
        discard;

    gl_FragData[0].xyz *= passLighting;

#ifdef LINEAR_LIGHTING
        gl_FragData[0].xyz = pow(gl_FragData[0].xyz, vec3(1.0/(2.2+(@gamma.0/1000.0)-1.0)));
        gl_FragData[0].xyz = SpecialContrast(gl_FragData[0].xyz, mix(connight, conday, lcalcDiffuse(0).x));
#endif

}

    gl_FragData[0].xyz = mix(gl_FragData[0].xyz, gl_Fog.color.xyz, fogValue);

#if (@gamma != 1000) && !defined(LINEAR_LIGHTING)
    gl_FragData[0].xyz = pow(gl_FragData[0].xyz, vec3(1.0/(@gamma.0/1000.0)));
#endif
}
