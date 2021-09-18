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

#include "helperutil.glsl"
#include "vertexcolors.glsl"

varying float depth;

#if !@radialFog
varying float linearDepth;
#endif

#if PER_PIXEL_LIGHTING
varying vec3 passNormal;
varying vec3 passViewPos;
#endif

  #include "lighting.glsl"

#if !PER_PIXEL_LIGHTING
     centroid varying vec3 passLighting;
#endif

#include "alpha.glsl"

void main()
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
	  gl_FragData[0].rgb = SRGBToLinear(gl_FragData[0].rgb);
#else
    gl_FragData[0] = vec4(1.0);
#endif

    if (depth > @groundcoverFadeStart)
        gl_FragData[0].a *= 1.0-smoothstep(@groundcoverFadeStart, @groundcoverFadeEnd, depth);

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


	float exposure = mix(4.6, 2.6, length(SRGBToLinearApprox(lcalcDiffuse(0).xyz) + SRGBToLinearApprox(gl_LightModel.ambient.xyz)) * 0.5);
	
	
	#ifdef PBRDEBUG
	gl_FragData[0].xyz *= 1.0;
	#else
	gl_FragData[0].xyz *= pow(2.0, exposure);
	
	
    // convert unbounded HDR color range to SDR color range
    gl_FragData[0].xyz = ACESFilm(gl_FragData[0].xyz);
 
    // convert from linear to sRGB for display
    gl_FragData[0].xyz = LinearToSRGB(gl_FragData[0].xyz);
	#endif

    gl_FragData[0].xyz = mix(gl_FragData[0].xyz, gl_Fog.color.xyz, fogValue);

    gl_FragData[0].xyz = pow(gl_FragData[0].xyz, vec3(1.0/@gamma));

}
