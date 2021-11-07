#version 120

#if @useUBO
    #extension GL_ARB_uniform_buffer_object : require
#endif

#if @useGPUShader4
    #extension GL_EXT_gpu_shader4: require
#endif

#define GROUNDCOVER

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
#define PER_PIXEL_LIGHTING @normalMap

varying float euclideanDepth;
varying float linearDepth;

#if PER_PIXEL_LIGHTING
varying vec3 passViewPos;
varying vec3 passNormal;
#else
centroid varying vec3 passLighting;
centroid varying vec3 shadowDiffuseLighting;
#endif

#include "helperutil.glsl"
#include "shadows_fragment.glsl"
#include "lighting.glsl"
#include "alpha.glsl"

uniform highp mat3 grassData;

void main()
{
#if @normalMap
    vec4 normalTex = texture2D(normalMap, normalMapUV);

    vec3 normalizedNormal = normalize(passNormal);
    vec3 normalizedTangent = normalize(passTangent.xyz);
    vec3 binormal = cross(normalizedTangent, normalizedNormal) * passTangent.w;
    mat3 tbnTranspose = mat3(normalizedTangent, binormal, normalizedNormal);

    vec3 viewNormal = gl_NormalMatrix * normalize(tbnTranspose * (normalTex.xyz * 2.0 - 1.0));
#endif

#if @diffuseMap
    gl_FragData[0] = texture2D(diffuseMap, diffuseMapUV);
	gl_FragData[0].rgb = texLoad(gl_FragData[0].rgb);
	#ifdef DEBUGLIGHTING
		#ifdef LINEAR
		gl_FragData[0].rgb = vec3(0.5 * 0.5);
		#else
		gl_FragData[0].rgb = vec3(0.5);
		#endif
	#endif
#else
    gl_FragData[0] = vec4(1.0);
#endif

    if (euclideanDepth > grassData[2].x)
        gl_FragData[0].a *= 1.0-smoothstep(grassData[2].x, grassData[2].y, euclideanDepth);

    alphaTest();

    float shadowing = unshadowedLightRatio(linearDepth);

    vec3 lighting;
#if !PER_PIXEL_LIGHTING
    lighting = passLighting + shadowDiffuseLighting * shadowing;
	gl_FragData[0].xyz *= lighting * Fd_Lambert();
#else
	vec4 param = vec4(0.0, 0.98, 0.5, 1.0);
	#if @specularMap
	param = texture2D(specularMap, adjustedDiffuseUV);
	#endif
    vec3 diffuseLight, ambientLight, specularLight;
    doLighting(passViewPos, normalize(viewNormal), param, shadowing, diffuseLight, ambientLight, specularLight);
    lighting = diffuseLight + ambientLight;
    //clampLightingResult(lighting);
	gl_FragData[0].xyz *= lighting * Fd_Lambert();
#endif

#if @radialFog
    float fogValue = clamp((euclideanDepth - gl_Fog.start) * gl_Fog.scale, 0.0, 1.0);
#else
    float fogValue = clamp((linearDepth - gl_Fog.start) * gl_Fog.scale, 0.0, 1.0);
#endif
	
	float exposure = getExposure(length(SRGBToLinearApprox(lcalcDiffuse(0).xyz) + SRGBToLinearApprox(gl_LightModel.ambient.xyz)) * 0.5);
	gl_FragData[0].xyz = toScreen(gl_FragData[0].xyz, exposure);
    gl_FragData[0].xyz = mix(gl_FragData[0].xyz, gl_Fog.color.xyz, fogValue);

    applyShadowDebugOverlay();

gl_FragData[0].xyz = pow(gl_FragData[0].xyz, vec3(1.0/@gamma));

}
