#version 120

#if @useUBO
    #extension GL_ARB_uniform_buffer_object : require
#endif

#if @useGPUShader4
    #extension GL_EXT_gpu_shader4: require
#endif

varying vec2 uv;

uniform sampler2D diffuseMap;

#if @normalMap
uniform sampler2D normalMap;
#endif

#if @blendMap
uniform sampler2D blendMap;
#endif

varying float euclideanDepth;
varying float linearDepth;

#define PER_PIXEL_LIGHTING (@normalMap || @forcePPL)

#if !PER_PIXEL_LIGHTING
centroid varying vec3 passLighting;
centroid varying vec3 shadowDiffuseLighting;
#endif
varying vec3 passViewPos;
varying vec3 passNormal;

#include "helperutil.glsl"
#include "vertexcolors.glsl"
#include "shadows_fragment.glsl"
#include "lighting.glsl"
#include "parallax.glsl"

void main()
{
    vec2 adjustedUV = (gl_TextureMatrix[0] * vec4(uv, 0.0, 1.0)).xy;

#if @normalMap
    vec4 normalTex = texture2D(normalMap, adjustedUV);

    vec3 normalizedNormal = normalize(passNormal);
    vec3 tangent = vec3(1.0, 0.0, 0.0);
    vec3 bitangent = normalize(cross(passNormal, tangent));
    vec3 binormal = normalize(cross(tangent, normalizedNormal));
    tangent = normalize(cross(normalizedNormal, binormal)); // note, now we need to re-cross to derive tangent again because it wasn't orthonormal
	
	mat3 tbnTranspose = mat3(tangent, binormal, normalizedNormal);
    mat3 tbnInverse = transpose2(gl_NormalMatrix * mat3(tangent, bitangent, normalizedNormal));

    vec3 viewNormal = normalize(gl_NormalMatrix * (tbnTranspose * (normalTex.xyz * 2.0 - 1.0)));
#endif

#if (!@normalMap && (@parallax || @forcePPL))
    vec3 viewNormal = gl_NormalMatrix * normalize(passNormal);
#endif

#if @parallax
	vec3 cameraPos = (gl_ModelViewMatrixInverse * vec4(0,0,0,1)).xyz;
    vec3 objectPos = (gl_ModelViewMatrixInverse * vec4(passViewPos, 1)).xyz;
	
	vec3 eyeDir = tbnTranspose * normalize(cameraPos - objectPos);
	
	vec3 lightdir = tbnInverse * normalize(vec4(lcalcPosition(0).xyz,1.0)).xyz;
	
	getParallaxOffset(adjustedUV, eyeDir, tbnInverse, normalMap, 1.f);
	
	vec2 shadowUV = adjustedUV;
    
    float h0 = 1.0 - normalTex.a;
    float h = h0;
    
    float dist = euclideanDepth*0.0001;
    float lod1 = 1.0 - step(0.1, dist);
    
    float shadowpara = 1.0;

	float soften = 5.0;
	
	
	
	vec2 lDir = (vec2(lightdir.x, lightdir.y)) * 0.04 * 0.75;
	
	h = min(1.0, 1.0 - texture2D(normalMap, shadowUV + lDir ).w);
	
	if(lod1 != 0.0)
	{
		h = min( h, 1.0 - texture2D(normalMap, shadowUV + 0.750 * lDir).w);
		h = min( h, 1.0 - texture2D(normalMap, shadowUV + 0.500 * lDir).w);
		h = min( h, 1.0 - texture2D(normalMap, shadowUV + 0.250 * lDir).w);
	}
	shadowpara =  min(shadowpara, 1.0 - saturate((h0 - h) * soften));

    // update normal using new coordinates
    normalTex = texture2D(normalMap, adjustedUV);
    viewNormal = normalize(gl_NormalMatrix * (tbnTranspose * (normalTex.xyz * 2.0 - 1.0)));
#endif

    vec4 diffuseTex = texture2D(diffuseMap, adjustedUV);
	diffuseTex.xyz = texLoad(diffuseTex.xyz);
	#ifdef DEBUGLIGHTING
		#ifdef LINEAR
		diffuseTex.rgb = vec3(0.5 * 0.5);
		#else
		diffuseTex.rgb = vec3(0.5);
		#endif
	#endif
    gl_FragData[0] = vec4(diffuseTex.xyz, 1.0);

#if @blendMap
    vec2 blendMapUV = (gl_TextureMatrix[1] * vec4(uv, 0.0, 1.0)).xy;
    gl_FragData[0].a *= texture2D(blendMap, blendMapUV).a;
#endif

    vec4 diffuseColor = getDiffuseColor();
	diffuseColor.rgb = colLoad(diffuseColor.rgb);
	
    gl_FragData[0].a *= diffuseColor.a;

    float shadowing = unshadowedLightRatio(linearDepth);
	
	#if @parallax
	shadowing *= shadowpara;
	#endif

    vec3 lighting;
#if !PER_PIXEL_LIGHTING
    lighting = passLighting + shadowDiffuseLighting * shadowing;
	gl_FragData[0].xyz *= lighting * Fd_Lambert();										   
#else
	vec4 param = vec4(0.0, 0.98, 0.5, 1.0);
	#if @specularMap
	param.y = diffuseTex.a;
	#endif
    vec3 diffuseLight, ambientLight,specularLight;
    doLighting(passViewPos, normalize(viewNormal), param, shadowing, diffuseLight, ambientLight, specularLight);
    lighting = diffuseColor.xyz * diffuseLight * Fd_Lambert() + vcolLoad(getAmbientColor().xyz) * ambientLight * Fd_Lambert() + colLoad(getEmissionColor().xyz);
    //clampLightingResult(lighting);
	#ifdef PBRDEBUG
	gl_FragData[0].xyz = specularLight;
	#else
	gl_FragData[0].xyz = gl_FragData[0].xyz * lighting + specularLight * vcolLoad(getAmbientColor().xyz);	
	#endif
#endif

#if @specularMap
    float shininess = 128.0; // TODO: make configurable
    vec3 matSpec = vec3(diffuseTex.a);
#else
    float shininess = gl_FrontMaterial.shininess;
    vec3 matSpec = getSpecularColor().xyz;
#endif

    if (matSpec != vec3(0.0))
    {
#if (!@normalMap && !@parallax && !@forcePPL)
        vec3 viewNormal = gl_NormalMatrix * normalize(passNormal);
#endif
        //gl_FragData[0].xyz += getSpecular(normalize(viewNormal), normalize(passViewPos), shininess, matSpec) * shadowing;
    }

#if @radialFog
    float fogValue = clamp((euclideanDepth - gl_Fog.start) * gl_Fog.scale, 0.0, 1.0);
#else
    float fogValue = clamp((linearDepth - gl_Fog.start) * gl_Fog.scale, 0.0, 1.0);
#endif

	float exposure = getExposure(length(SRGBToLinearApprox(lcalcDiffuse(0).xyz) + SRGBToLinearApprox(gl_LightModel.ambient.xyz)) * 0.5);
	
	#ifdef PBRDEBUG
	gl_FragData[0].xyz *= 1.0;
	#else
	gl_FragData[0].xyz = toScreen(gl_FragData[0].xyz, exposure);
	#endif
    gl_FragData[0].xyz = mix(gl_FragData[0].xyz, gl_Fog.color.xyz, fogValue);

    applyShadowDebugOverlay();

gl_FragData[0].xyz = pow(gl_FragData[0].xyz, vec3(1.0/@gamma));

}
