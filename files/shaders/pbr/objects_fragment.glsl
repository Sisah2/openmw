#version 120
#pragma import_defines(FORCE_OPAQUE)

#if @useUBO
    #extension GL_ARB_uniform_buffer_object : require
#endif

#if @useGPUShader4
    #extension GL_EXT_gpu_shader4: require
#endif

#if @diffuseMap
uniform sampler2D diffuseMap;
varying vec2 diffuseMapUV;
#endif

#if @darkMap
uniform sampler2D darkMap;
varying vec2 darkMapUV;
#endif

#if @detailMap
uniform sampler2D detailMap;
varying vec2 detailMapUV;
#endif

#if @decalMap
uniform sampler2D decalMap;
varying vec2 decalMapUV;
#endif

#if @emissiveMap
uniform sampler2D emissiveMap;
varying vec2 emissiveMapUV;
#endif

#if @normalMap
uniform sampler2D normalMap;
varying vec2 normalMapUV;
varying vec4 passTangent;
#endif

#if @envMap
uniform sampler2D envMap;
varying vec2 envMapUV;
uniform vec4 envMapColor;
#endif

#if @specularMap
uniform sampler2D specularMap;
varying vec2 specularMapUV;
#endif

#if @bumpMap
uniform sampler2D bumpMap;
varying vec2 bumpMapUV;
uniform vec2 envMapLumaBias;
uniform mat2 bumpMapMatrix;
#endif

uniform bool simpleWater;

varying float euclideanDepth;
varying float linearDepth;

#define PER_PIXEL_LIGHTING (@normalMap || @forcePPL)

#if !PER_PIXEL_LIGHTING
centroid varying vec3 passLighting;
centroid varying vec3 shadowDiffuseLighting;
#else
uniform float emissiveMult;
#endif
varying vec3 passViewPos;
varying vec3 passNormal;

#include "helperutil.glsl"
#include "vertexcolors.glsl"
#include "shadows_fragment.glsl"
#include "lighting.glsl"
#include "parallax.glsl"
#include "alpha.glsl"

void main()
{
#if @diffuseMap
    vec2 adjustedUV = diffuseMapUV;
#endif

#if @normalMap
    vec4 normalTex = texture2D(normalMap, normalMapUV);

    vec3 normalizedNormal = normalize(passNormal);
    vec3 normalizedTangent = normalize(passTangent.xyz);
    vec3 bitangent = normalize(cross(passNormal, passTangent.xyz) * passTangent.w);
    vec3 binormal = cross(normalizedTangent, normalizedNormal) * passTangent.w;
	mat3 tbnTranspose = mat3(normalizedTangent, binormal, normalizedNormal);
    mat3 tbnInverse = transpose2(gl_NormalMatrix * mat3(normalizedTangent, bitangent, normalizedNormal));
	
	

    vec3 viewNormal = gl_NormalMatrix * normalize(tbnTranspose * (normalTex.xyz * 2.0 - 1.0));
#endif

#if (!@normalMap && (@parallax || @forcePPL))
    vec3 viewNormal = gl_NormalMatrix * normalize(passNormal);
#endif

#if @parallax
	vec3 eyeDir = tbnInverse * -normalize(passViewPos);
	
	vec3 lightdir = tbnInverse * normalize(vec4(lcalcPosition(0).xyz,1.0)).xyz;
	
	
	float flip = 1.0; //(passTangent.w > 0.0) ? -1.f : 1.f;
	
	
    getParallaxOffset(adjustedUV, eyeDir, tbnInverse, normalMap, flip);
	
	
	vec2 shadowUV = adjustedUV;
    
    float h0 = 1.0 - normalTex.a;
    float h = h0;
    
    float dist = euclideanDepth*0.0001;
    float lod1 = 1.0 - step(0.1, dist);
    
    float shadowpara = 1.0;

	float soften = 5.0;
	
	
	
	vec2 lDir = (vec2(lightdir.x, lightdir.y * flip)) * 0.04 * 0.75;
	
	h = min(1.0, 1.0 - texture2D(normalMap, shadowUV + lDir ).w);
	
	if(lod1 != 0.0)
	{
		h = min( h, 1.0 - texture2D(normalMap, shadowUV + 0.750 * lDir).w);
		h = min( h, 1.0 - texture2D(normalMap, shadowUV + 0.500 * lDir).w);
		h = min( h, 1.0 - texture2D(normalMap, shadowUV + 0.250 * lDir).w);
	}
	shadowpara =  min(shadowpara, 1.0 - saturate((h0 - h) * soften));
	
    //adjustedUV += offset; // only offset diffuse for now, other textures are more likely to be using a completely different UV set

    // TODO: check not working as the same UV buffer is being bound to different targets
    // if diffuseMapUV == normalMapUV
#if 1
    // fetch a new normal using updated coordinates
    normalTex = texture2D(normalMap, adjustedUV);
    viewNormal = gl_NormalMatrix * normalize(tbnTranspose * (normalTex.xyz * 2.0 - 1.0));
#endif

#endif

#if @diffuseMap
    gl_FragData[0] = texture2D(diffuseMap, adjustedUV);
	gl_FragData[0].rgb = texLoad(gl_FragData[0].rgb);
	#ifdef DEBUGLIGHTING
		#ifdef LINEAR
		gl_FragData[0].rgb = vec3(0.5 * 0.5);
		#else
		gl_FragData[0].rgb = vec3(0.5);
		#endif
	#endif
    gl_FragData[0].a *= coveragePreservingAlphaScale(diffuseMap, adjustedUV);
#else
    gl_FragData[0] = vec4(1.0);
#endif

    vec4 diffuseColor = getDiffuseColor();
	diffuseColor.rgb = colLoad(diffuseColor.rgb);
    gl_FragData[0].a *= diffuseColor.a;

#if @darkMap
    gl_FragData[0] *= texture2D(darkMap, darkMapUV);
    gl_FragData[0].a *= coveragePreservingAlphaScale(darkMap, darkMapUV);
#endif

    alphaTest();

#if @detailMap
    gl_FragData[0].xyz *= texture2D(detailMap, detailMapUV).xyz * 2.0;
#endif

#if @decalMap
    vec4 decalTex = texture2D(decalMap, decalMapUV);
	decalTex.rgb = texLoad(decalTex.rgb);			   
    gl_FragData[0].xyz = mix(gl_FragData[0].xyz, decalTex.xyz, decalTex.a);
#endif

#if @envMap

    vec2 envTexCoordGen = envMapUV;
    float envLuma = 1.0;

#if @normalMap
    // if using normal map + env map, take advantage of per-pixel normals for envTexCoordGen
    vec3 viewVec = normalize(passViewPos.xyz);
    vec3 r = reflect( viewVec, viewNormal );
    float m = 2.0 * sqrt( r.x*r.x + r.y*r.y + (r.z+1.0)*(r.z+1.0) );
    envTexCoordGen = vec2(r.x/m + 0.5, r.y/m + 0.5);
#endif

#if @bumpMap
    vec4 bumpTex = texture2D(bumpMap, bumpMapUV);
    envTexCoordGen += bumpTex.rg * bumpMapMatrix;
    envLuma = clamp(bumpTex.b * envMapLumaBias.x + envMapLumaBias.y, 0.0, 1.0);
#endif

#if @preLightEnv
    gl_FragData[0].xyz += texture2D(envMap, envTexCoordGen).xyz * envMapColor.xyz * envLuma;
#endif

#endif

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
	param = texture2D(specularMap, adjustedUV);
	#endif
    vec3 diffuseLight, ambientLight, specularLight;
    doLighting(passViewPos, normalize(viewNormal), param, shadowing, diffuseLight, ambientLight, specularLight);
    vec3 emission = colLoad(getEmissionColor().xyz) * emissiveMult;
    lighting = diffuseColor.xyz * diffuseLight * Fd_Lambert() + vcolLoad(getAmbientColor().xyz) * ambientLight * Fd_Lambert() + emission;
    //clampLightingResult(lighting);
	
	#ifdef PBRDEBUG
	gl_FragData[0].xyz = specularLight;
	#else
	gl_FragData[0].xyz = gl_FragData[0].xyz * lighting + specularLight * vcolLoad(getAmbientColor().xyz);	
	#endif
#endif



#if @envMap && !@preLightEnv
    gl_FragData[0].xyz += texLoad(texture2D(envMap, envTexCoordGen).xyz) * envMapColor.xyz * envLuma;
#endif

#if @emissiveMap
    gl_FragData[0].xyz += texLoad(texture2D(emissiveMap, emissiveMapUV).xyz);
#endif

#if @specularMap
    vec4 specTex = texture2D(specularMap, specularMapUV);
    float shininess = specTex.a * 255.0;
    vec3 matSpec = specTex.xyz;
#else
    float shininess = gl_FrontMaterial.shininess;
    vec3 matSpec = getSpecularColor().xyz;
#endif

    if (matSpec != vec3(0.0))
    {
#if (!@normalMap && !@parallax && !@forcePPL)
        vec3 viewNormal = gl_NormalMatrix * normalize(passNormal);
#endif
        //gl_FragData[0].xyz += getSpecular(normalize(viewNormal), normalize(passViewPos.xyz), shininess, matSpec) * shadowing;
    }
#if @radialFog
    float depth;
    // For the less detailed mesh of simple water we need to recalculate depth on per-pixel basis
    if (simpleWater)
        depth = length(passViewPos);
    else
        depth = euclideanDepth;
    float fogValue = clamp((depth - gl_Fog.start) * gl_Fog.scale, 0.0, 1.0);
#else
    float fogValue = clamp((linearDepth - gl_Fog.start) * gl_Fog.scale, 0.0, 1.0);
#endif
	
	float exposure = getExposure(length(SRGBToLinearApprox(lcalcDiffuse(0).xyz) + SRGBToLinearApprox(gl_LightModel.ambient.xyz)) * 0.5);
	
	// spare maps and paper toy
    #if defined(FORCE_OPAQUE) && FORCE_OPAQUE
        exposure = 1.0;
    #endif
    
	if (gl_Fog.start > 9000000.0) {
		exposure = 1.0;
	}
	
	#ifdef PBRDEBUG
	gl_FragData[0].xyz *= 1.0;
	#else
	gl_FragData[0].xyz = toScreen(gl_FragData[0].xyz, exposure);
	#endif
	
    gl_FragData[0].xyz = mix(gl_FragData[0].xyz, gl_Fog.color.xyz, fogValue);

#if defined(FORCE_OPAQUE) && FORCE_OPAQUE
    // having testing & blending isn't enough - we need to write an opaque pixel to be opaque
        gl_FragData[0].a = 1.0;
#endif

    applyShadowDebugOverlay();

gl_FragData[0].xyz = pow(gl_FragData[0].xyz, vec3(1.0/@gamma));

}
