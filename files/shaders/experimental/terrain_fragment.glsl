#version 120

#define TERRAIN
#define PER_PIXEL_LIGHTING (@normalMap || @forcePPL)

#if @normalMap
uniform sampler2D normalMap;
#endif

#if @blendMap
uniform sampler2D blendMap;
#endif

#ifdef ANIMATED_HEIGHT_FOG
uniform float osg_SimulationTime;
#endif

varying vec2 uv;
uniform sampler2D diffuseMap;
varying highp float depth;
uniform highp mat4 osg_ViewMatrixInverse;
uniform bool skip;
varying vec3 passViewPos;

uniform bool parallaxShadows;
uniform bool underwaterFog;
uniform float gamma;

#include "helpsettings.glsl"
#include "tonemap.glsl"
#include "vertexcolors.glsl"
#include "lighting_util.glsl"
#include "effects.glsl"
#include "fog.glsl"

#if (PER_PIXEL_LIGHTING || @specularMap || defined(HEIGHT_FOG))
    varying vec3 passNormal;
#endif

#if !PER_PIXEL_LIGHTING
    centroid varying vec3 passLighting;
#else
    #include "lighting.glsl"
#endif


void main()
{
float underwaterFogValue;
if(underwaterFog) {
    bool isUnderwater = (osg_ViewMatrixInverse * vec4(passViewPos, 1.0)).z < -1.0 && osg_ViewMatrixInverse[3].z >= -1.0 && !skip;
    underwaterFogValue = (isUnderwater) ? getUnderwaterFogValue(depth) : 0.0;
}

    float fogValue = getFogValue(depth);

//if(fogValue != 1.0 && underwaterFogValue != 1.0)
{

    float shadowpara = 1.0;

    vec2 adjustedUV = (gl_TextureMatrix[0] * vec4(uv, 0.0, 1.0)).xy;

#if ((!@normalMap && @forcePPL) || (@normalMap && defined(NORMAL_MAP_FADING)) || @specularMap)
    vec3 viewNormal = gl_NormalMatrix * normalize(passNormal);
#endif

#if @normalMap
    #ifdef NORMAL_MAP_FADING
        if(!skip) {
    #endif
    vec4 normalTex = texture2D(normalMap, adjustedUV);

    vec3 normalizedNormal = normalize(passNormal);
    vec3 tangent = vec3(1.0, 0.0, 0.0);
    vec3 binormal = normalize(cross(tangent, normalizedNormal));
    tangent = normalize(cross(normalizedNormal, binormal)); // note, now we need to re-cross to derive tangent again because it wasn't orthonormal
    mat3 tbnTranspose = mat3(tangent, binormal, normalizedNormal);

#if !@parallax
    vec3 viewNormal = normalize(gl_NormalMatrix * (tbnTranspose * (normalTex.xyz * 2.0 - 1.0)));
#else
    vec3 cameraPos = (gl_ModelViewMatrixInverse * vec4(0,0,0,1)).xyz;
    vec3 objectPos = (gl_ModelViewMatrixInverse * vec4(passViewPos, 1)).xyz;
    vec3 eyeDir = normalize(cameraPos - objectPos);

    adjustedUV += getParallaxOffset(eyeDir, tbnTranspose, normalTex.a, 1.f);


    if(parallaxShadows) {
        //shadowpara = getParallaxShadow(normalTex.a, adjustedUV);

        vec3 bitangent = normalize(cross(passNormal, tangent));
        mat3 tbnInverse = transpose2(gl_NormalMatrix * mat3(tangent, bitangent, normalizedNormal));
        shadowpara = getParallaxShadow2(normalTex.a, adjustedUV, normalMap, tbnInverse);
    }


    // update normal using new coordinates
    normalTex = texture2D(normalMap, adjustedUV);
    vec3 viewNormal = normalize(gl_NormalMatrix * (tbnTranspose * (normalTex.xyz * 2.0 - 1.0)));
#endif
    #ifdef NORMAL_MAP_FADING
        }
    #endif
#endif

    vec4 diffuseTex = texture2D(diffuseMap, adjustedUV);
    gl_FragData[0] = vec4(diffuseTex.xyz, 1.0);

#if @blendMap
    vec2 blendMapUV = (gl_TextureMatrix[1] * vec4(uv, 0.0, 1.0)).xy;
    gl_FragData[0].a *= texture2D(blendMap, blendMapUV).a;
#endif

    gl_FragData[0].xyz = texLoad(gl_FragData[0].xyz);

    vec4 diffuseColor = getDiffuseColor();
    diffuseColor.rgb = colLoad(diffuseColor.rgb);
    gl_FragData[0].a *= diffuseColor.a;

    vec3 lighting;

#if !PER_PIXEL_LIGHTING
    lighting = passLighting * Fd_Lambert();
#else
    vec3 diffuseLight, ambientLight;
    doLighting(passViewPos, normalize(viewNormal), shadowpara, diffuseLight, ambientLight);
    lighting = diffuseColor.xyz * diffuseLight * Fd_Lambert() + vcolLoad(getAmbientColor().xyz) * ambientLight * Fd_Lambert() + colLoad(getEmissionColor().xyz);
    clampLightingResult(lighting);
#endif

#if @linearLighting
    gl_FragData[0].xyz *= lighting * vcolLoad(getAmbientColor().xyz);
#else
    gl_FragData[0].xyz *= lighting;
#endif

#if @specularMap
    float shininess = 128.0; // TODO: make configurable
    vec3 matSpec = vec3(diffuseTex.a);
    gl_FragData[0].xyz += getSpecular(normalize(viewNormal), normalize(passViewPos), shininess, matSpec) * shadowpara;
#endif

      float exposure = getExposure(length(colLoad(lcalcDiffuse(0).xyz) + colLoad(gl_LightModel.ambient.xyz)) * 0.5);
      gl_FragData[0].xyz = toneMap(gl_FragData[0].xyz, exposure);

}
/*
else
{
#if @blendMap // what?
     gl_FragData[0].a = texture2D(blendMap, (gl_TextureMatrix[1] * vec4(uv, 0.0, 1.0)).xy).a;
#endif
}
*/

    if(underwaterFog)
        gl_FragData[0].xyz = mix(gl_FragData[0].xyz, uwfogcolor, underwaterFogValue);

    gl_FragData[0].xyz = mix(gl_FragData[0].xyz, gl_Fog.color.xyz, fogValue);

    gl_FragData[0].xyz = pow(gl_FragData[0].xyz, vec3(1.0/ (@gamma + gamma - 1.0)));
}
