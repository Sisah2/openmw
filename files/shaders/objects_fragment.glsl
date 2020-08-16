#version 120

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
varying vec4 passTangent;
#endif

#if @envMap
uniform sampler2D envMap;
varying vec2 envMapUV;
uniform vec4 envMapColor;
#endif

#if @specularMap
uniform sampler2D specularMap;
#endif

#if @bumpMap
uniform sampler2D bumpMap;
uniform vec2 envMapLumaBias;
uniform mat2 bumpMapMatrix;
#endif

#if @radialFog
uniform bool simpleWater;
#endif

varying float depth;

#define PER_PIXEL_LIGHTING (@normalMap || (@forcePPL && (@particleHandling <= 2)))

#if !PER_PIXEL_LIGHTING
centroid varying vec4 lighting;
#endif

centroid varying vec4 passColor;
varying vec3 passViewPos;
varying vec3 passNormal;

uniform int colorMode;
#if PER_PIXEL_LIGHTING
#include "lighting.glsl"
#endif

#include "parallax.glsl"

void main()
{
#if @diffuseMap
    vec2 adjustedDiffuseUV = diffuseMapUV;
#endif

#if (!@normalMap && (@parallax || @forcePPL))
    vec3 viewNormal = gl_NormalMatrix * normalize(passNormal);
#endif

#if @normalMap
    vec4 normalTex = texture2D(normalMap, diffuseMapUV);

    vec3 normalizedNormal = normalize(passNormal);
    vec3 normalizedTangent = normalize(passTangent.xyz);
    vec3 binormal = cross(normalizedTangent, normalizedNormal) * passTangent.w;
    mat3 tbnTranspose = mat3(normalizedTangent, binormal, normalizedNormal);

#if !@parallax
    vec3 viewNormal = gl_NormalMatrix * normalize(tbnTranspose * (normalTex.xyz * 2.0 - 1.0));
#else

    vec3 cameraPos = (gl_ModelViewMatrixInverse * vec4(0,0,0,1)).xyz;
    vec3 objectPos = (gl_ModelViewMatrixInverse * vec4(passViewPos, 1)).xyz;
    vec3 eyeDir = normalize(cameraPos - objectPos);
    adjustedDiffuseUV += getParallaxOffset(eyeDir, tbnTranspose, normalTex.a, (passTangent.w > 0.0) ? -1.f : 1.f);

    normalTex = texture2D(normalMap, adjustedDiffuseUV);
    vec3 viewNormal = gl_NormalMatrix * normalize(tbnTranspose * (normalTex.xyz * 2.0 - 1.0));
#endif
#endif

#if @diffuseMap
    gl_FragData[0] = texture2D(diffuseMap, adjustedDiffuseUV);
#else
    gl_FragData[0] = vec4(1.0);
#endif

#if @detailMap
    gl_FragData[0].xyz *= texture2D(detailMap, detailMapUV).xyz * 2.0;
#endif

#if @darkMap
    gl_FragData[0].xyz *= texture2D(darkMap, darkMapUV).xyz;
#endif

#if @decalMap
    vec4 decalTex = texture2D(decalMap, decalMapUV);
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
    vec4 bumpTex = texture2D(bumpMap, diffuseMapUV);
    envTexCoordGen += bumpTex.rg * bumpMapMatrix;
    envLuma = clamp(bumpTex.b * envMapLumaBias.x + envMapLumaBias.y, 0.0, 1.0);
#endif

#if @preLightEnv
    gl_FragData[0].xyz += texture2D(envMap, envTexCoordGen).xyz * envMapColor.xyz * envLuma;
#endif

#endif

#if !PER_PIXEL_LIGHTING
    gl_FragData[0] *= lighting;
#else
    gl_FragData[0] *= doLighting(passViewPos, normalize(viewNormal), passColor, false);
#endif

#if @envMap && !@preLightEnv
    gl_FragData[0].xyz += texture2D(envMap, envTexCoordGen).xyz * envMapColor.xyz * envLuma;
#endif

#if @emissiveMap
    gl_FragData[0].xyz += texture2D(emissiveMap, emissiveMapUV).xyz;
#endif

#if @specularMap
    vec4 specTex = texture2D(specularMap, diffuseMapUV);
    float shininess = specTex.a * 255.0;
    vec3 matSpec = specTex.xyz;
#else
    float shininess = gl_FrontMaterial.shininess;
    vec3 matSpec = (colorMode == 5) ? passColor.xyz : gl_FrontMaterial.specular.xyz;
#endif

    if (matSpec != vec3(0.0))
    {
#if (!@normalMap && !@parallax && !@forcePPL)
        vec3 viewNormal = gl_NormalMatrix * normalize(passNormal);
#endif
        gl_FragData[0].xyz += getSpecular(normalize(viewNormal), normalize(passViewPos.xyz), shininess, matSpec);
    }

#if @radialFog
    float fogDepth = (simpleWater) ? length(passViewPos) : depth;
#else
    float fogDepth = depth;
#endif
    float fogValue = clamp((fogDepth - gl_Fog.start) * gl_Fog.scale, 0.0, 1.0);
    gl_FragData[0].xyz = mix(gl_FragData[0].xyz, gl_Fog.color.xyz, fogValue);

#if (@gamma != 1000)
    gl_FragData[0].xyz = pow(gl_FragData[0].xyz, vec3(1.0/(@gamma.0/1000.0)));
#endif
}
