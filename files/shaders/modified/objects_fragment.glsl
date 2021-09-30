#version 120

#define OBJECT

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

#define PER_PIXEL_LIGHTING (@normalMap || (@forcePPL))

uniform vec4 shaderSettings;
#include "tonemap.glsl"

#include "helpsettings.glsl"

uniform bool isInterior;

#include "vertexcolors.glsl"
#include "lighting_util.glsl"

uniform bool simpleWater;

uniform bool skip;

uniform mat4 osg_ViewMatrixInverse;
uniform bool isPlayer;

#ifdef ANIMATED_HEIGHT_FOG
uniform float osg_SimulationTime;
#endif

#if !PER_PIXEL_LIGHTING
centroid varying vec3 passLighting;
#endif

#if PER_PIXEL_LIGHTING || @specularMap
varying vec3 passNormal;
#endif

varying vec3 passViewPos;

#if @translucentFramebuffer
uniform bool noAlpha;
#endif

varying float depth;

#if PER_PIXEL_LIGHTING
  #ifdef LINEAR_LIGHTING
    #include "linear_lighting.glsl"
  #else
    #include "lighting.glsl"
  #endif
#endif

#include "effects.glsl"
#include "fog.glsl"
#include "alpha.glsl"

void main()
{
    bool underwaterFog = (shaderSettings.y == 1.0 || shaderSettings.y == 3.0 || shaderSettings.y == 5.0 || shaderSettings.y == 7.0) ? true : false;
    bool radialFog = (shaderSettings.y == 2.0 || shaderSettings.y == 3.0 || shaderSettings.y == 6.0 || shaderSettings.y == 7.0) ? true : false;
    bool clampLighting = (shaderSettings.y == 4.0 || shaderSettings.y == 5.0 || shaderSettings.y == 6.0 || shaderSettings.y == 7.0) ? true : false;

    bool isUnderwater = (osg_ViewMatrixInverse * vec4(passViewPos, 1.0)).z < -1.0 && osg_ViewMatrixInverse[3].z > -1.0 && !simpleWater && !skip && !isInterior && !isPlayer;
    float underwaterFogValue = (isUnderwater) ? getUnderwaterFogValue(depth) : 0.0;

    float fogValue = radialFog ? getFogValue((simpleWater) ? length(passViewPos) : depth) : getFogValue(depth);

if(fogValue < 1.0 && underwaterFogValue < 1.0)
{

float shadowpara = 1.0;

#if @diffuseMap
    vec2 adjustedDiffuseUV = diffuseMapUV;
#endif

#if (!@normalMap && (@specularMap || @forcePPL))
    vec3 viewNormal = gl_NormalMatrix * normalize(passNormal);
#endif

#ifdef NORMAL_MAP_FADING
    float nmFade = smoothstep(nmfader.x, nmfader.y, depth);
#endif

#if @normalMap
    vec3 viewNormal;

    #ifdef NORMAL_MAP_FADING
        if(nmFade < 1.0 && !skip){
    #endif

    vec4 normalTex = texture2D(normalMap, diffuseMapUV);

    #ifdef NORMAL_MAP_FADING
       if(nmFade != 0.0) normalTex = mix(normalTex, vec4(0.5, 0.5, 1.0, 0.5), nmFade);
    #endif

    vec3 normalizedNormal = normalize(passNormal);
    vec3 normalizedTangent = normalize(passTangent.xyz);
    vec3 binormal = cross(normalizedTangent, normalizedNormal) * passTangent.w;
    mat3 tbnTranspose = mat3(normalizedTangent, binormal, normalizedNormal);

#if !@parallax
    viewNormal = gl_NormalMatrix * normalize(tbnTranspose * (normalTex.xyz * 2.0 - 1.0));
    #ifdef NORMAL_MAP_FADING
        if(nmFade != 0.0) viewNormal = mix(viewNormal, gl_NormalMatrix * normalize(passNormal), nmFade);
    #endif
#else
    vec3 cameraPos = (gl_ModelViewMatrixInverse * vec4(0,0,0,1)).xyz;
    vec3 objectPos = (gl_ModelViewMatrixInverse * vec4(passViewPos, 1)).xyz;
    vec3 eyeDir = normalize(cameraPos - objectPos);
    adjustedDiffuseUV += getParallaxOffset(eyeDir, tbnTranspose, normalTex.a, (passTangent.w > 0.0) ? -1.f : 1.f);

    if(shaderSettings.z != 0.0){
        shadowpara = getParallaxShadow(normalTex.a, adjustedDiffuseUV);
        #ifdef NORMAL_MAP_FADING
            if(nmFade != 0.0) shadowpara = mix(shadowpara, 1.0, nmFade);
        #endif
    }

    //normalTex = texture2D(normalMap, adjustedDiffuseUV);
    viewNormal = gl_NormalMatrix * normalize(tbnTranspose * (normalTex.xyz * 2.0 - 1.0));
    #ifdef NORMAL_MAP_FADING
        if(nmFade != 0.0) viewNormal = mix(viewNormal, gl_NormalMatrix * normalize(passNormal), nmFade);
    #endif
#endif
    #ifdef NORMAL_MAP_FADING
        }
        else
        viewNormal = gl_NormalMatrix * normalize(passNormal);
    #endif
#endif

#if @diffuseMap
    gl_FragData[0] = texture2D(diffuseMap, adjustedDiffuseUV);
    gl_FragData[0].a *= coveragePreservingAlphaScale(diffuseMap, adjustedDiffuseUV);
#else
    gl_FragData[0] = vec4(1.0);
#endif

    vec4 diffuseColor = getDiffuseColor();
    gl_FragData[0].a *= diffuseColor.a;

#if @darkMap
    gl_FragData[0] *= texture2D(darkMap, darkMapUV);
    gl_FragData[0].a *= coveragePreservingAlphaScale(darkMap, darkMapUV);
#endif

    alphaTest();

if(gl_FragData[0].a != 0.0)
{

#if @detailMap
    gl_FragData[0].xyz *= texture2D(detailMap, detailMapUV).xyz * 2.0;
#endif

#if @decalMap
    vec4 decalTex = texture2D(decalMap, decalMapUV);
    gl_FragData[0].xyz = mix(gl_FragData[0].xyz, decalTex.xyz, decalTex.a);
#endif

#if @envMap

    #ifdef NORMAL_MAP_FADING
        if(nmFade < 1.0 && !skip){
    #endif

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
    #ifdef NORMAL_MAP_FADING
       if(nmFade != 0.0) bumpTex = mix(bumpTex, vec4(0.0, 0.0, 0.0, 1.0), nmFade);
    #endif
    envTexCoordGen += bumpTex.rg * bumpMapMatrix;
    envLuma = clamp(bumpTex.b * envMapLumaBias.x + envMapLumaBias.y, 0.0, 1.0);
#endif

    #ifdef NORMAL_MAP_FADING
        if(nmFade != 0.0) gl_FragData[0].xyz += mix(texture2D(envMap, envTexCoordGen).xyz * envMapColor.xyz * envLuma, vec3( 0.0, 0.0, 0.0), nmFade);
            else
    #endif
        gl_FragData[0].xyz += texture2D(envMap, envTexCoordGen).xyz * envMapColor.xyz * envLuma;

    #ifdef NORMAL_MAP_FADING
        }
    #endif

#endif

    gl_FragData[0].xyz = preLight(gl_FragData[0].xyz);

    vec3 lighting;
#if !PER_PIXEL_LIGHTING
    lighting = passLighting;
#else
#ifdef LINEAR_LIGHTING
    lighting = doLighting(passViewPos, normalize(viewNormal), passColor, shadowpara);
#else
    vec3 diffuseLight, ambientLight;
    doLighting(passViewPos, normalize(viewNormal), shadowpara, diffuseLight, ambientLight);
    lighting = diffuseColor.xyz * diffuseLight + getAmbientColor().xyz * ambientLight + getEmissionColor().xyz;
#endif
    clampLightingResult(lighting, clampLighting);
#endif

gl_FragData[0].xyz *= lighting;


#if @emissiveMap
    gl_FragData[0].xyz += pow(texture2D(emissiveMap, diffuseMapUV).xyz, vec3(2.2));
#endif

#if @specularMap
    #ifdef NORMAL_MAP_FADING
    if(nmFade < 1.0 && !skip) {
        vec4 specTex = texture2D(specularMap, diffuseMapUV);
        float shininess = (1.0-(nmFade*0.5)) * (specTex.a * 255.0);
        vec3 matSpec = mix(specTex.xyz, vec3(0.0, 0.0, 0.0), nmFade);
        gl_FragData[0].xyz += getSpecular(normalize(viewNormal), normalize(passViewPos.xyz), shininess, matSpec) * shadowpara;
    }
    #else
        vec4 specTex = texture2D(specularMap, diffuseMapUV);
        float shininess = specTex.a * 255.0;
        vec3 matSpec = specTex.xyz;
        gl_FragData[0].xyz += getSpecular(normalize(viewNormal), normalize(passViewPos.xyz), shininess, matSpec) * shadowpara;
    #endif
#endif

   gl_FragData[0].xyz = toneMap(gl_FragData[0].xyz);

#ifdef LINEAR_LIGHTING
        gl_FragData[0].xyz = SpecialContrast(gl_FragData[0].xyz, mix(connight, conday, lcalcDiffuse(0).x));
#endif

}

#if @translucentFramebuffer
// having testing & blending isn't enough - we need to write an opaque pixel to be opaque
    if (noAlpha)
         gl_FragData[0].a = 1.0;
#endif
 
}

if(underwaterFog)
    gl_FragData[0].xyz = mix(gl_FragData[0].xyz, uwfogcolor, underwaterFogValue);

if(!isUnderwater)
    gl_FragData[0].xyz = mix(gl_FragData[0].xyz, gl_Fog.color.xyz, fogValue);

    gl_FragData[0].xyz = pow(gl_FragData[0].xyz, vec3(1.0/shaderSettings.w));
}
