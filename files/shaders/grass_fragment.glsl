#version 120

#define GRASS

#if @diffuseMap
uniform sampler2D diffuseMap;
varying vec2 diffuseMapUV;
#endif

#define PER_PIXEL_LIGHTING @forcePPL

varying float euclideanDepth;
varying float linearDepth;

#if !@forcePPL
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

#if @forcePPL
    vec3 viewNormal = gl_NormalMatrix * normalize(passNormal);
#endif

#if @diffuseMap
    gl_FragData[0] = texture2D(diffuseMap, adjustedDiffuseUV);
#else
    gl_FragData[0] = vec4(1.0);
#endif

    if (euclideanDepth > @grassFadeStart)
        gl_FragData[0].a *= 1.0-smoothstep(@grassFadeStart, @grassFadeEnd, euclideanDepth);

#if !@forcePPL
    gl_FragData[0] *= lighting;
#else
    if(gl_FragData[0].a != 0.0)
        gl_FragData[0] *= doLighting(passViewPos, normalize(viewNormal), passColor);
#endif

    float shininess = gl_FrontMaterial.shininess;
    vec3 matSpec;
    if (colorMode == 5)
        matSpec = passColor.xyz;
    else
        matSpec = gl_FrontMaterial.specular.xyz;

    if (matSpec != vec3(0.0))
    {
#if !@forcePPL
        vec3 viewNormal = gl_NormalMatrix * normalize(passNormal);
#endif
        gl_FragData[0].xyz += getSpecular(normalize(viewNormal), normalize(passViewPos.xyz), shininess, matSpec);
    }
#if @radialFog
    float fogValue = clamp((euclideanDepth - gl_Fog.start) * gl_Fog.scale, 0.0, 1.0);
#else
    float fogValue = clamp((linearDepth - gl_Fog.start) * gl_Fog.scale, 0.0, 1.0);
#endif
    gl_FragData[0].xyz = mix(gl_FragData[0].xyz, gl_Fog.color.xyz, fogValue);

#if (@gamma != 1000)
    gl_FragData[0].xyz = pow(gl_FragData[0].xyz, vec3(1.0/(@gamma.0/1000.0)));
#endif
}
