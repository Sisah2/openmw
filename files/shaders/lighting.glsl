#if !@ffpLighting

#include "sun.glsl"

#define getLight PointLights

struct PointLight
{
    vec4 diffuse;
    vec4 ambient;
    vec4 position;
    vec4 attenuation;
};

uniform int PointLightCount;
uniform int PointLightIndex[@maxLights];

layout(std140) uniform PointLightBuffer
{
    PointLight PointLights[@maxLightsInScene];
};

#else
#define getLight gl_LightSource
#endif

void perLightSun(out vec3 ambientOut, out vec3 diffuseOut, vec3 viewPos, vec3 viewNormal)
{
    vec3 lightDir = @sunDirection.xyz;
    lightDir = normalize(lightDir);

    ambientOut = @sunAmbient.xyz;

    float lambert = dot(viewNormal.xyz, lightDir);
#ifndef GROUNDCOVER
    lambert = max(lambert, 0.0);
#else
    float eyeCosine = dot(normalize(viewPos), viewNormal.xyz);
    if (lambert < 0.0)
    {
        lambert = -lambert;
        eyeCosine = -eyeCosine;
    }
    lambert *= clamp(-8.0 * (1.0 - 0.3) * eyeCosine + 1.0, 0.3, 1.0);
#endif
    diffuseOut = @sunDiffuse.xyz * lambert;
}

void perLightPoint(out vec3 ambientOut, out vec3 diffuseOut, int lightIndex, vec3 viewPos, vec3 viewNormal)
{
    vec3 lightDir = getLight[lightIndex].position.xyz - viewPos;

    float lightDistance = length(lightDir);
    lightDir = normalize(lightDir);

#if @ffpLighting
    float illumination = clamp(1.0 / (getLight[lightIndex].constantAttenuation + getLight[lightIndex].linearAttenuation * lightDistance + getLight[lightIndex].quadraticAttenuation * lightDistance * lightDistance), 0.0, 1.0);
#else 
    float illumination = clamp(1.0 / (getLight[lightIndex].attenuation.x + getLight[lightIndex].attenuation.y * lightDistance + getLight[lightIndex].attenuation.z * lightDistance * lightDistance), 0, 1);
#endif

    ambientOut = getLight[lightIndex].ambient.xyz * illumination;

    float lambert = dot(viewNormal.xyz, lightDir) * illumination;
#ifndef GROUNDCOVER
    lambert = max(lambert, 0.0);
#else
    float eyeCosine = dot(normalize(viewPos), viewNormal.xyz);
    if (lambert < 0.0)
    {
        lambert = -lambert;
        eyeCosine = -eyeCosine;
    }
    lambert *= clamp(-8.0 * (1.0 - 0.3) * eyeCosine + 1.0, 0.3, 1.0);
#endif

    diffuseOut = getLight[lightIndex].diffuse.xyz * lambert;
}

#if PER_PIXEL_LIGHTING
void doLighting(vec3 viewPos, vec3 viewNormal, float shadowing, out vec3 diffuseLight, out vec3 ambientLight)
#else
void doLighting(vec3 viewPos, vec3 viewNormal, out vec3 diffuseLight, out vec3 ambientLight, out vec3 shadowDiffuse)
#endif
{
    vec3 ambientOut, diffuseOut;
    // This light gets added a second time in the loop to fix Mesa users' slowdown, so we need to negate its contribution here.
    perLightSun(ambientOut, diffuseOut, viewPos, viewNormal);

#if PER_PIXEL_LIGHTING
    diffuseLight = diffuseOut * shadowing - diffuseOut;
#else
    shadowDiffuse = diffuseOut;
    diffuseLight = -diffuseOut;
#endif
    ambientLight = gl_LightModel.ambient.xyz;

#if !@ffpLighting
    perLightSun(ambientOut, diffuseOut, viewPos, viewNormal);
    ambientLight += ambientOut;
    diffuseLight += diffuseOut;
    for (int i=0; i<PointLightCount; ++i)
    {
        perLightPoint(ambientOut, diffuseOut, PointLightIndex[i], viewPos, viewNormal);
#else
    for (int i=0; i<@maxLights; ++i)
    {
        perLightPoint(ambientOut, diffuseOut, i, viewPos, viewNormal);
#endif
        ambientLight += ambientOut;
        diffuseLight += diffuseOut;
    }
}

vec3 getSpecular(vec3 viewNormal, vec3 viewDirection, float shininess, vec3 matSpec)
{
    vec3 lightDir = normalize(@sunDirection.xyz);
    float NdotL = dot(viewNormal, lightDir);
    if (NdotL <= 0.0)
        return vec3(0.0);
    vec3 halfVec = normalize(lightDir - viewDirection);
    float NdotH = dot(viewNormal, halfVec);
    return pow(max(NdotH, 0.0), max(1e-4, shininess)) * @sunSpecular.xyz * matSpec;
}
